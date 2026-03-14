#include "sd_storage.h"

// ---- 私有: 单次初始化尝试 (被begin()和tryReinit()复用) ----
bool SDStorage::doBegin() {
  // 先重置内部状态与SD硬件层 (适应热插拔重入)
  if (dataFile) dataFile.close();
  SD.end();
  is_ready    = false;
  csv_buf_pos = 0;
  record_count = 0;
  data_filename = "";

  if (!SD.begin(SD_CS, SPI, SD_SPI_FREQ)) return false;

  data_filename = findNextFileName();
  dataFile = SD.open(data_filename.c_str(), FILE_WRITE);
  if (!dataFile) {
    SD.end();
    return false;
  }

  writeHeader();
  dataFile.flush();
  is_ready = true;
  return true;
}

// ---- 公开: 启动时初始化 (含重试) ----
bool SDStorage::begin() {
  Serial.print("[SD] Init starting...");
  for (int i = 0; i < 3; i++) {
    if (doBegin()) {
      Serial.println("OK!");
      Serial.printf("[SD] Log file: %s\n", data_filename.c_str());
      return true;
    }
    if (i < 2) {
      Serial.println("Retrying...");
      delay(500);
    }
  }
  Serial.println("Failed after 3 retries!");
  return false;
}

// ---- 公开: 热插拔 - 插卡重新初始化 (单次尝试, 无延迟) ----
bool SDStorage::tryReinit() {
  if (doBegin()) {
    Serial.printf("[SD] Card re-inserted! File: %s\n", data_filename.c_str());
    return true;
  }
  return false;
}

// ---- 公开: 热插拔 - 拔卡清理 (需在spiLock内调用) ----
void SDStorage::onCardRemoved() {
  uint32_t lost = record_count;
  size_t   lost_bytes = csv_buf_pos;

  if (dataFile) dataFile.close();   // 尽力关闭文件句柄 (SPI可能已失效, 会静默失败)
  SD.end();                          // 释放FATFS挂载状态

  is_ready      = false;
  csv_buf_pos   = 0;    // 丢弃缓冲区中未写入的数据
  record_count  = 0;
  data_filename = "";

  Serial.printf("[SD] Card removed! Records: %lu, buffer discarded: %u bytes\n",
                (unsigned long)lost, (unsigned)lost_bytes);
}



void SDStorage::writeHeader() {
  // 列名与 logData() 格式一一对应; 解码系数见 config.h
  const char *header =
    "timestamp_ms,"
    "raw_ax,raw_ay,raw_az,"
    "raw_gx,raw_gy,raw_gz,"
    "raw_roll,raw_pitch,raw_yaw,"
    "lat_e7,lon_e7,gps_fix\r\n";
  dataFile.write((const uint8_t *)header, strlen(header));
}

bool SDStorage::logData(const IMUData &imu_data, const GPSData &gps_data) {
  if (!is_ready) return false;

  // 格式化 CSV 整数行到栈缓冲区
  // IMU: 直接写 int16 原始值, 比 %f 浮点格式化快 ~3x, 行长 ~75B
  // GPS: lat/lon 乘 1e7 存为 int32, 精度 ~1cm, 无小数点格式化开销
  char row[120];
  int  len;

  if (gps_data.is_fixed) {
    int32_t lat_e7 = (int32_t)(gps_data.gcj02_lat * 1e7) * (gps_data.lat_dir == 'S' ? -1 : 1);
    int32_t lon_e7 = (int32_t)(gps_data.gcj02_lon * 1e7) * (gps_data.lon_dir == 'W' ? -1 : 1);
    len = snprintf(row, sizeof(row),
      "%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,1\r\n",
      imu_data.timestamp,
      imu_data.raw_ax, imu_data.raw_ay, imu_data.raw_az,
      imu_data.raw_gx, imu_data.raw_gy, imu_data.raw_gz,
      imu_data.raw_roll, imu_data.raw_pitch, imu_data.raw_yaw,
      (long)lat_e7, (long)lon_e7);
  } else {
    len = snprintf(row, sizeof(row),
      "%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,,,0\r\n",
      imu_data.timestamp,
      imu_data.raw_ax, imu_data.raw_ay, imu_data.raw_az,
      imu_data.raw_gx, imu_data.raw_gy, imu_data.raw_gz,
      imu_data.raw_roll, imu_data.raw_pitch, imu_data.raw_yaw);
  }

  if (len <= 0 || len >= (int)sizeof(row)) return false;

  if (csv_buf_pos + (size_t)len > CSV_BUF_SIZE) {
    Serial.println("[SD] WARN: write buffer full, data dropped!");
    return false;
  }

  memcpy(csv_buf + csv_buf_pos, row, len);
  csv_buf_pos += len;
  record_count++;
  // 此函数不访问SD卡，不需要SPI锁
  return true;
}

// 将 csv_buf 写入 FATFS 内部扇区缓冲 (通常0~5ms SPI, 需在spiLock内调用)
// 返回 false 表示写入失败 (卡满 / 卡故障)，调用方应触发卡移除流程
bool SDStorage::flushData() {
  if (!is_ready || !dataFile || csv_buf_pos == 0) return true;
  size_t to_write = csv_buf_pos;
  size_t written  = dataFile.write((const uint8_t *)csv_buf, to_write);
  csv_buf_pos = 0;  // 无论成功失败都丢弃缓冲，避免下次重复写入损坏文件
  if (written != to_write) {
    Serial.printf("[SD] WARN: flushData write error! wrote %u/%u bytes. Card full or removed?\n",
                  (unsigned)written, (unsigned)to_write);
    return false;
  }
  return true;
}

// flushData() + 强制FAT/目录项落盘 (10~30ms SPI, 需在spiLock内调用)
// 返回 false 表示底层写入失败，调用方应触发卡移除流程
bool SDStorage::flush() {
  if (!is_ready || !dataFile) return true;
  if (!flushData()) return false;
  dataFile.flush();
  return true;
}


const String& SDStorage::getFileName() const {
  return data_filename;
}

bool SDStorage::isReady() const {
  return is_ready;
}

uint32_t SDStorage::getRecordCount() const {
  return record_count;
}

String SDStorage::findNextFileName() {
  char fileName[32];
  for (int i = 1; i <= 9999; i++) {
    snprintf(fileName, sizeof(fileName), "/%d.csv", i);
    if (!SD.exists(fileName)) return String(fileName);
  }
  // 所有数字槽位已满，用毫秒时间戳生成唯一名称（避免覆盖已有文件）
  snprintf(fileName, sizeof(fileName), "/ov_%lu.csv", (unsigned long)millis());
  Serial.printf("[SD] WARN: all 9999 slots full, using fallback: %s\n", fileName);
  return String(fileName);
}
