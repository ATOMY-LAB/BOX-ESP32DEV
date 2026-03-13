#include "sd_storage.h"

bool SDStorage::begin() {
  Serial.print("[SD] Init starting...");

  for (int i = 0; i < 3; i++) {
    if (SD.begin(SD_CS, SPI, SD_SPI_FREQ)) {
      Serial.println("Success!");
      data_filename = findNextFileName();
      Serial.print("[SD] Log File: ");
      Serial.println(data_filename);

      // 打开文件并保持句柄 (避免每次写入时open/close)
      dataFile = SD.open(data_filename.c_str(), FILE_WRITE);
      if (dataFile) {
        writeHeader();
        dataFile.flush();
        is_ready = true;
        Serial.println("[SD] Binary header written (version=2, record_count protected)");
        return true;
      } else {
        Serial.println("[SD] Create file failed!");
        return false;
      }
    }
    delay(500);
    if (i < 2) Serial.println("Retrying...");
  }

  Serial.println("Failed after 3 retries!");
  is_ready = false;
  return false;
}

void SDStorage::writeHeader() {
  BinaryFileHeader hdr;
  memcpy(hdr.magic, "DBCB", 4);
  hdr.version = 2;  // version=2: record_count字段有效
  memset(hdr.reserved, 0, sizeof(hdr.reserved));
  hdr.acc_scale   = IMU_ACC_SCALE;
  hdr.gyro_scale  = IMU_GYRO_SCALE;
  hdr.angle_scale = IMU_ANGLE_SCALE;
  hdr.gps_scale   = GPS_FIXED_SCALE;
  hdr.record_count = 0;
  hdr.pad = 0;
  dataFile.write((const uint8_t*)&hdr, sizeof(hdr));
}

// 将record_count定点回写到文件头（seekable），然后跳回文件末尾
void SDStorage::updateHeaderCount() {
  if (!dataFile) return;
  dataFile.seek(HDR_RECORD_COUNT_OFFSET);
  dataFile.write((const uint8_t*)&record_count, sizeof(record_count));
  // 回到数据末尾，以便下次追加写入
  uint32_t dataEnd = sizeof(BinaryFileHeader) + (uint32_t)record_count * sizeof(BinaryRecord);
  dataFile.seek(dataEnd);
}

bool SDStorage::logData(const IMUData &imu_data, const GPSData &gps_data) {
  if (!is_ready || !dataFile) return false;

  BinaryRecord rec;
  rec.timestamp = (uint32_t)imu_data.timestamp;
  rec.ax  = imu_data.raw_ax;
  rec.ay  = imu_data.raw_ay;
  rec.az  = imu_data.raw_az;
  rec.gx  = imu_data.raw_gx;
  rec.gy  = imu_data.raw_gy;
  rec.gz  = imu_data.raw_gz;
  rec.roll  = imu_data.raw_roll;
  rec.pitch = imu_data.raw_pitch;
  rec.yaw   = imu_data.raw_yaw;

  if (gps_data.is_fixed) {
    rec.lat  = (int32_t)(gps_data.gcj02_lat * GPS_FIXED_SCALE);
    rec.lon  = (int32_t)(gps_data.gcj02_lon * GPS_FIXED_SCALE);
    uint8_t flags = 0x01;
    if (gps_data.lat_dir == 'S') flags |= 0x02;
    if (gps_data.lon_dir == 'W') flags |= 0x04;
    rec.flags = flags;
  } else {
    rec.lat   = INT32_MIN;
    rec.lon   = INT32_MIN;
    rec.flags = 0x00;
  }
  rec.pad = 0;

  size_t written = dataFile.write((const uint8_t*)&rec, sizeof(rec));
  if (written == sizeof(rec)) {
    record_count++;
    return true;
  }
  return false;
}

void SDStorage::flush() {
  if (!is_ready || !dataFile) return;
  // 先flush确保所有数据块写入SD
  dataFile.flush();
  // 再回写record_count到文件头 → 断电后可通过此字段定位有效记录范围
  updateHeaderCount();
  // updateHeaderCount会seek，flush再次确保头部更新落盘
  dataFile.flush();
}

void SDStorage::stopRecording() {
  if (!is_ready || !dataFile) return;
  // 完整flush + 回写计数 + 关闭句柄 (目录项文件大小将被正确更新)
  flush();
  dataFile.close();
  is_ready = false;
  Serial.printf("[SD] Recording stopped: %lu records in %s\n",
                (unsigned long)record_count, data_filename.c_str());
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
  int fileIndex = 1;
  char fileName[32];

  while (true) {
    snprintf(fileName, sizeof(fileName), "/%d.bin", fileIndex);
    if (!SD.exists(fileName)) {
      return String(fileName);
    }
    fileIndex++;
    if (fileIndex > 9999) return "/data.bin";
  }
}
