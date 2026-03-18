#include "sd_storage.h"

// 全局SdFat对象
SdFs sd;

bool SDStorage::begin() {
  Serial.print("[SD] Init starting...");
  
  // ESP32 SD卡初始化 - 使用 SPI 模式
  // 确保SD卡初始化成功（重试机制）
  for (int i = 0; i < 3; i++) {
    // 使用SdFat库的初始化方式
    if (sd.begin(SdSpiConfig(SD_CS, DEDICATED_SPI, 25000000))) {
      Serial.println("Success!");
      data_filename = findNextFileName();
      Serial.print("[SD] Log File: ");
      Serial.println(data_filename);

      // 创建文件并写入表头
      FsFile dataFile = sd.open(data_filename.c_str(), O_WRONLY | O_CREAT | O_TRUNC);
      if (dataFile) {
        if (dataFile.fileSize() == 0) {
          size_t header_written = dataFile.println("Timestamp(ms),Ax(m/s2),Ay(m/s2),Az(m/s2),Gx(deg/s),Gy(deg/s),Gz(deg/s),"
                          "Roll(deg),Pitch(deg),Yaw(deg),WGS84_Lat(deg),WGS84_Lon(deg),"
                          "GCJ02_Lat(deg),GCJ02_Lon(deg),Lat_Dir,Lon_Dir");
          if (header_written == 0) {
            Serial.println("[SD] ERROR: Header write failed!");
            dataFile.close();
            return false;
          }
          dataFile.flush();  // ✅ 确保表头写入SD卡
          delay(10);  // 等待SD卡写入完成
          Serial.print("[SD] CSV header written (");
          Serial.print(header_written);
          Serial.println(" bytes)");
        }
        dataFile.close();
        is_ready = true;
        return true;
      } else {
        Serial.println("[SD] Create file failed!");
        return false;
      }
    }
    delay(500);
    if (i < 2) {
      Serial.println("Retrying...");
    }
  }
  
  Serial.println("Failed after 3 retries!");
  is_ready = false;
  return false;
}

bool SDStorage::logData(const IMUData &imu_data, const GPSData &gps_data) {
  if (!is_ready) {
    Serial.println("[SD] SD card not ready!");
    return false;
  }

  // ✅ 使用O_WRONLY | O_CREAT | O_APPEND确保追加而不是覆盖
  FsFile dataFile = sd.open(data_filename.c_str(), O_WRONLY | O_CREAT | O_APPEND);
  if (!dataFile) {
    Serial.println("[SD] ERROR: File open failed!");
    return false;
  }

  // 记录写入前的文件大小
  size_t size_before = dataFile.fileSize();

  String csvLine = formatCSVLine(imu_data, gps_data);
  size_t written = dataFile.println(csvLine);
  
  if (written == 0) {
    Serial.println("[SD] ERROR: println() returned 0 - write failed!");
    dataFile.close();
    return false;
  }

  // ✅ 关键步骤：flush确保数据写入SD卡
  dataFile.flush();
  delay(5);  // 等待写入完成
  
  // 验证写入
  size_t size_after = dataFile.fileSize();
  dataFile.close();
  
  // 仅在出错时输出，正常只静默处理
  return true;
}

const String& SDStorage::getFileName() const {
  return data_filename;
}

bool SDStorage::isReady() const {
  return is_ready;
}

String SDStorage::findNextFileName() {
  int fileIndex = 1;
  char fileName[32];  // 固定缓冲区，避免频繁String分配
  
  while (true) {
    snprintf(fileName, sizeof(fileName), "/%d.csv", fileIndex);
    if (!sd.exists(fileName)) {
      return String(fileName);
    }
    fileIndex++;
    if (fileIndex > 1000) return "/data.csv";
  }
}

String SDStorage::formatCSVLine(const IMUData &imu_data, const GPSData &gps_data) {
  static char csvBuffer[256];  // 固定缓冲区，避免堆碎片
  
  if (gps_data.is_fixed) {
    snprintf(csvBuffer, sizeof(csvBuffer),
      "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.8f,%.8f,%.8f,%.8f,%s,%s",
      imu_data.timestamp,
      imu_data.ax, imu_data.ay, imu_data.az,
      imu_data.gx, imu_data.gy, imu_data.gz,
      imu_data.roll, imu_data.pitch, imu_data.yaw,
      gps_data.wgs84_lat, gps_data.wgs84_lon,
      gps_data.gcj02_lat, gps_data.gcj02_lon,
      gps_data.lat_dir.c_str(), gps_data.lon_dir.c_str()
    );
  } else {
    snprintf(csvBuffer, sizeof(csvBuffer),
      "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,NO_FIX,NO_FIX,NO_FIX,NO_FIX,NO_FIX,NO_FIX",
      imu_data.timestamp,
      imu_data.ax, imu_data.ay, imu_data.az,
      imu_data.gx, imu_data.gy, imu_data.gz,
      imu_data.roll, imu_data.pitch, imu_data.yaw
    );
  }
  
  return String(csvBuffer);
}

void SDStorage::checkFileStatus() {
  if (!is_ready) {
    Serial.println("[SD] SD card not ready!");
    return;
  }
  
  FsFile check = sd.open(data_filename.c_str(), O_RDONLY);
  if (check) {
    Serial.print("[SD] File: ");
    Serial.print(data_filename);
    Serial.print(" | Size: ");
    Serial.print(check.fileSize());
    Serial.println(" bytes");
    check.close();
  } else {
    Serial.print("[SD] ERROR: Cannot open file ");
    Serial.println(data_filename);
  }
}
