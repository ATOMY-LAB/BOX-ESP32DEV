#include "sd_storage.h"

bool SDStorage::begin() {
  Serial.print("[SD] Init starting...");
  
  // ESP32 SD卡初始化 - 使用 SPI 模式
  // 确保SD卡初始化成功（重试机制）
  for (int i = 0; i < 3; i++) {
    // 对于 ESP32，使用简化的 SD.begin() 调用
    // 也可以指定时钟频率：SD.begin(SD_CS, SPI, 4000000)
    if (SD.begin(SD_CS, SPI, 25000000)) {
      Serial.println("Success!");
      data_filename = findNextFileName();
      Serial.print("[SD] Log File: ");
      Serial.println(data_filename);

      // 创建文件并写入表头
      File dataFile = SD.open(data_filename.c_str(), FILE_WRITE);
      if (dataFile) {
        if (dataFile.size() == 0) {
          dataFile.println("Timestamp(ms),Ax(m/s2),Ay(m/s2),Az(m/s2),Gx(deg/s),Gy(deg/s),Gz(deg/s),"
                          "Roll(deg),Pitch(deg),Yaw(deg),WGS84_Lat(deg),WGS84_Lon(deg),"
                          "GCJ02_Lat(deg),GCJ02_Lon(deg),Lat_Dir,Lon_Dir");
          Serial.println("[SD] CSV header written");
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

  File dataFile = SD.open(data_filename.c_str(), FILE_WRITE);
  if (!dataFile) {
    Serial.println("[SD] File open failed!");
    return false;
  }

  String csvLine = formatCSVLine(imu_data, gps_data);
  dataFile.println(csvLine);
  dataFile.close();

  Serial.print("[SD] Log: ");
  Serial.println(csvLine);
  
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
    if (!SD.exists(fileName)) {
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
