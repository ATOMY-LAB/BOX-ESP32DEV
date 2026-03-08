#include "modules/sd_storage.h"

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
  String fileName;
  while (true) {
    fileName = "/" + String(fileIndex) + ".csv";
    if (!SD.exists(fileName.c_str())) {
      return fileName;
    }
    fileIndex++;
    if (fileIndex > 1000) return "/data.csv";
  }
}

String SDStorage::formatCSVLine(const IMUData &imu_data, const GPSData &gps_data) {
  String csvLine = "";
  
  // IMU数据（必录）
  csvLine += String(imu_data.timestamp) + ",";
  csvLine += String(imu_data.ax, 2) + ",";
  csvLine += String(imu_data.ay, 2) + ",";
  csvLine += String(imu_data.az, 2) + ",";
  csvLine += String(imu_data.gx, 2) + ",";
  csvLine += String(imu_data.gy, 2) + ",";
  csvLine += String(imu_data.gz, 2) + ",";
  csvLine += String(imu_data.roll, 2) + ",";
  csvLine += String(imu_data.pitch, 2) + ",";
  csvLine += String(imu_data.yaw, 2) + ",";
  
  // GPS数据（无定位填NO_FIX）
  if (gps_data.is_fixed) {
    csvLine += String(gps_data.wgs84_lat, 6) + ",";
    csvLine += String(gps_data.wgs84_lon, 6) + ",";
    csvLine += String(gps_data.gcj02_lat, 6) + ",";
    csvLine += String(gps_data.gcj02_lon, 6) + ",";
    csvLine += gps_data.lat_dir + ",";
    csvLine += gps_data.lon_dir;
  } else {
    csvLine += "NO_FIX,NO_FIX,NO_FIX,NO_FIX,NO_FIX,NO_FIX";
  }
  
  return csvLine;
}
