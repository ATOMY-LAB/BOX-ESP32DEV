#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

// ========================================
// 传感器数据结构定义
// ========================================

struct IMUData {
  float ax, ay, az;           // 加速度 (m/s²)
  float gx, gy, gz;           // 角速度 (°/s)
  float roll, pitch, yaw;     // 欧拉角 (°)
  int16_t raw_ax, raw_ay, raw_az;   // 原始16位加速度
  int16_t raw_gx, raw_gy, raw_gz;   // 原始16位角速度
  int16_t raw_roll, raw_pitch, raw_yaw; // 原始16位欧拉角
  unsigned long timestamp;
};

struct GPSData {
  double wgs84_lat;
  double wgs84_lon;
  double gcj02_lat;
  double gcj02_lon;
  char lat_dir;              // 'N'/'S' (单字符，避免String堆分配)
  char lon_dir;              // 'E'/'W'
  bool is_fixed;
  unsigned long timestamp;
};

struct SystemState {
  bool is_recording;
  bool is_card_ready;
  bool is_tft_ready;
};

#endif
