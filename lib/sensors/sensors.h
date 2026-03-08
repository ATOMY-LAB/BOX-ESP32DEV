#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

// ========================================
// 传感器数据结构定义
// ========================================

/**
 * @struct IMUData
 * @brief 惯性测量单元(IMU)数据结构
 * 
 * 用于存储加速度、角速度和欧拉角数据
 * 数据来源: JY901 IMU传感器 (I2C接口)
 */
struct IMUData {
  float ax, ay, az;           // 加速度 (m/s²) - 用于估计桨频
  float gx, gy, gz;           // 角速度 (°/s)
  float roll, pitch, yaw;     // 欧拉角 (°)
  unsigned long timestamp;    // 时间戳 (ms)
};

/**
 * @struct GPSData
 * @brief GPS定位数据结构
 * 
 * 用于存储位置和速度信息
 * 数据来源: GPS模块 (UART1接口，NMEA-0183格式)
 */
struct GPSData {
  double wgs84_lat;          // WGS84 原始纬度 (度)
  double wgs84_lon;          // WGS84 原始经度 (度)
  double gcj02_lat;          // GCJ02 转换后纬度 (度) - 中国坐标系
  double gcj02_lon;          // GCJ02 转换后经度 (度) - 中国坐标系
  String lat_dir;            // 纬度方向 (N/S)
  String lon_dir;            // 经度方向 (E/W)
  bool is_fixed;             // 是否定位成功
  unsigned long timestamp;   // 时间戳 (ms)
};

/**
 * @struct SystemState
 * @brief 系统运行状态
 * 
 * 用于跟踪各个子系统的状态
 */
struct SystemState {
  bool is_recording;         // 是否正在记录到SD卡
  bool is_card_ready;        // SD卡是否就绪
  // GPS定位状态由 gps.isFixed() 单一源提供，避免重复
  bool is_tft_ready;         // TFT屏幕是否就绪
};

#endif  // SENSORS_H
