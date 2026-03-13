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

// SD卡二进制存储格式 - 文件头 (32字节)
// version=2: 新增 record_count 字段支持断电恢复
// 断电恢复: 每次flush后会回写record_count，读取时以此为准
struct __attribute__((packed)) BinaryFileHeader {
  char magic[4];             // bytes 0-3:  "DBCB"
  uint8_t version;           // byte  4:    格式版本 = 2
  uint8_t reserved[3];       // bytes 5-7:  保留
  float acc_scale;           // bytes 8-11: 加速度缩放因子
  float gyro_scale;          // bytes 12-15:角速度缩放因子
  float angle_scale;         // bytes 16-19:欧拉角缩放因子
  int32_t gps_scale;         // bytes 20-23:GPS坐标缩放因子 (1e7)
  uint32_t record_count;     // bytes 24-27:已写入记录数 (flush时更新)
  uint32_t pad;              // bytes 28-31:对齐到32字节
};
// record_count在文件头中的字节偏移量 (用于flush时定点回写)
static constexpr uint32_t HDR_RECORD_COUNT_OFFSET = 24;

// SD卡二进制存储格式 - 数据记录 (每条32字节 vs CSV ~130字节)
struct __attribute__((packed)) BinaryRecord {
  uint32_t timestamp;        // 4B: millis()
  int16_t ax, ay, az;        // 6B: 原始加速度
  int16_t gx, gy, gz;        // 6B: 原始角速度
  int16_t roll, pitch, yaw;  // 6B: 原始欧拉角
  int32_t lat;               // 4B: 纬度 * 1e7 (INT32_MIN = no fix)
  int32_t lon;               // 4B: 经度 * 1e7
  uint8_t flags;             // 1B: bit0=fix, bit1=S, bit2=W
  uint8_t pad;               // 1B: 对齐到32字节
};

#endif
