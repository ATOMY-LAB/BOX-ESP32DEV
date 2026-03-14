#ifndef SD_STORAGE_H
#define SD_STORAGE_H

#include <Arduino.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include "sensors.h"
#include "config.h"

// ========================================
// CSV 整数格式存储 (int16 IMU + int32 GPS)
// 文件名: /1.csv, /2.csv, ...
//
// 列说明 (首行为标题):
//   timestamp_ms              — 毫秒时间戳 (uint32)
//   raw_ax/ay/az              — 加速度原始值 (int16), 乘 IMU_ACC_SCALE 得 m/s²
//   raw_gx/gy/gz              — 角速度原始值 (int16), 乘 IMU_GYRO_SCALE 得 °/s
//   raw_roll/pitch/yaw        — 欧拉角原始值 (int16), 乘 IMU_ANGLE_SCALE 得 °
//   lat_e7, lon_e7            — GCJ02坐标 × 10^7 (int32), 南纬/西经为负
//   gps_fix                   — GPS定位状态 (0/1)
//
// 数字格式化改进 (vs 浮点CSV):
//   snprintf %d/%ld 比 %f 快 ~3x; 行长 ~75B vs ~120B
//
// 两级flush (消除TFT抽搐根因):
//   logData()    — 只写RAM缓冲 csv_buf, 无SPI, 无锁 (50Hz)
//   flushData()  — csv_buf→FATFS内部缓冲, 通常0~5ms SPI (需spiLock, 5Hz)
//   flush()      — flushData() + dataFile.flush() FAT更新, 10~30ms (需spiLock, 0.2Hz)
//   根因: dataFile.flush()每次需更新FAT/目录项扇区, 降频到0.2Hz后每5s仅一次长阻塞
//
// 热插拔:
//   onCardRemoved() — 拔卡清理 (需spiLock)
//   tryReinit()     — 插卡重新初始化, 单次无延迟 (需spiLock)
// ========================================

static const size_t CSV_BUF_SIZE = 8192;  // 8KB RAM缓冲 (~10s数据量 @75B/行 50Hz)

class SDStorage {
public:
  bool begin();                  // 启动时初始化 (含3次重试)
  bool logData(const IMUData &imu_data, const GPSData &gps_data);
  bool flushData();              // csv_buf→FATFS层, 快速 (需在spiLock内调用, 5Hz); 写入失败返回false
  bool flush();                  // flushData() + FAT更新, 慢 (需在spiLock内调用, 0.2Hz); 写入失败返回false
  void onCardRemoved();          // 拔卡处理 (需在spiLock内调用)
  bool tryReinit();              // 插卡重新初始化, 单次无延迟 (需在spiLock内调用)
  const String& getFileName() const;
  bool isReady() const;
  uint32_t getRecordCount() const;

private:
  String   data_filename;
  fs::File dataFile;
  bool     is_ready    = false;
  uint32_t record_count = 0;

  char   csv_buf[CSV_BUF_SIZE];
  size_t csv_buf_pos = 0;

  bool   doBegin();
  String findNextFileName();
  void   writeHeader();
};

#endif
