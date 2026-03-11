#ifndef SD_STORAGE_H
#define SD_STORAGE_H

#include <Arduino.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include "sensors.h"
#include "config.h"

// ========================================
// SD 卡存储模块
// ========================================

/**
 * @class SDStorage
 * @brief SD卡数据记录类
 * 
 * 功能:
 * - 初始化SD卡（带重试机制）
 * - CSV格式数据记录
 * - 自动生成递增文件名
 * - 文件表头管理
 * 
 * CSV数据格式:
 *   Timestamp, Ax, Ay, Az, Gx, Gy, Gz, Roll, Pitch, Yaw,
 *   WGS84_Lat, WGS84_Lon, GCJ02_Lat, GCJ02_Lon, Lat_Dir, Lon_Dir
 * 
 * 应用场景:
 * - 记录完整的传感器原始数据用于后期分析
 * - 导入到MATLAB/Python进行桨频和船速算法开发
 */
class SDStorage {
public:
  /**
   * @brief 初始化SD卡
   * 
   * 内部包含3次重试机制，确保初始化成功
   * 自动调用 findNextFileName() 生成新文件名
   * 自动写入CSV表头
   * 
   * @return true - 初始化成功, false - 初始化失败
   */
  bool begin();

  /**
   * @brief 记录IMU + GPS数据到SD卡
   * 
   * 将当前采集到的IMU和GPS数据以CSV行格式写入文件
   * GPS无定位时，对应字段填入"NO_FIX"
   * 
   * @param imu_data - IMU传感器数据
   * @param gps_data - GPS定位数据
   * @return true - 写入成功, false - 写入失败
   */
  bool logData(const IMUData &imu_data, const GPSData &gps_data);

  /**
   * @brief 获取当前日志文件名
   * @return 文件名字符串 (例: "/1.csv")
   */
  const String& getFileName() const;

  /**
   * @brief 查询SD卡就绪状态
   * @return true - 就绪, false - 未就绪
   */
  bool isReady() const;

  /**
   * @brief 检查当前日志文件大小
   * 
   * 用于诊断，打印文件状态信息到串口
   */
  void checkFileStatus();

private:
  String data_filename;
  bool is_ready;

  /**
   * @brief 查找下一个可用的文件名
   * 
   * 扫描SD卡，从1.csv开始查找不存在的文件名
   * 最多尝试1000个文件，超出限制则使用 data.csv
   * 
   * @return 新文件名字符串
   */
  String findNextFileName();

  /**
   * @brief 格式化CSV数据行
   * 
   * @param imu_data - IMU数据
   * @param gps_data - GPS数据
   * @return CSV格式的数据行
   */
  String formatCSVLine(const IMUData &imu_data, const GPSData &gps_data);
};

#endif  // SD_STORAGE_H
