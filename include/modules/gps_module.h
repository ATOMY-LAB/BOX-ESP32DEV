#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "sensors.h"
#include "../config.h"

// ========================================
// GPS 定位模块
// ========================================

// 全局 GPS 串口对象
extern HardwareSerial GPS_Serial;

/**
 * @class GPSModule
 * @brief GPS定位和速度采集类
 * 
 * 功能:
 * - UART1接收GPS模块NMEA-0183格式数据
 * - 解析GGA语句（位置信息）
 * - WGS84到GCJ02坐标系转换
 * - 提供定位状态和速度信息
 * 
 * 应用场景:
 * - 从GPS SOG字段获取龙舟实时船速
 * - 记录采集地点的地理位置
 * - 调试数据回放时用于时空参照
 */
class GPSModule {
public:
  /**
   * @brief 初始化GPS模块
   * @return true - 初始化成功, false - 初始化失败
   */
  bool begin();

  /**
   * @brief 处理UART接收到的GPS数据
   * 
   * 应在loop()中频繁调用，处理接收缓冲区数据
   * 自动识别NMEA语句并解析
   */
  void processSerialData();

  /**
   * @brief 获取最新的GPS数据
   * @return GPSData 结构体引用
   */
  const GPSData& getData() const;

  /**
   * @brief 查询GPS定位状态
   * @return true - 已定位, false - 未定位
   */
  bool isFixed() const;

private:

  GPSData gps_data;
  char nmea_buffer[256];
  int buffer_index;
  bool is_gga_valid;

  /**
   * @brief 解析GGA语句
   * 
   * GGA格式: $GPGGA,hhmmss.ss,ddmm.mmmm,a,dddmm.mmmm,a,x,xx,...*hh
   * 提取: 时间、纬度、纬度方向、经度、经度方向、定位质量
   * 
   * @param gga_str - 完整的GGA语句字符串
   */
  void parseGGA(const String &gga_str);

  /**
   * @brief DDM(度分)格式转换为DD(十进制度)
   * 
   * @param ddm_str - 度分格式字符串 (例: "3137.2345")
   * @return 十进制度数值
   */
  double ddm2dd(const String &ddm_str);

  /**
   * @brief WGS84到GCJ02坐标转换
   * 
   * 中国地图采用GCJ02坐标系统，此函数用于将GPS获取的
   * WGS84坐标转换为GCJ02坐标，以便在高德地图等应用中使用
   * 
   * @param wgs_lat - WGS84纬度
   * @param wgs_lon - WGS84经度
   * @param gcj_lat - [输出] GCJ02纬度
   * @param gcj_lon - [输出] GCJ02经度
   */
  void wgs84ToGcj02(double wgs_lat, double wgs_lon, 
                     double &gcj_lat, double &gcj_lon);

  /**
   * @brief 检查坐标是否在中国范围内
   * @param lat - 纬度
   * @param lon - 经度
   * @return true - 在中国范围外, false - 在中国范围内
   */
  bool outOfChina(double lat, double lon);

  /**
   * @brief 纬度变换辅助函数
   */
  double transformLat(double x, double y);

  /**
   * @brief 经度变换辅助函数
   */
  double transformLon(double x, double y);
};

#endif  // GPS_MODULE_H
