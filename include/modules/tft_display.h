#ifndef TFT_DISPLAY_H
#define TFT_DISPLAY_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "sensors.h"
#include "../config.h"

// ========================================
// TFT 显示屏模块
// ========================================

/**
 * @class TFTDisplay
 * @brief ST7735S TFT液晶屏显示类
 * 
 * 功能:
 * - 管理160x128 ST7735S TFT屏幕
 * - 实时显示IMU、GPS、系统状态信息
 * - SPI总线共享管理（与SD卡共用）
 * 
 * UI布局:
 * - 顶部: 标题栏 + 记录状态指示灯
 * - 中间: IMU数据 (加速度、角速度、欧拉角)
 * - 中下: GPS定位状态
 * - 底部: 记录状态 + 文件信息
 */
class TFTDisplay {
public:
  /**
   * @brief 构造函数
   */
  TFTDisplay();

  /**
   * @brief 初始化TFT屏幕
   * @return true - 初始化成功, false - 初始化失败
   */
  bool begin();

  /**
   * @brief 绘制静态界面框架
   * 
   * 包含: 标题栏、所有静态标签文字
   * 应在系统启动时只调用一次
   */
  void drawStaticUI();

  /**
   * @brief 更新动态数据显示（实时刷新）
   * 
   * @param imu_data - IMU传感器数据
   * @param gps_data - GPS定位数据
   * @param sys_state - 系统状态
   * @param filename - 当前SD卡日志文件名
   */
  void updateDisplay(const IMUData &imu_data, 
                    const GPSData &gps_data,
                    const SystemState &sys_state,
                    const String &filename);

  /**
   * @brief 显示错误消息
   * @param error_msg - 错误文本
   */
  void showError(const String &error_msg);

  /**
   * @brief 显示启动信息
   * @param msg - 启动信息文本
   */
  void showStartupMessage(const String &msg);

private:
  Adafruit_ST7735 tft;

  /**
   * @brief SPI总线获取（CS拉低）
   * 
   * TFT和SD卡共用SPI总线，使用CS信号选择
   */
  void busTake();

  /**
   * @brief SPI总线释放（CS拉高）
   */
  void busRelease();

  /**
   * @brief 绘制记录状态指示灯
   * @param is_recording - 是否正在记录
   */
  void drawStatusLED(bool is_recording);

  /**
   * @brief 更新加速度显示
   * @param ax, ay, az - 加速度值
   */
  void updateAcceleration(float ax, float ay, float az);

  /**
   * @brief 更新角速度显示
   * @param gx, gy, gz - 角速度值
   */
  void updateGyroscope(float gx, float gy, float gz);

  /**
   * @brief 更新欧拉角显示
   * @param roll, pitch, yaw - 欧拉角值
   */
  void updateAttitude(float roll, float pitch, float yaw);

  /**
   * @brief 更新GPS显示
   * @param gps_data - GPS数据
   */
  void updateGPS(const GPSData &gps_data);

  /**
   * @brief 更新记录状态文本
   * @param is_recording - 是否正在记录
   */
  void updateRecordingState(bool is_recording);

  /**
   * @brief 更新文件名显示
   * @param filename - 文件名
   */
  void updateFileName(const String &filename);
};

#endif  // TFT_DISPLAY_H
