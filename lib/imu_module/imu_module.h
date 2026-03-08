#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <Arduino.h>
#include <Wire.h>
#include "sensors.h"
#include "config.h"

// ========================================
// IMU 传感器模块 - JY901
// ========================================

/**
 * @class IMUModule
 * @brief 惯性测量单元(IMU)驱动类
 * 
 * 功能:
 * - 通过I2C读取JY901 IMU传感器数据
 * - 解析加速度、角速度、欧拉角
 * - 数据缩放转换
 * 
 * 应用场景:
 * - 通过加速度变化估计龙舟桨频
 * - 通过角速度和欧拉角监测船体运动
 */
class IMUModule {
public:
  /**
   * @brief 初始化IMU模块
   * @return true - 初始化成功, false - 初始化失败
   */
  bool begin();

  /**
   * @brief 读取IMU数据
   * @param data - 传出参数，包含所有IMU数据
   * @return true - 读取成功, false - 读取失败
   */
  bool readData(IMUData &data);

  /**
   * @brief 获取上一次读取的IMU数据
   * @return IMUData 结构体引用
   */
  const IMUData& getLastData() const;

private:
  IMUData currentData;

  /**
   * @brief 从JY901寄存器读取16位有符号数据
   * @param dev_addr - 设备地址
   * @param reg_addr - 寄存器地址
   * @return 16位有符号数值
   */
  int16_t readRegister(uint8_t dev_addr, uint8_t reg_addr);

  /**
   * @brief 数据缩放转换
   * @param raw_value - 原始值
   * @param scale - 缩放因子
   * @return 转换后的物理量值
   */
  float scaleData(int16_t raw_value, float scale);
};

#endif  // IMU_MODULE_H
