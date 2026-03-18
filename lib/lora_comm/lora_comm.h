#ifndef LORA_COMM_H
#define LORA_COMM_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "sensors.h"
#include "config.h"

// ========================================
// LoRa 无线通信模块
// ========================================

// 全局 LoRa 串口对象
extern HardwareSerial LoRaSerial;

/**
 * @typedef LoRaCommandCallback
 * @brief LoRa命令接收回调函数指针
 * 
 * @param cmd - 接收到的命令字符
 */
typedef void (*LoRaCommandCallback)(char cmd);

/**
 * @class LoRaCommunication
 * @brief LoRa无线通信类
 * 
 * 功能:
 * - UART2接收LoRa模块数据
 * - 命令接收与处理 ('1'=开始记录, '2'=停止记录)
 * - 周期性发送IMU+GPS数据到LoRa模块
 * - 支持命令回调机制
 * 
 * 应用场景:
 * - 岸边远程控制龙舟盒子的记录开关
 * - 实时接收采集盒子的IMU和GPS数据
 * - 无线数据链路，支持多个接收器
 * 
 * 新LoRa协议格式:
 *   发送: $LORA,DEV2,DATA,TS:12345|AX:9.80|AY:1.23|ROLL:0.45|PITCH:2.34|LAT:31.2345|LON:120.5678\n
 *   接收: $LORA,DEV2,CMD,START\n 或 $LORA,DEV2,CMD,STOP\n
 *   (设备ID从config.h的DEVICE_ID读取，支持多设备过滤)
 */
class LoRaCommunication {
public:
  /**
   * @brief 初始化LoRa通信
   * @return true - 初始化成功, false - 初始化失败
   */
  bool begin();

  /**
   * @brief 处理UART接收到的LoRa数据
   * 
   * 应在loop()中频繁调用，处理接收缓冲区数据
   * 自动识别命令字符并触发回调
   */
  void processSerialData();

  /**
   * @brief 发送IMU + GPS数据
   * 
   * 将当前IMU和GPS数据打包为单行文本格式并发送
   * 格式: TS:xxx,AX:xxx,AY:xxx,ROLL:xxx,PITCH:xxx,LAT:xxx,LON:xxx\r\n
   * 
   * @param imu_data - IMU传感器数据
   * @param gps_data - GPS定位数据
   */
  void sendData(const IMUData &imu_data, const GPSData &gps_data);

  /**
   * @brief 注册命令接收回调函数
   * 
   * 当接收到'1'或'2'命令时，会调用注册的回调函数
   * 
   * @param callback - 回调函数指针
   */
  void setCommandCallback(LoRaCommandCallback callback);

  /**
   * @brief 发送自定义文本数据
   * 
   * 用于调试或发送其他自定义格式的数据
   * 
   * @param data - 文本数据
   */
  void sendCustomData(const String &data);

private:
  LoRaCommandCallback command_callback;
  static const uint16_t RX_BUFFER_SIZE = 256;
  char rx_buffer[RX_BUFFER_SIZE];
  uint16_t rx_buffer_index;

  /**
   * @brief 处理接收到的LoRa包
   * 新协议: $LORA,设备ID,CMD,内容\n
   * 
   * @param packet - 完整的接收包
   */
  void processLoRaPacket(const String &packet);

  /**
   * @brief 处理接收到的命令
   * 
   * @param cmd - 命令字符 ('1' 或 '2')
   */
  void handleCommand(char cmd);
};

#endif  // LORA_COMM_H
