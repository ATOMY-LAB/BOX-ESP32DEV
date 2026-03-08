#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "imu_module.h"
#include "gps_module.h"
#include "tft_display.h"
#include "sd_storage.h"
#include "lora_comm.h"

// ========================================
// 龙舟桨频/船速采集盒子 - 主控程序
// ========================================

// 全局串口对象
HardwareSerial GPS_Serial(1);
HardwareSerial LoRaSerial(2);

// 创建各模块实例
IMUModule imu;
GPSModule gps;
TFTDisplay display;
SDStorage sd_storage;
LoRaCommunication lora;

// 系统状态
SystemState sys_state = {false, false, false};
IMUData current_imu_data;
GPSData current_gps_data;

// 时间管理变量
unsigned long lastIMURead = 0;
unsigned long lastTFTRefresh = 0;
unsigned long lastSDLog = 0;
unsigned long lastLoRaSend = 0;

/**
 * @brief LoRa命令回调函数
 * 
 * 处理来自LoRa的命令:
 * '1' - 开始记录
 * '2' - 停止记录
 * 
 * @param cmd - 命令字符
 */
void onLoRaCommand(char cmd) {
  if (cmd == '1') {
    sys_state.is_recording = true;
    Serial.println("[System] Recording started");
  } else if (cmd == '2') {
    sys_state.is_recording = false;
    Serial.println("[System] Recording stopped");
  }
}

/**
 * @brief 系统初始化
 * 
 * 初始化顺序:
 * 1. Serial通信（调试输出）
 * 2. SPI总线（TFT和SD卡共用）
 * 3. I2C总线（IMU）
 * 4. TFT显示屏
 * 5. IMU传感器
 * 6. GPS模块
 * 7. SD卡存储
 * 8. LoRa通信
 */
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial) delay(10);
  
  Serial.println("\n\n===== DragonBoat Collection Box =====");
  Serial.println("===== System Initialization Start =====\n");

  // 初始化SPI总线（TFT+TF卡共用）
  SPI.begin();
  Serial.println("[Init] SPI Bus OK");

  // 初始化I2C总线（IMU）
  Wire.begin(IIC_SDA, IIC_SCL);
  Wire.setClock(400000);
  delay(500);
  Serial.println("[Init] I2C Bus OK");

  // 初始化TFT显示屏（支持重试机制）
  int tft_retries = 3;
  while (tft_retries > 0 && !sys_state.is_tft_ready) {
    if (display.begin()) {
      sys_state.is_tft_ready = true;
      display.drawStaticUI();
      display.showStartupMessage("Initializing...");
      Serial.println("[Init] TFT Display OK");
      break;
    } else {
      tft_retries--;
      if (tft_retries > 0) {
        Serial.print("[Init] TFT Display FAILED! Retrying... (");
        Serial.print(tft_retries);
        Serial.println(" retries left)");
        delay(1000);
      }
    }
  }
  
  if (!sys_state.is_tft_ready) {
    Serial.println("[Init] WARNING: TFT Display unavailable. Continuing without display.");
    Serial.println("       All data will be logged to SD card and LoRa transmission.");
  }

  // 初始化IMU
  if (imu.begin()) {
    Serial.println("[Init] IMU Sensor OK");
  } else {
    Serial.println("[Init] IMU Sensor FAILED!");
  }

  // 初始化GPS
  if (gps.begin()) {
    Serial.println("[Init] GPS Module OK");
  } else {
    Serial.println("[Init] GPS Module FAILED!");
  }

  // 初始化SD卡
  if (sd_storage.begin()) {
    sys_state.is_card_ready = true;
    Serial.println("[Init] SD Card OK");
  } else {
    Serial.println("[Init] SD Card FAILED!");
    display.showError("SD Card Error");
  }

  // 初始化LoRa通信
  if (lora.begin()) {
    lora.setCommandCallback(onLoRaCommand);
    Serial.println("[Init] LoRa Module OK");
  } else {
    Serial.println("[Init] LoRa Module FAILED!");
  }

  delay(500);
  Serial.println("\n===== System Initialization Complete =====");
  Serial.println("Functions:");
  Serial.println("  1. LoRa Command Control (1=Start, 2=Stop)");
  Serial.println("  2. IMU Data Acquisition");
  Serial.println("  3. GPS Positioning");
  Serial.println("  4. TFT Real-time Display");
  Serial.println("  5. SD Card Data Logging");
  Serial.println("  6. LoRa Data Transmission");
  Serial.println("=====================================\n");
}

/**
 * @brief 主循环
 * 
 * 时序说明:
 * - IMU采集: 100ms间隔
 * - TFT刷新: 200ms间隔
 * - SD卡写入: 100ms间隔（与IMU采集同步）
 * - LoRa发送: 500ms间隔
 * - GPS处理: 连续处理接收缓冲区
 * - LoRa命令: 连续处理接收缓冲区
 */
void loop() {
  unsigned long now = millis();

  // 1. 处理LoRa命令（优先级最高）
  lora.processSerialData();

  // 2. 处理GPS数据（实时接收）
  gps.processSerialData();

  // 3. 采集IMU数据（100ms周期）
  if (now - lastIMURead >= IMU_READ_INTERVAL) {
    imu.readData(current_imu_data);
    lastIMURead = now;
  }

  // 4. 更新GPS状态指示
  if (gps.isFixed()) {
    current_gps_data = gps.getData();
  }

  // 5. 刷新TFT显示（200ms周期，实时显示，不受记录状态影响）
  if (now - lastTFTRefresh >= TFT_REFRESH_INTERVAL) {
    display.updateDisplay(current_imu_data, current_gps_data, 
                         sys_state, sd_storage.getFileName());
    lastTFTRefresh = now;
  }

  // 6. 写入SD卡（100ms周期，仅在记录状态且SD卡就绪时）
  if (sys_state.is_recording && sys_state.is_card_ready && 
      (now - lastSDLog >= SD_LOG_INTERVAL)) {
    sd_storage.logData(current_imu_data, current_gps_data);
    lastSDLog = now;
  }

  // 7. 定时发送LoRa数据（500ms周期，持续发送）
  if (now - lastLoRaSend >= LORA_SEND_INTERVAL) {
    lora.sendData(current_imu_data, current_gps_data);
    lastLoRaSend = now;
  }

  delay(5);  // CPU休息，防止看门狗超时
}
