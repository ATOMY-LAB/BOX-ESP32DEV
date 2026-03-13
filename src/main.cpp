#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "imu_module.h"
#include "gps_module.h"
#include "tft_display.h"
#include "sd_storage.h"
#include "lora_comm.h"

// ========================================
// 龙舟桨频/船速采集盒子 - 主控程序 (双核架构)
//
// Core 0: TFT显示任务 (30fps, GFXcanvas16离屏渲染)
// Core 1: 传感器采集 + SD存储 + LoRa通信 (Arduino主循环)
// ========================================

// 全局串口对象
HardwareSerial GPS_Serial(1);
HardwareSerial LoRaSerial(2);

// 模块实例
IMUModule imu;
GPSModule gps;
TFTDisplay display;
SDStorage sd_storage;
LoRaCommunication lora;

// ====== 共享数据 (Core 0/1之间通过互斥锁同步) ======
static SemaphoreHandle_t dataMutex;    // 传感器数据互斥锁
static SemaphoreHandle_t spiMutex;     // SPI总线互斥锁 (TFT与SD共用)

static IMUData shared_imu_data;
static GPSData shared_gps_data;
static SystemState shared_sys_state = {false, false, false};
static String shared_filename;
static uint32_t shared_record_count = 0;

// Core 1本地数据
static IMUData current_imu_data;
static GPSData current_gps_data;
static SystemState sys_state = {false, false, false};

// 时间管理
static unsigned long lastIMURead = 0;
static unsigned long lastSDLog = 0;
static unsigned long lastSDFlush = 0;
static unsigned long lastLoRaSend = 0;

// 录制状态切换检测 (用于停止时立即flush，确保断电安全)
static bool prev_recording = false;

// ====== SPI互斥锁辅助 ======
static inline void spiLock()   { xSemaphoreTake(spiMutex, portMAX_DELAY); }
static inline void spiUnlock() { xSemaphoreGive(spiMutex); }

// ====== LoRa命令回调 ======
void onLoRaCommand(char cmd) {
  if (cmd == '1') {
    sys_state.is_recording = true;
    Serial.println("[System] Recording started");
  } else if (cmd == '2') {
    sys_state.is_recording = false;
    Serial.println("[System] Recording stopped");
  }
}

// ====== 显示任务 (运行在Core 0, 30fps) ======
void displayTask(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  // 本地快照 (避免长时间持有锁)
  IMUData localIMU;
  GPSData localGPS;
  SystemState localState;
  String localFilename;
  uint32_t localRecCount = 0;

  while (true) {
    // 1. 快速拷贝共享数据
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      localIMU = shared_imu_data;
      localGPS = shared_gps_data;
      localState = shared_sys_state;
      localFilename = shared_filename;
      localRecCount = shared_record_count;
      xSemaphoreGive(dataMutex);
    }

    // 2. 渲染到canvas (纯内存操作，不持有SPI锁)
    display.renderFrame(localIMU, localGPS, localState, localFilename, localRecCount);

    // 3. SPI传输到屏幕 (需要SPI锁)
    spiLock();
    display.pushFrame();
    spiUnlock();

    // 4. 等待下一帧 (~33ms = 30fps)
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TFT_REFRESH_INTERVAL));
  }
}

// ====== 系统初始化 (运行在Core 1) ======
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial) delay(10);

  Serial.println("\n===== DragonBoat Collection Box =====");
  Serial.println("===== Dual-Core Architecture =====\n");

  // 创建互斥锁
  dataMutex = xSemaphoreCreateMutex();
  spiMutex  = xSemaphoreCreateMutex();

  // 初始化SPI总线
  SPI.begin();
  Serial.println("[Init] SPI Bus OK");

  // 初始化I2C总线
  Wire.begin(IIC_SDA, IIC_SCL);
  Wire.setClock(400000);
  delay(500);
  Serial.println("[Init] I2C Bus OK");

  // 初始化TFT (此时还在单核，无需SPI锁)
  int tft_retries = 3;
  while (tft_retries > 0 && !sys_state.is_tft_ready) {
    if (display.begin()) {
      sys_state.is_tft_ready = true;
      display.showStartupScreen();
      Serial.println("[Init] TFT Display OK (Canvas 160x128)");
      break;
    } else {
      tft_retries--;
      if (tft_retries > 0) {
        Serial.printf("[Init] TFT FAILED! Retrying (%d left)\n", tft_retries);
        delay(1000);
      }
    }
  }

  if (!sys_state.is_tft_ready) {
    Serial.println("[Init] WARNING: TFT unavailable");
  }

  // 初始化IMU
  if (imu.begin()) {
    Serial.println("[Init] IMU OK (burst read)");
  } else {
    Serial.println("[Init] IMU FAILED!");
  }

  // 初始化GPS
  if (gps.begin()) {
    Serial.println("[Init] GPS OK");
  } else {
    Serial.println("[Init] GPS FAILED!");
  }

  // 初始化SD卡
  if (sd_storage.begin()) {
    sys_state.is_card_ready = true;
    Serial.println("[Init] SD Card OK (binary format)");
  } else {
    Serial.println("[Init] SD Card FAILED!");
    if (sys_state.is_tft_ready) display.showError("SD Card Error");
  }

  // 初始化LoRa
  if (lora.begin()) {
    lora.setCommandCallback(onLoRaCommand);
    Serial.println("[Init] LoRa OK");
  } else {
    Serial.println("[Init] LoRa FAILED!");
  }

  // 初始化共享数据
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  shared_sys_state = sys_state;
  shared_filename = sd_storage.getFileName();
  xSemaphoreGive(dataMutex);

  // 启动显示任务 (Core 0)
  if (sys_state.is_tft_ready) {
    xTaskCreatePinnedToCore(
      displayTask,
      "DisplayTask",
      DISPLAY_TASK_STACK,
      NULL,
      DISPLAY_TASK_PRIO,
      NULL,
      DISPLAY_TASK_CORE
    );
    Serial.println("[Init] Display task started on Core 0 (30fps)");
  }

  delay(500);
  Serial.println("\n===== System Ready =====");
  Serial.println("Core 0: Display (30fps canvas)");
  Serial.println("Core 1: Sensors@50Hz + SD@50Hz + LoRa@2Hz");
  Serial.println("========================\n");
}

// ====== 主循环 (Core 1: 传感器 + SD + LoRa) ======
void loop() {
  unsigned long now = millis();

  // 1. 连续处理串口数据
  lora.processSerialData();
  gps.processSerialData();

  // 2. 采集IMU (50Hz = 20ms间隔)
  if (now - lastIMURead >= IMU_READ_INTERVAL) {
    imu.readData(current_imu_data);
    lastIMURead = now;

    // 更新GPS
    if (gps.isFixed()) {
      current_gps_data = gps.getData();
    }

    // 同步共享数据到显示任务
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      shared_imu_data = current_imu_data;
      shared_gps_data = current_gps_data;
      shared_sys_state = sys_state;
      shared_record_count = sd_storage.getRecordCount();
      xSemaphoreGive(dataMutex);
    }
  }

  // 3. SD卡写入 (50Hz，仅录制中)
  if (sys_state.is_recording && sys_state.is_card_ready &&
      (now - lastSDLog >= SD_LOG_INTERVAL)) {
    spiLock();
    sd_storage.logData(current_imu_data, current_gps_data);
    spiUnlock();
    lastSDLog = now;
  }

  // 4. SD卡定期flush (1Hz) + 回写record_count到文件头 (断电保护)
  if (sys_state.is_recording && sys_state.is_card_ready &&
      (now - lastSDFlush >= SD_FLUSH_INTERVAL)) {
    spiLock();
    sd_storage.flush();
    spiUnlock();
    lastSDFlush = now;
  }

  // 5. 录制停止时立即flush一次 (确保最后一批数据落盘)
  if (prev_recording && !sys_state.is_recording && sys_state.is_card_ready) {
    spiLock();
    sd_storage.flush();
    spiUnlock();
    Serial.println("[SD] Stop-flush done");
  }
  prev_recording = sys_state.is_recording;

  // 6. LoRa发送 (2Hz)
  if (now - lastLoRaSend >= LORA_SEND_INTERVAL) {
    lora.sendData(current_imu_data, current_gps_data);
    lastLoRaSend = now;
  }
}
