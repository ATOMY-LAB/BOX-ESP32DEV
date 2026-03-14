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
//
// Core 0: displayTask — TFT 24fps (GFXcanvas16离屏渲染)
// Core 1: imuTask    — IMU精确50Hz采样 (vTaskDelayUntil, 优先级2)
//         loopTask   — GPS/LoRa/SD/热插拔 (Arduino loop, 优先级1)
//
// IMU与GPS/SD/LoRa任务分离是解决IMU数据抖动的根本手段:
//   原因: loop()中GPS串口解析、mutex等待等随机延迟污染millis()轮询的采样时机
//   修复: imuTask用vTaskDelayUntil精确调度, 高优先级抢占loop()保证50Hz到达时间
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

// 时间管理 (loopTask用; imuTask使用vTaskDelayUntil,不需要millis)
static unsigned long lastSDLog    = 0;
static unsigned long lastSDFlush  = 0;   // flushData (5Hz)
static unsigned long lastSDMeta   = 0;   // flush/FAT更新 (0.2Hz)
static unsigned long lastSDCheck  = 0;   // 有卡时拔出检测 (1Hz)
static unsigned long lastSDReinit = 0;   // 无卡时插卡重试 (0.2Hz, SD.begin()无卡阻塞慢)
static unsigned long lastLoRaSend = 0;

// 录制状态切换检测 (用于停止时立即flush，确保断电安全)
static bool prev_recording = false;

// ====== SPI互斥锁辅助 ======
static inline void spiLock()   { xSemaphoreTake(spiMutex, portMAX_DELAY); }
static inline void spiUnlock() { xSemaphoreGive(spiMutex); }

// ====== LoRa命令回调 ======
void onLoRaCommand(char cmd) {
  if (cmd == '1') {
    if (!sys_state.is_card_ready) {
      Serial.println("[System] Recording start IGNORED: no SD card");
      return;
    }
    sys_state.is_recording = true;
    Serial.println("[System] Recording started");
  } else if (cmd == '2') {
    sys_state.is_recording = false;
    Serial.println("[System] Recording stopped");
  }
}

// ====== IMU采集任务 (运行在Core 1, 精确50Hz) ======
// 使用 vTaskDelayUntil 实现无漂移采样, 优先级2高于loopTask(1)
// 即使 loopTask 正在处理GPS/LoRa/SD, imuTask 也能在20ms到期时准时抢占执行
void imuTask(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    // 1. 读取IMU原始数据 (I2C 400kHz, ~0.5ms)
    imu.readData(current_imu_data);

    // 2. 更新GPS快照 (gps.processSerialData()在loopTask中运行, getData()线程安全)
    if (gps.isFixed()) {
      current_gps_data = gps.getData();
    }

    // 3. 同步共享数据到显示任务 (Core 0 via dataMutex)
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      shared_imu_data     = current_imu_data;
      shared_gps_data     = current_gps_data;
      shared_sys_state    = sys_state;
      shared_record_count = sd_storage.getRecordCount();
      xSemaphoreGive(dataMutex);
    }

    // 4. 精确等待到下一个20ms周期 (无漂移)
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(IMU_READ_INTERVAL));
  }
}

// ====== 显示任务 (运行在Core 0, 24fps) ======
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
    // 超时 SPI_MUTEX_TIMEOUT_MS(25ms): flushData()通常0~5ms内完成,超时充裕
    // flush()每5s一次最多30ms, 偶发超时时跳过本帧(每5s丢1帧,视觉可接受)
    if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(SPI_MUTEX_TIMEOUT_MS)) == pdTRUE) {
      display.pushFrame();
      xSemaphoreGive(spiMutex);
    }

    // 4. 等待下一帧 (~42ms = 24fps)
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
    Serial.println("[Init] SD Card OK (CSV format, 8KB write buffer)");
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

  // 启动IMU采集任务 (Core 1, 优先级2, 确保50Hz精确采样)
  xTaskCreatePinnedToCore(
    imuTask,
    "IMUTask",
    IMU_TASK_STACK,
    NULL,
    IMU_TASK_PRIO,
    NULL,
    IMU_TASK_CORE
  );
  Serial.println("[Init] IMU task started on Core 1 (50Hz vTaskDelayUntil)");

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
    Serial.println("[Init] Display task started on Core 0 (24fps)");
  }

  delay(500);
  Serial.println("\n===== System Ready =====");
  Serial.println("Core 0: displayTask  — 24fps canvas push");
  Serial.println("Core 1: imuTask      — 50Hz vTaskDelayUntil (prio 2)");
  Serial.println("Core 1: loopTask     — GPS/LoRa/SD/hotswap (prio 1)");
  Serial.println("SD: logData@50Hz(RAM) flushData@5Hz(sector) flush@0.2Hz(FAT)");
  Serial.println("========================\n");
}

// ====== SD写入错误统一处理 (写入失败视同拔卡，触发热插拔恢复流程) ======
static void handleSDWriteError() {
  Serial.println("[SD] Write error! Treating as card removal, stopping recording.");
  sys_state.is_recording  = false;
  sys_state.is_card_ready = false;
  spiLock();
  sd_storage.onCardRemoved();   // 关闭文件句柄, SD.end(), 清缓冲
  spiUnlock();
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  shared_sys_state = sys_state;
  shared_filename  = "";
  xSemaphoreGive(dataMutex);
}

// ====== 主循环 (Core 1 loopTask: GPS/LoRa/SD/热插拔) ======
// IMU采集已移至独立的 imuTask (50Hz精确调度)
// 本循环只处理串口数据流、SD写入、LoRa通信、热插拔检测
void loop() {
  unsigned long now = millis();

  // 1. 连续处理串口数据 (GPS NMEA解析, LoRa命令接收)
  lora.processSerialData();
  gps.processSerialData();

  // 2. SD卡写入 (50Hz，仅录制中) — logData()只写内存缓冲区，无需SPI锁
  //    current_imu_data/current_gps_data 由imuTask在Core 1上更新, 同Core串行安全
  if (sys_state.is_recording && sys_state.is_card_ready &&
      (now - lastSDLog >= SD_LOG_INTERVAL)) {
    sd_storage.logData(current_imu_data, current_gps_data);
    lastSDLog = now;
  }

  // 3. SD数据预写 (5Hz) — csv_buf→FATFS内部扇区缓冲, 通常0~5ms SPI
  if (sys_state.is_recording && sys_state.is_card_ready &&
      (now - lastSDFlush >= SD_FLUSH_INTERVAL)) {
    spiLock();
    bool ok = sd_storage.flushData();
    spiUnlock();
    lastSDFlush = now;
    if (!ok) handleSDWriteError();
  }

  // 4. SD元数据FAT更新 (0.2Hz) — FAT/目录项落盘, 10~30ms SPI, 每5s一次
  //    这是 dataFile.flush() 调用的唯一时机, 降频后不再频繁抢占显示帧
  if (sys_state.is_recording && sys_state.is_card_ready &&
      (now - lastSDMeta >= SD_META_FLUSH_INTERVAL)) {
    spiLock();
    bool ok = sd_storage.flush();
    spiUnlock();
    lastSDMeta = now;
    if (!ok) handleSDWriteError();
  }

  // 5. 录制停止时立即完整flush (确保最后一批数据落盘)
  if (prev_recording && !sys_state.is_recording && sys_state.is_card_ready) {
    spiLock();
    bool ok = sd_storage.flush();
    spiUnlock();
    if (ok) {
      Serial.println("[SD] Stop-flush done");
    } else {
      handleSDWriteError();
    }
  }
  prev_recording = sys_state.is_recording;

  // 6. LoRa发送 (2Hz)
  if (now - lastLoRaSend >= LORA_SEND_INTERVAL) {
    lora.sendData(current_imu_data, current_gps_data);
    lastLoRaSend = now;
  }

  // 7a. SD卡拔出检测 (有卡时, 1Hz) — cardType()只需一次SPI查询, 非常快
  if (sys_state.is_card_ready && (now - lastSDCheck >= SD_CHECK_INTERVAL)) {
    lastSDCheck = now;

    spiLock();
    bool still_present = (SD.cardType() != CARD_NONE);
    spiUnlock();

    if (!still_present) {
      Serial.println("[SD] Card removed! Stopping recording.");
      sys_state.is_recording  = false;
      sys_state.is_card_ready = false;

      spiLock();
      sd_storage.onCardRemoved();   // 关闭文件, SD.end(), 清内存缓冲
      spiUnlock();

      xSemaphoreTake(dataMutex, portMAX_DELAY);
      shared_sys_state = sys_state;
      shared_filename  = "";
      xSemaphoreGive(dataMutex);

      lastSDReinit = now;  // 从这一刻开始计5s, 避免紧接着就重试
    }
  }

  // 7b. SD卡插入检测 (无卡时, 0.2Hz) — SD.begin()无卡时需100ms+总线超时, 降频至5s一次
  //     避免每秒抢占spiMutex导致displayTask丢帧卡顿
  if (!sys_state.is_card_ready && (now - lastSDReinit >= SD_REINIT_INTERVAL)) {
    lastSDReinit = now;

    spiLock();
    bool ok = sd_storage.tryReinit();
    spiUnlock();

    if (ok) {
      sys_state.is_card_ready = true;
      // 插卡后不自动开始录制，需通过LoRa命令显式启动

      xSemaphoreTake(dataMutex, portMAX_DELAY);
      shared_sys_state = sys_state;
      shared_filename  = sd_storage.getFileName();
      xSemaphoreGive(dataMutex);
    }
  }
}
