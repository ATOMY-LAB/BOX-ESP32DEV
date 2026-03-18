#include <Arduino.h>
#include <SPI.h>
#include "SdFat.h"

// ************************ 硬件定义 ************************
HardwareSerial LoRaSerial(2);
#define DBG_SERIAL Serial

#define BUTTON1_PIN 2
#define BUTTON2_PIN 4

#define LED_R 14
#define LED_G 12
#define LED_B 13

#define SD_CS 5
SdFs sd;
FsFile dataFile;

// ************************ 核心配置 ************************
// 【重要】烧录第二套设备前请修改为 "DEV2"
#define DEVICE_ID "DEV1"
#define LORA_BAUD_RATE 9600
#define DEBOUNCE_TIME 20
#define DBG_BAUD_RATE 115200
#define SD_INIT_RETRY 5
#define SD_SPI_SPEED 10000000
#define PROTOCOL_HEADER "$LORA"

// 【优化】接收缓冲区配置（零阻塞核心）
#define RX_BUFFER_SIZE 4096    // 串口硬件RX缓冲区大小
#define PACKET_BUFFER_SIZE 256 // 单包最大长度
#define DATA_QUEUE_SIZE 50     // 待写入SD卡的数据包队列长度

// 状态枚举
enum {
  STATE_INIT_SD,
  STATE_SD_ERROR,
  STATE_IDLE,
  STATE_RECORDING,
  STATE_STOPPED
};

// ************************ 全局变量 ************************
// 按键相关
unsigned long btn1LastTime = 0, btn2LastTime = 0;
bool btn1State = LOW, btn2State = LOW;

// 【优化】零阻塞接收相关
char rxPacketBuffer[PACKET_BUFFER_SIZE]; // 单包缓存
int packetIndex = 0;
char dataQueue[DATA_QUEUE_SIZE][PACKET_BUFFER_SIZE]; // 待写入SD卡的队列
volatile int queueWriteIndex = 0;
int queueReadIndex = 0;

// 系统状态
bool isRecording = false;
String data_filename;
bool sd_is_ready = false;
unsigned long ledBlinkStart = 0;
bool isBlinking = false;
int currentSystemState = STATE_INIT_SD;

// 非阻塞延时相关
unsigned long stateChangeTime = 0;
bool pendingStateChange = false;
int nextState = STATE_IDLE;
unsigned long lastSdFlushTime = 0; // SD卡定时刷写计时

// ************************ 函数声明 ************************
void setLEDColor(bool r, bool g, bool b);
void showLEDState(int state);
void triggerDataReceiveLED();
bool initSDCard();
void startRecording();
void stopRecording();
String findNextFileName();
void sendLoRaCommand(String cmd);
void parseLoRaPacket(char* packet);
void processDataQueue(); // 处理待写入SD卡的队列

// ************************ 串口接收中断（零阻塞核心） ************************
// 收到字节时立刻存入缓冲区，不做任何耗时操作
void onLoRaDataReceived() {
  while (LoRaSerial.available()) {
    char c = LoRaSerial.read();
    
    // 检测换行符，一包结束
    if (c == '\n' || c == '\r') {
      if (packetIndex > 0) {
        rxPacketBuffer[packetIndex] = '\0';
        // 复制到待处理队列（仅内存拷贝，无耗时操作）
        int nextWriteIndex = (queueWriteIndex + 1) % DATA_QUEUE_SIZE;
        if (nextWriteIndex != queueReadIndex) {
          strncpy(dataQueue[queueWriteIndex], rxPacketBuffer, PACKET_BUFFER_SIZE);
          queueWriteIndex = nextWriteIndex;
        }
        packetIndex = 0;
        memset(rxPacketBuffer, 0, PACKET_BUFFER_SIZE);
      }
    } else {
      // 存入单包缓存，防止溢出
      if (packetIndex < PACKET_BUFFER_SIZE - 1) {
        rxPacketBuffer[packetIndex++] = c;
      }
    }
  }
}

void setup() {
  // LED初始化
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  setLEDColor(false, false, false);

  // 调试串口初始化
  DBG_SERIAL.begin(DBG_BAUD_RATE);
  while (!DBG_SERIAL) {}
  
  DBG_SERIAL.println("");
  DBG_SERIAL.println("========================================");
  DBG_SERIAL.print("  系统启动 (零阻塞高速模式) - ID: ");
  DBG_SERIAL.println(DEVICE_ID);
  DBG_SERIAL.println("========================================");

  // 初始化状态灯
  showLEDState(STATE_INIT_SD);

  // 【优化】LoRa串口初始化：增大RX缓冲区，注册接收回调
  LoRaSerial.setRxBufferSize(RX_BUFFER_SIZE);
  LoRaSerial.begin(LORA_BAUD_RATE, SERIAL_8N1, 16, 17);
  LoRaSerial.onReceive(onLoRaDataReceived); // 中断级接收，零延迟

  // 按键初始化
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);

  // TF卡初始化
  DBG_SERIAL.print("[状态] 正在初始化 TF 卡...");
  if (!initSDCard()) {
    sd_is_ready = false;
    showLEDState(STATE_SD_ERROR);
    DBG_SERIAL.println(" [失败]");
  } else {
    sd_is_ready = true;
    currentSystemState = STATE_IDLE;
    showLEDState(STATE_IDLE);
    DBG_SERIAL.println(" [成功]");
    DBG_SERIAL.println("[状态] 系统就绪");
  }
}

void loop() {
  // 非阻塞状态切换
  if (pendingStateChange && (millis() - stateChangeTime > 2000)) {
    pendingStateChange = false;
    currentSystemState = nextState;
    showLEDState(currentSystemState);
    DBG_SERIAL.println("[状态] 恢复待机模式");
  }

  // LED闪烁恢复
  if (isBlinking && (millis() - ledBlinkStart > 100)) {
    isBlinking = false;
    showLEDState(currentSystemState);
  }

  // 【核心】处理待写入SD卡的数据包（空闲时执行，不阻塞接收）
  processDataQueue();

  // 按键检测
  int btn1Read = digitalRead(BUTTON1_PIN);
  if (btn1Read != btn1State && millis() - btn1LastTime > DEBOUNCE_TIME) {
    btn1State = btn1Read;
    btn1LastTime = millis();
    if (btn1State == HIGH) {
      DBG_SERIAL.println("[按键] 开始记录");
      sendLoRaCommand("START");
      if (sd_is_ready) startRecording();
    }
  }

  int btn2Read = digitalRead(BUTTON2_PIN);
  if (btn2Read != btn2State && millis() - btn2LastTime > DEBOUNCE_TIME) {
    btn2State = btn2Read;
    btn2LastTime = millis();
    if (btn2State == HIGH) {
      DBG_SERIAL.println("[按键] 停止记录");
      sendLoRaCommand("STOP");
      stopRecording();
    }
  }

  // 【优化】SD卡定时刷写，每秒1次，避免频繁阻塞
  if (isRecording && dataFile.isOpen() && (millis() - lastSdFlushTime > 1000)) {
    dataFile.flush();
    lastSdFlushTime = millis();
  }
}

// ************************ LED控制函数 ************************
void setLEDColor(bool r, bool g, bool b) {
  digitalWrite(LED_R, r ? HIGH : LOW);
  digitalWrite(LED_G, g ? HIGH : LOW);
  digitalWrite(LED_B, b ? HIGH : LOW);
}

void showLEDState(int state) {
  switch (state) {
    case STATE_INIT_SD: setLEDColor(false, false, true); break;
    case STATE_SD_ERROR: setLEDColor(true, true, false); break;
    case STATE_IDLE: setLEDColor(false, true, true); break;
    case STATE_RECORDING: setLEDColor(false, true, false); break;
    case STATE_STOPPED: setLEDColor(true, false, false); break;
    default: setLEDColor(false, false, false); break;
  }
}

void triggerDataReceiveLED() {
  setLEDColor(true, false, true);
  ledBlinkStart = millis();
  isBlinking = true;
}

// ************************ LoRa协议函数 ************************
void sendLoRaCommand(String cmd) {
  LoRaSerial.print(PROTOCOL_HEADER);
  LoRaSerial.print(",");
  LoRaSerial.print(DEVICE_ID);
  LoRaSerial.print(",CMD,");
  LoRaSerial.println(cmd);
}

void parseLoRaPacket(char* packet) {
  // 检查包头
  if (strstr(packet, PROTOCOL_HEADER) != packet) return;
  
  // 分割数据包（纯C语言实现，无String开销）
  char* parts[4];
  int partIndex = 0;
  char* token = strtok(packet, ",");
  while (token != NULL && partIndex < 4) {
    parts[partIndex++] = token;
    token = strtok(NULL, ",");
  }

  // 字段不足，直接忽略
  if (partIndex < 4) return;

  // 设备ID不匹配，直接忽略
  if (strcmp(parts[1], DEVICE_ID) != 0) return;

  // 处理控制指令
  if (strcmp(parts[2], "CMD") == 0) {
    if (strcmp(parts[3], "START") == 0 && sd_is_ready) {
      startRecording();
    } else if (strcmp(parts[3], "STOP") == 0) {
      stopRecording();
    }
  }
  // 处理数据：仅标记LED，不做写入操作
  else if (strcmp(parts[2], "DATA") == 0) {
    if (isRecording && sd_is_ready) {
      triggerDataReceiveLED();
    }
  }
}

// ************************ SD卡相关函数 ************************
bool initSDCard() {
  delay(3000); 
  for (int i = 0; i < SD_INIT_RETRY; i++) {
    if (sd.begin(SdSpiConfig(SD_CS, SHARED_SPI, SD_SPI_SPEED))) {
      uint64_t sectorCount = sd.card()->sectorCount();
      double cardSizeGB = sectorCount * 512.0 / (1024.0 * 1024.0 * 1024.0);
      DBG_SERIAL.print(" (容量: ");
      DBG_SERIAL.print(cardSizeGB);
      DBG_SERIAL.print(" GB, 文件系统: ");
      DBG_SERIAL.print(sd.fatType() == FAT_TYPE_EXFAT ? "exFAT" : "FAT32");
      DBG_SERIAL.print(") ");
      return true;
    }
    delay(500);
  }
  return false;
}

String findNextFileName() {
  int fileIndex = 1;
  char fileName[32];
  while (true) {
    snprintf(fileName, sizeof(fileName), "/%d.csv", fileIndex);
    if (!sd.exists(fileName)) return String(fileName);
    if (++fileIndex > 1000) return "/data.csv";
  }
}

void startRecording() {
  if (isRecording || !sd_is_ready) return;
  
  // 清空队列，避免旧数据写入
  queueReadIndex = queueWriteIndex = 0;
  
  // 打开文件并保持打开
  data_filename = findNextFileName();
  dataFile = sd.open(data_filename.c_str(), O_WRONLY | O_CREAT | O_TRUNC);
  
  if (!dataFile) {
    DBG_SERIAL.println("[错误] 文件打开失败!");
    return;
  }

  // 写入表头
  if (dataFile.fileSize() == 0) {
    dataFile.println("Timestamp(ms),LoRa_Raw_Data");
    dataFile.flush();
  }
  
  DBG_SERIAL.print("[记录] 开始 -> ");
  DBG_SERIAL.println(data_filename);
  
  isRecording = true;
  currentSystemState = STATE_RECORDING;
  showLEDState(STATE_RECORDING);
  lastSdFlushTime = millis();
}

void stopRecording() {
  if (!isRecording) return;
  
  isRecording = false;
  
  // 写入队列中剩余的所有数据
  processDataQueue();
  
  // 关闭文件
  if (dataFile.isOpen()) {
    dataFile.close();
    DBG_SERIAL.println("[记录] 停止 -> 文件已保存");
  }
  
  currentSystemState = STATE_STOPPED;
  showLEDState(STATE_STOPPED);
  
  // 非阻塞切换回待机
  stateChangeTime = millis();
  nextState = STATE_IDLE;
  pendingStateChange = true;
}

// 【优化】批量处理待写入SD卡的数据包
void processDataQueue() {
  // 没有待写入数据，或未在记录状态，直接返回
  if (queueReadIndex == queueWriteIndex || !isRecording || !dataFile.isOpen()) {
    return;
  }

  // 批量写入队列中的所有数据
  while (queueReadIndex != queueWriteIndex) {
    char* packet = dataQueue[queueReadIndex];
    
    // 仅处理DATA类型的数据包
    if (strstr(packet, PROTOCOL_HEADER) == packet) {
      char* parts[4];
      int partIndex = 0;
      char* token = strtok(packet, ",");
      while (token != NULL && partIndex < 4) {
        parts[partIndex++] = token;
        token = strtok(NULL, ",");
      }
      
      // 只写入匹配ID的DATA包
      if (partIndex == 4 && strcmp(parts[1], DEVICE_ID) == 0 && strcmp(parts[2], "DATA") == 0) {
        dataFile.print(millis());
        dataFile.print(",");
        dataFile.println(parts[3]);
      }
    }

    // 移动读指针
    queueReadIndex = (queueReadIndex + 1) % DATA_QUEUE_SIZE;
  }
}