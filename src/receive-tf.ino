#include <Arduino.h>
#include <SPI.h>
#include <SD.h>  // 替换SdFat为SD库，与第二份代码保持一致

// ************************ 硬件定义 ************************
// LoRa通信：UART2 (GPIO16=RX2, GPIO17=TX2)
HardwareSerial LoRaSerial(2);
// USB串口：调试打印
#define DBG_SERIAL Serial

// 按键定义（高电平按下）
#define BUTTON1_PIN 2  // 按下发1（开始记录）
#define BUTTON2_PIN 4  // 按下发2（停止记录）

// TF卡配置 (请根据实际接线修改CS引脚)
#define SD_CS 5
File dataFile;  // SD库的File类型，替换SdFat的FsFile

// ************************ 配置参数 ************************
#define LORA_BAUD_RATE 9600    // LoRa波特率
#define DEBOUNCE_TIME 20       // 按键消抖时间(ms)
#define DBG_BAUD_RATE 115200   // 调试串口波特率
#define SD_INIT_RETRY 3        // SD卡初始化重试次数
#define SD_SPI_SPEED 18000000  // SD卡SPI时钟频率

// ************************ 全局变量 ************************
// 按键消抖
unsigned long btn1LastTime = 0, btn2LastTime = 0;
bool btn1State = LOW, btn2State = LOW;

// LoRa接收缓存（按行解析）
char loraRecvBuf[256];
int recvBufIndex = 0;

// 记录状态控制
bool isRecording = false;
String data_filename;  // 存储当前使用的文件名（与第二份代码对齐）
bool sd_is_ready = false;  // SD卡就绪状态标识

// ************************ 函数声明 ************************
bool initSDCard();
void startRecording();
void stopRecording();
String findNextFileName();  // 替换原getCurrentFilename
void checkFileStatus();     // 新增：文件状态检查（参考第二份代码）

void setup() {
  // 初始化调试串口
  DBG_SERIAL.begin(DBG_BAUD_RATE);
  while (!DBG_SERIAL) {}
  
  // 初始化LoRa串口
  LoRaSerial.begin(LORA_BAUD_RATE, SERIAL_8N1, 16, 17);
  
  // 初始化按键
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);

  // 初始化TF卡（带重试机制）
  if (!initSDCard()) {
    DBG_SERIAL.println("[ERROR] TF卡初始化失败，将无法保存数据");
    sd_is_ready = false;
  } else {
    sd_is_ready = true;
  }

  // 初始化提示
  DBG_SERIAL.println("===== LoRa接收端初始化完成 =====");
  DBG_SERIAL.println("功能：1. 按键1发1(开始) 按键2发2(停止)");
  DBG_SERIAL.println("      2. 接收LoRa数据，按行保存到TF卡");
  DBG_SERIAL.println("================================\n");
}

void loop() {
  /********** 第一部分：按键检测，发送1/2 **********/
  // 按键1：发送1（开始记录）
  int btn1Read = digitalRead(BUTTON1_PIN);
  if (btn1Read != btn1State && millis() - btn1LastTime > DEBOUNCE_TIME) {
    btn1State = btn1Read;
    btn1LastTime = millis();
    if (btn1State == HIGH) {
      LoRaSerial.write('1');  // 发送字符1（无换行）
      DBG_SERIAL.println("[发送指令] 开始记录 (1)");
      if (sd_is_ready) {
        startRecording(); // 本地也开始写入文件
      } else {
        DBG_SERIAL.println("[ERROR] SD卡未就绪，无法开始记录");
      }
    }
  }

  // 按键2：发送2（停止记录）
  int btn2Read = digitalRead(BUTTON2_PIN);
  if (btn2Read != btn2State && millis() - btn2LastTime > DEBOUNCE_TIME) {
    btn2State = btn2Read;
    btn2LastTime = millis();
    if (btn2State == HIGH) {
      LoRaSerial.write('2');  // 发送字符2（无换行）
      DBG_SERIAL.println("[发送指令] 停止记录 (2)");
      stopRecording(); // 本地停止写入文件
    }
  }

  /********** 第二部分：接收LoRa数据，按行解析并存储 **********/
  while (LoRaSerial.available() > 0) {
    char recvData = LoRaSerial.read();
    
    // 检测换行符，代表一行数据结束
    if (recvData == '\n' || recvData == '\r') {
      if (recvBufIndex > 0) {
        loraRecvBuf[recvBufIndex] = '\0'; // 字符串结束符
        
        // 打印整行数据
        DBG_SERIAL.print("[接收数据] ");
        DBG_SERIAL.println(loraRecvBuf);

        // 如果正在记录且SD卡就绪，则写入文件
        if (isRecording && sd_is_ready) {
          // 参考第二份代码：使用FILE_APPEND追加写入，避免覆盖
          File dataFile = SD.open(data_filename.c_str(), FILE_APPEND);
          if (!dataFile) {
            DBG_SERIAL.println("[SD] ERROR: 文件打开失败！");
            return;
          }

          // 固定缓冲区拼接数据，避免堆碎片（参考第二份代码的formatCSVLine）
          char csvBuffer[256];
          snprintf(csvBuffer, sizeof(csvBuffer), "%lu,%s", millis(), loraRecvBuf);
          
          // 写入数据并验证写入结果
          size_t written = dataFile.println(csvBuffer);
          if (written == 0) {
            DBG_SERIAL.println("[SD] ERROR: 数据写入失败！");
            dataFile.close();
            return;
          }

          // 关键：强制刷新+短暂延迟，确保数据写入SD卡（参考第二份代码）
          dataFile.flush();
          delay(5);

          // 验证文件大小变化（可选，增强可靠性）
          size_t size_after = dataFile.size();
          DBG_SERIAL.print("[SD] 写入成功 - 字节数: ");
          DBG_SERIAL.print(written);
          DBG_SERIAL.print(" | 文件总大小: ");
          DBG_SERIAL.print(size_after);
          DBG_SERIAL.println(" bytes");

          dataFile.close();
        }
        
        // 清空缓存
        recvBufIndex = 0;
        memset(loraRecvBuf, 0, sizeof(loraRecvBuf));
      }
    } else {
      // 缓存数据（防止溢出）
      if (recvBufIndex < sizeof(loraRecvBuf) - 1) {
        loraRecvBuf[recvBufIndex++] = recvData;
      }
    }
  }

  delay(5);
}

// ==================== 底层功能函数 ====================

/**
 * @brief 初始化SD卡（带重试机制，参考第二份代码）
 * @return 成功返回true，失败返回false
 */
bool initSDCard() {
  DBG_SERIAL.println("初始化TF卡...");
  
  // 参考第二份代码：3次重试机制
  for (int i = 0; i < SD_INIT_RETRY; i++) {
    if (SD.begin(SD_CS, SPI, SD_SPI_SPEED)) {
      DBG_SERIAL.println("TF卡初始化成功!");
      
      // 打印SD卡基本信息
      uint64_t cardSize = SD.cardSize() / (1024 * 1024);
      DBG_SERIAL.printf("存储卡容量: %.2f GB\n", (float)cardSize / 1024);
      
      // 预查找下一个可用文件名（提前初始化）
      data_filename = findNextFileName();
      DBG_SERIAL.print("[SD] 下一个日志文件: ");
      DBG_SERIAL.println(data_filename);
      
      return true;
    }
    delay(500);
    if (i < SD_INIT_RETRY - 1) {
      DBG_SERIAL.println("TF卡初始化失败，重试中...");
    }
  }
  
  DBG_SERIAL.println("TF卡初始化失败（3次重试后）!");
  return false;
}

/**
 * @brief 查找下一个可用的数字文件名（参考第二份代码findNextFileName）
 * @return 可用的文件名（如/1.csv, /2.csv...）
 */
String findNextFileName() {
  int fileIndex = 1;
  char fileName[32];  // 固定缓冲区，避免String堆碎片
  
  while (true) {
    snprintf(fileName, sizeof(fileName), "/%d.csv", fileIndex);
    if (!SD.exists(fileName)) {
      return String(fileName);
    }
    fileIndex++;
    // 防止无限循环，最多查找1000个文件
    if (fileIndex > 1000) {
      DBG_SERIAL.println("[SD] 警告：文件数超过1000，使用默认文件名");
      return "/data.csv";
    }
  }
}

/**
 * @brief 开始记录数据（参考第二份代码的文件创建+表头写入逻辑）
 */
void startRecording() {
  if (isRecording || !sd_is_ready) return;
  
  // 重新查找最新文件名（避免多次启动时重复使用旧文件名）
  data_filename = findNextFileName();
  DBG_SERIAL.print("[记录] 尝试创建文件: ");
  DBG_SERIAL.println(data_filename);

  // 参考第二份代码：打开文件并写入表头（仅文件为空时）
  File dataFile = SD.open(data_filename.c_str(), FILE_WRITE);
  if (!dataFile) {
    DBG_SERIAL.println("[ERROR] 创建数据文件失败!");
    return;
  }

  // 仅当文件为空时写入表头（避免重复写入）
  if (dataFile.size() == 0) {
    size_t header_written = dataFile.println("Timestamp(ms),LoRa_Raw_Data");
    if (header_written == 0) {
      DBG_SERIAL.println("[SD] ERROR: 表头写入失败!");
      dataFile.close();
      return;
    }
    // 强制刷新+延迟，确保表头写入（参考第二份代码）
    dataFile.flush();
    delay(10);
    DBG_SERIAL.print("[SD] CSV表头写入成功 (");
    DBG_SERIAL.print(header_written);
    DBG_SERIAL.println(" bytes)");
  }
  
  dataFile.close();
  isRecording = true;
  DBG_SERIAL.println("[记录] 已开始记录数据...");
  
  // 检查文件状态（可选，增强可靠性）
  checkFileStatus();
}

/**
 * @brief 停止记录数据
 */
void stopRecording() {
  if (!isRecording) return;
  
  isRecording = false;
  
  // 检查并关闭可能未关闭的文件
  if (dataFile) {
    dataFile.close();
  }
  
  DBG_SERIAL.println("[记录] 已停止记录，文件已保存。");
  
  // 停止后检查文件最终状态
  checkFileStatus();
}

/**
 * @brief 检查文件状态（参考第二份代码checkFileStatus）
 */
void checkFileStatus() {
  if (!sd_is_ready) {
    DBG_SERIAL.println("[SD] SD卡未就绪，无法检查文件状态");
    return;
  }
  
  File checkFile = SD.open(data_filename.c_str());
  if (checkFile) {
    DBG_SERIAL.print("[SD] 文件状态 - 名称: ");
    DBG_SERIAL.print(data_filename);
    DBG_SERIAL.print(" | 大小: ");
    DBG_SERIAL.print(checkFile.size());
    DBG_SERIAL.println(" bytes");
    checkFile.close();
  } else {
    DBG_SERIAL.print("[SD] ERROR: 无法打开文件 ");
    DBG_SERIAL.println(data_filename);
  }
}