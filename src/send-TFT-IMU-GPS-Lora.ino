#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <SD.h>
#include <HardwareSerial.h>
#include <math.h>

// ************************ 硬件引脚定义 ************************
// --- IMU (JY901) I2C ---
#define JY901_ADDR 0x50
#define IIC_SDA 21
#define IIC_SCL 22

// --- TFT (ST7735S) VSPI ---
#define TFT_CS    5
#define TFT_RST   4
#define TFT_DC    2
// TFT硬件SPI引脚: CLK=18, MOSI=23, MISO=19（与TF卡共用）

// --- TF Card (共用VSPI) ---
#define SD_CS     15

// --- GPS (UART1) ---
#define GPS_RX_PIN 32  // ESP32 UART1_RX <-- GPS_TX
#define GPS_TX_PIN 33  // ESP32 UART1_TX --> GPS_RX
HardwareSerial GPS_Serial(1);

// --- LoRa (UART2) ---
#define LORA_RX_PIN 16  // ESP32 UART2_RX <-- LoRa_TX
#define LORA_TX_PIN 17  // ESP32 UART2_TX --> LoRa_RX
HardwareSerial LoRaSerial(2);

// ************************ 配置参数 ************************
#define BAUD_RATE 115200
#define IMU_READ_INTERVAL 100     // IMU采集间隔(ms)
#define TFT_REFRESH_INTERVAL 200  // TFT刷新间隔(ms)
#define SD_LOG_INTERVAL 100       // TF卡写入间隔(ms)（与IMU采集同步）
#define LORA_SEND_INTERVAL 500    // LoRa数据发送间隔(ms)
#define GPS_BAUD_RATE 9600        // GPS默认波特率
#define LORA_BAUD_RATE 9600       // LoRa默认波特率

// ************************ 状态定义 ************************
bool isRecording = false;       // 记录状态：true=开始，false=停止
bool isCardReady = false;       // TF卡就绪状态
bool isGPSFixed = false;        // GPS定位状态

// ************************ IMU寄存器地址 ************************
#define AX_ADDR 0x34
#define AY_ADDR 0x35
#define AZ_ADDR 0x36
#define GX_ADDR 0x37
#define GY_ADDR 0x38
#define GZ_ADDR 0x39
#define ROLL_ADDR 0x3D
#define PITCH_ADDR 0x3E
#define YAW_ADDR 0x3F

// ************************ GPS相关定义 ************************
char nmeaBuffer[256];
int gpsBufferIndex = 0;
bool isGGA = false;
double wgs84_lat = 0.0;  // 原始WGS84纬度
double wgs84_lon = 0.0;  // 原始WGS84经度
double gcj02_lat = 0.0;  // 转换后GCJ02纬度
double gcj02_lon = 0.0;  // 转换后GCJ02经度
String latDir = "";      // 纬度方向 N/S
String lonDir = "";      // 经度方向 E/W

// ************************ 全局对象与变量 ************************
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// 颜色定义
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define ORANGE  0xFBE0
#define DARKGREY 0x4208

// IMU数据结构体
struct IMUData {
  float ax, ay, az;     // 加速度 m/s²
  float gx, gy, gz;     // 角速度 °/s
  float roll, pitch, yaw; // 角度 °
  unsigned long timestamp; // 时间戳 ms
};

IMUData currentData;
String dataFileName;
unsigned long lastLoRaSend = 0; // LoRa发送计时

// 时间管理变量
unsigned long lastIMURead = 0;
unsigned long lastTFTRefresh = 0;
unsigned long lastSDLog = 0;

// ************************ GCJ02转换算法 ************************
const double a = 6378245.0;
const double ee = 0.00669342162296594323;

bool outOfChina(double lat, double lon) {
  if (lon < 72.004 || lon > 137.8347) return true;
  if (lat < 0.8293 || lat > 55.8271) return true;
  return false;
}

double transformLat(double x, double y) {
  double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(fabs(x));
  ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;
  ret += (20.0 * sin(y * PI) + 40.0 * sin(y / 3.0 * PI)) * 2.0 / 3.0;
  ret += (160.0 * sin(y / 12.0 * PI) + 320 * sin(y * PI / 30.0)) * 2.0 / 3.0;
  return ret;
}

double transformLon(double x, double y) {
  double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(fabs(x));
  ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;
  ret += (20.0 * sin(x * PI) + 40.0 * sin(x / 3.0 * PI)) * 2.0 / 3.0;
  ret += (150.0 * sin(x / 12.0 * PI) + 300.0 * sin(x / 30.0 * PI)) * 2.0 / 3.0;
  return ret;
}

void wgs84_to_gcj02(double wgs_lat, double wgs_lon, double &gcj_lat, double &gcj_lon) {
  if (outOfChina(wgs_lat, wgs_lon)) {
    gcj_lat = wgs_lat;
    gcj_lon = wgs_lon;
    return;
  }
  double dLat = transformLat(wgs_lon - 105.0, wgs_lat - 35.0);
  double dLon = transformLon(wgs_lon - 105.0, wgs_lat - 35.0);
  double radLat = wgs_lat / 180.0 * PI;
  double magic = sin(radLat);
  magic = 1 - ee * magic * magic;
  double sqrtMagic = sqrt(magic);
  dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * PI);
  dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * PI);
  gcj_lat = wgs_lat + dLat;
  gcj_lon = wgs_lon + dLon;
}

double ddm2dd(String ddmStr) {
  if (ddmStr.isEmpty()) return 0.0;
  double ddm = atof(ddmStr.c_str());
  int deg = (int)(ddm / 100);
  double min = ddm - deg * 100;
  return deg + min / 60.0;
}

void parseGGA(String ggaStr) {
  int comma1 = ggaStr.indexOf(',', 7);
  int comma2 = ggaStr.indexOf(',', comma1 + 1);
  int comma3 = ggaStr.indexOf(',', comma2 + 1);
  int comma4 = ggaStr.indexOf(',', comma3 + 1);

  String latDDM = ggaStr.substring(comma1 + 1, comma2);
  latDir = ggaStr.substring(comma2 + 1, comma3);
  String lonDDM = ggaStr.substring(comma3 + 1, comma4);
  lonDir = ggaStr.substring(comma4 + 1, comma4 + 2);

  wgs84_lat = ddm2dd(latDDM);
  if (latDir == "S") wgs84_lat = -wgs84_lat;
  wgs84_lon = ddm2dd(lonDDM);
  if (lonDir == "W") wgs84_lon = -wgs84_lon;

  wgs84_to_gcj02(wgs84_lat, wgs84_lon, gcj02_lat, gcj02_lon);
  isGPSFixed = (wgs84_lat != 0.0 && wgs84_lon != 0.0);
}

// ************************ 函数声明 ************************
bool initTFT();
bool initSDCard();
bool initGPS();
bool initLoRa();
String findNextFileName();
void readIMUData();
void logDataToSD();
void drawStaticUI();
void updateDynamicData();
void tftBusTake();
void tftBusRelease();
int16_t readJY901Reg(uint8_t devAddr, uint8_t regAddr);
void handleLoRaCommand();
void sendDataByLoRa(); // LoRa发送IMU+GPS数据

void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial) delay(10);
  Serial.println("===== 接收端初始化开始 =====");

  // 初始化SPI总线（TFT+TF卡共用）
  SPI.begin();

  // 初始化I2C（IMU）
  Wire.begin(IIC_SDA, IIC_SCL);
  Wire.setClock(400000);
  delay(500);

  // 初始化外设
  if (!initTFT()) {
    Serial.println("TFT Init Failed!");
    while (1);
  }
  drawStaticUI(); // 绘制静态界面

  isCardReady = initSDCard();
  initGPS();
  initLoRa();

  Serial.println("===== 接收端初始化完成 =====");
  Serial.println("功能：1. 解析LoRa指令1/2控制记录 2. 发送IMU+GPS数据 3. TFT显示 4. SD卡记录");
}

void loop() {
  unsigned long now = millis();

  // 1. 处理LoRa指令（1=开始，2=停止）
  handleLoRaCommand();

  // 2. 读取GPS数据
  while (GPS_Serial.available() > 0) {
    char ch = GPS_Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (gpsBufferIndex > 0) {
        nmeaBuffer[gpsBufferIndex] = '\0';
        String nmeaStr = String(nmeaBuffer);

        if (nmeaStr.startsWith("$GPGGA") || nmeaStr.startsWith("$BDGGA") || nmeaStr.startsWith("$GNGGA")) {
          isGGA = true;
          parseGGA(nmeaStr);
        }

        if (isGGA && isGPSFixed) {
          isGGA = false;
        }

        gpsBufferIndex = 0;
        memset(nmeaBuffer, 0, sizeof(nmeaBuffer));
      }
    } else {
      if (gpsBufferIndex < sizeof(nmeaBuffer) - 1) {
        nmeaBuffer[gpsBufferIndex++] = ch;
      }
    }
  }

  // 3. 读取IMU数据（实时采集，不受记录状态影响）
  if (now - lastIMURead >= IMU_READ_INTERVAL) {
    readIMUData();
    lastIMURead = now;
  }

  // 4. 刷新TFT显示（实时显示，不受记录状态影响）
  if (now - lastTFTRefresh >= TFT_REFRESH_INTERVAL) {
    updateDynamicData();
    lastTFTRefresh = now;
  }

  // 5. 写入TF卡（仅记录状态+TF卡就绪，无需GPS定位）
  if (isRecording && isCardReady && (now - lastSDLog >= SD_LOG_INTERVAL)) {
    logDataToSD();
    lastSDLog = now;
  }

  // 6. 定时发送IMU+GPS数据到LoRa
  if (now - lastLoRaSend >= LORA_SEND_INTERVAL) {
    sendDataByLoRa();
    lastLoRaSend = now;
  }

  delay(5);
}

// ************************ 初始化函数 ************************
bool initTFT() {
  tftBusTake();
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1); // 横屏 160x128
  tft.fillScreen(BLACK);
  tftBusRelease();
  return true;
}

bool initSDCard() {
  Serial.print("Init SD Card...");
  // 确保SD卡初始化成功（重试机制）
  for (int i = 0; i < 3; i++) {
    if (SD.begin(SD_CS, SPI, 4000000)) {
      Serial.println("Success!");
      break;
    }
    delay(500);
    if (i == 2) {
      Serial.println("Failed!");
      return false;
    }
  }

  dataFileName = findNextFileName();
  Serial.print("Log File: "); Serial.println(dataFileName);

  // 创建文件并写入表头（确保表头只写一次）
  File dataFile = SD.open(dataFileName.c_str(), FILE_WRITE);
  if (dataFile) {
    // 检查文件大小，为空时才写表头
    if (dataFile.size() == 0) {
      dataFile.println("Timestamp(ms),Ax(m/s2),Ay(m/s2),Az(m/s2),Gx(deg/s),Gy(deg/s),Gz(deg/s),Roll(deg),Pitch(deg),Yaw(deg),"
                       "WGS84_Lat(deg),WGS84_Lon(deg),GCJ02_Lat(deg),GCJ02_Lon(deg),Lat_Dir,Lon_Dir");
      Serial.println("CSV表头已写入");
    }
    dataFile.close();
    return true;
  } else {
    Serial.println("Create file failed!");
    return false;
  }
}

bool initGPS() {
  Serial.print("Init GPS...");
  GPS_Serial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("Success!");
  return true;
}

bool initLoRa() {
  Serial.print("Init LoRa...");
  LoRaSerial.begin(LORA_BAUD_RATE, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  Serial.println("Success!");
  return true;
}

String findNextFileName() {
  int fileIndex = 1;
  String fileName;
  while (true) {
    fileName = "/" + String(fileIndex) + ".csv";
    if (!SD.exists(fileName.c_str())) {
      return fileName;
    }
    fileIndex++;
    if (fileIndex > 1000) return "/data.csv";
  }
}

// ************************ SPI总线管理 ************************
void tftBusTake() {
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, LOW);
}

void tftBusRelease() {
  digitalWrite(TFT_CS, HIGH);
}

// ************************ IMU数据读取 ************************
void readIMUData() {
  int16_t raw_ax = readJY901Reg(JY901_ADDR, AX_ADDR);
  int16_t raw_ay = readJY901Reg(JY901_ADDR, AY_ADDR);
  int16_t raw_az = readJY901Reg(JY901_ADDR, AZ_ADDR);
  int16_t raw_gx = readJY901Reg(JY901_ADDR, GX_ADDR);
  int16_t raw_gy = readJY901Reg(JY901_ADDR, GY_ADDR);
  int16_t raw_gz = readJY901Reg(JY901_ADDR, GZ_ADDR);
  int16_t raw_roll = readJY901Reg(JY901_ADDR, ROLL_ADDR);
  int16_t raw_pitch = readJY901Reg(JY901_ADDR, PITCH_ADDR);
  int16_t raw_yaw = readJY901Reg(JY901_ADDR, YAW_ADDR);

  currentData.timestamp = millis();
  currentData.ax = (float)raw_ax / 32768 * 16 * 9.8;
  currentData.ay = (float)raw_ay / 32768 * 16 * 9.8;
  currentData.az = (float)raw_az / 32768 * 16 * 9.8;
  currentData.gx = (float)raw_gx / 32768 * 2000;
  currentData.gy = (float)raw_gy / 32768 * 2000;
  currentData.gz = (float)raw_gz / 32768 * 2000;
  currentData.roll = (float)raw_roll / 32768 * 180;
  currentData.pitch = (float)raw_pitch / 32768 * 180;
  currentData.yaw = (float)raw_yaw / 32768 * 180;

  // 串口打印IMU数据（调试用）
  Serial.print("IMU采集: TS="); Serial.print(currentData.timestamp);
  Serial.print(" AX="); Serial.print(currentData.ax, 2);
  Serial.print(" ROLL="); Serial.println(currentData.roll, 2);
}

int16_t readJY901Reg(uint8_t devAddr, uint8_t regAddr) {
  int16_t data = 0;
  uint8_t buf[2] = {0};
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.endTransmission(false);
  Wire.requestFrom(devAddr, 2);
  if (Wire.available() == 2) {
    buf[0] = Wire.read();
    buf[1] = Wire.read();
    data = (int16_t)((buf[1] << 8) | buf[0]);
  }
  return data;
}

// ************************ TF卡存储（核心修复） ************************
void logDataToSD() {
  // 以追加模式打开文件（FILE_WRITE = O_APPEND）
  File dataFile = SD.open(dataFileName.c_str(), FILE_WRITE);
  if (!dataFile) {
    Serial.println("[SD错误] 文件打开失败");
    return;
  }

  // 组装CSV行数据：IMU必录，GPS无定位则填NO_FIX
  String csvLine = "";
  
  // 1. IMU数据（必录）
  csvLine += String(currentData.timestamp) + ",";
  csvLine += String(currentData.ax, 2) + ",";
  csvLine += String(currentData.ay, 2) + ",";
  csvLine += String(currentData.az, 2) + ",";
  csvLine += String(currentData.gx, 2) + ",";
  csvLine += String(currentData.gy, 2) + ",";
  csvLine += String(currentData.gz, 2) + ",";
  csvLine += String(currentData.roll, 2) + ",";
  csvLine += String(currentData.pitch, 2) + ",";
  csvLine += String(currentData.yaw, 2) + ",";
  
  // 2. GPS数据（无定位填NO_FIX）
  if (isGPSFixed) {
    csvLine += String(wgs84_lat, 6) + ",";
    csvLine += String(wgs84_lon, 6) + ",";
    csvLine += String(gcj02_lat, 6) + ",";
    csvLine += String(gcj02_lon, 6) + ",";
    csvLine += latDir + ",";
    csvLine += lonDir;
  } else {
    csvLine += "NO_FIX,NO_FIX,NO_FIX,NO_FIX,NO_FIX,NO_FIX";
  }
  
  // 3. 写入并换行
  dataFile.println(csvLine);
  dataFile.close(); // 立即关闭，确保数据写入闪存

  // 调试日志
  Serial.print("[SD写入] "); Serial.println(csvLine);
}

// ************************ LoRa指令处理（1=开始，2=停止） ************************
void handleLoRaCommand() {
  while (LoRaSerial.available() > 0) {
    char loraData = LoRaSerial.read();
    if (loraData == '1') {
      isRecording = true;
      Serial.println("[LoRa指令] 开始记录到TF卡 (1)");
      // 记录开始时打印文件信息
      Serial.print("[记录信息] 文件路径："); Serial.println(dataFileName);
    } else if (loraData == '2') {
      isRecording = false;
      Serial.println("[LoRa指令] 停止记录到TF卡 (2)");
    } else {
      // 非1/2数据，忽略（防止误码）
      Serial.print("[LoRa指令] 无效数据：");
      Serial.write(loraData);
      Serial.println();
    }
  }
}

// ************************ LoRa发送IMU+GPS数据（单行格式） ************************
void sendDataByLoRa() {
  // 打包数据：单行格式，逗号分隔，结尾换行（方便发射端按行解析）
  String dataStr = "";
  dataStr += "TS:" + String(currentData.timestamp) + ",";
  dataStr += "AX:" + String(currentData.ax, 2) + ",";
  dataStr += "AY:" + String(currentData.ay, 2) + ",";
  dataStr += "ROLL:" + String(currentData.roll, 2) + ",";
  dataStr += "PITCH:" + String(currentData.pitch, 2) + ",";
  if (isGPSFixed) {
    dataStr += "LAT:" + String(gcj02_lat, 6) + ",";
    dataStr += "LON:" + String(gcj02_lon, 6);
  } else {
    dataStr += "LAT:NO_FIX,LON:NO_FIX";
  }
  dataStr += "\r\n"; // 换行符，标记一行结束

  // 发送数据
  LoRaSerial.print(dataStr);
  Serial.print("[LoRa发送] ");
  Serial.print(dataStr);
}

// ************************ TFT显示（实时刷新，不受记录状态影响） ************************
void drawStaticUI() {
  tftBusTake();
  tft.fillScreen(BLACK);
  
  // 1. 顶部标题栏
  tft.fillRect(0, 0, 160, 18, BLUE);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  int titleX = (160 - 8 * 10) / 2;
  tft.setCursor(titleX, 4);
  tft.print("IMU+GPS+LOG");

  // 2. 静态标签
  tft.setCursor(5, 25); tft.print("Acc:");
  tft.setCursor(5, 40); tft.print("Gyro:");
  tft.setCursor(5, 55); tft.print("Angle:");
  tft.setCursor(5, 70); tft.print("GPS:");
  tft.setCursor(5, 85); tft.print("State:");
  tft.setCursor(5, 100); tft.print("File:");

  tftBusRelease();
}

void updateDynamicData() {
  tftBusTake();
  
  // 1. 记录状态指示灯（绿色=记录中，红色=停止）
  tft.fillCircle(145, 9, 5, BLACK);
  tft.fillCircle(145, 9, 5, isRecording ? GREEN : RED);

  // 2. 加速度（清除残影）
  tft.fillRect(30, 22, 125, 10, BLACK);
  tft.setTextColor(RED);
  tft.setCursor(30, 27);
  tft.print("X:"); tft.print(currentData.ax, 1);
  tft.print(" Y:"); tft.print(currentData.ay, 1);
  tft.print(" Z:"); tft.print(currentData.az, 1);

  // 3. 角速度
  tft.fillRect(30, 37, 125, 10, BLACK);
  tft.setTextColor(GREEN);
  tft.setCursor(30, 42);
  tft.print("X:"); tft.print(currentData.gx, 1);
  tft.print(" Y:"); tft.print(currentData.gy, 1);
  tft.print(" Z:"); tft.print(currentData.gz, 1);

  // 4. 角度
  tft.fillRect(30, 52, 125, 10, BLACK);
  tft.setTextColor(YELLOW);
  tft.setCursor(30, 57);
  tft.print("R:"); tft.print(currentData.roll, 1);
  tft.print(" P:"); tft.print(currentData.pitch, 1);
  tft.print(" Y:"); tft.print(currentData.yaw, 1);

  // 5. GPS（实时显示，无定位标NO_FIX）
  tft.fillRect(30, 67, 125, 10, BLACK);
  tft.setTextColor(ORANGE);
  tft.setCursor(30, 72);
  if (isGPSFixed) {
    tft.print("L:"); tft.print(gcj02_lat, 4);
    tft.print(" Lo:"); tft.print(gcj02_lon, 4);
  } else {
    tft.print("No Fix");
  }

  // 6. 记录状态（实时显示）
  tft.fillRect(30, 82, 125, 10, BLACK);
  tft.setTextColor(CYAN);
  tft.setCursor(30, 87);
  if (isRecording) {
    tft.print("Recording (ON)");
  } else {
    tft.print("Stopped (OFF)");
  }

  // 7. 文件名
  tft.fillRect(30, 97, 125, 10, BLACK);
  tft.setTextColor(WHITE);
  tft.setCursor(30, 102);
  if (isCardReady) {
    tft.print(dataFileName.substring(1));
  } else {
    tft.print("SD Error");
  }

  tftBusRelease();
}