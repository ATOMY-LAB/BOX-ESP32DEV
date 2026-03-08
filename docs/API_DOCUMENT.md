# 龙舟桨频/船速采集盒子 - API 接口文档

**版本:** v1.0  
**日期:** 2026-03-08  
**项目代号:** DragonBoat Collection Box  

---

## 目录

1. [概述](#概述)
2. [IMU 模块 API](#imu-模块-api)
3. [GPS 模块 API](#gps-模块-api)
4. [TFT 显示模块 API](#tft-显示模块-api)
5. [SD 卡存储模块 API](#sd-卡存储模块-api)
6. [LoRa 通信模块 API](#lora-通信模块-api)
7. [数据结构](#数据结构)
8. [系统集成 API](#系统集成-api)

---

## 概述

本文档定义了龙舟采集盒子中各传感器和通信模块的公开 API 接口。开发人员可以通过这些接口集成新功能或扩展现有功能。

**核心特点：**
- 模块化设计，各功能独立
- 基于 C++ 类设计，提供清晰的对象接口
- 支持非阻塞式串行通信
- 自动时序管理，无需手动控制各模块同步

---

## IMU 模块 API

### 类：`IMUModule`

低功耗 MEMS 传感器（JY901）驱动模块，获取加速度、角速度和欧拉角。

#### 依赖库
- `Wire.h` - I2C 通信

#### 工作参数
- **I2C 地址:** 0x50
- **I2C 时钟:** 400 kHz
- **采集周期:** 100 ms（可配置）

#### 公开方法

##### `bool begin()`
**功能:** 初始化 IMU 模块

**参数:** 无

**返回值:** 
- `true` - 初始化成功
- `false` - 初始化失败

**示例:**
```cpp
if (imu.begin()) {
  Serial.println("IMU initialized successfully");
}
```

---

##### `bool readData(IMUData &data)`
**功能:** 从 JY901 读取一帧 IMU 数据

**参数:**
- `data` - [输出] IMUData 结构体引用，包含所有传感器数据

**返回值:**
- `true` - 读取成功
- `false` - I2C 通信失败

**数据有效性:** 即使返回 `false`，`data` 结构中的时间戳仍会更新为调用时刻

**注意:** 该函数不会阻塞。若 I2C 总线被占用，会自动等待

**示例:**
```cpp
IMUData data;
if (imu.readData(data)) {
  Serial.print("Ax = "); Serial.println(data.ax);
  Serial.print("Roll = "); Serial.println(data.roll);
}
```

---

##### `const IMUData& getLastData() const`
**功能:** 获取上一次读取的完整 IMU 数据（不进行新的读取操作）

**参数:** 无

**返回值:** IMUData 结构体常引用

**应用场景:** 当需要快速查询上一帧数据而不想阻塞时使用

**示例:**
```cpp
const IMUData& last_data = imu.getLastData();
Serial.print("Last timestamp: "); Serial.println(last_data.timestamp);
```

---

## GPS 模块 API

### 类：`GPSModule`

GPS 定位模块驱动，解析 NMEA-0183 格式的 GGA 语句，支持 WGS84→GCJ02 坐标转换。

#### 依赖库
- `HardwareSerial.h` - UART 通信
- `math.h` - 三角函数（坐标转换）

#### 工作参数
- **UART 通道:** UART1
- **波特率:** 9600 bps
- **数据格式:** NMEA-0183（$GPGGA 或 $BDGGA 或 $GNGGA）
- **处理周期:** 连续处理接收缓冲

#### 公开方法

##### `bool begin()`
**功能:** 初始化 GPS 模块

**参数:** 无

**返回值:**
- `true` - 初始化成功
- `false` - 初始化失败

**示例:**
```cpp
if (gps.begin()) {
  Serial.println("GPS module started");
}
```

---

##### `void processSerialData()`
**功能:** 处理 UART1 接收缓冲中的 GPS 数据

**参数:** 无

**返回值:** 无

**特点:**
- 非阻塞
- 每次调用处理所有可用数据
- 自动识别 GGA 语句并解析
- 自动更新 GPS 结构体

**重要:** 该函数应在 `loop()` 中频繁调用（最好每 5-10 ms 一次）

**示例:**
```cpp
void loop() {
  gps.processSerialData();  // 在每个循环中调用
  // ... 其他代码
}
```

---

##### `const GPSData& getData() const`
**功能:** 获取最新的 GPS 定位数据

**参数:** 无

**返回值:** GPSData 结构体常引用

**数据有效性:** 可通过 `GPSData.is_fixed` 判断是否定位成功

**示例:**
```cpp
const GPSData& gps_info = gps.getData();
if (gps_info.is_fixed) {
  Serial.print("Latitude: "); Serial.println(gps_info.gcj02_lat);
  Serial.print("Longitude: "); Serial.println(gps_info.gcj02_lon);
} else {
  Serial.println("GPS not fixed");
}
```

---

##### `bool isFixed() const`
**功能:** 查询 GPS 定位状态

**参数:** 无

**返回值:**
- `true` - GPS 已定位
- `false` - GPS 未定位

**示例:**
```cpp
if (gps.isFixed()) {
  // 使用 GPS 数据进行后续处理
}
```

---

### 坐标系说明

**WGS84（World Geodetic System 1984）:**
- 国际标准坐标系
- GPS 原生输出格式
- 地理信息系统、学术研究采用

**GCJ02（高德/谷歌地图坐标系）:**
- 中国地图的官方坐标系
- 由 GPS 坐标加密得来
- 与高德地图、腾讯地图等兼容
- 本系统自动转换并存储

**存储规则:**
- CSV 文件中同时记录 WGS84 和 GCJ02 坐标
- LoRa 无线链路发送 GCJ02 坐标

---

## TFT 显示模块 API

### 类：`TFTDisplay`

ST7735S 液晶屏驱动模块，实时显示 IMU、GPS、系统状态。

#### 依赖库
- `Adafruit_GFX.h`
- `Adafruit_ST7735.h`

#### 硬件参数
- **分辨率:** 160×128 像素
- **接口:** SPI（与 SD 卡共享）
- **刷新率:** 推荐 200 ms

#### 公开方法

##### `bool begin()`
**功能:** 初始化 TFT 显示屏

**参数:** 无

**返回值:**
- `true` - 初始化成功
- `false` - 初始化失败

**示例:**
```cpp
if (display.begin()) {
  Serial.println("TFT ready");
}
```

---

##### `void drawStaticUI()`
**功能:** 绘制静态界面框架

**参数:** 无

**返回值:** 无

**内容:**
- 顶部蓝色标题栏
- 各数据区域的标签文字
- 静态背景元素

**重要:** 仅在初始化时调用一次

**示例:**
```cpp
display.begin();
display.drawStaticUI();  // 系统启动时调用
```

---

##### `void updateDisplay(const IMUData &imu_data, const GPSData &gps_data, const SystemState &sys_state, const String &filename)`
**功能:** 更新动态数据显示

**参数:**
- `imu_data` - IMU 传感器数据
- `gps_data` - GPS 定位数据
- `sys_state` - 系统运行状态
- `filename` - 当前日志文件名

**返回值:** 无

**更新内容:**
| 区域 | 显示内容 | 颜色 |
|------|--------|------|
| 状态灯 | 绿灯(记录中)/红灯(停止) | 绿/红 |
| 加速度 | Ax/Ay/Az 三轴值 | 红色 |
| 角速度 | Gx/Gy/Gz 三轴值 | 绿色 |
| 欧拉角 | Roll/Pitch/Yaw | 黄色 |
| GPS信息 | 纬度/经度或"No Fix" | 橙色 |
| 记录状态 | Recording(ON)/Stopped(OFF) | 青色 |
| 文件名 | 当前 CSV 文件名 | 白色 |

**推荐调用周期:** 200 ms

**示例:**
```cpp
void loop() {
  unsigned long now = millis();
  
  if (now - lastTFTRefresh >= 200) {
    display.updateDisplay(current_imu_data, current_gps_data, 
                         sys_state, sd_storage.getFileName());
    lastTFTRefresh = now;
  }
}
```

---

##### `void showError(const String &error_msg)`
**功能:** 显示错误消息

**参数:**
- `error_msg` - 错误文本（推荐不超过 20 个字符）

**返回值:** 无

**外观:** 红色背景，白色文字

**示例:**
```cpp
display.showError("SD Card Error");
```

---

##### `void showStartupMessage(const String &msg)`
**功能:** 显示启动信息

**参数:**
- `msg` - 信息文本

**返回值:** 无

**外观:** 黑色背景，青色文字

**示例:**
```cpp
display.showStartupMessage("System Ready");
```

---

## SD 卡存储模块 API

### 类：`SDStorage`

SD 卡数据记录模块，支持 CSV 格式存储和自动文件管理。

#### 依赖库
- `SD.h` - Arduino SD 库

#### 硬件参数
- **通讯协议:** SPI（与 TFT 共享）
- **文件格式:** CSV
- **字符编码:** UTF-8
- **行结束符:** `\n`（Unix 风格）

#### CSV 文件格式

**表头：**
```
Timestamp(ms),Ax(m/s2),Ay(m/s2),Az(m/s2),Gx(deg/s),Gy(deg/s),Gz(deg/s),Roll(deg),Pitch(deg),Yaw(deg),WGS84_Lat(deg),WGS84_Lon(deg),GCJ02_Lat(deg),GCJ02_Lon(deg),Lat_Dir,Lon_Dir
```

**数据行示例：**
```
1234567,9.80,0.23,1.45,5.12,-3.21,2.10,0.45,2.34,15.23,31.234567,120.456789,31.235101,120.457123,N,E
```

**GPS 无定位时：**
```
1234567,9.80,0.23,1.45,5.12,-3.21,2.10,0.45,2.34,15.23,NO_FIX,NO_FIX,NO_FIX,NO_FIX,NO_FIX,NO_FIX
```

#### 公开方法

##### `bool begin()`
**功能:** 初始化 SD 卡

**参数:** 无

**返回值:**
- `true` - 初始化成功
- `false` - 初始化失败

**特点:**
- 包含 3 次重试机制
- 每次重试间隔 500 ms
- 自动生成新文件名（1.csv, 2.csv, ...）
- 自动写入 CSV 表头

**示例:**
```cpp
if (sd_storage.begin()) {
  Serial.println("SD card ready");
} else {
  Serial.println("SD card initialization failed");
}
```

---

##### `bool logData(const IMUData &imu_data, const GPSData &gps_data)`
**功能:** 将 IMU + GPS 数据以 CSV 行格式写入 SD 卡

**参数:**
- `imu_data` - IMU 传感器数据
- `gps_data` - GPS 定位数据

**返回值:**
- `true` - 写入成功
- `false` - 写入失败（可能原因：SD 卡未就绪、文件打开失败）

**特点:**
- 自动处理 GPS 无定位情况（填充 "NO_FIX"）
- 立即 close 文件以确保数据落盘
- 一次调用写入一行数据
- 非阻塞（IO 驱动保证）

**推荐调用周期:** 100 ms（与 IMU 采集同步）

**示例:**
```cpp
if (sys_state.is_recording && sd_storage.isReady()) {
  sd_storage.logData(current_imu_data, current_gps_data);
}
```

---

##### `const String& getFileName() const`
**功能:** 获取当前日志文件名

**参数:** 无

**返回值:** 文件名字符串（例："1.csv"）

**示例:**
```cpp
Serial.print("Logging to: ");
Serial.println(sd_storage.getFileName());
```

---

##### `bool isReady() const`
**功能:** 查询 SD 卡是否就绪

**参数:** 无

**返回值:**
- `true` - 就绪
- `false` - 未就绪

**示例:**
```cpp
if (sd_storage.isReady()) {
  // 可以进行记录操作
}
```

---

## LoRa 通信模块 API

### 类：`LoRaCommunication`

LoRa 无线通信模块驱动，支持命令接收和数据发送。

#### 依赖库
- `HardwareSerial.h` - UART 通信

#### 硬件参数
- **UART 通道:** UART2
- **波特率:** 9600 bps
- **通讯格式:** 文本行格式（ASCII）

#### 命令协议

**接收命令格式：**
| 命令 | 功能 | 触发事件 |
|------|------|--------|
| `'1'` | 开始记录 | `sys_state.is_recording = true` |
| `'2'` | 停止记录 | `sys_state.is_recording = false` |

**发送数据格式（自动生成）：**
```
TS:12345,AX:9.80,AY:0.23,ROLL:0.45,PITCH:2.34,LAT:31.234567,LON:120.456789\r\n
```

当 GPS 无定位时：
```
TS:12345,AX:9.80,AY:0.23,ROLL:0.45,PITCH:2.34,LAT:NO_FIX,LON:NO_FIX\r\n
```

#### 公开方法

##### `bool begin()`
**功能:** 初始化 LoRa 通信模块

**参数:** 无

**返回值:**
- `true` - 初始化成功
- `false` - 初始化失败

**示例:**
```cpp
if (lora.begin()) {
  Serial.println("LoRa communication ready");
}
```

---

##### `void processSerialData()`
**功能:** 处理 UART2 接收缓冲中的 LoRa 数据

**参数:** 无

**返回值:** 无

**特点:**
- 非阻塞
- 自动识别命令字符（'1'、'2'）
- 自动触发注册的回调函数
- 忽略无效命令

**重要:** 该函数应在 `loop()` 中频繁调用

**示例:**
```cpp
void loop() {
  lora.processSerialData();  // 在每个循环中调用
  // ... 其他代码
}
```

---

##### `void sendData(const IMUData &imu_data, const GPSData &gps_data)`
**功能:** 发送 IMU + GPS 数据

**参数:**
- `imu_data` - IMU 传感器数据
- `gps_data` - GPS 定位数据

**返回值:** 无

**发送内容:**
- 时间戳
- Ax, Ay 加速度
- Roll, Pitch 欧拉角
- 纬度、经度（GCJ02 坐标系）

**推荐发送周期:** 500 ms

**示例:**
```cpp
if (millis() - lastLoRaSend >= 500) {
  lora.sendData(current_imu_data, current_gps_data);
  lastLoRaSend = millis();
}
```

---

##### `void setCommandCallback(LoRaCommandCallback callback)`
**功能:** 注册命令接收回调函数

**参数:**
- `callback` - 回调函数指针（类型：`void(*)(char cmd)`）

**返回值:** 无

**回调时机:** 每次接收到有效命令时（'1' 或 '2'）

**示例:**
```cpp
void onLoRaCommand(char cmd) {
  if (cmd == '1') {
    sys_state.is_recording = true;
    Serial.println("Recording started");
  } else if (cmd == '2') {
    sys_state.is_recording = false;
    Serial.println("Recording stopped");
  }
}

void setup() {
  // ... 初始化代码 ...
  lora.setCommandCallback(onLoRaCommand);
}
```

---

##### `void sendCustomData(const String &data)`
**功能:** 发送自定义格式数据

**参数:**
- `data` - 自定义文本数据

**返回值:** 无

**用途:** 调试、固件升级、特殊协议等

**示例:**
```cpp
lora.sendCustomData("DEBUG:System Status OK\r\n");
```

---

## 数据结构

### IMUData

```cpp
struct IMUData {
  float ax, ay, az;           // 加速度 (m/s²)
  float gx, gy, gz;           // 角速度 (°/s)
  float roll, pitch, yaw;     // 欧拉角 (°)
  unsigned long timestamp;    // 时间戳 (ms)
};
```

**数据范围:**
- **加速度:** ±16g (-156.8 ~ +156.8 m/s²)
- **角速度:** ±2000 °/s
- **欧拉角:** ±180°
- **时间戳:** 从 ESP32 启动时开始计时

**精度:**
- 浮点数精度：3-4 位有效数字（由 JY901 传感器决定）
- 建议在后期处理中使用低通滤波

---

### GPSData

```cpp
struct GPSData {
  double wgs84_lat;          // WGS84 纬度 (度)
  double wgs84_lon;          // WGS84 经度 (度)
  double gcj02_lat;          // GCJ02 纬度 (度) - 中国坐标系
  double gcj02_lon;          // GCJ02 经度 (度) - 中国坐标系
  String lat_dir;            // 纬度方向 ("N" 或 "S")
  String lon_dir;            // 经度方向 ("E" 或 "W")
  bool is_fixed;             // 定位状态
  unsigned long timestamp;   // 时间戳 (ms)
};
```

**数据说明:**
- **纬度范围:** 0° ~ 90°（南半球为负）
- **经度范围:** 0° ~ 180°（西半球为负）
- **精度:** 6 位小数（约 0.1 米）

---

### SystemState

```cpp
struct SystemState {
  bool is_recording;         // 是否正在记录到 SD 卡
  bool is_card_ready;        // SD 卡是否就绪
  bool is_gps_fixed;         // GPS 是否定位
  bool is_tft_ready;         // TFT 屏幕是否就绪
};
```

---

## 系统集成 API

### 全局对象

```cpp
extern IMUModule imu;
extern GPSModule gps;
extern TFTDisplay display;
extern SDStorage sd_storage;
extern LoRaCommunication lora;
extern SystemState sys_state;
```

### 回调函数

#### `void onLoRaCommand(char cmd)`
**功能:** LoRa 命令回调（由开发者实现）

**参数:**
- `cmd` - 命令字符 ('1' 或 '2')

**默认实现:**
```cpp
void onLoRaCommand(char cmd) {
  if (cmd == '1') {
    sys_state.is_recording = true;
  } else if (cmd == '2') {
    sys_state.is_recording = false;
  }
}
```

---

### 时序管理

**推荐的采样和刷新周期：**

```cpp
#define IMU_READ_INTERVAL 100       // IMU 采集：100ms
#define TFT_REFRESH_INTERVAL 200    // TFT 显示：200ms
#define SD_LOG_INTERVAL 100         // SD 卡记录：100ms（与 IMU 同步）
#define LORA_SEND_INTERVAL 500      // LoRa 发送：500ms
```

**时序图：**
```
时间 (ms)  0   50   100  150  200  250  300  350  400  450  500
IMU采集    |    ✓              ✓              ✓              ✓
TFT刷新    |         ✓                   ✓                   ✓
SD记录     |    ✓              ✓              ✓              ✓
LoRa发送   |                        ✓                        ✓
```

---

### 错误处理建议

**初始化阶段错误：**
```cpp
if (!display.begin()) {
  // TFT 必须就绪，系统无法启动
  while (1);  // 死循环，等待手动重启
}

if (!sd_storage.begin()) {
  // SD 卡可选，继续启动但禁用记录功能
  sys_state.is_card_ready = false;
  display.showError("SD Card Error");
}
```

**运行阶段错误：**
```cpp
if (!imu.readData(current_imu_data)) {
  Serial.println("[ERROR] IMU read failed");
  // 继续使用上一帧数据
}

if (!sd_storage.logData(current_imu_data, current_gps_data)) {
  Serial.println("[ERROR] SD card write failed");
  // 尝试检查 SD 卡是否仍然就绪，或显示警告
}
```

---

## 常见问题

### Q1：为什么 GPS 坐标同时存储 WGS84 和 GCJ02？
**A:** GPS 模块输出的是标准 WGS84 坐标，但中国大陆的地图应用（高德、腾讯）采用 GCJ02 坐标系。双坐标存储可满足国内外应用需求，后期分析时可灵活选择。

### Q2：TFT 显示和 SD 卡为什么要共用 SPI 总线？
**A:** ESP32 仅有一个 VSPI，为节省硬件成本和 PCB 面积，通过 CS 信号分选。`busTake()`/`busRelease()` 函数自动管理，开发者无需手动干预。

### Q3：LoRa 命令为什么不能有更多功能？
**A:** 当前设计简化协议复杂度，专注于记录控制。后期可扩展为 0xFF 帧头 + 长度 + 命令/数据 + 校验和的二进制协议。

### Q4：可以同时运行多个采集盒子吗？
**A:** 可以。只需为每个 LoRa 模块独立配置频率/ID（具体步骤参考 LoRa 硬件手册）。多盒子数据会并行采集并无线发送。

---

## 附录：引脚配置表

| 功能 | 引脚号 | 信号 | 方向 |
|------|--------|------|------|
| **IMU (I2C)** |
| SDA | 21 | I2C_SDA | 双向 |
| SCL | 22 | I2C_SCL | 双向 |
| **TFT (SPI)** |
| CS | 5 | TFT_CS | OUT |
| DC | 2 | TFT_DC | OUT |
| RST | 4 | TFT_RST | OUT |
| CLK | 18 | SPI_CLK | OUT |
| MOSI | 23 | SPI_MOSI | OUT |
| MISO | 19 | SPI_MISO | IN |
| **SD Card (SPI)** |
| CS | 15 | SD_CS | OUT |
| CLK | 18 | SPI_CLK | OUT (共用) |
| MOSI | 23 | SPI_MOSI | OUT (共用) |
| MISO | 19 | SPI_MISO | IN (共用) |
| **GPS (UART1)** |
| RX | 32 | GPS_RX | IN |
| TX | 33 | GPS_TX | OUT |
| **LoRa (UART2)** |
| RX | 16 | LoRa_RX | IN |
| TX | 17 | LoRa_TX | OUT |

---

**文档完成日期:** 2026-03-08  
**最后更新:** 2026-03-08
