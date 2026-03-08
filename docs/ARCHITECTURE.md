# 龙舟采集盒子 - 架构设计文档

**文档版本:** v1.0
**设计日期:** 2026-03-08
**作者:** 项目团队
**适用系统:** DragonBoat-Collection-Box v1.0

---

## 目录

1. [架构概览](#架构概览)
2. [模块设计](#模块设计)
3. [时序管理](#时序管理)
4. [通信协议](#通信协议)
5. [数据流](#数据流)
6. [设计模式](#设计模式)
7. [扩展性分析](#扩展性分析)
8. [性能分析](#性能分析)
9. [安全考虑](#安全考虑)

---

## 架构概览

### 分层架构

```
┌────────────────────────────────────────────────────────────┐
│                   应用层 (Application)                      │
│  - 数据采集逻辑                                              │
│  - 业务流程控制（记录开/关）                                  │
│  - 事件回调处理                                              │
└─────────────────────────────────────────────────────────────┘
                            │
┌────────────────────────────────────────────────────────────┐
│                   模块层 (Modules)                           │
│  ┌─────────┬────────┬────────┬────────┬─────────┐          │
│  │ IMU Module    │ GPS Module   │ TFT Display  │ SD Storage  │ LoRa Comm  │
│  │ (I2C)        │ (UART1)      │ (SPI)       │ (SPI)       │ (UART2)    │
│  └─────────┴────────┴────────┴────────┴─────────┘          │
└────────────────────────────────────────────────────────────┘
                            │
┌────────────────────────────────────────────────────────────┐
│                  驱动层 (Drivers)                            │
│  - Arduino Wire (I2C)                                       │
│  - Arduino Serial (UART)                                    │
│  - Arduino SPI                                              │
│  - Adafruit GFX/ST7735 Library                              │
│  - Arduino SD Library                                       │
└────────────────────────────────────────────────────────────┘
                            │
┌────────────────────────────────────────────────────────────┐
│                  硬件层 (Hardware)                           │
│  - ESP32 MCU + Sensors                                      │
└────────────────────────────────────────────────────────────┘
```

### 设计哲学

| 原则             | 说明         | 实现                           |
| ---------------- | ------------ | ------------------------------ |
| **模块化** | 功能完全独立 | 每个模块一个类，只负责一项工作 |
| **非阻塞** | 不阻塞主循环 | 所有操作都设计为即时返回       |
| **可测试** | 容易单元测试 | 清晰的接口，最小化依赖         |
| **可扩展** | 易于添加功能 | 标准的初始化/处理模式          |
| **通用化** | 减少硬编码   | 配置全部在 config.h 中         |

---

## 模块设计

### 1. IMU 模块

**职责：** I2C 通信 + 传感器数据读取 + 物理量缩放

```
┌──────────────┐
│ readJY901Reg │  (private)
│ 读取 16 位    │
└──────┬───────┘
       │
┌──────▼────────┐
│ scaleData()   │  (private)
│ 缩放转换      │
└──────┬────────┘
       │
┌──────▼──────────────┐
│ readData()           │  (public)
│ 触发一帧完整读取     │
└──────┬───────────────┘
       │
    IMUData
    (ax, ay, az, gx, gy, gz, roll, pitch, yaw, timestamp)
```

**关键设计决策：**

- ✅ 不包含卡尔曼滤波（保留给上层应用）
- ✅ 直接返回 JY901 缩放后的值
- ✅ 错误处理：I2C 失败时返回 false，但保持上次数据

**关键方法：**

```cpp
bool begin()                     // 初始化 I2C 和传感器
bool readData(IMUData &data)     // 读取一帧数据
const IMUData& getLastData()     // 获取缓存的上一帧
```

### 2. GPS 模块

**职责：** NMEA 解析 + 坐标转换 + 定位状态管理

```
NMEA 句子 (UART1)
    ↓
processSerialData()  (public, 非阻塞)
    ↓
detect $GPGGA / $BDGGA / $GNGGA
    ↓
parseGGA()  (private)
    ├─ ddm2dd()       → 度分→十进制度
    ├─ WGS84→GCJ02()  → 坐标系转换
    └─ update GPSData
    ↓
GPSData
(wgs84_lat, wgs84_lon, gcj02_lat, gcj02_lon, is_fixed, ...)
```

**坐标转换算法：**

- **输入:** WGS84 (GPS 原生输出)
- **过程:** 三角函数 + 椭球体参数
- **输出:** GCJ02 (中国地图标准)
- **无转换:** 坐标在中国范围外时原样输出

**关键设计决策：**

- ✅ 自动缓冲和行解析（无需上层处理缓冲区）
- ✅ 支持多种 GNSS 前缀（GPS、BDS、GNSS）
- ✅ 双坐标系存储（WGS84 + GCJ02）
- ⚠️ 暂未实现 SOG (Speed Over Ground) 提取

### 3. TFT 显示模块

**职责：** SPI 通信 + UI 管理 + 动态更新

```
drawStaticUI()
    │ 绘制一次性元素
    ├─ 标题栏
    ├─ 标签文字
    │
updateDisplay()
    │ 定期更新动态数据（200ms 周期）
    ├─ drawStatusLED()        → 状态灯
    ├─ updateAcceleration()   → 加速度区域
    ├─ updateGyroscope()      → 角速度区域
    ├─ updateAttitude()       → 欧拉角区域
    ├─ updateGPS()            → GPS 区域
    ├─ updateRecordingState() → 状态文本
    └─ updateFileName()       → 文件名
```

**SPI 总线仲裁：**

```
TFT 和 SD 卡共用 VSPI:
  CLK  = GPIO18
  MOSI = GPIO23
  MISO = GPIO19

使用 CS 信号选择设备:
  TFT_CS (GPIO5)   = 0 时选中 TFT
  SD_CS  (GPIO15)  = 0 时选中 SD 卡

busTake()    → digitalWrite(TFT_CS, LOW)
busRelease() → digitalWrite(TFT_CS, HIGH)
```

**关键设计决策：**

- ✅ 分离静态和动态绘制（初始化时绘制一次，之后只更新数据区域）
- ✅ 区域清除防止残影（fillRect 清除后再写入）
- ✅ 颜色编码区分数据类别
- ⚠️ 未实现触摸交互（可扩展）

### 4. SD 存储模块

**职责：** 文件管理 + CSV 格式化 + 数据落盘

```
findNextFileName()
    │ 扫描已有文件
    └─ 返回 1.csv / 2.csv / ... / data.csv

formatCSVLine()
    │ 组装数据行
    ├─ IMU 数据（必录）
    └─ GPS 数据（有定位记录，否则填 NO_FIX）

logData()
    │ 打开文件 → 写行 → 关闭
    ├─ 使用 FILE_WRITE 追加模式
    └─ 立即 close() 确保落盘
```

**CSV 格式设计：**

```
16 列数据 (可扩展)
Timestamp(ms), Ax-Gz (9 列), Roll/Pitch/Yaw (3 列),
WGS84_Lat, WGS84_Lon, GCJ02_Lat, GCJ02_Lon,
Lat_Dir, Lon_Dir
```

**文件管理策略：**

- 启动时自动下一个可用文件名
- GPS 无定位时填充 "NO_FIX"，不影响 CSV 格式
- 每行立即写入，无缓冲（确保掉电数据不丢失）

**关键设计决策：**

- ✅ 3 次重试 + 延迟确保初始化成功
- ✅ 自动拼装 CSV 行（无需上层手动格式化）
- ✅ 数据完整性优先（即时落盘）
- ⚠️ 未实现压缩或增量备份

### 5. LoRa 通信模块

**职责：** 命令接收 + 数据打包 + 异步发送

```
processSerialData()  (public, 定期调用)
    │ 从 UART2 接收缓冲读数据
    ├─ 识别命令 '1' / '2'
    ├─ 触发 command_callback()
    │
sendData()           (public, 周期调用)
    │ 打包 IMU + GPS 数据
    ├─ 格式: TS:xxx,AX:xxx,...\r\n
    └─ 发送到 UART2

sendCustomData()     (public, 按需调用)
    │ 发送自定义文本
    └─ 用于调试或扩展协议
```

**协议设计：**

**接收协议：**

```
命令字符: '1' 或 '2' (各 1 字节)
处理: 识别后触发回调
特点: 简单、可靠，适合龙舟场景
```

**发送协议：**

```
格式: 关键字:值,关键字:值,...\r\n

示例 (GPS 已定位):
TS:12345,AX:9.80,AY:0.23,ROLL:0.45,PITCH:2.34,LAT:31.2345,LON:120.5678\r\n

示例 (GPS 无定位):
TS:12345,AX:9.80,AY:0.23,ROLL:0.45,PITCH:2.34,LAT:NO_FIX,LON:NO_FIX\r\n

字段说明:
  TS    - 时间戳(ms)         [整数]
  AX/AY - 加速度(m/s²)       [浮点]
  ROLL  - 滚动角(°)          [浮点]
  PITCH - 俯仰角(°)          [浮点]
  LAT   - 纬度(GCJ02)        [浮点 或 NO_FIX]
  LON   - 经度(GCJ02)        [浮点 或 NO_FIX]
```

**关键设计决策：**

- ✅ 文本协议易于调试和扩展
- ✅ 每条消息独立（可丢失而不影响整体）
- ✅ 回调机制松耦合
- ⚠️ 无校验位（假设 LoRa 链路可靠）

---

## 时序管理

### 任务调度

```cpp
// main.cpp 中的 loop() 循环
unsigned long now = millis();

// 1. 异步处理(无周期)
lora.processSerialData();       // 连续处理接收缓冲
gps.processSerialData();        // 连续处理接收缓冲

// 2. 同步采集(有周期)
if (now - lastIMURead >= IMU_READ_INTERVAL) {
    imu.readData(current_imu_data);
    lastIMURead = now;
}

if (now - lastTFTRefresh >= TFT_REFRESH_INTERVAL) {
    display.updateDisplay(...);
    lastTFTRefresh = now;
}

if (sys_state.is_recording && (now - lastSDLog >= SD_LOG_INTERVAL)) {
    sd_storage.logData(...);
    lastSDLog = now;
}

if (now - lastLoRaSend >= LORA_SEND_INTERVAL) {
    lora.sendData(...);
    lastLoRaSend = now;
}

delay(5);  // CPU 休息
```

### 时序图（毫秒精度）

```
时间 (ms)   |  0   50  100  150  200  250  300  350  400  450  500
────────────┼─────────────────────────────────────────────────────
GPS数据接收  | ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
            |
LoRa命令    | ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
接收        |
            |
IMU读取      |  ✓           ✓           ✓           ✓           ✓
            |
TFT刷新      |           ✓               ✓               ✓
            |
SD卡记录     |  ✓           ✓           ✓           ✓           ✓
(仅记录中)   |
            |
LoRa发送    |                     ✓                       ✓
            |

图例:
  ▓ = 连续处理基于中断/缓冲的异步任务
  ✓ = 周期性同步任务的执行点
```

### 时间余量分析

**总周期:** 100 ms (最小公倍数)

**任务分布：**

- GPS/LoRa 异步处理: ~5-10 ms/周期
- IMU 读取: ~10-20 ms (I2C @ 400kHz)
- TFT 更新: ~50-100 ms (SPI 驱动)
- SD 写入: ~10-30 ms (SPI 驱动)
- LoRa 发送: ~5 ms (UART 异步)

**可用余量:** 约 30-50 ms 空闲时间，足以应对突发任务或缓冲处理

---

## 通信协议

### I2C 协议 (IMU)

```
Master (ESP32) ←→ Slave (JY901 @ 0x50)

SEND 寄存器地址:
  Master | Start | Addr+W | RegAddr | Stop |
         | Bit  | (0x50<<1+0) | (0x34-0x3F) |
       
RECV 数据 (16位):
  Master | Start | Addr+R | Data_Low | Data_Hi | NAK | Stop |
         | Bit   | (0x50<<1+1) | (8位)  | (8位)    |    |

时序: SCL = 400 kHz, 典型周期 = 25 µs
耗时(每寄存器): ~1 ms
总耗时(9个寄存器): ~10-15 ms per readData()
```

### UART 协议 (GPS & LoRa)

```
参数: 9600 bps, 8N1 (8 bits, No parity, 1 stop bit)
       典型周期 = 104 µs per bit

GPS (UART1) from device:
  $GPGGA,hhmmss.ss,ddmm.mmmm,N,dddmm.mmmm,E,1,12,0.8,10.2,M,...*hh<CR><LF>
  耗时: ~100-200 ms (整句)

LoRa (UART2):
  - 输入: '1' 或 '2' (1 byte = ~1 ms)
  - 输出: "TS:xxx,AX:xxx,...\r\n" (~80 bytes = ~80 ms)
```

### SPI 协议 (TFT & SD)

```
参数: 4 MHz, Mode 0 (CPOL=0, CPHA=0)
      典型周期 = 250 ns per bit

TFT 显示更新: ~50-100 ms (取决于更新区域大小)
SD 卡写入: ~10-30 ms (依赖 SD 卡性能)

CS 仲裁:
  1. digitalWrite(TFT_CS, LOW)   → 拉低 CS，选中 TFT
  2. [SPI 操作]
  3. digitalWrite(TFT_CS, HIGH)  → 拉高 CS，释放总线
```

---

## 数据流

### 数据采集链

```
传感器 ────────────────→ 模块读取 ────────→ 应用层缓存 ────────→ 消费者
                        (驱动级)        (main.cpp)      (显示/存储/通信)

IMU:  JY901 @I2C  →  imu.readData()  →  current_imu_data  →  TFT/SD/LoRa

GPS:  Module @UART1 → gps.processSerialData() → current_gps_data → TFT/SD/LoRa
                      [自动在后台更新]

TFT:  [显存]  ←────────────────────────────────── display.updateDisplay()

SD:   [文件]  ←────────────────────────────────── sd_storage.logData()

LoRa: [UART2] ←────────────────────────────────── lora.sendData()
```

### 数据一致性

**关键问题:** IMU 和 GPS 数据时间戳不同步

**当前解决方案:**

- 每个模块独立时间戳
- 上层应用负责关联（例如：后期分析时按时间戳对齐）

**可选改进:**

- 增加全局时间戳缓冲
- 使用活动缓冲区 (double buffering) 原子交换

---

## 设计模式

### 1. 单例模式（隐式）

```cpp
// 全局实例
IMUModule imu;        // 唯一实例，main.cpp 中创建
GPSModule gps;
TFTDisplay display;
// ...

// 特点:
// - 隐式管理生命周期（由 Arduino 框架控制）
// - 不验证唯一性（需要开发者自觉遵守）
// - 优点：简洁，适合嵌入式环境
// - 缺点：难以跨文件使用（需要 extern 声明）
```

### 2. 观察者模式（回调）

```cpp
// 定义
typedef void (*LoRaCommandCallback)(char cmd);

// 使用
lora.setCommandCallback(onLoRaCommand);

void onLoRaCommand(char cmd) {
    if (cmd == '1') sys_state.is_recording = true;
    // ...
}

// 特点:
// - 解耦 LoRa 模块和业务逻辑
// - 支持多个观察者（扩展：改用 std::vector<callback>）
// - 缺点：硬编码函数指针（无运行时灵活性）
```

### 3. RAII 模式（隐式）

```cpp
// File 对象自动管理资源
File dataFile = SD.open(filename, FILE_WRITE);
if (dataFile) {
    dataFile.println(content);
    dataFile.close();  // 显式关闭确保落盘
}

// 特点:
// - 资源获取即初始化 (RAII)
// - Arduino SD 库的 File 类自动在析构时清理
// - 风险：若忘记 close()，缓冲区延迟写入
```

### 4. 配置对象模式

```cpp
// config.h 中定义所有常数
#define IMU_READ_INTERVAL 100
#define BAUD_RATE 115200

// 避免:
// - 硬编码数值分散各地
// - 调整参数需要编辑多个源文件

// 优点:
// - 集中管理配置
// - 易于快速调参和版本控制
```

---

## 扩展性分析

### 添加新传感器

**难度:** 低

**步骤:**

1. include/modules/sensors.h → 新数据结构
2. include/modules/xxx_module.h → 驱动类声明
3. src/modules/xxx_module.cpp → 实现
4. src/main.cpp → 创建实例 + loop() 集成
5. docs/ → 更新文档

**示例:** 气压计（Barometer）

```cpp
// sensors.h
struct BarometerData {
    float pressure;      // kPa
    float altitude;      // m
    float temperature;   // °C
    unsigned long timestamp;
};

// barometer_module.h / .cpp
class BarometerModule { /* ... */ };

// main.cpp
BarometerModule barometer;
// ...
if (!barometer.begin()) { /* error */ }
// ...
barometer.readData(current_barometer_data);
```

### 修改通信协议

**LoRa 协议升级** （当前：简单命令 → 目标：帧格式）

```cpp
// 当前协议
'1' or '2'

// 升级协议 (示例：二进制帧)
[SYNC:0xFF] [TYPE:0x01] [LENGTH:0x0A] [PAYLOAD:10字节] [CRC:0xAB] [END:0x0D]

优点:
  - 支持更多命令类型 (SYNC+TYPE 编码)
  - 校验机制 (CRC)
  - 长数据支持 (LENGTH 字段)

修改点:
  - lora_comm.cpp: processSerialData() 改为帧解析
  - sendData() 改为帧打包
  - API 兼容性破坏（需版本升级）
```

### 添加新的显示器

**替换 TFT 为 OLED** （当前：TFT 1.8" → 新：OLED 0.96"）

```cpp
// 方案 1: 新建 OLEDDisplay 类
// - 优点：保留原 TFTDisplay，并行维护
// - 缺点：代码重复，两套实现

// 方案 2: 抽象显示接口
//
// 在 include/modules/ 中新建 display_interface.h:
// class IDisplay {
// public:
//   virtual void drawStaticUI() = 0;
//   virtual void updateDisplay(...) = 0;
// };
//
// 然后:
// class TFTDisplay : public IDisplay { /* ... */ };
// class OLEDDisplay : public IDisplay { /* ... */ };

// main.cpp 中使用多态:
IDisplay *display;
if (USE_OLED) {
    display = new OLEDDisplay();
} else {
    display = new TFTDisplay();
}
```

---

## 性能分析

### CPU 负载

**主循环周期:** 5 ms (delay)

**每 100 ms 的任务分解：**

| 任务      | 耗时  | CPU占比 |
| --------- | ----- | ------- |
| IMU 读取  | 15 ms | 15%     |
| TFT 刷新  | 50 ms | 50%     |
| SD 写入   | 20 ms | 20%     |
| LoRa 发送 | 5 ms  | 5%      |
| GPS 异步  | ~5 ms | 5%      |
| 其他开销  | 10 ms | 10%     |

**总计:** ~100 ms 内基本完成，CPU 频率 240 MHz 足够

### 内存占用

**动态分配：**

```
- IMUData 结构: ~40 字节
- GPSData 结构: ~60 字节
- SystemState 结构: ~4 字节
- 全局变量 & 对象: ~500 字节
- Stack 余量: 520 KB 中的 ~100-200 KB 可用

结论：内存充足
```

### 功耗估算

**单位时间能耗：**

| 模块                    | 电流(mA) | 占比 |
| ----------------------- | -------- | ---- |
| ESP32 @ 240MHz, Working | 80       | 60%  |
| IMU (常开)              | 10       | 7%   |
| GPS (常开)              | 25       | 19%  |
| TFT (5分钟内)           | 10       | 7%   |
| LoRa (工作)             | 5        | 3%   |
| 其他                    | 3        | 4%   |

**总计:** 约 100-150 mA

**建议电源:**

- 2000 mAh Li-Po 电池 = 13-20 小时续航
- 或接 solar panel 充电

---

## 安全考虑

### 1. 数据完整性

**风险:** SD 卡数据丢失（掉电、卡拔出）

**对策：**

- ✅ 每行立即 flush() (当前实现)
- ✅ 定期备份（多文件轮转）
- ✅ CSV 校验和（可选扩展）

**不采用：**

- ❌ 驱动器级 RAID (价格昂贵，内存不足)
- ❌ 网络备份 (延迟高，龙舟场景网络不可靠)

### 2. 通信安全

**风险:** LoRa 被截获/篡改

**当前状态:** 无加密（龙舟训练场景，信息非机密）

**如需加密：**

```cpp
// AES-128 加密库: tiny-AES-c
// 代码示例:
uint8_t key[16] = {/* 32个十六进制数 */};
uint8_t plaintext[16] = data;
uint8_t ciphertext[16];
AES_ECB_encrypt(plaintext, key, ciphertext);
LoRaSerial.write(ciphertext, 16);
```

### 3. 实时性保证

**风险:** 传感器采样不准确（主循环被阻塞）

**已采用措施：**

- ✅ 所有操作非阻塞
- ✅ 独立 UART 接收缓冲（不丢失数据）
- ✅ 独立 SPI 驱动（硬件级别同步）

**无法完全保证：**

- ❌ 未使用实时操作系统 (RTOS)
- ❌ 中断优先级固定（Arduino 框架限制）

**改进方案** (Advanced):

```cpp
// 使用 FreeRTOS 多任务
BaseType_t xReturned;
TaskHandle_t xHandle = NULL;

xReturned = xTaskCreate(
    imuTaskFunction,       // 任务函数
    "IMU Task",            // 任务名
    2048,                  // 栈大小
    NULL,                  // 参数
    3,                     // 优先级 (高)
    &xHandle               // 任务句柄
);

// 优点：保证 IMU 采样频率
// 缺点：代码复杂度大幅上升，需要互斥量 (mutex) 保护共享数据
```

### 4. 环境耐受

**水分防护：**

- 代码无关，需要 PCB 级别的防水设计
- 推荐：环氧树脂灌封或 IP67 等级外壳

**温度范围：**

- ESP32: -40°C ~ +85°C (规格书)
- 传感器各异（参考数据手册）
- 代码级别无特殊处理

---

## 版本管理

### 语义版本 (Semantic Versioning)

**当前版本:** v1.0.0

**格式:** MAJOR.MINOR.PATCH

- **MAJOR** (1): 架构生成或不兼容的 API 变更 → v2.0.0
- **MINOR** (0): 新增功能但向后兼容 → v1.1.0
  - 示例: 新增温度传感器
- **PATCH** (0): 修复 BUG 或代码优化 → v1.0.1
  - 示例: 修复 GPS 坐标转换精度问题

### 发布记录

| 版本  | 日期       | 主要变更               |
| ----- | ---------- | ---------------------- |
| 1.0.0 | 2026-03-08 | 初始版本：5 模块集成   |
| 1.1.0 | TBD        | 添加气压计模块         |
| 2.0.0 | TBD        | 升级 LoRa 协议至帧格式 |

---

## 参考资源

- [ESP32 Datasheet](https://www.espressif.com)
- [JY901 IMU Manual](manufacturer)
- [Arduino Wire Library](https://www.arduino.cc)
- [Arduino SD Library](https://www.arduino.cc)
- [Adafruit ST7735 Library](https://github.com/adafruit)
- [WGS84/GCJ02 坐标转换算法](https://github.com/wandergis/coordtransform)

---

**文档完成:** 2026-03-08
