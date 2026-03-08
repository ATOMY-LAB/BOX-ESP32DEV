# 龙舟桨频/船速采集盒子 - 项目说明文档

**项目编码:** DragonBoat-Collection-Box-v1.0  
**硬件平台:** ESP32-DevKit  
**开发工具:** PlatformIO + Visual Studio Code  
**编程语言:** C/C++ (Arduino Framework)  
**文档版本:** v1.0  
**更新日期:** 2026-03-08  

---

## 目录

1. [项目概述](#项目概述)
2. [系统架构](#系统架构)
3. [硬件配置](#硬件配置)
4. [软件结构](#软件结构)
5. [快速开始](#快速开始)
6. [功能说明](#功能说明)
7. [数据流向](#数据流向)
8. [团队协作指南](#团队协作指南)
9. [故障排查](#故障排查)
10. [扩展开发](#扩展开发)

---

## 项目概述

### 应用背景

本项目为**龙舟训练/比赛实时数据采集系统**。在龙舟上固定安装此采集盒子，可实时捕捉以下关键指标：

- **桨频估计**（通过 IMU 加速度变化）
- **船速测量**（通过 GPS 实时速度）
- **船体运动**（通过 IMU 欧拉角和角速度）
- **地理位置追踪**（通过 GPS 定位）

### 核心功能

| 功能模块 | 传感器 | 数据输出 | 更新频率 |
|---------|--------|--------|---------|
| **惯性测量** | JY901 IMU | 加速度、角速度、欧拉角 | 100 ms |
| **定位导航** | GPS 模块 | 纬度、经度、速度向 | 连续 |
| **实时显示** | 1.8" TFT 屏幕 | 所有传感器数据 + 系统状态 | 200 ms |
| **数据记录** | SD 卡 | CSV 格式完整原始数据 | 100 ms |
| **无线通信** | LoRa 模块 | 远程控制 + 数据回传 | 命令/500ms |

### 主要特点

✅ **模块化设计** - 各功能独立，易于扩展  
✅ **非阻塞架构** - 无死锁，实时性好  
✅ **本地存储** - SD 卡记录完整数据用于后期分析  
✅ **无线链接** - LoRa 远程控制，支持岸边指挥  
✅ **实时监控** - 现场 TFT 显示关键指标  
✅ **坐标适配** - 自动 WGS84→GCJ02 转换，兼容国内地图  

---

## 系统架构

### 硬件拓扑

```
┌─────────────────────────────────────┐
│      ESP32-DevKit (主控)             │
│  - ARM Xtensa 32-bit @ 240 MHz      │
│  - 520 KB RAM                       │
│  - Wi-Fi + Bluetooth                │
└─────────────────────────────────────┘
         │        │        │        │
    SPI  │        │ I2C    │ UART1  │ UART2
    共享 │   ┌────┘        │        │
         │   │             │        │
    ┌────▼─┬▼──┐     ┌──────▼──┐  ┌▼─────┐
    │  TFT │SD │     │  JY901  │  │ GPS  │
    │1.8"  │卡 │     │  IMU    │  │模块  │
    │160x  │   │     │         │  │      │
    │128   │   │     │         │  │      │
    └──────┴───┘     └─────────┘  └──────┘
                           
                     ┌────────────┐
                     │   LoRa     │
                     │  模块      │ (UART2)
                     │ 无线链路   │
                     └────────────┘
```

### 通信总线

| 总线 | 连接设备 | 速率 | 模式 | 备注 |
|------|---------|------|------|------|
| **I2C** | IMU(JY901) | 400 kHz | Master | ESP32 内部控制器 |
| **SPI (VSPI)** | TFT + SD 卡 | 4 MHz | CS 分选 | 共享硬件引脚 18/23/19 |
| **UART1** | GPS 模块 | 9600 bps | 异步 | NMEA-0183 格式 |
| **UART2** | LoRa 模块 | 9600 bps | 异步 | 文本协议 |

---

## 硬件配置

### 传感器清单

| 编号 | 器件 | 规格 | 功能 |
|------|------|------|------|
| 1 | JY901 IMU | 6 轴 MEMS | 加速度 + 角速度 + 欧拉角 |
| 2 | GPS 模块 | GNSS (GPS/BDS) | 全球定位 + 速度 |
| 3 | ST7735S TFT | 1.8", 160×128 | 实时显示 |
| 4 | SD 卡 | >= 4GB | 数据记录 |
| 5 | LoRa 模块 * | 868/915 MHz | 无线通信（可选） |

**引脚分配：**
```
ESP32-DevKit
├─ I2C (GPIO21=SDA, GPIO22=SCL)
│  └─ JY901 (Addr: 0x50)
├─ SPI VSPI (GPIO18=CLK, GPIO23=MOSI, GPIO19=MISO)
│  ├─ TFT (CS=GPIO5, DC=GPIO2, RST=GPIO4)
│  └─ SD Card (CS=GPIO15)
├─ UART1 (GPIO32=RX, GPIO33=TX)
│  └─ GPS Module
└─ UART2 (GPIO16=RX, GPIO17=TX)
   └─ LoRa Module
```

---

## 软件结构

### 项目目录树

```
Box-esp32dev/
├── include/
│   ├── config.h                      # 全局配置（引脚、波特率、常量）
│   ├── modules/
│   │   ├── sensors.h                 # 数据结构定义（IMUData, GPSData 等）
│   │   ├── imu_module.h              # IMU 驱动类
│   │   ├── gps_module.h              # GPS 驱动类
│   │   ├── tft_display.h             # TFT 显示类
│   │   ├── sd_storage.h              # SD 卡存储类
│   │   └── lora_comm.h               # LoRa 通信类
│   └── README
├── src/
│   ├── main.cpp                      # 主程序（集成所有模块）
│   ├── modules/
│   │   ├── imu_module.cpp            # IMU 实现
│   │   ├── gps_module.cpp            # GPS 实现
│   │   ├── tft_display.cpp           # TFT 实现
│   │   ├── sd_storage.cpp            # SD 卡实现
│   │   └── lora_comm.cpp             # LoRa 实现
│   ├── send-TFT-IMU-GPS-Lora.ino     # 原始项目代码（备份）
│   └── README
├── docs/
│   ├── README.md                     # 项目说明（本文件）
│   ├── API_DOCUMENT.md               # API 接口文档
│   ├── ARCHITECTURE.md               # 架构详细设计
│   └── DEVELOPMENT_GUIDE.md          # 开发指南（可选）
├── platformio.ini                    # PlatformIO 配置文件
├── lib/
│   └── README
└── test/
    └── README
```

### 模块设计原则

**1. 单一职责**
- 每个模块只负责一个传感器或功能
- IMUModule 仅处理 I2C 通信和数据转换
- GPSModule 仅处理 UART 数据和坐标解析

**2. 即插即用**
- 模块初始化相互独立
- 故障不影响其他模块运行
- 支持热插拔扩展

**3. 非阻塞设计**
- 无论何时都不会"卡住"主循环
- 使用回调机制处理异步事件
- 时序由 `main.cpp` 统一管理

**4. 文档驱动**
- 每个类都有详细的 Doxygen 注释
- 每个公开方法都有参数说明和示例代码

---

## 快速开始

### 第 1 步：环境配置

**安装 PlatformIO：**
```bash
# 在 VS Code 中
1. 打开 Extension(扩展) 市场
2. 搜索 "PlatformIO IDE"
3. 点击 Install
4. 重启 VS Code
```

**打开项目：**
```bash
1. File > Open Folder
2. 选择 Box-esp32dev 目录
3. PlatformIO 自动识别 platformio.ini
```

### 第 2 步：编译和烧写

**编译项目：**
```bash
# PlatformIO 侧边栏 > Build
# 或者终端命令：
pio run -e Box-esp32dev
```

**烧写到 ESP32：**
```bash
# PlatformIO 侧边栏 > Upload
# 或者终端命令：
pio run -e Box-esp32dev -t upload
```

**打开串口监视器：**
```
# PlatformIO 侧边栏 > Serial Monitor (波特率: 115200)
```

### 第 3 步：验证功能

**查看串口输出：**
```
===== DragonBoat Collection Box =====
===== System Initialization Start =====

[Init] SPI Bus OK
[Init] I2C Bus OK
[Init] TFT Display OK
[Init] IMU Sensor OK
[Init] GPS Module OK
[Init] SD Card OK
[Init] LoRa Module OK

===== System Initialization Complete =====
```

**TFT 屏幕显示：**
- 顶部：蓝色标题栏 "DragonBoat Box"
- 中间：各传感器实时数据
- 右上角：红灯（停止录制）

---

## 功能说明

### 1. IMU 惯性测量

**采集内容：**
- 三轴加速度（Ax/Ay/Az）- 用于估计桨频
- 三轴角速度（Gx/Gy/Gz）- 船体旋转速率
- 欧拉角（Roll/Pitch/Yaw）- 船体姿态

**采集周期：** 100 ms

**应用示例：**
```
加速度 Z 轴振幅 → 估计桨频（Hz）
  假设周期为 1 秒内 Az 变化 3 次，则桨频 ≈ 3 Hz

欧拉角 Roll → 龙舟侧倾角度
  用于评估船队协调度
```

### 2. GPS 定位导航

**采集内容：**
- 实时纬度/经度（WGS84 + GCJ02）
- 地理位置
- 定位精度

**定位需求：**
- 开阔水域（河流）
- 卫星数 >= 4 个（通常 60-90 秒获得首次定位）

**应用示例：**
```
GPS 速度（SOG） → 船速测量
  从 GGA 语句第 9 字段提取（节数），转换为 m/s
  
GCJ02 坐标 → 赛道回放
  结合 IMU 数据，生成龙舟运动轨迹地图
```

### 3. TFT 实时显示

**显示内容：**

| 区域 | 显示内容 | 更新频率 |
|------|--------|--------|
| 状态灯 | 绿灯=录制 / 红灯=停止 | 200 ms |
| 加速度 | X/Y/Z 三轴值（m/s²） | 200 ms |
| 角速度 | X/Y/Z 三轴值（°/s） | 200 ms |
| 欧拉角 | Roll/Pitch/Yaw（°） | 200 ms |
| GPS | 纬度/经度 或 "No Fix" | 200 ms |
| 状态 | Recording(ON) / Stopped | 200 ms |
| 文件名 | 当前日志文件 | 200 ms |

### 4. SD 卡数据记录

**记录格式：** CSV（逗号分隔值）

**触发条件：** 仅在以下两个条件同时满足时才记录
1. 收到 LoRa 命令 '1'（开始录制）
2. SD 卡就绪

**CSV 内容（单行示例）：**
```
1234567,9.80,0.23,1.45,5.12,-3.21,2.10,0.45,2.34,15.23,31.234567,120.456789,31.235101,120.457123,N,E
```

**文件管理：**
- 自动命名：1.csv, 2.csv, 3.csv...
- 每次启动自动选择最小未使用编号
- 最多支持 1000 个文件，超出后使用 data.csv

**后期处理：**
```python
# Python 示例：导入 CSV 到 Pandas
import pandas as pd

df = pd.read_csv('1.csv')
print(df[['Timestamp', 'Ax', 'Ay', 'Roll', 'Pitch']])

# 计算船速变化
# df['GPS_Lat'] 和 'GPS_Lon' 用于轨迹回放
```

### 5. LoRa 无线通信

**通信模式：**

```
岸边接收器                          龙舟采集盒子
(LoRa 收发模块)                     (本项目)
       │
     ①│ 发送命令 '1'(开始录制)
       └─────────────────────────────>  processSerialData()
                                          ├─识别命令 '1'
                                          └─触发 onLoRaCommand()
                                               ├─设置 is_recording=true
                                               └─TFT 绿灯亮
       │
     ②│ 接收传感器数据 (500ms 间隔)
       <───────────────────────────────  sendData()
           TS:12345,AX:9.80,...
           LAT:31.2345,LON:120.5678\r\n
       │
     ③│ 发送命令 '2'(停止录制)
       └─────────────────────────────>  processSerialData()
                                          └─设置 is_recording=false
```

**发送数据格式：**
```
关键字:值,关键字:值,...\r\n

TS:     时间戳(ms)
AX/AY:  加速度(m/s²)
ROLL:   滚动角(°)
PITCH:  俯仰角(°)
LAT:    纬度(GCJ02)
LON:    经度(GCJ02)
```

---

## 数据流向

### 数据采集流程

```
┌─────────────┐
│   IMU 传感器 │
│ 加速度/角速度 │
└──────┬──────┘
       │ I2C (400 kHz)
       ▼
┌──────────────────┐
│ imu.readData()   │
│ 缩放转换、时间戳  │
└──────┬───────────┘
       │ IMUData
       ▼
┌────────────────────────────┐
│  main.cpp (loop)           │
│  100ms 采集一次 IMU         │
│ 200ms 刷新一次 TFT         │
│ 100ms 记录一次 SD 卡       │
│ 500ms 发送一次 LoRa        │
└──────┬─────────────────────┘
       │
      ┌┴────────────────────────┬────────────────┬────────────────┐
      │                         │                │                │
      ▼                         ▼                ▼                ▼
┌──────────────┐      ┌───────────────┐  ┌────────────┐  ┌─────────────┐
│ TFT Display  │      │ SD Card CSV   │  │ LoRa send  │  │ 内存存储    │
│ 实时显示数据  │      │ 本地存储原始  │  │ 无线回传    │  │ (上一帧)    │
└──────────────┘      │ 数据用分析    │  │ 数据       │  └─────────────┘
                      └───────────────┘  └────────────┘
```

### GPS 数据流程

```
GPS 模块（UART1）
    │
    ├─ $GPGGA 语句（每秒多次）
    │
    ▼
gps.processSerialData()
    │
    ├─ 识别 GGA 语句
    ├─ 解析纬度/经度（DDM 格式）
    ├─ WGS84 → GCJ02 坐标转换
    ├─ 设置 is_fixed = true
    │
    ▼
GPSData 结构体
    │
    ├─ wgs84_lat, wgs84_lon
    ├─ gcj02_lat, gcj02_lon
    └─ is_fixed, timestamp

（loop 中 200ms 调用 gps.getData()）
    │
    ▼
├─ TFT Display（显示 GCJ02 坐标）
├─ SD Card（记录双坐标系）
└─ LoRa Send（发送 GCJ02 坐标）
```

---

## 团队协作指南

### 代码提交规范

**分支管理：**
```
main              # 主分支（稳定版本，Release）
  ├─ develop     # 开发分支（集成新功能）
  └─ feature/*   # 功能分支（单个功能开发）
```

**提交信息格式：**
```
[模块名] 简短描述

更详细的说明（可选）

示例：
[IMU] 修复寄存器读取超时问题
[GPS] 添加 BDGGA 语句支持
[API] 更新 TFT 显示方法注释
```

### 文件修改清单

**修改 `include/config.h` 时：**
- 同时更新 API 文档中的"引脚配置表"
- 同时更新本文档中的"硬件配置"章节

**修改 `src/main.cpp` 时：**
- 保持初始化顺序（TFT > IMU > GPS > SD > LoRa）
- 保持时序周期（100/200/100/500 ms）
- 添加详细的代码注释

**修改类实现时（.cpp 文件）：**
- 保持头文件中的方法签名一致
- 遵循 Doxygen 注释规范
- 测试所有公开方法

**新增传感器时：**
1. 在 `include/modules/sensors.h` 中添加数据结构
2. 创建 `include/modules/xxxx_module.h` 和 `src/modules/xxxx_module.cpp`
3. 在 `src/main.cpp` 中创建实例和集成逻辑
4. 更新 `docs/API_DOCUMENT.md`

### 代码审查重点

- ✅ 是否遵循模块化设计（不跨模块访问私有数据）
- ✅ 是否使用常量而非硬编码数值（所有配置应在 config.h）
- ✅ 是否有充分的错误检查（begin() 返回值检查）
- ✅ 是否有详细的注释和 API 文档
- ✅ 是否会导致主循环阻塞（避免 delay(>10ms)）

### 测试清单

**集成测试：**
- [ ] TFT 能正常显示初始化信息
- [ ] IMU 能读取数据并在 TFT 上实时显示
- [ ] GPS 定位成功后纬度/经度显示正确
- [ ] SD 卡记录的 CSV 文件格式正确
- [ ] LoRa 能接收命令 '1'（绿灯亮，开始录制）
- [ ] LoRa 能接收命令 '2'（红灯亮，停止录制）
- [ ] LoRa 能定期发送传感器数据

---

## 故障排查

### 问题 1：TFT 屏幕不显示或显示花屏

**可能原因：**
- SPI 总线初始化顺序错误
- CS 信号触发时序不对
- 显示屏初始化代码 bug

**排查方法：**
```cpp
// 在 setup() 中逐步测试
Serial.println("Step 1: SPI.begin()");
SPI.begin();  // 检查是否有错误

Serial.println("Step 2: display.begin()");
if (!display.begin()) {
  Serial.println("ERROR: TFT init failed!");
  // 检查硬件连接和引脚定义
}
```

### 问题 2：IMU 数据不变或全为 0

**可能原因：**
- I2C 地址错误（JY901 默认 0x50）
- I2C 引脚连接松动
- IMU 未加电或损坏

**排查方法：**
```cpp
// 添加 I2C 扫描代码
Wire.begin(IIC_SDA, IIC_SCL);
Wire.setClock(400000);

for (byte addr = 1; addr < 127; addr++) {
  Wire.beginTransmission(addr);
  if (Wire.endTransmission() == 0) {
    Serial.print("I2C device found at 0x");
    Serial.println(addr, HEX);
  }
}
```

### 问题 3：GPS 始终无法定位

**可能原因：**
- GPS 模块无卫星信号（室内环境）
- GPS 波特率设置错误
- GPS 模块损坏或 RX/TX 接反

**排查方法：**
```cpp
// 串口透传测试 GPS 原始数据
void gps_passthrough() {
  while (GPS_Serial.available()) {
    Serial.write(GPS_Serial.read());
  }
  while (Serial.available()) {
    GPS_Serial.write(Serial.read());
  }
}

// 在 loop() 中调用，观察原始 NMEA 句子
// 应该能看到 $GPGGA,hhmmss,...
```

### 问题 4：SD 卡无法初始化

**可能原因：**
- SD 卡损坏或 FAT32 格式错误
- CS 引脚与 TFT 冲突
- SPI 时钟过快（试试降速）

**排查方法：**
```cpp
// 尝试降低 SPI 时钟速率
if (SD.begin(SD_CS, SPI, 1000000)) {  // 从 4MHz 降到 1MHz
  Serial.println("SD card initialized at lower speed");
}

// Windows 中格式化 SD 卡为 FAT32
// 使用 SD Card Formatter 工具
```

### 问题 5：LoRa 无法接收命令

**可能原因：**
- LoRa 模块与主机之间没有通信
- UART2 引脚定义错误
- 波特率不匹配（9600 bps）

**排查方法：**
```cpp
// 直接读取 UART2 数据
void loop() {
  if (LoRaSerial.available()) {
    char c = LoRaSerial.read();
    Serial.print("LoRa received: ");
    Serial.write(c);  // 应该显示接收到的字符
  }
}
```

---

## 扩展开发

### 添加新的传感器模块

**步骤 1：定义数据结构**
```cpp
// include/modules/sensors.h 中添加：
struct NewSensorData {
  float value1;
  float value2;
  unsigned long timestamp;
};
```

**步骤 2：创建模块类**
```cpp
// include/modules/new_sensor.h
class NewSensorModule {
public:
  bool begin();
  bool readData(NewSensorData &data);
private:
  NewSensorData current_data;
};
```

**步骤 3：实现模块**
```cpp
// src/modules/new_sensor.cpp
// ... 实现各个方法
```

**步骤 4：集成到主程序**
```cpp
// src/main.cpp 中：
NewSensorModule new_sensor;

void setup() {
  if (!new_sensor.begin()) {
    Serial.println("New sensor init failed!");
  }
}

void loop() {
  if (now - lastNewSensorRead >= NEW_SENSOR_INTERVAL) {
    new_sensor.readData(new_data);
    lastNewSensorRead = now;
  }
}
```

### 修改数据记录格式

**当前 CSV 格式：**
```
Timestamp,Ax,Ay,Az,Gx,Gy,Gz,Roll,Pitch,Yaw,WGS84_Lat,...
```

**扩展示例（添加新传感器）：**
```
Timestamp,Ax,Ay,Az,Gx,Gy,Gz,Roll,Pitch,Yaw,WGS84_Lat,...,NewSensor_Val1,NewSensor_Val2
```

**修改文件：** `src/modules/sd_storage.cpp` 中的 `formatCSVLine()` 方法

### 优化性能

**当前瓶颈：**
- SPI 共享（TFT 和 SD 存在竞争）
- I2C 速率（可提升到 1 MHz）

**优化方案：**
1. **增加硬件 SPI 通道** - 使用 HSPI 或 FSPI
2. **提升 I2C 时钟** - Wire.setClock(1000000)
3. **数据缓冲** - 采集多帧后批量写入 SD（提升写入速度）
4. **并行采样** - 使用 FreeRTOS 多任务（advanced）

---

## 常见问题

**Q: 能否将龙舟盒子装在船以外的地方？**  
A: 可以。只需确保信号到达：GPS 需要天空开阔（河面上方），LoRa 距离取决于天线和环境（通常 1-10 km）。

**Q: 数据丢失怎么办？**  
A: 检查 SD 卡是否已满。若容量不够，可每 20 分钟启动一个新文件（通过 LoRa 命令序列）。

**Q: 可以用 Wi-Fi 代替 LoRa 吗？**  
A: 可以，但需要：(1) 路由器覆盖、(2) 低延迟网络、(3) 修改 lora_comm.h 使用 WiFi 库。

**Q: 如何在 Python 中分析 CSV 数据？**  
A: 见《扩展开发》章节的 Python 示例代码。

---

## 文档历史

| 版本 | 日期 | 作者 | 变更内容 |
|------|------|------|--------|
| 1.0 | 2026-03-08 | Team | 初始版本 |

---

**项目主页:** [TODO: 添加 GitHub 链接]  
**问题报告:** [TODO: 添加 Issues 链接]  
**联系方式:** [TODO: 添加联系方式]

