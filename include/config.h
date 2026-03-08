#ifndef CONFIG_H
#define CONFIG_H

// ========================================
// 龙舟桨频/船速采集盒子 - 全局配置
// ========================================

// ====== 硬件引脚定义 ======
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

// --- LoRa (UART2) ---
#define LORA_RX_PIN 16  // ESP32 UART2_RX <-- LoRa_TX
#define LORA_TX_PIN 17  // ESP32 UART2_TX --> LoRa_RX

// ====== 通信波特率 ======
#define BAUD_RATE 115200
#define GPS_BAUD_RATE 9600        // GPS默认波特率
#define LORA_BAUD_RATE 9600       // LoRa默认波特率

// ====== 采集间隔参数 ======
#define IMU_READ_INTERVAL 100     // IMU采集间隔(ms)
#define TFT_REFRESH_INTERVAL 200  // TFT显示刷新间隔(ms)
#define SD_LOG_INTERVAL 100       // TF卡写入间隔(ms)（与IMU采集同步）
#define LORA_SEND_INTERVAL 500    // LoRa数据发送间隔(ms)

// ====== IMU (JY901) 寄存器地址 ======
#define AX_ADDR 0x34
#define AY_ADDR 0x35
#define AZ_ADDR 0x36
#define GX_ADDR 0x37
#define GY_ADDR 0x38
#define GZ_ADDR 0x39
#define ROLL_ADDR 0x3D
#define PITCH_ADDR 0x3E
#define YAW_ADDR 0x3F

// ====== GCJ02 坐标转换常数 ======
#define EARTH_RADIUS_A 6378245.0
#define EARTH_ECCENTRICITY 0.00669342162296594323

#endif  // CONFIG_H
