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

// --- TF Card (共用VSPI) ---
#define SD_CS     15

// --- GPS (UART1) ---
#define GPS_RX_PIN 32
#define GPS_TX_PIN 33

// --- LoRa (UART2) ---
#define LORA_RX_PIN 16
#define LORA_TX_PIN 17

// ====== 通信波特率 ======
#define BAUD_RATE 115200
#define GPS_BAUD_RATE 9600
#define LORA_BAUD_RATE 9600

// ====== 采集间隔参数 ======
#define IMU_READ_INTERVAL 20      // IMU采集间隔(ms) → 50Hz
#define TFT_REFRESH_INTERVAL 33   // TFT刷新间隔(ms) ~30fps
#define SD_LOG_INTERVAL 20        // TF卡写入间隔(ms) → 50Hz (与IMU同步)
#define SD_FLUSH_INTERVAL 1000    // SD卡flush间隔(ms)
#define LORA_SEND_INTERVAL 500    // LoRa发送间隔(ms)

// ====== SPI速度 ======
#define TFT_SPI_FREQ  27000000
#define SD_SPI_FREQ   25000000

// ====== 屏幕尺寸 ======
#define TFT_WIDTH  160
#define TFT_HEIGHT 128

// ====== FreeRTOS任务配置 ======
#define DISPLAY_TASK_STACK  8192
#define DISPLAY_TASK_PRIO   1
#define DISPLAY_TASK_CORE   0      // 显示任务运行在Core 0

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

// ====== IMU缩放常数 ======
#define IMU_ACC_SCALE  (16.0f * 9.8f / 32768.0f)
#define IMU_GYRO_SCALE (2000.0f / 32768.0f)
#define IMU_ANGLE_SCALE (180.0f / 32768.0f)
#define GPS_FIXED_SCALE 10000000

// ====== GCJ02 坐标转换常数 ======
#define EARTH_RADIUS_A 6378245.0
#define EARTH_ECCENTRICITY 0.00669342162296594323

#endif
