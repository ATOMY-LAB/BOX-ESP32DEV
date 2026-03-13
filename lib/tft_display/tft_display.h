#ifndef TFT_DISPLAY_H
#define TFT_DISPLAY_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "sensors.h"
#include "config.h"

// ====== 新版UI布局 (160×128) ======
//  Y= 0-12:  标题栏 (13px)
//  Y=13:     分割线
//  Y=14-21:  列标签 "X   Y   Z" (8px)
//  Y=22-31:  ACC行 (10px)
//  Y=32-41:  GYR行 (10px)
//  Y=42-51:  ANG行 = R P Y顺序 (10px)
//  Y=52:     分割线
//  Y=53-91:  XYZ加速度曲线图 (39px, ≈10s @15fps采样)
//  Y=92:     分割线
//  Y=93-103: GPS行1 (11px)
//  Y=104-114:GPS行2 (11px)
//  Y=115:    分割线
//  Y=116-127:状态栏 (12px)

// 曲线图采样点数 = 图表像素宽度 (153)，每2帧采样一次 ≈ 15Hz → ~10s数据
static const uint16_t CHART_SAMPLES = 153;

class TFTDisplay {
public:
  TFTDisplay();
  bool begin();

  // 渲染一帧到canvas (纯内存操作，不涉及SPI)
  void renderFrame(const IMUData &imu_data,
                   const GPSData &gps_data,
                   const SystemState &sys_state,
                   const String &filename,
                   uint32_t record_count);

  // 将canvas推送到屏幕 (SPI传输，需要SPI互斥锁保护)
  void pushFrame();

  // 启动/错误画面 (直接绘制到屏幕，仅在初始化阶段使用)
  void showStartupScreen();
  void showError(const char *error_msg);

private:
  Adafruit_ST7735 tft;
  GFXcanvas16 *canvas;           // 160x128 离屏缓冲区 (~40KB)

  // 曲线图环形缓冲区 (静态成员，避免栈溢出)
  static float _chart_ax[CHART_SAMPLES];
  static float _chart_ay[CHART_SAMPLES];
  static float _chart_az[CHART_SAMPLES];
  static uint16_t _chart_head;   // 下一个写入位置
  static uint16_t _chart_count;  // 已存入的有效样本数

  uint8_t _frame_cnt;            // 帧计数器，用于降频采样

  // 内部: 向环形缓冲区追加一组加速度样本
  void addChartSample(float ax, float ay, float az);

  // UI绘制子函数 (均绘制到canvas)
  void drawTitleBar(bool is_recording);
  void drawColHeaders();
  void drawDataSection(const IMUData &imu_data);
  void drawChart();
  void drawGPSSection(const GPSData &gps_data);
  void drawStatusBar(const SystemState &sys_state, const String &filename, uint32_t record_count);
  void drawDivider(int16_t y, uint16_t color);
};

#endif
