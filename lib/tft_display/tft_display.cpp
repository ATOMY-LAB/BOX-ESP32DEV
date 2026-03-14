#include "tft_display.h"

// ====== 配色方案 (暗色主题) ======
#define C_BG          0x0841   // 深灰背景
#define C_TITLE_BG    0x1926   // 深蓝标题栏
#define C_TITLE_TEXT  0xFFFF   // 白色标题
#define C_DIVIDER     0x2965   // 暗灰分割线
#define C_LABEL       0x8C51   // 中灰标签
#define C_ACC         0xFB6D   // 珊瑚红 - 加速度
#define C_GYRO        0x67EA   // 浅绿 - 角速度
#define C_ANGLE       0xFDE0   // 金色 - 欧拉角
#define C_CHART_AX    0xF800   // 红 - 曲线图ax
#define C_CHART_AY    0x07E0   // 绿 - 曲线图ay
#define C_CHART_AZ    0x1C9F   // 蓝 - 曲线图az
#define C_CHART_GRID  0x18C3   // 深灰网格线
#define C_GPS_FIX     0x5D5F   // 天蓝 - GPS定位
#define C_GPS_NOFIX   0x8410   // 灰色
#define C_REC_ON      0x07E0   // 亮绿
#define C_REC_OFF     0x8410   // 灰色
#define C_NO_SD       0xFD20   // 橙色 (255,168,0) — 无SD卡状态
#define C_STATUS_BG   0x10A2   // 深灰底色
#define C_WHITE       0xFFFF
#define C_RED         0xF800
#define C_BLACK       0x0000

// ====== 布局常数 ======
#define TITLE_H       13      // 标题栏高度
#define HDR_Y         14      // 列标签行Y
#define DATA_Y        22      // 数据区起始Y
#define ROW_H         10      // 每行数据高度
#define DATA_SEP_Y    52      // 数据区底部分割线Y
#define CHART_Y       53      // 曲线图起始Y
#define CHART_H       39      // 曲线图高度
#define GPS_SEP_Y     92      // 曲线图底部分割线Y
#define GPS_Y         93      // GPS区域起始Y
#define GPS_ROW_H     11      // GPS行高
#define STATUS_SEP_Y  115     // 状态栏分割线Y
#define STATUS_Y      116     // 状态栏起始Y

// 数据列X坐标
#define LABEL_X       2       // 行标签左边距
#define VAL_X         26      // "%6.1f %6.1f %6.1f" 起始X

// 列标签 "X" / "Y" / "Z" 的像素X坐标 (居中对齐到 "%6.1f" 各字段)
// 每字段6字符×6px=36px；字段1:[26,61] 字段2:[68,103] 字段3:[110,145]
#define COL1_LABEL_X  40      // "X" 字符起始X (居中于[26,61])
#define COL2_LABEL_X  82      // "Y"
#define COL3_LABEL_X  124     // "Z"

// 曲线图内部绘制区 (留3px上边距给标签，4px下边距)
#define CHART_DATA_X  3                    // 数据起始X
#define CHART_DATA_W  153                  // 数据宽度 (= CHART_SAMPLES)
#define CHART_DATA_Y  (CHART_Y + 7)        // 数据起始Y (留7px上边距)
#define CHART_DATA_H  28                   // 数据高度
// 0线 Y像素: 中间
#define CHART_ZERO_Y  (CHART_DATA_Y + CHART_DATA_H / 2)

// ====== 静态成员定义 ======
float    TFTDisplay::_chart_ax[CHART_SAMPLES];
float    TFTDisplay::_chart_ay[CHART_SAMPLES];
float    TFTDisplay::_chart_az[CHART_SAMPLES];
uint16_t TFTDisplay::_chart_head  = 0;
uint16_t TFTDisplay::_chart_count = 0;

// ====== 构造 ======
TFTDisplay::TFTDisplay()
  : tft(TFT_CS, TFT_DC, TFT_RST), canvas(nullptr), _frame_cnt(0) {}

bool TFTDisplay::begin() {
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.setSPISpeed(TFT_SPI_FREQ);
  tft.fillScreen(C_BLACK);

  canvas = new GFXcanvas16(TFT_WIDTH, TFT_HEIGHT);
  if (!canvas) {
    Serial.println("[TFT] Canvas alloc failed!");
    return false;
  }
  canvas->fillScreen(C_BG);
  return true;
}

void TFTDisplay::showStartupScreen() {
  tft.fillScreen(C_TITLE_BG);
  tft.setTextColor(C_WHITE);
  tft.setTextSize(2);
  tft.setCursor(16, 30);
  tft.print("DragonBoat");
  tft.setCursor(52, 55);
  tft.print("Box");
  tft.setTextSize(1);
  tft.setTextColor(C_LABEL);
  tft.setCursor(28, 90);
  tft.print("Initializing...");
}

void TFTDisplay::showError(const char *error_msg) {
  tft.fillRect(0, 110, 160, 18, C_RED);
  tft.setTextColor(C_WHITE);
  tft.setTextSize(1);
  tft.setCursor(5, 114);
  tft.print(error_msg);
}

// ====== 向曲线图环形缓冲区追加样本 ======
void TFTDisplay::addChartSample(float ax, float ay, float az) {
  _chart_ax[_chart_head] = ax;
  _chart_ay[_chart_head] = ay;
  _chart_az[_chart_head] = az;
  _chart_head = (_chart_head + 1) % CHART_SAMPLES;
  if (_chart_count < CHART_SAMPLES) _chart_count++;
}

// ====== 渲染一帧到Canvas (纯内存操作,无SPI) ======
void TFTDisplay::renderFrame(const IMUData &imu_data,
                              const GPSData &gps_data,
                              const SystemState &sys_state,
                              const String &filename,
                              uint32_t record_count) {
  if (!canvas) return;

  // 每两帧采样一次 → ~15Hz → 153 samples ≈ 10.2s
  _frame_cnt++;
  if (_frame_cnt & 1) {
    addChartSample(imu_data.ax, imu_data.ay, imu_data.az);
  }

  // 清屏
  canvas->fillScreen(C_BG);

  // 标题栏
  drawTitleBar(sys_state.is_recording);
  drawDivider(TITLE_H, C_DIVIDER);

  // 列标签 + IMU数据区
  drawColHeaders();
  drawDataSection(imu_data);
  drawDivider(DATA_SEP_Y, C_DIVIDER);

  // 曲线图
  drawChart();
  drawDivider(GPS_SEP_Y, C_DIVIDER);

  // GPS区域
  drawGPSSection(gps_data);
  drawDivider(STATUS_SEP_Y, C_DIVIDER);

  // 状态栏
  drawStatusBar(sys_state, filename, record_count);
}

// ====== 将Canvas推送到屏幕 (SPI传输) ======
void TFTDisplay::pushFrame() {
  if (!canvas) return;
  tft.startWrite();
  tft.setAddrWindow(0, 0, TFT_WIDTH, TFT_HEIGHT);
  tft.writePixels(canvas->getBuffer(), TFT_WIDTH * TFT_HEIGHT);
  tft.endWrite();
}

// ====== 标题栏 ======
void TFTDisplay::drawTitleBar(bool is_recording) {
  canvas->fillRect(0, 0, TFT_WIDTH, TITLE_H, C_TITLE_BG);
  canvas->setTextColor(C_TITLE_TEXT);
  canvas->setTextSize(1);
  canvas->setCursor(4, 3);
  canvas->print("DragonBoat Box");

  uint16_t led_color = is_recording ? C_REC_ON : C_REC_OFF;
  canvas->fillCircle(151, 6, 5, led_color);
  canvas->fillCircle(149, 4, 1, C_WHITE);  // 高亮点
}

// ====== 列标签行 (X / Y / Z，与"%6.1f"各字段居中对齐) ======
void TFTDisplay::drawColHeaders() {
  canvas->setTextColor(C_LABEL);
  canvas->setTextSize(1);

  // 行左侧留3字符空出 "ACC"/"GYR"/"ANG" 标签位置
  // X 列
  canvas->setCursor(COL1_LABEL_X, HDR_Y);
  canvas->print("X");
  // Y 列
  canvas->setCursor(COL2_LABEL_X, HDR_Y);
  canvas->print("Y");
  // Z 列 (ANG行对应 Yaw，仍显示Z以保持一致，ANG标签隐含RPY)
  canvas->setCursor(COL3_LABEL_X, HDR_Y);
  canvas->print("Z");
}

// ====== IMU数据区 (ACC / GYR / ANG 三行) ======
void TFTDisplay::drawDataSection(const IMUData &imu_data) {
  static char buf[24];
  canvas->setTextSize(1);

  // === ACC行 ===
  int y = DATA_Y + 1;
  canvas->setTextColor(C_LABEL);
  canvas->setCursor(LABEL_X, y);
  canvas->print("ACC");
  canvas->setTextColor(C_ACC);
  snprintf(buf, sizeof(buf), "%6.1f %6.1f %6.1f",
           imu_data.ax, imu_data.ay, imu_data.az);
  canvas->setCursor(VAL_X, y);
  canvas->print(buf);

  // === GYR行 ===
  y += ROW_H;
  canvas->setTextColor(C_LABEL);
  canvas->setCursor(LABEL_X, y);
  canvas->print("GYR");
  canvas->setTextColor(C_GYRO);
  snprintf(buf, sizeof(buf), "%6.1f %6.1f %6.1f",
           imu_data.gx, imu_data.gy, imu_data.gz);
  canvas->setCursor(VAL_X, y);
  canvas->print(buf);

  // === ANG行 (Roll/Pitch/Yaw → 对应 X/Y/Z列) ===
  y += ROW_H;
  canvas->setTextColor(C_LABEL);
  canvas->setCursor(LABEL_X, y);
  canvas->print("RPY");
  canvas->setTextColor(C_ANGLE);
  snprintf(buf, sizeof(buf), "%6.1f %6.1f %6.1f",
           imu_data.roll, imu_data.pitch, imu_data.yaw);
  canvas->setCursor(VAL_X, y);
  canvas->print(buf);
}

// ====== XYZ加速度曲线图 ======
void TFTDisplay::drawChart() {
  // 图表背景
  canvas->fillRect(0, CHART_Y, TFT_WIDTH, CHART_H, C_BG);

  // === 顶部标签行 (y=CHART_Y+1, 利用7px上边距) ===
  canvas->setTextSize(1);
  // 量程标注
  canvas->setTextColor(C_LABEL);
  canvas->setCursor(3, CHART_Y + 1);
  canvas->print("\xb1""20");   // ±20 m/s²

  // 颜色图例 (与数值颜色对应)
  canvas->setTextColor(C_CHART_AX);
  canvas->setCursor(60, CHART_Y + 1);
  canvas->print("X");
  canvas->setTextColor(C_CHART_AY);
  canvas->setCursor(72, CHART_Y + 1);
  canvas->print("Y");
  canvas->setTextColor(C_CHART_AZ);
  canvas->setCursor(84, CHART_Y + 1);
  canvas->print("Z");
  canvas->setTextColor(C_LABEL);
  canvas->setCursor(96, CHART_Y + 1);
  canvas->print("accel");

  // === 零线 (水平虚线，位于数据区中间) ===
  for (int16_t x = CHART_DATA_X; x < CHART_DATA_X + CHART_DATA_W; x += 4) {
    canvas->drawPixel(x, CHART_ZERO_Y, C_CHART_GRID);
  }

  // === 绘制曲线数据 ===
  if (_chart_count == 0) return;

  // 计算最旧样本在环形缓冲区中的索引
  uint16_t oldest = (_chart_head + CHART_SAMPLES - _chart_count) % CHART_SAMPLES;

  // 值 → 像素Y（量程 ±20 m/s²，顶=+20，底=-20）
  auto valToY = [](float v) -> int16_t {
    float norm = (20.0f - v) / 40.0f;
    int16_t py = CHART_DATA_Y + (int16_t)(norm * (CHART_DATA_H - 1));
    if (py < CHART_DATA_Y) py = CHART_DATA_Y;
    if (py >= CHART_DATA_Y + CHART_DATA_H) py = CHART_DATA_Y + CHART_DATA_H - 1;
    return py;
  };

  uint16_t pts = (_chart_count < (uint16_t)CHART_DATA_W) ? _chart_count : (uint16_t)CHART_DATA_W;
  for (uint16_t x = 0; x < pts; x++) {
    uint16_t idx = (oldest + x) % CHART_SAMPLES;
    int16_t px = (int16_t)(CHART_DATA_X + x);
    canvas->drawPixel(px, valToY(_chart_ax[idx]), C_CHART_AX);
    canvas->drawPixel(px, valToY(_chart_ay[idx]), C_CHART_AY);
    canvas->drawPixel(px, valToY(_chart_az[idx]), C_CHART_AZ);
  }
}

// ====== GPS区域 ======
void TFTDisplay::drawGPSSection(const GPSData &gps_data) {
  static char buf[24];
  canvas->setTextSize(1);

  // "GPS" 标签
  canvas->setTextColor(C_LABEL);
  canvas->setCursor(LABEL_X, GPS_Y + 2);
  canvas->print("GPS");

  if (gps_data.is_fixed) {
    canvas->setTextColor(C_GPS_FIX);
    // 纬度
    snprintf(buf, sizeof(buf), "%9.5f%c", gps_data.gcj02_lat, gps_data.lat_dir);
    canvas->setCursor(VAL_X, GPS_Y + 2);
    canvas->print(buf);
    // 经度
    snprintf(buf, sizeof(buf), "%9.5f%c", gps_data.gcj02_lon, gps_data.lon_dir);
    canvas->setCursor(VAL_X, GPS_Y + GPS_ROW_H + 2);
    canvas->print(buf);
    // 定位有效小圆点
    canvas->fillCircle(LABEL_X + 12, GPS_Y + GPS_ROW_H + 6, 2, C_REC_ON);
  } else {
    canvas->setTextColor(C_GPS_NOFIX);
    canvas->setCursor(VAL_X, GPS_Y + 2);
    canvas->print("Searching...");
    // 动态点动画
    int dots = (millis() / 500) % 4;
    canvas->setCursor(VAL_X, GPS_Y + GPS_ROW_H + 2);
    for (int i = 0; i < dots; i++) canvas->print(".");
  }
}

// ====== 底部状态栏 ======
void TFTDisplay::drawStatusBar(const SystemState &sys_state,
                                const String &filename,
                                uint32_t record_count) {
  static char buf[24];

  canvas->fillRect(0, STATUS_Y, TFT_WIDTH, TFT_HEIGHT - STATUS_Y, C_STATUS_BG);
  canvas->setTextSize(1);

  // 录制状态圆点 + 状态文字 (三态: 录制中 / 停止 / 无卡)
  if (sys_state.is_recording) {
    bool blink = (millis() / 500) % 2;
    if (blink) canvas->fillCircle(8, STATUS_Y + 6, 3, C_RED);
    canvas->setTextColor(C_REC_ON);
    canvas->setCursor(14, STATUS_Y + 2);
    canvas->print("REC");
  } else if (!sys_state.is_card_ready) {
    canvas->fillCircle(8, STATUS_Y + 6, 3, C_NO_SD);
    canvas->setTextColor(C_NO_SD);
    canvas->setCursor(14, STATUS_Y + 2);
    canvas->print("NO SD");
  } else {
    canvas->fillCircle(8, STATUS_Y + 6, 3, C_REC_OFF);
    canvas->setTextColor(C_REC_OFF);
    canvas->setCursor(14, STATUS_Y + 2);
    canvas->print("STP");
  }

  // 文件名
  canvas->setTextColor(C_WHITE);
  canvas->setCursor(46, STATUS_Y + 2);
  if (filename.length() > 1) {
    canvas->print(filename.substring(1));  // 去掉前导/
  }

  // 记录数
  if (record_count > 0) {
    snprintf(buf, sizeof(buf), "#%lu", (unsigned long)record_count);
    canvas->setTextColor(C_LABEL);
    canvas->setCursor(110, STATUS_Y + 2);
    canvas->print(buf);
  }
}

// ====== 分割线 ======
void TFTDisplay::drawDivider(int16_t y, uint16_t color) {
  canvas->drawFastHLine(2, y, TFT_WIDTH - 4, color);
}
