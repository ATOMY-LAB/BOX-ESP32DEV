#include "tft_display.h"

// 颜色定义
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define ORANGE  0xFBE0

TFTDisplay::TFTDisplay() : 
  tft(TFT_CS, TFT_DC, TFT_RST) {}

bool TFTDisplay::begin() {
  busTake();
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);  // 横屏 160x128
  tft.fillScreen(BLACK);
  busRelease();
  return true;
}

void TFTDisplay::drawStaticUI() {
  busTake();
  tft.fillScreen(BLACK);
  
  // ✅ 新排版：紧凑布局，每行8像素，充分利用160x128屏幕
  
  // 顶部标题栏（0-15）
  tft.fillRect(0, 0, 160, 16, BLUE);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  int titleX = (160 - 8 * 11) / 2;
  tft.setCursor(titleX, 4);
  tft.print("DragonBoat Box");

  // 数据区域静态标签（无需顶部标签，直接显示数据）
  // 排版规划：
  // 16-23: Acc
  // 24-31: Gyro
  // 32-39: Angle
  // 40-47: GPS
  // 48-55: State
  // 56-63: File
  // 64-127: 空白区域
  
  // 不再显示静态标签，数据直接显示，节省空间

  busRelease();
}

void TFTDisplay::updateDisplay(const IMUData &imu_data, 
                               const GPSData &gps_data,
                               const SystemState &sys_state,
                               const String &filename) {
  busTake();
  
  // 状态指示灯
  drawStatusLED(sys_state.is_recording);
  
  // 更新各数据区域
  updateAcceleration(imu_data.ax, imu_data.ay, imu_data.az);
  updateGyroscope(imu_data.gx, imu_data.gy, imu_data.gz);
  updateAttitude(imu_data.roll, imu_data.pitch, imu_data.yaw);
  updateGPS(gps_data);
  updateRecordingState(sys_state.is_recording);
  updateFileName(filename);
  
  busRelease();
}

void TFTDisplay::showError(const String &error_msg) {
  busTake();
  tft.fillRect(0, 110, 160, 18, RED);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.setCursor(5, 112);
  tft.println(error_msg);
  busRelease();
}

void TFTDisplay::showStartupMessage(const String &msg) {
  busTake();
  tft.fillRect(0, 60, 160, 20, BLACK);
  tft.setTextColor(CYAN);
  tft.setCursor(10, 70);
  tft.println(msg);
  busRelease();
}

void TFTDisplay::busTake() {
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, LOW);
}

void TFTDisplay::busRelease() {
  digitalWrite(TFT_CS, HIGH);
}

void TFTDisplay::drawStatusLED(bool is_recording) {
  tft.fillCircle(145, 9, 5, BLACK);
  tft.fillCircle(145, 9, 5, is_recording ? GREEN : RED);
}

void TFTDisplay::updateAcceleration(float ax, float ay, float az) {
  static char buf[50];
  // ✅ 加速度：第1行显示X、Y，第2行显示Z
  tft.fillRect(0, 16, 160, 12, BLACK);
  tft.setTextColor(RED);
  tft.setTextSize(1);
  tft.setCursor(2, 18);
  snprintf(buf, sizeof(buf), "Acc: X:%6.1f Y:%6.1f", ax, ay);
  tft.print(buf);
  
  tft.fillRect(0, 28, 160, 12, BLACK);
  tft.setCursor(2, 30);
  snprintf(buf, sizeof(buf), "     Z:%6.1f", az);
  tft.print(buf);
}

void TFTDisplay::updateGyroscope(float gx, float gy, float gz) {
  static char buf[50];
  // ✅ 角速度：第1行显示X、Y，第2行显示Z
  tft.fillRect(0, 40, 160, 12, BLACK);
  tft.setTextColor(GREEN);
  tft.setTextSize(1);
  tft.setCursor(2, 42);
  snprintf(buf, sizeof(buf), "Gyro: X:%6.1f Y:%6.1f", gx, gy);
  tft.print(buf);
  
  tft.fillRect(0, 52, 160, 12, BLACK);
  tft.setCursor(2, 54);
  snprintf(buf, sizeof(buf), "      Z:%6.1f", gz);
  tft.print(buf);
}

void TFTDisplay::updateAttitude(float roll, float pitch, float yaw) {
  static char buf[50];
  // ✅ 欧拉角：第1行显示R、P，第2行显示Y
  tft.fillRect(0, 64, 160, 12, BLACK);
  tft.setTextColor(YELLOW);
  tft.setTextSize(1);
  tft.setCursor(2, 66);
  snprintf(buf, sizeof(buf), "Att: R:%6.1f P:%6.1f", roll, pitch);
  tft.print(buf);
  
  tft.fillRect(0, 76, 160, 12, BLACK);
  tft.setCursor(2, 78);
  snprintf(buf, sizeof(buf), "     Y:%6.1f", yaw);
  tft.print(buf);
}

void TFTDisplay::updateGPS(const GPSData &gps_data) {
  static char buf[60];
  // ✅ GPS：第1行显示纬度，第2行显示经度
  tft.fillRect(0, 88, 160, 12, BLACK);
  tft.setTextColor(ORANGE);
  tft.setTextSize(1);
  tft.setCursor(2, 90);
  if (gps_data.is_fixed) {
    snprintf(buf, sizeof(buf), "GPS Lat:%8.3f %c", gps_data.gcj02_lat, gps_data.lat_dir[0]);
    tft.print(buf);
  } else {
    tft.print("GPS: No Fix");
  }
  
  tft.fillRect(0, 100, 160, 12, BLACK);
  tft.setCursor(2, 102);
  if (gps_data.is_fixed) {
    snprintf(buf, sizeof(buf), "    Lon:%8.3f %c", gps_data.gcj02_lon, gps_data.lon_dir[0]);
    tft.print(buf);
  } else {
    tft.print("    --:-- --");
  }
}

void TFTDisplay::updateRecordingState(bool is_recording) {
  static char buf[100];
  // ✅ 清除整行（0-159）
  tft.fillRect(0, 112, 160, 12, BLACK);
  
  // ✅ 状态部分（前半部分，青色）
  tft.setTextColor(CYAN);
  tft.setTextSize(1);
  tft.setCursor(2, 114);
  if (is_recording) {
    tft.print("Status: [REC]");
  } else {
    tft.print("Status: STOP");
  }
}

void TFTDisplay::updateFileName(const String &filename) {
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.setCursor(80, 114);  // ✅ 往左挪，距离状态部分更近
  tft.print("File: ");
  tft.print(filename.substring(1));
}
