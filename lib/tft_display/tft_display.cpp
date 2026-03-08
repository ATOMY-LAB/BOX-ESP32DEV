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
  
  // 顶部标题栏
  tft.fillRect(0, 0, 160, 18, BLUE);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  int titleX = (160 - 8 * 11) / 2;
  tft.setCursor(titleX, 4);
  tft.print("DragonBoat Box");

  // 静态标签
  tft.setTextColor(WHITE);
  tft.setCursor(5, 25); tft.print("Acc:");
  tft.setCursor(5, 40); tft.print("Gyro:");
  tft.setCursor(5, 55); tft.print("Angle:");
  tft.setCursor(5, 70); tft.print("GPS:");
  tft.setCursor(5, 85); tft.print("State:");
  tft.setCursor(5, 100); tft.print("File:");

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
  tft.fillRect(30, 22, 125, 10, BLACK);
  tft.setTextColor(RED);
  tft.setTextSize(1);
  tft.setCursor(30, 27);
  tft.print("X:"); tft.print(ax, 1);
  tft.print(" Y:"); tft.print(ay, 1);
  tft.print(" Z:"); tft.print(az, 1);
}

void TFTDisplay::updateGyroscope(float gx, float gy, float gz) {
  tft.fillRect(30, 37, 125, 10, BLACK);
  tft.setTextColor(GREEN);
  tft.setCursor(30, 42);
  tft.print("X:"); tft.print(gx, 1);
  tft.print(" Y:"); tft.print(gy, 1);
  tft.print(" Z:"); tft.print(gz, 1);
}

void TFTDisplay::updateAttitude(float roll, float pitch, float yaw) {
  tft.fillRect(30, 52, 125, 10, BLACK);
  tft.setTextColor(YELLOW);
  tft.setCursor(30, 57);
  tft.print("R:"); tft.print(roll, 1);
  tft.print(" P:"); tft.print(pitch, 1);
  tft.print(" Y:"); tft.print(yaw, 1);
}

void TFTDisplay::updateGPS(const GPSData &gps_data) {
  tft.fillRect(30, 67, 125, 10, BLACK);
  tft.setTextColor(ORANGE);
  tft.setCursor(30, 72);
  if (gps_data.is_fixed) {
    tft.print("L:"); tft.print(gps_data.gcj02_lat, 4);
    tft.print(" Lo:"); tft.print(gps_data.gcj02_lon, 4);
  } else {
    tft.print("No Fix");
  }
}

void TFTDisplay::updateRecordingState(bool is_recording) {
  tft.fillRect(30, 82, 125, 10, BLACK);
  tft.setTextColor(CYAN);
  tft.setCursor(30, 87);
  if (is_recording) {
    tft.print("Recording (ON)");
  } else {
    tft.print("Stopped (OFF)");
  }
}

void TFTDisplay::updateFileName(const String &filename) {
  tft.fillRect(30, 97, 125, 10, BLACK);
  tft.setTextColor(WHITE);
  tft.setCursor(30, 102);
  tft.print(filename.substring(1));
}
