#include "imu_module.h"

bool IMUModule::begin() {
  // Wire已在 main.cpp 中初始化，此处跳过
  // 测试I2C通信
  int16_t test_val = readRegister(JY901_ADDR, AX_ADDR);
  Serial.print("[IMU] Init - Test read: ");
  Serial.println(test_val);
  
  return true;
}

bool IMUModule::readData(IMUData &data) {
  int16_t raw_ax = readRegister(JY901_ADDR, AX_ADDR);
  int16_t raw_ay = readRegister(JY901_ADDR, AY_ADDR);
  int16_t raw_az = readRegister(JY901_ADDR, AZ_ADDR);
  int16_t raw_gx = readRegister(JY901_ADDR, GX_ADDR);
  int16_t raw_gy = readRegister(JY901_ADDR, GY_ADDR);
  int16_t raw_gz = readRegister(JY901_ADDR, GZ_ADDR);
  int16_t raw_roll = readRegister(JY901_ADDR, ROLL_ADDR);
  int16_t raw_pitch = readRegister(JY901_ADDR, PITCH_ADDR);
  int16_t raw_yaw = readRegister(JY901_ADDR, YAW_ADDR);

  currentData.timestamp = millis();
  currentData.ax = scaleData(raw_ax, 16 * 9.8 / 32768.0);
  currentData.ay = scaleData(raw_ay, 16 * 9.8 / 32768.0);
  currentData.az = scaleData(raw_az, 16 * 9.8 / 32768.0);
  currentData.gx = scaleData(raw_gx, 2000.0 / 32768.0);
  currentData.gy = scaleData(raw_gy, 2000.0 / 32768.0);
  currentData.gz = scaleData(raw_gz, 2000.0 / 32768.0);
  currentData.roll = scaleData(raw_roll, 180.0 / 32768.0);
  currentData.pitch = scaleData(raw_pitch, 180.0 / 32768.0);
  currentData.yaw = scaleData(raw_yaw, 180.0 / 32768.0);

  data = currentData;
  return true;
}

const IMUData& IMUModule::getLastData() const {
  return currentData;
}

int16_t IMUModule::readRegister(uint8_t dev_addr, uint8_t reg_addr) {
  int16_t data = 0;
  uint8_t buf[2] = {0};
  
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  byte error = Wire.endTransmission(false);
  
  // 检查传输错误
  if (error != 0) {
    Serial.print("[IMU] I2C transmission error: ");
    Serial.println(error);
    return 0;  // 返回哨兵值，表示错误
  }
  
  Wire.requestFrom(dev_addr, (uint8_t)2);
  if (Wire.available() == 2) {
    buf[0] = Wire.read();
    buf[1] = Wire.read();
    data = (int16_t)((buf[1] << 8) | buf[0]);
  } else {
    Serial.println("[IMU] I2C read timeout or incomplete read");
    return 0;
  }
  
  return data;
}

float IMUModule::scaleData(int16_t raw_value, float scale) {
  return (float)raw_value * scale;
}
