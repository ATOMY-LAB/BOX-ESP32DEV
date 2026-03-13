#include "imu_module.h"

bool IMUModule::begin() {
  int16_t test_val = readRegister(JY901_ADDR, AX_ADDR);
  Serial.print("[IMU] Init - Test read: ");
  Serial.println(test_val);
  return true;
}

bool IMUModule::readData(IMUData &data) {
  // 批量读取加速度+角速度 (AX=0x34..GZ=0x39, 连续6个寄存器)
  int16_t accel_gyro[6];
  if (!burstRead(JY901_ADDR, AX_ADDR, accel_gyro, 6)) {
    return false;
  }

  // 批量读取欧拉角 (ROLL=0x3D..YAW=0x3F, 连续3个寄存器)
  int16_t angles[3];
  if (!burstRead(JY901_ADDR, ROLL_ADDR, angles, 3)) {
    return false;
  }

  currentData.timestamp = millis();

  // 保存原始int16值 (用于二进制存储)
  currentData.raw_ax = accel_gyro[0];
  currentData.raw_ay = accel_gyro[1];
  currentData.raw_az = accel_gyro[2];
  currentData.raw_gx = accel_gyro[3];
  currentData.raw_gy = accel_gyro[4];
  currentData.raw_gz = accel_gyro[5];
  currentData.raw_roll = angles[0];
  currentData.raw_pitch = angles[1];
  currentData.raw_yaw = angles[2];

  // 缩放为物理量 (用于显示和LoRa传输)
  currentData.ax = (float)accel_gyro[0] * IMU_ACC_SCALE;
  currentData.ay = (float)accel_gyro[1] * IMU_ACC_SCALE;
  currentData.az = (float)accel_gyro[2] * IMU_ACC_SCALE;
  currentData.gx = (float)accel_gyro[3] * IMU_GYRO_SCALE;
  currentData.gy = (float)accel_gyro[4] * IMU_GYRO_SCALE;
  currentData.gz = (float)accel_gyro[5] * IMU_GYRO_SCALE;
  currentData.roll  = (float)angles[0] * IMU_ANGLE_SCALE;
  currentData.pitch = (float)angles[1] * IMU_ANGLE_SCALE;
  currentData.yaw   = (float)angles[2] * IMU_ANGLE_SCALE;

  data = currentData;
  return true;
}

const IMUData& IMUModule::getLastData() const {
  return currentData;
}

bool IMUModule::burstRead(uint8_t dev_addr, uint8_t start_reg, int16_t *out, uint8_t count) {
  uint8_t bytes_needed = count * 2;

  Wire.beginTransmission(dev_addr);
  Wire.write(start_reg);
  byte error = Wire.endTransmission(false);
  if (error != 0) {
    return false;
  }

  Wire.requestFrom(dev_addr, bytes_needed);
  if (Wire.available() < bytes_needed) {
    return false;
  }

  for (uint8_t i = 0; i < count; i++) {
    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    out[i] = (int16_t)((hi << 8) | lo);
  }
  return true;
}

int16_t IMUModule::readRegister(uint8_t dev_addr, uint8_t reg_addr) {
  int16_t val = 0;
  burstRead(dev_addr, reg_addr, &val, 1);
  return val;
}
