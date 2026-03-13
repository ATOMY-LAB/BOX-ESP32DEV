#include "imu_module.h"

bool IMUModule::begin() {
  int16_t test_val = readRegister(JY901_ADDR, AX_ADDR);
  Serial.print("[IMU] Init - Test read: ");
  Serial.println(test_val);
  return true;
}

bool IMUModule::readData(IMUData &data) {
  // 单次批量读取 0x34-0x3F (12个寄存器, 24字节), 覆盖:
  //   [0-5]  = AX AY AZ GX GY GZ (加速度+角速度)
  //   [6-8]  = HX HY HZ (磁力计, 跳过不用)
  //   [9-11] = ROLL PITCH YAW (欧拉角)
  // 单次事务消除两次读取之间传感器刷新寄存器导致的字节撕裂/数据不一致问题
  int16_t buf[12];
  if (!burstRead(JY901_ADDR, AX_ADDR, buf, 12)) {
    return false;
  }

  currentData.timestamp = millis();

  // 保存原始int16值 (用于二进制存储)
  currentData.raw_ax    = buf[0];
  currentData.raw_ay    = buf[1];
  currentData.raw_az    = buf[2];
  currentData.raw_gx    = buf[3];
  currentData.raw_gy    = buf[4];
  currentData.raw_gz    = buf[5];
  // buf[6-8] = 磁力计, 跳过
  currentData.raw_roll  = buf[9];
  currentData.raw_pitch = buf[10];
  currentData.raw_yaw   = buf[11];

  // 缩放为物理量 (用于显示和LoRa传输)
  currentData.ax    = (float)buf[0] * IMU_ACC_SCALE;
  currentData.ay    = (float)buf[1] * IMU_ACC_SCALE;
  currentData.az    = (float)buf[2] * IMU_ACC_SCALE;
  currentData.gx    = (float)buf[3] * IMU_GYRO_SCALE;
  currentData.gy    = (float)buf[4] * IMU_GYRO_SCALE;
  currentData.gz    = (float)buf[5] * IMU_GYRO_SCALE;
  currentData.roll  = (float)buf[9]  * IMU_ANGLE_SCALE;
  currentData.pitch = (float)buf[10] * IMU_ANGLE_SCALE;
  currentData.yaw   = (float)buf[11] * IMU_ANGLE_SCALE;

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
