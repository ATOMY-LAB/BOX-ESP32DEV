#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <Arduino.h>
#include <Wire.h>
#include "sensors.h"
#include "config.h"

class IMUModule {
public:
  bool begin();
  bool readData(IMUData &data);
  const IMUData& getLastData() const;

private:
  IMUData currentData;

  // 批量读取连续寄存器 (减少I2C事务次数)
  bool burstRead(uint8_t dev_addr, uint8_t start_reg, int16_t *out, uint8_t count);
  int16_t readRegister(uint8_t dev_addr, uint8_t reg_addr);
};

#endif
