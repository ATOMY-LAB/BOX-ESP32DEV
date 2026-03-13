#include "lora_comm.h"

bool LoRaCommunication::begin() {
  LoRaSerial.begin(LORA_BAUD_RATE, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  command_callback = nullptr;
  
  Serial.println("[LoRa] Init Success");
  return true;
}

void LoRaCommunication::processSerialData() {
  while (LoRaSerial.available() > 0) {
    char loraData = LoRaSerial.read();
    if (loraData == '1' || loraData == '2') {
      handleCommand(loraData);
    }
  }
}

void LoRaCommunication::sendData(const IMUData &imu_data, const GPSData &gps_data) {
  static char loraBuffer[384];  // ✅ 增大缓冲区到384字节确保所有数据完整
  
  if (gps_data.is_fixed) {
    // ✅ 使用固定宽度格式确保字符数量一致
    snprintf(loraBuffer, sizeof(loraBuffer),
      "TS:%10lu,AX:%7.2f,AY:%7.2f,AZ:%7.2f,GX:%7.2f,GY:%7.2f,GZ:%7.2f,ROLL:%7.2f,PITCH:%7.2f,YAW:%7.2f,LAT:%11.8f,LAT_DIR:%c,LON:%11.8f,LON_DIR:%c\r\n",
      imu_data.timestamp,
      imu_data.ax, imu_data.ay, imu_data.az,
      imu_data.gx, imu_data.gy, imu_data.gz,
      imu_data.roll, imu_data.pitch, imu_data.yaw,
      gps_data.gcj02_lat, gps_data.lat_dir,
      gps_data.gcj02_lon, gps_data.lon_dir
    );
  } else {
    snprintf(loraBuffer, sizeof(loraBuffer),
      "TS:%10lu,AX:%7.2f,AY:%7.2f,AZ:%7.2f,GX:%7.2f,GY:%7.2f,GZ:%7.2f,ROLL:%7.2f,PITCH:%7.2f,YAW:%7.2f,LAT:   NO_FIX,LON:   NO_FIX\r\n",
      imu_data.timestamp,
      imu_data.ax, imu_data.ay, imu_data.az,
      imu_data.gx, imu_data.gy, imu_data.gz,
      imu_data.roll, imu_data.pitch, imu_data.yaw
    );
  }
  
  LoRaSerial.print(loraBuffer);
  
  Serial.print("[LoRa Send] ");
  Serial.print(loraBuffer);
}

void LoRaCommunication::setCommandCallback(LoRaCommandCallback callback) {
  command_callback = callback;
}

void LoRaCommunication::sendCustomData(const String &data) {
  LoRaSerial.print(data);
  Serial.print("[LoRa Custom] ");
  Serial.println(data);
}

void LoRaCommunication::handleCommand(char cmd) {
  Serial.print("[LoRa Command] Received: ");
  Serial.println(cmd);
  
  if (command_callback) {
    command_callback(cmd);  // 所有命令处理逻辑委托给回调，避免重复
  }
}
