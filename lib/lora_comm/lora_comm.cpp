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
    } else {
      Serial.print("[LoRa] Invalid data: ");
      Serial.write(loraData);
      Serial.println();
    }
  }
}

void LoRaCommunication::sendData(const IMUData &imu_data, const GPSData &gps_data) {
  static char loraBuffer[128];  // 固定缓冲区，避免堆碎片
  
  if (gps_data.is_fixed) {
    snprintf(loraBuffer, sizeof(loraBuffer),
      "TS:%lu,AX:%.2f,AY:%.2f,ROLL:%.2f,PITCH:%.2f,LAT:%.6f,LON:%.6f\r\n",
      imu_data.timestamp, imu_data.ax, imu_data.ay,
      imu_data.roll, imu_data.pitch,
      gps_data.gcj02_lat, gps_data.gcj02_lon
    );
  } else {
    snprintf(loraBuffer, sizeof(loraBuffer),
      "TS:%lu,AX:%.2f,AY:%.2f,ROLL:%.2f,PITCH:%.2f,LAT:NO_FIX,LON:NO_FIX\r\n",
      imu_data.timestamp, imu_data.ax, imu_data.ay,
      imu_data.roll, imu_data.pitch
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
