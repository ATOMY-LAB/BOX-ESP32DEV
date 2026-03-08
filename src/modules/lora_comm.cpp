#include "modules/lora_comm.h"

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
  String dataStr = "";
  dataStr += "TS:" + String(imu_data.timestamp) + ",";
  dataStr += "AX:" + String(imu_data.ax, 2) + ",";
  dataStr += "AY:" + String(imu_data.ay, 2) + ",";
  dataStr += "ROLL:" + String(imu_data.roll, 2) + ",";
  dataStr += "PITCH:" + String(imu_data.pitch, 2) + ",";
  
  if (gps_data.is_fixed) {
    dataStr += "LAT:" + String(gps_data.gcj02_lat, 6) + ",";
    dataStr += "LON:" + String(gps_data.gcj02_lon, 6);
  } else {
    dataStr += "LAT:NO_FIX,LON:NO_FIX";
  }
  
  dataStr += "\r\n";
  LoRaSerial.print(dataStr);
  
  Serial.print("[LoRa Send] ");
  Serial.print(dataStr);
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
    command_callback(cmd);
  }
  
  if (cmd == '1') {
    Serial.println("[LoRa] Start recording (1)");
  } else if (cmd == '2') {
    Serial.println("[LoRa] Stop recording (2)");
  }
}
