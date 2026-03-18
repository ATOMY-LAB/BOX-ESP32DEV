#include "lora_comm.h"

bool LoRaCommunication::begin() {
  LoRaSerial.begin(LORA_BAUD_RATE, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  command_callback = nullptr;
  rx_buffer_index = 0;  // 初始化接收缓冲区
  
  Serial.println("[LoRa] Init Success");
  return true;
}

void LoRaCommunication::processSerialData() {
  // 持续读取LoRa模块的串口数据
  while (LoRaSerial.available() > 0) {
    char c = LoRaSerial.read();
    
    // 检查缓冲区溢出
    if (rx_buffer_index >= RX_BUFFER_SIZE - 1) {
      rx_buffer_index = 0;  // 缓冲区满，重置
      Serial.println("[LoRa] RX Buffer overflow!");
      return;
    }
    
    // 检测包尾 (换行符)
    if (c == '\n' || c == '\r') {
      if (rx_buffer_index > 0) {
        rx_buffer[rx_buffer_index] = '\0';  // 字符串终止符
        String packet(rx_buffer);
        
        // 打印接收到的包用于调试
        Serial.print("[LoRa RCV] ");
        Serial.println(packet);
        
        // 处理完整的包
        processLoRaPacket(packet);
        
        rx_buffer_index = 0;  // 重置缓冲区指针
      }
    } else {
      // 累积数据到缓冲区
      rx_buffer[rx_buffer_index++] = c;
    }
  }
}

void LoRaCommunication::sendData(const IMUData &imu_data, const GPSData &gps_data) {
  // 新LoRa协议格式: $LORA,设备ID,DATA,数据内容\n
  // 数据字段用 | 分隔：TS|AX|AY|AZ|GX|GY|GZ|ROLL|PITCH|YAW|LAT|LON
  
  static char loraBuffer[512];  // 增大缓冲区以容纳新格式
  
  if (gps_data.is_fixed) {
    snprintf(loraBuffer, sizeof(loraBuffer),
      "%s,%s,DATA,TS:%lu|AX:%.2f|AY:%.2f|AZ:%.2f|GX:%.2f|GY:%.2f|GZ:%.2f|ROLL:%.2f|PITCH:%.2f|YAW:%.2f|LAT:%.6f|LON:%.6f\n",
      PROTOCOL_HEADER,
      DEVICE_ID,
      imu_data.timestamp,
      imu_data.ax, imu_data.ay, imu_data.az,
      imu_data.gx, imu_data.gy, imu_data.gz,
      imu_data.roll, imu_data.pitch, imu_data.yaw,
      gps_data.gcj02_lat, gps_data.gcj02_lon
    );
  } else {
    snprintf(loraBuffer, sizeof(loraBuffer),
      "%s,%s,DATA,TS:%lu|AX:%.2f|AY:%.2f|AZ:%.2f|GX:%.2f|GY:%.2f|GZ:%.2f|ROLL:%.2f|PITCH:%.2f|YAW:%.2f|LAT:NO_FIX|LON:NO_FIX\n",
      PROTOCOL_HEADER,
      DEVICE_ID,
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

void LoRaCommunication::processLoRaPacket(const String &packet) {
  // 新LoRa协议格式: $LORA,设备ID,CMD,内容 或 $LORA,设备ID,DATA,内容
  // 只处理发往这个设备的包，其他设备的包自动过滤
  
  // 检查包头
  if (!packet.startsWith(PROTOCOL_HEADER)) {
    return;  // 不是LoRa协议包，忽略
  }
  
  // 查找逗号进行字段分割
  int comma1 = packet.indexOf(',');
  if (comma1 < 0) return;  // 格式错误
  
  int comma2 = packet.indexOf(',', comma1 + 1);
  if (comma2 < 0) return;  // 格式错误
  
  int comma3 = packet.indexOf(',', comma2 + 1);
  if (comma3 < 0) return;  // 格式错误
  
  // 提取各个字段
  String deviceID = packet.substring(comma1 + 1, comma2);
  String dataType = packet.substring(comma2 + 1, comma3);
  String dataContent = packet.substring(comma3 + 1);
  
  // 检查是否是针对本设备的包
  if (deviceID != DEVICE_ID) {
    Serial.print("[LoRa] Ignored packet for device: ");
    Serial.println(deviceID);
    return;  // 不是发给我们的包，忽略
  }
  
  // 处理命令包
  if (dataType == "CMD") {
    Serial.print("[LoRa] Command received: ");
    Serial.println(dataContent);
    
    // 识别命令内容
    if (dataContent == "START") {
      handleCommand('1');  // 开始记录
    } else if (dataContent == "STOP") {
      handleCommand('2');  // 停止记录
    }
  } else if (dataType == "DATA") {
    // DATA包可选处理（如果需要接收数据的话）
    Serial.print("[LoRa] Data from remote: ");
    Serial.println(dataContent);
  }
}

void LoRaCommunication::handleCommand(char cmd) {
  Serial.print("[LoRa Command] Received: ");
  Serial.println(cmd);
  
  if (command_callback) {
    command_callback(cmd);  // 所有命令处理逻辑委托给回调，避免重复
  }
}
