#ifndef SD_STORAGE_H
#define SD_STORAGE_H

#include <Arduino.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include "sensors.h"
#include "config.h"

// 二进制存储格式说明 (version=2):
// Python读取: numpy.fromfile(f, dtype=record_dtype)
// 文件头32字节(BinaryFileHeader) + N条32字节记录(BinaryRecord)
//
// 断电恢复机制:
//   每次flush()时将record_count回写到文件头偏移24处，
//   即使断电后FAT目录项文件大小有误，也可通过header.record_count
//   定位到最后一条有效记录。

class SDStorage {
public:
  bool begin();
  bool logData(const IMUData &imu_data, const GPSData &gps_data);
  void flush();                    // 定期flush + 回写record_count
  void stopRecording();            // 停止录制：flush并关闭文件句柄
  const String& getFileName() const;
  bool isReady() const;
  uint32_t getRecordCount() const;

private:
  String data_filename;
  fs::File dataFile;               // 持久文件句柄 (避免每次open/close)
  bool is_ready = false;
  uint32_t record_count = 0;

  String findNextFileName();
  void writeHeader();
  void updateHeaderCount();        // 将record_count回写到文件头
};

#endif
