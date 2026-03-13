#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "sensors.h"
#include "config.h"

extern HardwareSerial GPS_Serial;

class GPSModule {
public:
  bool begin();
  void processSerialData();
  const GPSData& getData() const;
  bool isFixed() const;

private:
  GPSData gps_data;
  char nmea_buffer[256];
  int buffer_index;
  bool is_gga_valid;

  // 使用char*避免String堆分配
  void parseGGA(const char *gga_str);
  double ddm2dd(const char *ddm_str, int len);
  void wgs84ToGcj02(double wgs_lat, double wgs_lon, double &gcj_lat, double &gcj_lon);
  bool outOfChina(double lat, double lon);
  double transformLat(double x, double y);
  double transformLon(double x, double y);

  // 在char数组中查找第n个逗号的位置
  static int findComma(const char *str, int n);
};

#endif
