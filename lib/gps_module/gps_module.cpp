#include "gps_module.h"
#include <math.h>

bool GPSModule::begin() {
  GPS_Serial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  gps_data.is_fixed = false;
  gps_data.lat_dir = 'N';
  gps_data.lon_dir = 'E';
  buffer_index = 0;
  is_gga_valid = false;
  memset(nmea_buffer, 0, sizeof(nmea_buffer));
  Serial.println("[GPS] Init Success");
  return true;
}

void GPSModule::processSerialData() {
  while (GPS_Serial.available() > 0) {
    char ch = GPS_Serial.read();

    if (ch == '\r' || ch == '\n') {
      if (buffer_index > 0) {
        nmea_buffer[buffer_index] = '\0';

        // 直接用strncmp比较前缀，避免String分配
        if (strncmp(nmea_buffer, "$GPGGA", 6) == 0 ||
            strncmp(nmea_buffer, "$BDGGA", 6) == 0 ||
            strncmp(nmea_buffer, "$GNGGA", 6) == 0) {
          is_gga_valid = true;
          parseGGA(nmea_buffer);
        }

        if (is_gga_valid && gps_data.is_fixed) {
          is_gga_valid = false;
        }

        buffer_index = 0;
      }
    } else {
      if (buffer_index < (int)sizeof(nmea_buffer) - 1) {
        nmea_buffer[buffer_index++] = ch;
      }
    }
  }
}

const GPSData& GPSModule::getData() const {
  return gps_data;
}

bool GPSModule::isFixed() const {
  return gps_data.is_fixed;
}

int GPSModule::findComma(const char *str, int n) {
  int count = 0;
  for (int i = 0; str[i] != '\0'; i++) {
    if (str[i] == ',') {
      count++;
      if (count == n) return i;
    }
  }
  return -1;
}

void GPSModule::parseGGA(const char *gga_str) {
  // GGA: $GPGGA,time,lat,N/S,lon,E/W,quality,...
  // 找到各逗号位置
  int c1 = findComma(gga_str, 2);  // lat start
  int c2 = findComma(gga_str, 3);  // N/S
  int c3 = findComma(gga_str, 4);  // lon start
  int c4 = findComma(gga_str, 5);  // E/W
  int c5 = findComma(gga_str, 6);  // quality

  if (c1 < 0 || c2 < 0 || c3 < 0 || c4 < 0 || c5 < 0) return;

  double wgs_lat = ddm2dd(gga_str + c1 + 1, c2 - c1 - 1);
  char latDir = gga_str[c2 + 1];
  if (latDir == 'S') wgs_lat = -wgs_lat;

  double wgs_lon = ddm2dd(gga_str + c3 + 1, c4 - c3 - 1);
  char lonDir = gga_str[c4 + 1];
  if (lonDir == 'W') wgs_lon = -wgs_lon;

  gps_data.wgs84_lat = wgs_lat;
  gps_data.wgs84_lon = wgs_lon;
  gps_data.lat_dir = latDir;
  gps_data.lon_dir = lonDir;
  gps_data.timestamp = millis();

  wgs84ToGcj02(wgs_lat, wgs_lon, gps_data.gcj02_lat, gps_data.gcj02_lon);
  gps_data.is_fixed = (wgs_lat != 0.0 && wgs_lon != 0.0);
}

double GPSModule::ddm2dd(const char *ddm_str, int len) {
  if (len <= 0) return 0.0;
  // 用固定缓冲区做atof，避免修改原始缓冲区
  char tmp[20];
  int copyLen = (len < 19) ? len : 19;
  memcpy(tmp, ddm_str, copyLen);
  tmp[copyLen] = '\0';

  double ddm = atof(tmp);
  int deg = (int)(ddm / 100);
  double min = ddm - deg * 100;
  return deg + min / 60.0;
}

void GPSModule::wgs84ToGcj02(double wgs_lat, double wgs_lon,
                              double &gcj_lat, double &gcj_lon) {
  if (outOfChina(wgs_lat, wgs_lon)) {
    gcj_lat = wgs_lat;
    gcj_lon = wgs_lon;
    return;
  }

  double dLat = transformLat(wgs_lon - 105.0, wgs_lat - 35.0);
  double dLon = transformLon(wgs_lon - 105.0, wgs_lat - 35.0);
  double radLat = wgs_lat / 180.0 * PI;
  double magic = sin(radLat);
  magic = 1 - 0.00669 * magic * magic;
  double sqrtMagic = sqrt(magic);
  dLat = (dLat * 180.0) / ((6370996.81 * (1 - 0.00669)) / (magic * sqrtMagic) * PI);
  dLon = (dLon * 180.0) / (6370996.81 / sqrtMagic * cos(radLat) * PI);
  gcj_lat = wgs_lat + dLat;
  gcj_lon = wgs_lon + dLon;
}

bool GPSModule::outOfChina(double lat, double lon) {
  return (lon < 72.004 || lon > 137.8347 || lat < 0.8293 || lat > 55.8271);
}

double GPSModule::transformLat(double x, double y) {
  double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(abs(x));
  ret += ((20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0);
  ret += ((20.0 * sin(y * PI) + 40.0 * sin(y / 3.0 * PI)) * 2.0 / 3.0);
  ret += ((160.0 * sin(y / 12.0 * PI) + 320 * sin(y * PI / 30.0)) * 2.0 / 3.0);
  return ret;
}

double GPSModule::transformLon(double x, double y) {
  double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(abs(x));
  ret += ((20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0);
  ret += ((20.0 * sin(x * PI) + 40.0 * sin(x / 3.0 * PI)) * 2.0 / 3.0);
  ret += ((150.0 * sin(x / 12.0 * PI) + 300.0 * sin(x / 30.0 * PI)) * 2.0 / 3.0);
  return ret;
}
