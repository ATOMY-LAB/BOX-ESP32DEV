#include "modules/gps_module.h"
#include <math.h>

bool GPSModule::begin() {
  GPS_Serial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  gps_data.is_fixed = false;
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
        String nmeaStr = String(nmea_buffer);
        
        if (nmeaStr.startsWith("$GPGGA") || nmeaStr.startsWith("$BDGGA") || nmeaStr.startsWith("$GNGGA")) {
          is_gga_valid = true;
          parseGGA(nmeaStr);
        }
        
        if (is_gga_valid && gps_data.is_fixed) {
          is_gga_valid = false;
        }
        
        buffer_index = 0;
        memset(nmea_buffer, 0, sizeof(nmea_buffer));
      }
    } else {
      if (buffer_index < sizeof(nmea_buffer) - 1) {
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

void GPSModule::parseGGA(const String &ggaStr) {
  // GGA格式: $GPGGA,hhmmss.ss,lat,N/S,lon,E/W,quality,...
  int comma1 = ggaStr.indexOf(',', 7);
  int comma2 = ggaStr.indexOf(',', comma1 + 1);
  int comma3 = ggaStr.indexOf(',', comma2 + 1);
  int comma4 = ggaStr.indexOf(',', comma3 + 1);

  String latDDM = ggaStr.substring(comma1 + 1, comma2);
  String latDir = ggaStr.substring(comma2 + 1, comma3);
  String lonDDM = ggaStr.substring(comma3 + 1, comma4);
  int comma5 = ggaStr.indexOf(',', comma4 + 1);
  String lonDir = ggaStr.substring(comma4 + 1, comma5);

  double wgs_lat = ddm2dd(latDDM);
  if (latDir == "S") wgs_lat = -wgs_lat;
  
  double wgs_lon = ddm2dd(lonDDM);
  if (lonDir == "W") wgs_lon = -wgs_lon;

  gps_data.wgs84_lat = wgs_lat;
  gps_data.wgs84_lon = wgs_lon;
  gps_data.lat_dir = latDir;
  gps_data.lon_dir = lonDir;
  gps_data.timestamp = millis();

  wgs84ToGcj02(wgs_lat, wgs_lon, gps_data.gcj02_lat, gps_data.gcj02_lon);
  gps_data.is_fixed = (wgs_lat != 0.0 && wgs_lon != 0.0);
}

double GPSModule::ddm2dd(const String &ddmStr) {
  if (ddmStr.isEmpty()) return 0.0;
  double ddm = atof(ddmStr.c_str());
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
  magic = 1 - EARTH_ECCENTRICITY * magic * magic;
  double sqrtMagic = sqrt(magic);
  
  dLat = (dLat * 180.0) / ((EARTH_RADIUS_A * (1 - EARTH_ECCENTRICITY)) / (magic * sqrtMagic) * PI);
  dLon = (dLon * 180.0) / (EARTH_RADIUS_A / sqrtMagic * cos(radLat) * PI);
  
  gcj_lat = wgs_lat + dLat;
  gcj_lon = wgs_lon + dLon;
}

bool GPSModule::outOfChina(double lat, double lon) {
  if (lon < 72.004 || lon > 137.8347) return true;
  if (lat < 0.8293 || lat > 55.8271) return true;
  return false;
}

double GPSModule::transformLat(double x, double y) {
  double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(fabs(x));
  ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;
  ret += (20.0 * sin(y * PI) + 40.0 * sin(y / 3.0 * PI)) * 2.0 / 3.0;
  ret += (160.0 * sin(y / 12.0 * PI) + 320 * sin(y * PI / 30.0)) * 2.0 / 3.0;
  return ret;
}

double GPSModule::transformLon(double x, double y) {
  double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(fabs(x));
  ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;
  ret += (20.0 * sin(x * PI) + 40.0 * sin(x / 3.0 * PI)) * 2.0 / 3.0;
  ret += (150.0 * sin(x / 12.0 * PI) + 300.0 * sin(x / 30.0 * PI)) * 2.0 / 3.0;
  return ret;
}
