/*
 Name:		ESP32 APRS Internet Gateway
 Created:	1-Nov-2021 14:27:23
 Author:	HS5TQA/Atten
 Support IS: host:aprs.dprns.com port:14580
 Support IS monitor: http://aprs.dprns.com:14501
 Support in LINE Group APRS Only
*/

#include <Arduino.h>
#include "main.h"
#include <LibAPRSesp.h>
#include <limits.h>
#include <KISS.h>
#include "webservice.h"
#include <WiFiUdp.h>
#include "ESP32Ping.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClient.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "XPowersLib.h"
#include "cppQueue.h"
#include "digirepeater.h"
#include "igate.h"

#include "time.h"

#include <TinyGPSPlus.h>
#include <pbuf.h>
#include <parse_aprs.h>
// #include <Fonts/FreeSansBold9pt7b.h>
// #include <Fonts/FreeSerifItalic9pt7b.h>
// #include <Fonts/Seven_Segment24pt7b.h>

#include <WiFiUdp.h>

#include <WiFiClientSecure.h>

#include "AFSK.h"

#include "gui_lcd.h"

#define DEBUG_TNC

#define EEPROM_SIZE 2048

#ifdef SDCARD
#include <SPI.h> //SPI.h must be included as DMD is written by SPI (the IDE complains otherwise)
#include "FS.h"
#include "SPIFFS.h"
#define SDCARD_CS 10
#define SDCARD_CLK 12
#define SDCARD_MOSI 11
#define SDCARD_MISO 13
#endif

#ifdef OLED
#include <Wire.h>
#include "Adafruit_SSD1306.h"
#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// For a connection via I2C using the Arduino Wire include:
// #include <Wire.h>		 // Only needed for Arduino 1.6.5 and earlier
// #include "SSD1306Wire.h" // legacy: #include "SSD1306.h"
// OR #include "SH1106Wire.h"   // legacy: #include "SH1106.h"

// Initialize the OLED display using Arduino Wire:
// SSD1306Wire display(0x3c, SDA, SCL, GEOMETRY_128_64); // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h
// SSD1306Wire display(0x3c, D3, D5);  // ADDRESS, SDA, SCL  -  If not, they can be specified manually.
// SSD1306Wire display(0x3c, SDA, SCL, GEOMETRY_128_32);  // ADDRESS, SDA, SCL, OLEDDISPLAY_GEOMETRY  -  Extra param required for 128x32 displays.
// SH1106Wire display(0x3c, SDA, SCL);     // ADDRESS, SDA, SCL

#endif

struct pbuf_t aprs;
ParseAPRS aprsParse;

TinyGPSPlus gps;

#ifdef SA818
#define VBAT_PIN 36
// #define WIRE 4
#define POWER_PIN SA868_PWR_PIN
#define PULLDOWN_PIN SA868_PD_PIN
#define SQL_PIN -1
HardwareSerial SerialRF(1);
#endif

#define PWR_VDD 4

#define MODEM_PWRKEY 5
#define MODEM_TX 13
#define MODEM_RX 14

#define LED_TX -1
#define LED_RX 5

#define PPP_APN "internet"
#define PPP_USER ""
#define PPP_PASS ""

char send_aprs_table, send_aprs_symbol;

time_t systemUptime = 0;
time_t wifiUptime = 0;

boolean KISS = false;
bool aprsUpdate = false;

boolean gotPacket = false;
AX25Msg incomingPacket;

bool lastPkg = false;
bool afskSync = false;
String lastPkgRaw = "";
float dBV = 0;
int mVrms = 0;

float vbat;

uint8_t dispFlagTX = 0;

long timeNetwork, timeAprs, timeGui;

unsigned long previousMillis = 0; // กำหนดตัวแปรเก็บค่า เวลาสุดท้ายที่ทำงาน
long interval = 10000;            // กำหนดค่าตัวแปร ให้ทำงานทุกๆ 10 วินาที
int conStat = 0;
int conStatNetwork = 0;

cppQueue PacketBuffer(sizeof(AX25Msg), 10, IMPLEMENTATION); // Instantiate queue
// cppQueue dispBuffer(300, 100, IMPLEMENTATION);

statusType status;
RTC_DATA_ATTR igateTLMType igateTLM;
#ifdef BOARD_HAS_PSRAM
txQueueType *txQueue;
#else
RTC_DATA_ATTR txQueueType txQueue[PKGTXSIZE];
#endif

RTC_DATA_ATTR double LastLat, LastLng;
RTC_DATA_ATTR time_t lastTimeStamp;

extern RTC_DATA_ATTR uint8_t digiCount;

XPowersAXP2101 PMU;

Configuration config;

pkgListType *pkgList;

TelemetryType *Telemetry;

TaskHandle_t taskNetworkHandle;
TaskHandle_t taskAPRSHandle;
TaskHandle_t mainDisplayHandle;
TaskHandle_t taskGpsHandle;

bool firstGpsTime = true;
time_t startTime = 0;
// HardwareSerial SerialTNC(2);
HardwareSerial SerialGPS(2);
// #define SerialGPS Serial
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool BTdeviceConnected = false;
bool BToldDeviceConnected = false;
uint8_t BTtxValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

// #define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
// #define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
// #define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    BTdeviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    BTdeviceConnected = false;
  }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0)
    {
      // Serial.println("*********");
      // Serial.print("Received Value: ");
      char raw[300];
      memset(raw, 0, sizeof(raw));
      for (int i = 0; i < rxValue.length(); i++)
      {
        if (i > sizeof(raw))
          break;
        raw[i] = (char)rxValue[i];
        if (raw[i] == 0)
          break;
      }
      if (config.bt_mode == 1)
      { // TNC2RAW MODE
        pkgTxPush(raw, 1);
      }
      // for (int i = 0; i < rxValue.length(); i++)
      //   Serial.print(rxValue[i]);

      // Serial.println();
      // Serial.println("*********");
    }
  }
};

// Set your Static IP address for wifi AP
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 254);
IPAddress subnet(255, 255, 255, 0);

IPAddress vpn_IP(192, 168, 44, 195);

int pkgTNC_count = 0;

/// Degrees to radians.
#define DEG2RAD(x) (x / 360 * 2 * PI)
/// Radians to degrees.
#define RAD2DEG(x) (x * (180 / PI))

double direction(double lon0, double lat0, double lon1, double lat1)
{
  double direction;

  /* Convert degrees into radians. */
  lon0 = DEG2RAD(lon0);
  lat0 = DEG2RAD(lat0);
  lon1 = DEG2RAD(lon1);
  lat1 = DEG2RAD(lat1);

  /* Direction from Aviation Formulary V1.42 by Ed Williams by way of
   * http://mathforum.org/library/drmath/view/55417.html */
  direction = atan2(sin(lon1 - lon0) * cos(lat1), cos(lat0) * sin(lat1) - sin(lat0) * cos(lat1) * cos(lon1 - lon0));
  if (direction < 0)
  {
    /* Make direction positive. */
    direction += 2 * PI;
  }

  return RAD2DEG(direction);
}

double distance(double lon0, double lat0, double lon1, double lat1)
{
  double dlon;
  double dlat;
  double a, c;
  /* Convert degrees into radians. */
  lon0 = DEG2RAD(lon0);
  lat0 = DEG2RAD(lat0);
  lon1 = DEG2RAD(lon1);
  lat1 = DEG2RAD(lat1);

  /* Use the haversine formula for distance calculation
   * http://mathforum.org/library/drmath/view/51879.html */
  dlon = lon1 - lon0;
  dlat = lat1 - lat0;
  a = pow(sin(dlat / 2), 2) + cos(lat0) * cos(lat1) * pow(sin(dlon / 2), 2);
  c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return c * 6366.71; /* in kilometers */
}

void GPS_INIT()
{
  SerialGPS.begin(9600, SERIAL_8N1, GNSS_RX, GNSS_TX);
  SerialGPS.flush();
  while (SerialGPS.available() > 0)
    SerialGPS.read();
  // SerialGPS.print("\r\n");
  // SerialGPS.print("$PMTK161,1*28\r\n");
  // SerialGPS.print("$PMTK225,0*2B\r\n");
  // SerialGPS.print("$PMTK869,1,0*34\r\n");
}

// SmartBeaconing(tm) algorithm adapted from HamHUD
char EVENT_TX_POSITION = 0;
unsigned char SB_SPEED = 0, SB_SPEED_OLD = 0;
int16_t SB_HEADING = 0;
uint16_t tx_interval = 0, igate_tx_interval, digi_tx_interval; // How often we transmit, in seconds
unsigned int tx_counter, igate_tx_counter, digi_tx_counter;    // Incremented every second
int16_t last_heading, heading_change;                          // Keep track of corner pegging
uint16_t trk_interval = 15;

void smartbeacon(void)
{
  // Adaptive beacon rate
  if (SB_SPEED >= config.trk_hspeed)
  {
    // Speed is greater than upper limit - use maximum rate
    // Send position now as we're stopping
    //				if (tx_interval != config->tx_slow_interval) flags |= F_XMIT_PENDING;
    tx_interval = (uint16_t)config.trk_maxinterval;
  }
  else
  {
    if (SB_SPEED <= config.trk_lspeed)
    {
      // Speed is below lower limit - use minimum rate
      tx_interval = (uint16_t)config.trk_slowinterval;
    }
    else
    {
      // In between, do SmartBeaconing calculations
      tx_interval = (trk_interval * config.trk_hspeed) / SB_SPEED;
      if (tx_interval < 5)
        tx_interval = 5;
      if (tx_interval > config.trk_slowinterval)
        tx_interval = (uint16_t)config.trk_mininterval;
      // Corner pegging - note that we use two-degree units
      if (last_heading > SB_HEADING)
        heading_change = last_heading - SB_HEADING;
      else
        heading_change = SB_HEADING - last_heading;
      if (heading_change > 90)
        heading_change = 180 - heading_change;
      if (heading_change > config.trk_minangle)
      {
        // Make sure last heading is updated during turns to avoid immedate xmit
        if (tx_counter > config.trk_mininterval)
        {
          EVENT_TX_POSITION = 3;
        }
        else
        {
          // last_heading = SB_HEADING;
          tx_interval = config.trk_mininterval;
        }
      }
    }
  }
}

// void printTime()
// {
// 	SerialLOG.print("[");
// 	SerialLOG.print(hour());
// 	SerialLOG.print(":");
// 	SerialLOG.print(minute());
// 	SerialLOG.print(":");
// 	SerialLOG.print(second());
// 	SerialLOG.print("]");
// }

String deg2lat(double deg)
{
  char sign;
  if (deg > 0.0F)
  {
    sign = 'N';
  }
  else
  {
    sign = 'S';
    deg *= -1;
  }

  uint id = (uint)floor(deg);
  uint im = (uint)((deg - (double)id) * 60);
  uint imm = (uint)round((((deg - (double)id) * 60) - (double)im) * 100);
  char dmm[10];
  sprintf(dmm, "%02d%02d.%02d%c", id, im, imm, sign);
  return String(dmm);
}

String deg2lon(double deg)
{
  char sign;
  if (deg > 0.0F)
  {
    sign = 'E';
  }
  else
  {
    sign = 'W';
    deg *= -1;
  }
  uint id = (uint)floor(deg);
  uint im = (uint)((deg - (double)id) * 60);
  uint imm = (uint)round((((deg - (double)id) * 60) - (double)im) * 100);
  char dmm[10];
  sprintf(dmm, "%03d%02d.%02d%c", id, im, imm, sign);
  return String(dmm);
}

time_t setGpsTime()
{
  time_t time;
  tmElements_t timeinfo;
  if (gps.time.isValid())
  {
    timeinfo.Year = (gps.date.year()) - 1970;
    timeinfo.Month = gps.date.month();
    timeinfo.Day = gps.date.day();
    timeinfo.Hour = gps.time.hour();
    timeinfo.Minute = gps.time.minute();
    timeinfo.Second = gps.time.second();
    time_t timeStamp = makeTime(timeinfo);
    time = timeStamp + TZ * SECS_PER_HOUR;
    setTime(time);
    // setTime(timeinfo.Hour,timeinfo.Minute,timeinfo.Second,timeinfo.Day, timeinfo.Month, timeinfo.Year);
    return time;
  }
  return 0;
}

time_t getGpsTime()
{
  time_t time;
  tmElements_t timeinfo;
  if (gps.time.isValid())
  {
    timeinfo.Year = (gps.date.year()) - 1970;
    timeinfo.Month = gps.date.month();
    timeinfo.Day = gps.date.day();
    timeinfo.Hour = gps.time.hour();
    timeinfo.Minute = gps.time.minute();
    timeinfo.Second = gps.time.second();
    time_t timeStamp = makeTime(timeinfo);
    time = timeStamp + TZ * SECS_PER_HOUR;
    return time;
  }
  return 0;
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length();

  for (int i = 0; i <= maxIndex && found <= index; i++)
  {
    if (data.charAt(i) == separator || i == maxIndex)
    {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
} // END

boolean isValidNumber(String str)
{
  for (byte i = 0; i < str.length(); i++)
  {
    if (isDigit(str.charAt(i)))
      return true;
  }
  return false;
}

uint8_t checkSum(uint8_t *ptr, size_t count)
{
  uint8_t lrc, tmp;
  uint16_t i;
  lrc = 0;
  for (i = 0; i < count; i++)
  {
    tmp = *ptr++;
    lrc = lrc ^ tmp;
  }
  return lrc;
}

void saveEEPROM()
{
  uint8_t chkSum = 0;
  byte *ptr;
  ptr = (byte *)&config;
  EEPROM.writeBytes(1, ptr, sizeof(Configuration));
  chkSum = checkSum(ptr, sizeof(Configuration));
  EEPROM.write(0, chkSum);
  EEPROM.commit();
#ifdef DEBUG
  log_d("Save EEPROM ChkSUM=");
  log_d("%0X\n", chkSum);
#endif
}

void defaultConfig()
{
  log_d("Default configure mode!");
  config.synctime = true;
  config.timeZone = 7;
  config.tx_timeslot = 2000; // ms

  config.wifi_mode = WIFI_AP_STA_FIX;
  config.wifi_power = 44;
  config.wifi_ap_ch = 6;
  sprintf(config.wifi_ssid, "APRSTH");
  sprintf(config.wifi_pass, "aprsthnetwork");
  sprintf(config.wifi_ap_ssid, "ESP32IGate");
  sprintf(config.wifi_ap_pass, "aprsthnetwork");
  // Blutooth
  config.bt_slave = false;
  config.bt_master = false;
  config.bt_mode = 1; // 0-None,1-TNC2RAW,2-KISS
  config.bt_power = 1;
  sprintf(config.bt_uuid, "6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
  sprintf(config.bt_uuid_rx, "6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
  sprintf(config.bt_uuid_tx, "6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
  sprintf(config.bt_name, "ESP32APRS");
  sprintf(config.bt_pin, "0000");

  //--RF Module
  config.rf_en = true;
  config.freq_rx = 144.3900;
  config.freq_tx = 144.3900;
  config.offset_rx = 0;
  config.offset_tx = 0;
  config.tone_rx = 0;
  config.tone_tx = 0;
  config.band = 0;
  config.sql_level = 1;
  config.rf_power = LOW;
  config.volume = 6;
  config.mic = 8;
  config.input_hpf = false;
  input_HPF = config.input_hpf;

  // IGATE
  config.igate_bcn = false;
  config.igate_en = false;
  config.rf2inet = true;
  config.inet2rf = false;
  config.igate_tlm = true;
  config.igate_loc2rf = false;
  config.igate_loc2inet = true;
  //--APRS-IS
  config.aprs_ssid = 1;
  config.aprs_port = 14580;
  sprintf(config.aprs_mycall, "NOCALL");
  sprintf(config.aprs_host, "aprs.dprns.com");
  sprintf(config.aprs_passcode, "");
  sprintf(config.aprs_moniCall, "%s-%d", config.aprs_mycall, config.aprs_ssid);
  sprintf(config.aprs_filter, "m/10");
  //--Position
  config.igate_gps = false;
  config.igate_lat = 13.7555;
  config.igate_lon = 100.4930;
  config.igate_alt = 1;
  config.igate_interval = 600;
  config.igate_tlm_interval = 600;
  sprintf(config.igate_symbol, "/&");
  sprintf(config.igate_object, "");
  sprintf(config.igate_phg, "");
  sprintf(config.igate_path, "WIDE1-1");
  sprintf(config.igate_comment, "IGate MODE");

  // DIGI REPEATER
  config.digi_en = false;
  config.digi_loc2rf = true;
  config.digi_loc2inet = false;
  config.digi_ssid = 3;
  sprintf(config.digi_mycall, "NOCALL");
  sprintf(config.digi_path, "WIDE1-1");
  //--Position
  config.digi_phg = false;
  config.digi_gps = false;
  config.digi_lat = 13.7555;
  config.digi_lon = 100.4930;
  config.digi_alt = 1;
  config.digi_interval = 600;
  config.digi_delay = 0;
  sprintf(config.digi_symbol, "/#");
  // sprintf(config.digi_phg,"");
  sprintf(config.digi_comment, "DIGI MODE");

  // Tracker
  config.trk_en = false;
  config.trk_loc2rf = true;
  config.trk_loc2inet = false;
  config.trk_bat = false;
  config.trk_sat = false;
  config.trk_dx = false;
  config.trk_ssid = 9;
  sprintf(config.digi_mycall, "NOCALL");
  sprintf(config.digi_path, "WIDE1-1,WIDE2-1");
  sprintf(config.digi_comment, "TRACKER MODE");
  //--Position
  config.trk_gps = false;
  config.trk_lat = 13.7555;
  config.trk_lon = 100.4930;
  config.trk_alt = 1;
  config.trk_interval = 600;
  // Smart beacon
  config.trk_smartbeacon = true;
  config.trk_compress = true;
  config.trk_altitude = true;
  config.trk_speed = true;
  config.trk_hspeed = 120;
  config.trk_lspeed = 5;
  config.trk_maxinterval = 30;
  config.trk_mininterval = 5;
  config.trk_minangle = 25;
  config.trk_slowinterval = 600;

  sprintf(config.trk_symbol, "\\>");
  sprintf(config.trk_symmove, "/>");
  sprintf(config.trk_symstop, "\\>");
  sprintf(config.trk_btext, "");
  sprintf(config.trk_mycall, "NOCALL");
  sprintf(config.trk_comment, "");
  sprintf(config.trk_item, "");
  sprintf(config.trk_object, "");

  // OLED DISPLAY
  config.oled_enable = true;
  config.oled_timeout = 60;
  config.dim = 0;
  config.contrast = 0;
  config.startup = 0;

  // Display
  config.dispDelay = 3; // Popup display 3 sec
  config.dispRF = true;
  config.dispINET = false;
  config.filterDistant = 0;
  config.h_up = true;
  config.tx_status = true;
  config.filterMessage = true;
  config.filterStatus = false;
  config.filterTelemetry = false;
  config.filterWeather = true;
  config.filterTracker = true;
  config.filterMove = true;
  config.filterPosition = true;

  sprintf(config.path[0], "WIDE1-1");
  sprintf(config.path[1], "WIDE1-1,WIDE2-1");
  sprintf(config.path[2], "TRACK3-3");
  sprintf(config.path[3], "RS0ISS");

  // VPN Wireguard
  //  config.wg_port = 51820;
  //  sprintf(config.wg_peer_address, "203.150.19.23");
  //  sprintf(config.wg_local_address, "44.63.31.223");
  //  sprintf(config.wg_netmask_address, "255.255.255.255");
  //  sprintf(config.wg_gw_address, "44.63.31.193");
  //  sprintf(config.wg_public_key, "");
  //  sprintf(config.wg_private_key, "");

  config.gpio_sql_pin = -1;

  saveEEPROM();
}

unsigned long NTP_Timeout;
unsigned long pingTimeout;

const char *lastTitle = "LAST HERT";

int tlmList_Find(char *call)
{
  int i;
  for (i = 0; i < TLMLISTSIZE; i++)
  {
    if (strstr(Telemetry[i].callsign, call) != NULL)
      return i;
  }
  return -1;
}

int tlmListOld()
{
  int i, ret = 0;
  time_t minimum = Telemetry[0].time;
  for (i = 1; i < TLMLISTSIZE; i++)
  {
    if (Telemetry[i].time < minimum)
    {
      minimum = Telemetry[i].time;
      ret = i;
    }
    if (Telemetry[i].time > time(NULL))
      Telemetry[i].time = 0;
  }
  return ret;
}

char pkgList_Find(char *call)
{
  char i;
  for (i = 0; i < PKGLISTSIZE; i++)
  {
    if (strstr(pkgList[(int)i].calsign, call) != NULL)
      return i;
  }
  return -1;
}

int pkgList_Find(char *call, uint8_t type)
{
  int i;
  for (i = 0; i < PKGLISTSIZE; i++)
  {
    // if (type == 0) {
    //	if (strstr(pkgList[i].calsign, call) != NULL) return i;
    // }
    // else {
    if ((strstr(pkgList[i].calsign, call) != NULL) && (pkgList[i].type == type))
      return i;
    //}
  }
  return -1;
}

char pkgListOld()
{
  char i, ret = 0;
  time_t minimum = pkgList[0].time;
  for (i = 1; i < PKGLISTSIZE; i++)
  {
    if (pkgList[(int)i].time < minimum)
    {
      minimum = pkgList[(int)i].time;
      ret = i;
    }
  }
  return ret;
}

void sort(pkgListType a[], int size)
{
  pkgListType t;
  char *ptr1;
  char *ptr2;
  char *ptr3;
  ptr1 = (char *)&t;
  for (int i = 0; i < (size - 1); i++)
  {
    for (int o = 0; o < (size - (i + 1)); o++)
    {
      if (a[o].time < a[o + 1].time)
      {
        ptr2 = (char *)&a[o];
        ptr3 = (char *)&a[o + 1];
        memcpy(ptr1, ptr2, sizeof(pkgListType));
        memcpy(ptr2, ptr3, sizeof(pkgListType));
        memcpy(ptr3, ptr1, sizeof(pkgListType));
      }
    }
  }
}

void sortPkgDesc(pkgListType a[], int size)
{
  pkgListType t;
  char *ptr1;
  char *ptr2;
  char *ptr3;
  ptr1 = (char *)&t;
  for (int i = 0; i < (size - 1); i++)
  {
    for (int o = 0; o < (size - (i + 1)); o++)
    {
      if (a[o].pkg < a[o + 1].pkg)
      {
        ptr2 = (char *)&a[o];
        ptr3 = (char *)&a[o + 1];
        memcpy(ptr1, ptr2, sizeof(pkgListType));
        memcpy(ptr2, ptr3, sizeof(pkgListType));
        memcpy(ptr3, ptr1, sizeof(pkgListType));
      }
    }
  }
}

uint8_t pkgType(const char *raw)
{
  uint8_t type = 0;
  char packettype = 0;
  const char *info_start, *body;
  int paclen = strlen(raw);
  // info_start = (char*)strchr(raw, ':');
  // if (info_start == NULL) return 0;
  // info_start=0;
  packettype = (char)raw[0];
  body = &raw[1];

  switch (packettype)
  {
  case '=':
  case '/':
  case '@':
    if (strchr(body, 'r') != NULL)
    {
      if (strchr(body, 'g') != NULL)
      {
        if (strchr(body, 't') != NULL)
        {
          if (strchr(body, 'P') != NULL)
          {
            type = PKG_WX;
          }
        }
      }
    }
    break;
  case ':':
    type = PKG_MESSAGE;
    if (body[9] == ':' &&
        (memcmp(body + 9, ":PARM.", 6) == 0 ||
         memcmp(body + 9, ":UNIT.", 6) == 0 ||
         memcmp(body + 9, ":EQNS.", 6) == 0 ||
         memcmp(body + 9, ":BITS.", 6) == 0))
    {
      type = PKG_TELEMETRY;
    }
    break;
  case '>':
    type = PKG_STATUS;
    break;
  case '?':
    type = PKG_QUERY;
    break;
  case ';':
    type = PKG_OBJECT;
    break;
  case ')':
    type = PKG_ITEM;
    break;
  case 'T':
    type = PKG_TELEMETRY;
    break;
  case '#': /* Peet Bros U-II Weather Station */
  case '*': /* Peet Bros U-I  Weather Station */
  case '_': /* Weather report without position */
    type = PKG_WX;
    break;
  default:
    type = 0;
    break;
  }
  return type;
}

int TNC2Raw[PKGLISTSIZE];
int raw_count = 0, raw_idx_rd = 0, raw_idx_rw = 0;

void pushTNC2Raw(int raw)
{
  if (raw < 0)
    return;
  if (raw_count > PKGLISTSIZE)
    return;
  if (++raw_idx_rw >= PKGLISTSIZE)
    raw_idx_rw = 0;
  TNC2Raw[raw_idx_rw] = raw;
  raw_count++;
}

void popTNC2Raw(int &ret)
{
  String raw = "";
  int idx = 0;
  if (raw_count <= 0)
    return;
  if (++raw_idx_rd >= PKGLISTSIZE)
    raw_idx_rd = 0;
  idx = TNC2Raw[raw_idx_rd];
  if (idx < PKGLISTSIZE)
    ret = idx;
  // raw = String(pkgList[idx].raw);
  if (raw_count > 0)
    raw_count--;
  // return raw;
}

int pkgListUpdate(char *call, char *raw, uint8_t type)
{
  if (*call == 0)
    return -1;
  if (*raw == 0)
    return -1;

  int i = pkgList_Find(call, type);

  if (i > -1)
  { // Found call in old pkg
    pkgList[i].time = time(NULL);
    pkgList[i].pkg++;
    pkgList[i].type = type;
    strcpy(pkgList[i].raw, raw);
    // SerialLOG.print("Update: ");
  }
  else
  {
    i = pkgListOld();
    memset(&pkgList[i], 0, sizeof(pkgListType));
    pkgList[i].time = time(NULL);
    pkgList[i].pkg = 1;
    pkgList[i].type = type;
    strcpy(pkgList[i].calsign, call);
    strcpy(pkgList[i].raw, raw);
    pkgList[i].calsign[10] = 0;
    // SerialLOG.print("NEW: ");
  }
  return i;
  // pkgLast_array[3] = pkgLast_array[2];
  // pkgLast_array[2] = pkgLast_array[1];
  // pkgLast_array[1] = pkgLast_array[0];
  // pkgLast_array[0] = i;
  // SerialLOG.print(i, DEC);
  // SerialLOG.println(call);
}

bool pkgTxPush(const char *info, int delay)
{
  char *ecs = strstr(info, ">");
  if (ecs == NULL)
    return false;
  // Replace
  for (int i = 0; i < PKGTXSIZE; i++)
  {
    if (txQueue[i].Active)
    {
      if (!(strncmp(&txQueue[i].Info[0], info, info - ecs)))
      {
        strcpy(&txQueue[i].Info[0], info);
        txQueue[i].Delay = delay;
        txQueue[i].timeStamp = millis();
        return true;
      }
    }
  }

  // Add
  for (int i = 0; i < PKGTXSIZE; i++)
  {
    if (txQueue[i].Active == false)
    {
      strcpy(&txQueue[i].Info[0], info);
      txQueue[i].Delay = delay;
      txQueue[i].Active = true;
      txQueue[i].timeStamp = millis();
      break;
    }
  }
  return true;
}

bool pkgTxSend()
{
  for (int i = 0; i < PKGTXSIZE; i++)
  {
    if (txQueue[i].Active)
    {
      int decTime = millis() - txQueue[i].timeStamp;
      if (decTime > txQueue[i].Delay)
      {
        txQueue[i].Active = false;
#ifdef SA818
        digitalWrite(POWER_PIN, config.rf_power); // RF Power LOW
#endif
        APRS_setPreamble(350L);
        APRS_sendTNC2Pkt(String(txQueue[i].Info)); // Send packet to RF
        for (int i = 0; i < 100; i++)
        {
          if (digitalRead(PTT_PIN))
            break;
          delay(50); // TOT 5sec
        }
        LED_Color(0, 0, 0);
        digitalWrite(POWER_PIN, 0); // set RF Power Low
#ifdef DEBUG_TNC
        // printTime();
        log_d("TX->RF: %s\n", txQueue[i].Info);
#endif
        return true;
      }
    }
  }
  return false;
}

uint8_t *packetData;
// ฟังชั่นถูกเรียกมาจาก ax25_decode
void aprs_msg_callback(struct AX25Msg *msg)
{
  AX25Msg pkg;

  memcpy(&pkg, msg, sizeof(AX25Msg));
  PacketBuffer.push(&pkg); // ใส่แพ็จเก็จจาก TNC ลงคิวบัพเฟอร์
  // String tnc2;
  //         packet2Raw(tnc2, pkg);
  //         log_d("RX: %s", tnc2.c_str());
}

void printTime()
{
  struct tm tmstruct;
  getLocalTime(&tmstruct, 500);
  Serial.print("[");
  Serial.print(tmstruct.tm_hour);
  Serial.print(":");
  Serial.print(tmstruct.tm_min);
  Serial.print(":");
  Serial.print(tmstruct.tm_sec);
  Serial.print("]");
}

uint8_t gwRaw[PKGLISTSIZE][66];
uint8_t gwRawSize[PKGLISTSIZE];
int gwRaw_count = 0, gwRaw_idx_rd = 0, gwRaw_idx_rw = 0;

void pushGwRaw(uint8_t *raw, uint8_t size)
{
  if (gwRaw_count > PKGLISTSIZE)
    return;
  if (++gwRaw_idx_rw >= PKGLISTSIZE)
    gwRaw_idx_rw = 0;
  if (size > 65)
    size = 65;
  memcpy(&gwRaw[gwRaw_idx_rw][0], raw, size);
  gwRawSize[gwRaw_idx_rw] = size;
  gwRaw_count++;
}

uint8_t popGwRaw(uint8_t *raw)
{
  uint8_t size = 0;
  if (gwRaw_count <= 0)
    return 0;
  if (++gwRaw_idx_rd >= PKGLISTSIZE)
    gwRaw_idx_rd = 0;
  size = gwRawSize[gwRaw_idx_rd];
  memcpy(raw, &gwRaw[gwRaw_idx_rd][0], size);
  if (gwRaw_count > 0)
    gwRaw_count--;
  return size;
}

String myBeacon(String Path)
{
  // sprintf(raw, "%s-%d>APEI%s:=%s%c%s%c%s%s", Config.mycallsign, Config.myssid, VERSION, Config.mylat, Config.mysymbol[0], Config.mylon, Config.mysymbol[1], Config.myphg, Config.mycomment);
  String rawData = "";
  String lat, lon;
  double nowLat, nowLng;
  char rawTNC[300];
  char aprs_table, aprs_symbol;
  char timestamp[10];
  struct tm tmstruct;
  double dist, course, speed;
  time_t nowTime;

  memset(rawTNC, 0, sizeof(rawTNC));
  getLocalTime(&tmstruct, 5000);
  // if (config.mygps==true) {
  // if (gps.location.isValid())
  // {
  sprintf(timestamp, "%02d%02d%02d%02d", (tmstruct.tm_mon + 1), tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min);
  time(&nowTime);
  // nowTime = gps.time.value();
  if (lastTimeStamp == 0)
    lastTimeStamp = nowTime;

  // lastTimeStamp=10000;
  // nowTime=lastTimeStamp+1800;
  time_t tdiff = nowTime - lastTimeStamp;

  // aprs_table = config.aprs_table;
  // aprs_symbol = config.aprs_symbol;
  if (config.trk_smartbeacon)
  {
    if (SB_SPEED < config.trk_lspeed)
    {
      aprs_table = config.trk_symstop[0];
      aprs_symbol = config.trk_symstop[1];
      SB_SPEED = 0;
    }
    else
    {
      aprs_table = config.trk_symmove[0];
      aprs_symbol = config.trk_symmove[1];
    }
  }
  else
  {
    tx_interval = config.trk_slowinterval = 60;
    aprs_table = config.igate_symbol[0];
    aprs_symbol = config.igate_symbol[1];
  }

  char object[10];
  memset(object, 0, 10);
  memcpy(object, config.igate_object, strlen(config.igate_object));
  object[9] = 0;

  if (gps.location.isValid())
  {
    nowLat = gps.location.lat();
    nowLng = gps.location.lng();

    // Distant= 17m
    //  LastLat=13.726841f;
    //  LastLng=100.434987f;
    //  nowLat=13.726841f;
    //  nowLng=100.434987f;

    // if (LastLng == 0)
    // 	LastLng = nowLng;
    // if (LastLat == 0)
    // 	LastLat = nowLat;
    uint16_t spdKnot;
    if (LastLng == 0 || LastLat == 0)
    {
      course = gps.course.deg();
      spdKnot = (uint16_t)gps.speed.knots();
    }
    else
    {
      dist = distance(LastLng, LastLat, nowLng, nowLat);
      course = direction(LastLng, LastLat, nowLng, nowLat);
      if (dist > 50.0F)
        dist = 0;
      if (tdiff > 10 && (nowTime > lastTimeStamp))
        speed = dist / ((double)tdiff / 3600);
      else
        speed = 0.0F;

      if (speed > 999)
        speed = 999.0F;
      // uint16_t spdMph=(uint16_t)(speed / 1.609344);
      spdKnot = (uint16_t)(speed * 0.53996F);
      if (spdKnot > 94)
        spdKnot = 0;
    }

    lat = deg2lat(nowLat);
    lon = deg2lon(nowLng);

    LastLat = nowLat;
    LastLng = nowLng;
    lastTimeStamp = nowTime;

    ESP_LOGE("GPS", "Aprs Compress");
    // Translate from semicircles to Base91 format
    char aprs_position[13];
    long latitude = semicircles((char *)lat.c_str(), (nowLat < 0));
    long longitude = semicircles((char *)lon.c_str(), (nowLat < 0));
    long ltemp = 1073741824L - latitude; // 90 degrees - latitude
    ESP_LOGE("GPS", "lat=%u lon=%u", latitude, longitude);
    memset(aprs_position, 0, sizeof(aprs_position));

    base91encode(ltemp, aprs_position);
    ltemp = 1073741824L + (longitude >> 1); // 180 degrees + longitude
    base91encode(ltemp, aprs_position + 4);
    // Encode heading
    uint8_t c = (uint8_t)(course / 4);
    // Scan lookup table to encode speed
    uint8_t s = (uint8_t)(log(spdKnot + 1) / log(1.08));
    if ((spdKnot <= 0.1) && (config.trk_altitude))
    {
      if (gps.altitude.isValid())
      {
        // Send Altitude
        aprs_position[11] = '!' + 0x30; // t current,GGA
        int alt = (int)gps.altitude.feet();
        int cs = (int)(log(alt) / log(1.002));
        c = (uint8_t)cs / 91;
        // c+='!';
        s = (uint8_t)(cs - ((int)c * 91));
        if (s > 91)
          s = 91;
        // s+='!';
        aprs_position[9] = '!' + c;  // c
        aprs_position[10] = '!' + s; // s
      }
      else
      {
        // Send Range
        aprs_position[11] = '!' + 0x00; // t current,GGA
        aprs_position[9] = '{';         // c = {
        if (!config.rf_power)
        {
          s = 10;
        }
        else
        {
          s = 30;
        }
        aprs_position[10] = '!' + s; // s
      }
    }
    else
    {
      aprs_position[9] = '!' + c;  // c
      aprs_position[10] = '!' + s; // s

      if (gps.location.isValid())
      {
        aprs_position[11] = '!' + 0x20 + 0x18 + 0x06; // t 0x20 1=current,0x18 11=RMC,0x06 110=Other tracker
      }
      else
      {
        aprs_position[11] = '!' + 0x00 + 0x18 + 0x06; // t
      }
    }
    aprs_position[12] = 0;
    // waveFlag = false;
    aprs_position[8] = aprs_symbol; // Symbol
    ESP_LOGE("GPS", "Compress=%s", aprs_position);
    if (strlen(object) >= 3)
    {
      sprintf(rawTNC, "%s-%d>APTWR1,%s:)%s!%c%s", config.aprs_mycall, config.aprs_ssid, Path.c_str(), object, aprs_table, aprs_position);
    }
    else
    {
      sprintf(rawTNC, "%s-%d>APTWR1,%s:!%c%s", config.aprs_mycall, config.aprs_ssid, Path.c_str(), aprs_table, aprs_position);
    }
  }
  else
  {
    sprintf(rawTNC, "%s-%d>APTWR1,%s:>%s ", config.aprs_mycall, config.aprs_ssid, Path.c_str(), object);
    // sprintf(rawTNC, "%s-%d>APDRH1,%s:)%s_%s", config.aprs_mycall, config.aprs_ssid, Path.c_str(), object, timestamp);
    //  sprintf(rawTNC, "%s-%d>APBT01-1%s:>%s_%s", config.aprs_mycall, config.aprs_ssid, config.aprs_path, object, timestamp);
  }

  char batStr[50];
  // uint16_t Vb = (uint16_t)(vbat * 100);
  sprintf(batStr, "SAT:%d,BAT:%0.2fV", gps.satellites.value(), vbat);
  strcat(rawTNC, batStr);
  rawData = String(rawTNC);
  return rawData;
}

#ifdef SA818
unsigned long SA818_Timeout = 0;
void SA818_INIT(bool boot)
{
#ifdef SR_FRS
  log_d("Radio Module SR_FRS Init");
#else
  log_d("Radio Module SA818/SA868 Init");
#endif
  if (boot)
  {
    SerialRF.begin(9600, SERIAL_8N1, SA868_RX_PIN, SA868_TX_PIN);
    pinMode(POWER_PIN, OUTPUT);
    pinMode(PULLDOWN_PIN, OUTPUT);
    // pinMode(SQL_PIN, INPUT);
    pinMode(PTT_PIN, OUTPUT);

    digitalWrite(PTT_PIN, LOW);
    digitalWrite(POWER_PIN, LOW);
    digitalWrite(PULLDOWN_PIN, LOW);
    delay(1000);
    digitalWrite(PTT_PIN, HIGH);
    digitalWrite(PULLDOWN_PIN, HIGH);
    delay(1500);
    SerialRF.println();
    delay(500);
  }
  SerialRF.println();
  delay(500);
  char str[100];
  if (config.sql_level > 8)
    config.sql_level = 8;
#ifdef SR_FRS
  sprintf(str, "AT+DMOSETGROUP=%01d,%0.4f,%0.4f,%d,%01d,%d,0", config.band, config.freq_tx + ((float)config.offset_tx / 1000000), config.freq_rx + ((float)config.offset_rx / 1000000), config.tone_rx, config.sql_level, config.tone_tx);
  SerialRF.println(str);
  delay(500);
  // Module auto power save setting
  SerialRF.println("AT+DMOAUTOPOWCONTR=1");
  delay(500);
  SerialRF.println("AT+DMOSETVOX=0");
  delay(500);
  SerialRF.println("AT+DMOSETMIC=6,0");
  delay(500);
  SerialRF.printf("AT+DMOSETVOLUME=%d\r\n", config.volume);
#else
  sprintf(str, "AT+DMOSETGROUP=%01d,%0.4f,%0.4f,%04d,%01d,%04d", config.band, config.freq_tx + ((float)config.offset_tx / 1000000), config.freq_rx + ((float)config.offset_rx / 1000000), config.tone_tx, config.sql_level, config.tone_rx);
  SerialRF.println(str);
  delay(500);
  SerialRF.println("AT+SETTAIL=0");
  delay(500);
  SerialRF.println("AT+SETFILTER=1,1,1");
  delay(500);
  SerialRF.printf("AT+DMOSETVOLUME=%d\r\n", config.volume);
#endif
  // SerialRF.println(str);
  // delay(500);
  // if (config.volume > 8)
  //     config.volume = 8;
  // SerialRF.printf("AT+DMOSETVOLUME=%d\r\n", config.volume);
}

void SA818_SLEEP()
{
  digitalWrite(POWER_PIN, LOW);
  digitalWrite(PULLDOWN_PIN, LOW);
  // SerialGPS.print("$PMTK161,0*28\r\n");
  AFSK_TimerEnable(false);
}

void SA818_CHECK()
{
  while (SerialRF.available() > 0)
    SerialRF.read();
  SerialRF.println("AT+DMOCONNECT");
  delay(100);
  if (SerialRF.available() > 0)
  {
    String ret = SerialRF.readString();
    if (ret.indexOf("DMOCONNECT") > 0)
    {
      SA818_Timeout = millis();
#ifdef DEBUG
      // Serial.println(SerialRF.readString());
      log_d("Radio SA818/SR_FRS Activated");
#endif
    }
  }
  else
  {
    log_d("Radio SA818/SR_FRS deActive");
    digitalWrite(POWER_PIN, LOW);
    digitalWrite(PULLDOWN_PIN, LOW);
    delay(500);
    SA818_INIT(true);
  }
  // SerialGPS.print("$PMTK161,0*28\r\n");
  // AFSK_TimerEnable(false);
}
#endif
// #ifdef SA818
// unsigned long SA818_Timeout = 0;
// void SA818_INIT(uint8_t HL)
// {

//     pinMode(0, INPUT);
//     pinMode(POWER_PIN, OUTPUT);
//     pinMode(PULLDOWN_PIN, OUTPUT);
//     pinMode(SQL_PIN, INPUT_PULLUP);

//     SerialRF.begin(9600, SERIAL_8N1, 14, 13);

//     digitalWrite(PULLDOWN_PIN, HIGH);
//     digitalWrite(POWER_PIN, LOW);
//     delay(500);
//     // AT+DMOSETGROUP=1,144.3900,144.3900,0,1,0,0
//     SerialRF.println("AT+DMOSETGROUP=0,144.3900,144.3900,0,1,0,0");
//     delay(10);
//     SerialRF.println("AT+DMOAUTOPOWCONTR=1");
//     delay(10);
//     SerialRF.println("AT+DMOSETVOLUME=9");
//     delay(10);
//     SerialRF.println("AT+DMOSETVOX=0");
//     delay(10);
//     SerialRF.println("AT+DMOSETMIC=8,0,0");
//     delay(100);
//     // AFSK_TimerEnable(true);
//     digitalWrite(POWER_PIN, HL);
// }

// void SA818_SLEEP()
// {
//     digitalWrite(POWER_PIN, LOW);
//     digitalWrite(PULLDOWN_PIN, LOW);
//     // SerialGPS.print("$PMTK161,0*28\r\n");
//     // AFSK_TimerEnable(false);
// }

// void SA818_CHECK()
// {
//     while (SerialRF.available() > 0)
//         SerialRF.read();
//     SerialRF.println("AT+DMOCONNECT");
//     delay(100);
//     if (SerialRF.available() > 0)
//     {
//         String ret = SerialRF.readString();
//         if (ret.indexOf("DMOCONNECT") > 0)
//         {
//             SA818_Timeout = millis();
// #ifdef DEBUG
//             // Serial.println(SerialRF.readString());
//             Serial.println("SA818 Activated");
// #endif
//         }
//     }
//     else
//     {
//         Serial.println("SA818 deActive");
//         digitalWrite(POWER_PIN, LOW);
//         digitalWrite(PULLDOWN_PIN, LOW);
//         delay(500);
//         SA818_INIT(LOW);
//     }
//     // SerialGPS.print("$PMTK161,0*28\r\n");
//     // AFSK_TimerEnable(false);
// }
// #endif

WiFiClient aprsClient;

boolean APRSConnect()
{
  // Serial.println("Connect TCP Server");
  String login = "";
  int cnt = 0;
  uint8_t con = aprsClient.connected();
  // Serial.println(con);
  if (con <= 0)
  {
    if (!aprsClient.connect(config.aprs_host, config.aprs_port)) // เชื่อมต่อกับเซิร์ฟเวอร์ TCP
    {
      // Serial.print(".");
      delay(100);
      cnt++;
      if (cnt > 50) // วนร้องขอการเชื่อมต่อ 50 ครั้ง ถ้าไม่ได้ให้รีเทิร์นฟังค์ชั่นเป็น False
        return false;
    }

    if (strcmp("NOCALL", config.aprs_mycall) == 0)
    {
      config.igate_en = false;
      return false;
    }

    // ขอเชื่อมต่อกับ aprsc
    if (strlen(config.igate_object) >= 3)
    {
      uint16_t passcode = aprsParse.passCode(config.igate_object);
      login = "user " + String(config.igate_object) + " pass " + String(passcode, DEC) + " vers ESP32APRS T-TWR+ " + String(VERSION) + " filter " + String(config.aprs_filter);
    }
    else
    {
      uint16_t passcode = aprsParse.passCode(config.aprs_mycall);
      if (config.aprs_ssid == 0)
        login = "user " + String(config.aprs_mycall) + " pass " + String(passcode, DEC) + " vers ESP32APRS T-TWR+ " + String(VERSION) + " filter " + String(config.aprs_filter);
      else
        login = "user " + String(config.aprs_mycall) + "-" + String(config.aprs_ssid) + " pass " + String(passcode, DEC) + " vers ESP32APRS T-TWR+ " + String(VERSION) + " filter " + String(config.aprs_filter);
    }
    aprsClient.println(login);
    // Serial.println(login);
    // Serial.println("Success");
    delay(500);
  }
  return true;
}

// boolean dataAct = false;
// uint8_t x = 0, y = 0;
// char str[300];
// String callSign;
// char callDest[8];
// char path[64];
// char raw[128];
// int posNow = 0;
// int timeHalfSec = 0;
String NMEA;

// String myBeacon(String Path)
// {
//     // sprintf(raw, "%s-%d>APEI%s:=%s%c%s%c%s%s", config.aprs_mycall, config.aprs_ssid, VERSION, config.mylat, config.mysymbol[0], config.mylon, config.mysymbol[1], config.myphg, config.mycomment);
//     String rawData = "";
//     String lat, lon;
//     char rawTNC[300];
//     char aprs_table, aprs_symbol;

//     if (config.mygps == true)
//     {
//         if (gps.location.isValid())
//         {
//             lat = aprsParse.deg2lat(gps.location.lat());
//             lon = aprsParse.deg2lon(gps.location.lng());
//             if (config.trk_smartbeacon)
//             {
//                 if (SB_SPEED < config.trk_lspeed)
//                 {
//                     aprs_table = config.trk_symstop[0];
//                     aprs_symbol = config.trk_symstop[1];
//                     SB_SPEED = 0;
//                 }
//                 else
//                 {
//                     aprs_table = config.trk_symmove[0];
//                     aprs_symbol = config.trk_symmove[1];
//                 }
//                 if (config.trk_speed && gps.speed.isValid())
//                 {
//                     if (config.trk_altitude)
//                         sprintf(rawTNC, "%s-%d>APEI%s%s:!%s%c%s%c%03d/%03d/A=%06d%s", config.aprs_mycall, config.aprs_ssid, VERSION, Path.c_str(), lat.c_str(), aprs_table, lon.c_str(), aprs_symbol, (uint16_t)gps.course.deg(), (uint16_t)gps.speed.mph(), (int16_t)gps.altitude.feet(), config.aprs_comment);
//                     else
//                         sprintf(rawTNC, "%s-%d>APEI%s%s:!%s%c%s%c%03d/%03d%s", config.aprs_mycall, config.aprs_ssid, VERSION, Path.c_str(), lat.c_str(), aprs_table, lon.c_str(), aprs_symbol, (uint16_t)gps.course.deg(), (uint16_t)gps.speed.mph(), config.aprs_comment);
//                 }
//                 else
//                 {
//                     sprintf(rawTNC, "%s-%d>APEI%s%s:!%s%c%s%c%s", config.aprs_mycall, config.aprs_ssid, VERSION, Path.c_str(), lat.c_str(), aprs_table, lon.c_str(), aprs_symbol, config.aprs_comment);
//                 }
//             }
//             else
//             {
//                 tx_interval = config.trk_slowinterval = 600;
//                 send_aprs_table = config.mysymbol[0];
//                 send_aprs_symbol = config.mysymbol[1];
//                 sprintf(rawTNC, "%s-%d>APEI%s%s:=%s%c%s%c%s", config.aprs_mycall, config.aprs_ssid, VERSION, Path.c_str(), lat.c_str(), config.mysymbol[0], lon.c_str(), config.mysymbol[1], config.aprs_comment);
//             }
//         }
//         else
//         {
//             tx_interval = config.trk_slowinterval;
//             sprintf(rawTNC, ">Wait GPS Active.");
//         }
//         send_aprs_table = aprs_table;
//         send_aprs_symbol = aprs_symbol;
//     }
//     else
//     {
//         lat = aprsParse.deg2lat(config.gps_lat);
//         lon = aprsParse.deg2lon(config.gps_lon);
//         tx_interval = config.trk_slowinterval = 600;
//         send_aprs_table = config.mysymbol[0];
//         send_aprs_symbol = config.mysymbol[1];
//         sprintf(rawTNC, "%s-%d>APEI%s%s:=%s%c%s%c%s", config.aprs_mycall, config.aprs_ssid, VERSION, Path.c_str(), lat.c_str(), config.mysymbol[0], lon.c_str(), config.mysymbol[1], config.aprs_comment);
//     }
//     // sprintf(rawTNC, "%s-%d>APEI%s:=%s%c%s%c%s%s", config.aprs_mycall, config.aprs_ssid, VERSION, lat.c_str(), config.mysymbol[0], lon.c_str(), config.mysymbol[1], config.myphg, config.mycomment);
//     return String(rawTNC);
// }

// boolean confirmRequestPending = true;

// void BTConfirmRequestCallback(uint32_t numVal)
// {
//   confirmRequestPending = true;
//   // log_d(numVal);
// }

// void BTAuthCompleteCallback(boolean success)
// {
//   confirmRequestPending = false;
//   if (success)
//   {
//     log_d("Pairing success!!");
//   }
//   else
//   {
//     log_d("Pairing failed, rejected by user!!");
//   }
// }

// void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
// {
//   if (event == ESP_SPP_SRV_OPEN_EVT)
//   {
//     log_d("Client Connected");
//   }
// }

bool powerEvent = true;

void powerSave()
{
  if (config.trk_smartbeacon)
  {
    display.dim(true);
  }
  else
  {
    // if (digitalRead(PWR_VDD))
    // {
    //   powerEvent = false;
    //   display.clearDisplay();
    //   display.display();
    //   // if (!config.trk_smartbeacon)
    //   //{
    //   digitalWrite(PWR_VDD, LOW);
    //   //}
    // }
  }
}

void powerWakeup()
{
  if (config.trk_smartbeacon)
  {
    display.dim(false);
  }
  else
  {
    // if (!digitalRead(PWR_VDD))
    // {
    //   powerEvent = true;
    //   digitalWrite(PWR_VDD, HIGH);
    //   delay(10);
    //   display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false);
    //   display.clearDisplay();
    //   display.display();
    //   GPS_INIT();
    // }
  }
}

bool powerStatus()
{
  return 1;
  // digitalRead(PWR_VDD);
}

void setupPower()
{
  bool result = PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
  if (result == false)
  {
    while (1)
    {
      Serial.println("PMU is not online...");
      delay(500);
    }
  }

  // Set the minimum common working voltage of the PMU VBUS input,
  // below this value will turn off the PMU
  PMU.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_3V88);

  // Set the maximum current of the PMU VBUS input,
  // higher than this value will turn off the PMU
  PMU.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_2000MA);

  // Get the VSYS shutdown voltage
  uint16_t vol = PMU.getSysPowerDownVoltage();
  Serial.printf("->  getSysPowerDownVoltage:%u\n", vol);

  // Set VSY off voltage as 2600mV , Adjustment range 2600mV ~ 3300mV
  PMU.setSysPowerDownVoltage(2600);

  //! DC1 ESP32S3 Core VDD , Don't change
  // PMU.setDC1Voltage(3300);

  //! DC3 Radio & Pixels VDD , Don't change
  PMU.setDC3Voltage(3400);

  //! ALDO2 MICRO TF Card VDD, Don't change
  PMU.setALDO2Voltage(3300);

  //! ALDO4 GNSS VDD, Don't change
  PMU.setALDO4Voltage(3300);

  //! BLDO1 MIC VDD, Don't change
  PMU.setBLDO1Voltage(3300);

  //! The following supply voltages can be controlled by the user
  // DC5 IMAX=2A
  // 1200mV
  // 1400~3700mV,100mV/step,24steps
  PMU.setDC5Voltage(3300);

  // ALDO1 IMAX=300mA
  // 500~3500mV, 100mV/step,31steps
  PMU.setALDO1Voltage(3300);

  // ALDO3 IMAX=300mA
  // 500~3500mV, 100mV/step,31steps
  PMU.setALDO3Voltage(3300);

  // BLDO2 IMAX=300mA
  // 500~3500mV, 100mV/step,31steps
  PMU.setBLDO2Voltage(3300);

  //! END

  // Turn on the power that needs to be used
  //! DC1 ESP32S3 Core VDD , Don't change
  // PMU.enableDC3();

  //! External pin power supply
  PMU.enableDC5();
  PMU.enableALDO1();
  PMU.enableALDO3();
  PMU.enableBLDO2();

  //! ALDO2 MICRO TF Card VDD
  PMU.enableALDO2();

  //! ALDO4 GNSS VDD
  PMU.enableALDO4();

  //! BLDO1 MIC VDD
  PMU.enableBLDO1();

  //! DC3 Radio & Pixels VDD
  PMU.enableDC3();

  // power off when not in use
  PMU.disableDC2();
  PMU.disableDC4();
  PMU.disableCPUSLDO();
  PMU.disableDLDO1();
  PMU.disableDLDO2();

  Serial.println("DCDC=======================================================================");
  Serial.printf("DC1  : %s   Voltage:%u mV \n", PMU.isEnableDC1() ? "+" : "-", PMU.getDC1Voltage());
  Serial.printf("DC2  : %s   Voltage:%u mV \n", PMU.isEnableDC2() ? "+" : "-", PMU.getDC2Voltage());
  Serial.printf("DC3  : %s   Voltage:%u mV \n", PMU.isEnableDC3() ? "+" : "-", PMU.getDC3Voltage());
  Serial.printf("DC4  : %s   Voltage:%u mV \n", PMU.isEnableDC4() ? "+" : "-", PMU.getDC4Voltage());
  Serial.printf("DC5  : %s   Voltage:%u mV \n", PMU.isEnableDC5() ? "+" : "-", PMU.getDC5Voltage());
  Serial.println("ALDO=======================================================================");
  Serial.printf("ALDO1: %s   Voltage:%u mV\n", PMU.isEnableALDO1() ? "+" : "-", PMU.getALDO1Voltage());
  Serial.printf("ALDO2: %s   Voltage:%u mV\n", PMU.isEnableALDO2() ? "+" : "-", PMU.getALDO2Voltage());
  Serial.printf("ALDO3: %s   Voltage:%u mV\n", PMU.isEnableALDO3() ? "+" : "-", PMU.getALDO3Voltage());
  Serial.printf("ALDO4: %s   Voltage:%u mV\n", PMU.isEnableALDO4() ? "+" : "-", PMU.getALDO4Voltage());
  Serial.println("BLDO=======================================================================");
  Serial.printf("BLDO1: %s   Voltage:%u mV\n", PMU.isEnableBLDO1() ? "+" : "-", PMU.getBLDO1Voltage());
  Serial.printf("BLDO2: %s   Voltage:%u mV\n", PMU.isEnableBLDO2() ? "+" : "-", PMU.getBLDO2Voltage());
  Serial.println("===========================================================================");

  // Set the time of pressing the button to turn off
  PMU.setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
  uint8_t opt = PMU.getPowerKeyPressOffTime();
  Serial.print("PowerKeyPressOffTime:");
  switch (opt)
  {
  case XPOWERS_POWEROFF_4S:
    Serial.println("4 Second");
    break;
  case XPOWERS_POWEROFF_6S:
    Serial.println("6 Second");
    break;
  case XPOWERS_POWEROFF_8S:
    Serial.println("8 Second");
    break;
  case XPOWERS_POWEROFF_10S:
    Serial.println("10 Second");
    break;
  default:
    break;
  }
  // Set the button power-on press time
  PMU.setPowerKeyPressOnTime(XPOWERS_POWERON_128MS);
  opt = PMU.getPowerKeyPressOnTime();
  Serial.print("PowerKeyPressOnTime:");
  switch (opt)
  {
  case XPOWERS_POWERON_128MS:
    Serial.println("128 Ms");
    break;
  case XPOWERS_POWERON_512MS:
    Serial.println("512 Ms");
    break;
  case XPOWERS_POWERON_1S:
    Serial.println("1 Second");
    break;
  case XPOWERS_POWERON_2S:
    Serial.println("2 Second");
    break;
  default:
    break;
  }

  Serial.println("===========================================================================");
  // It is necessary to disable the detection function of the TS pin on the board
  // without the battery temperature detection function, otherwise it will cause abnormal charging
  PMU.disableTSPinMeasure();

  // Enable internal ADC detection
  PMU.enableBattDetection();
  PMU.enableVbusVoltageMeasure();
  PMU.enableBattVoltageMeasure();
  PMU.enableSystemVoltageMeasure();

  /*
    The default setting is CHGLED is automatically controlled by the PMU.
  - XPOWERS_CHG_LED_OFF,
  - XPOWERS_CHG_LED_BLINK_1HZ,
  - XPOWERS_CHG_LED_BLINK_4HZ,
  - XPOWERS_CHG_LED_ON,
  - XPOWERS_CHG_LED_CTRL_CHG,
  * */
  PMU.setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);

  // Force add pull-up
  pinMode(PMU_IRQ, INPUT_PULLUP);
  // attachInterrupt(PMU_IRQ, setFlag, FALLING);

  // Disable all interrupts
  PMU.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
  // Clear all interrupt flags
  PMU.clearIrqStatus();
  // Enable the required interrupt function
  PMU.enableIRQ(
      XPOWERS_AXP2101_BAT_INSERT_IRQ | XPOWERS_AXP2101_BAT_REMOVE_IRQ |    // BATTERY
      XPOWERS_AXP2101_VBUS_INSERT_IRQ | XPOWERS_AXP2101_VBUS_REMOVE_IRQ |  // VBUS
      XPOWERS_AXP2101_PKEY_SHORT_IRQ | XPOWERS_AXP2101_PKEY_LONG_IRQ |     // POWER KEY
      XPOWERS_AXP2101_BAT_CHG_DONE_IRQ | XPOWERS_AXP2101_BAT_CHG_START_IRQ // CHARGE
  );

  // Set the precharge charging current
  PMU.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_150MA);

  // Set constant current charge current limit
  //! Using inferior USB cables and adapters will not reach the maximum charging current.
  //! Please pay attention to add a suitable heat sink above the PMU when setting the charging current to 1A
  PMU.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_1000MA);

  // Set stop charging termination current
  PMU.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_150MA);

  // Set charge cut-off voltage
  PMU.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);

  // Disable the PMU long press shutdown function
  PMU.disableLongPressShutdown();

  // Get charging target current
  const uint16_t currTable[] = {
      0, 0, 0, 0, 100, 125, 150, 175, 200, 300, 400, 500, 600, 700, 800, 900, 1000};
  uint8_t val = PMU.getChargerConstantCurr();
  Serial.print("Val = ");
  Serial.println(val);
  Serial.print("Setting Charge Target Current : ");
  Serial.println(currTable[val]);

  // Get charging target voltage
  const uint16_t tableVoltage[] = {
      0, 4000, 4100, 4200, 4350, 4400, 255};
  val = PMU.getChargeTargetVoltage();
  Serial.print("Setting Charge Target Voltage : ");
  Serial.println(tableVoltage[val]);
}

void setupSDCard()
{
  // SPI Bus
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  if (SD.begin(SD_CS, SPI))
  {
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE)
    {
      Serial.println("No SD_MMC card attached");
      return;
    }
    else
    {
      Serial.print("SD_MMC Card Type: ");
      if (cardType == CARD_MMC)
      {
        Serial.println("MMC");
      }
      else if (cardType == CARD_SD)
      {
        Serial.println("SDSC");
      }
      else if (cardType == CARD_SDHC)
      {
        Serial.println("SDHC");
      }
      else
      {
        Serial.println("UNKNOWN");
      }
      uint32_t cardSize = SD.cardSize() / (1024 * 1024);
      uint32_t cardTotal = SD.totalBytes() / (1024 * 1024);
      uint32_t cardUsed = SD.usedBytes() / (1024 * 1024);
      Serial.printf("SD Card Size: %lu MB\n", cardSize);
      Serial.printf("Total space: %lu MB\n", cardTotal);
      Serial.printf("Used space: %lu MB\n", cardUsed);
    }
  }
}

long oledSleepTimeout = 0;
bool showDisp = false;
// uint8_t curTab=0;

char *htmlBuffer;
void setup()
{
  // htmlBuffer = (char*)ps_malloc(1000000);
  byte *ptr;
#ifdef BOARD_HAS_PSRAM
  pkgList = (pkgListType *)ps_malloc(sizeof(pkgListType) * PKGLISTSIZE);
  Telemetry = (TelemetryType *)ps_malloc(sizeof(TelemetryType) * TLMLISTSIZE);
  txQueue = (txQueueType *)ps_malloc(sizeof(txQueueType) * PKGTXSIZE);
#endif

  memset(pkgList, 0, sizeof(pkgListType) * PKGLISTSIZE);
  memset(Telemetry, 0, sizeof(TelemetryType) * TLMLISTSIZE);
  memset(txQueue, 0, sizeof(txQueueType) * PKGTXSIZE);

  pinMode(ENCODER_OK_PIN, INPUT_PULLUP);
  // pinMode(SQL_PIN, INPUT);
  //  pinMode(0, INPUT_PULLUP); // BOOT Button
  // pinMode(LED_RX, OUTPUT);
  //  pinMode(LED_TX, OUTPUT);
  //  pinMode(PWR_VDD, OUTPUT);
  //  digitalWrite(PWR_VDD, HIGH);
  //  pinMode(39, INPUT);

  // digitalWrite(keyA, LOW);
  // digitalWrite(keyB, LOW);
  // digitalWrite(keyPush, HIGH);

  // Set up serial port
  Serial.begin(115200); // debug
  // Serial.setRxBufferSize(256);
  //  SerialGPS.setRxBufferSize(256);
  //  SerialGPS.begin(9600, SERIAL_8N1, 18, 19);
  //   SerialTNC.setRxBufferSize(256);
  //   SerialTNC.begin(9600, SERIAL_8N1, 16, 17);

  // Serial.println();
  log_d("Start ESP32IGate V%s", String(VERSION).c_str());
  log_d("Push BOOT after 3 sec for Factory Default config.");
  log_d("Total heap: %d", ESP.getHeapSize());

  Wire.begin(I2C_SDA, I2C_SCL, 400000L);

  // Setup Power PMU AXP2101
  setupPower();
  setupSDCard();

#ifdef OLED
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false); // initialize with the I2C addr 0x3C (for the 128x64)
  // Initialising the UI will init the display too.
  // clear the display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setTextSize(1);
  display.setFont(&FreeSansBold9pt7b);
  display.setCursor(0, 15);
  display.print("APRS");
  display.setCursor(55, 32);
  display.print("T-TWR+");
  display.drawYBitmap(0, 16, LOGO, 48, 48, WHITE);
  display.drawRoundRect(52, 16, 75, 22, 3, WHITE);

  display.setFont();
  display.setTextColor(WHITE);

  display.setCursor(55, 40);
  display.print("Version " + String(VERSION));
  display.setCursor(60, 55);
  display.print("Copy@2023");
  // display.drawLine(10, 30, 110, 30, WHITE);
  // display.setCursor(1, 40);
  // display.print("Push B Factory reset");
  // topBar(-100);
  display.display();
#endif

  if (!EEPROM.begin(EEPROM_SIZE))
  {
    log_d("failed to initialise EEPROM"); // delay(100000);
  }

#ifdef OLED
  delay(1000);
  // digitalWrite(LED_TX, HIGH);
  // display.setCursor(50, 50);
  // display.print("3 Sec");
  // display.display();
  // delay(1000);
  // digitalWrite(LED_RX, HIGH);
  // display.setCursor(40, 50);
  // display.print("        ");
  // display.display();
  // display.setCursor(50, 50);
  // display.print("2 Sec");
  // display.display();
  // delay(1000);
  // display.setCursor(40, 50);
  // display.print("        ");
  // display.display();
  // display.setCursor(50, 50);
  // display.print("1 Sec");
  // display.display();
#else
  delay(1000);
  digitalWrite(LED_TX, HIGH);
  delay(1000);
  digitalWrite(LED_RX, HIGH);
  delay(1000);
#endif
  if (digitalRead(ENCODER_OK_PIN) == LOW)
  {
    defaultConfig();
    log_d("Manual Default configure!");
#ifdef OLED
    display.clearDisplay();
    display.setCursor(10, 22);
    display.print("Factory Reset!");
    display.display();
#endif
    //   while (digitalRead(0) == LOW)
    //   {
    //     delay(500);
    //     digitalWrite(LED_TX, LOW);
    //     digitalWrite(LED_RX, LOW);
    //     delay(500);
    //     digitalWrite(LED_TX, HIGH);
    //     digitalWrite(LED_RX, HIGH);
    //   }
  }

  // digitalWrite(LED_TX, LOW);
  // digitalWrite(LED_RX, LOW);

  // ตรวจสอบคอนฟิกซ์ผิดพลาด
  ptr = (byte *)&config;
  EEPROM.readBytes(1, ptr, sizeof(Configuration));
  uint8_t chkSum = checkSum(ptr, sizeof(Configuration));
  log_d("EEPROM Check %0Xh=%0Xh(%dByte)\n", EEPROM.read(0), chkSum, sizeof(Configuration));
  if (EEPROM.read(0) != chkSum)
  {
    log_d("Config EEPROM Error!");
    display.clearDisplay();
    display.setCursor(12, 22);
    display.print("EEPROM Error!");
    display.setCursor(10, 42);
    display.print("Factory Default");
    display.display();
    defaultConfig();
  }
  input_HPF = config.input_hpf;

  if (config.bt_master == true)
  {
    // Create the BLE Device
    BLEDevice::init(config.bt_name);

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(config.bt_uuid);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
        config.bt_uuid_tx,
        BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
        config.bt_uuid_rx,
        BLECharacteristic::PROPERTY_WRITE);
    pRxCharacteristic->setCallbacks(new MyCallbacks());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
  }

#ifdef SA818
  SA818_INIT(true);
#endif

#ifdef OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.display();
#endif

  // pkgListType pkgList[PKGLISTSIZE];
  // pkgList = (pkgListType*)ps_malloc(sizeof(pkgListType)*PKGLISTSIZE);

  log_d("Free heap: %d", ESP.getFreeHeap());
  log_d("Total PSRAM: %d", ESP.getPsramSize());
  log_d("Free PSRAM: %d", ESP.getFreePsram());
  // log_d("Used PSRAM: %d", ESP.getPsramSize() - ESP.getFreePsram());

  // enableLoopWDT();
  // enableCore0WDT();
  // enableCore1WDT();
  firstGpsTime = true;

  // Task 1
  xTaskCreatePinnedToCore(
      taskAPRS,        /* Function to implement the task */
      "taskAPRS",      /* Name of the task */
      8192,            /* Stack size in words */
      NULL,            /* Task input parameter */
      1,               /* Priority of the task */
      &taskAPRSHandle, /* Task handle. */
      0);              /* Core where the task should run */

  xTaskCreatePinnedToCore(
      taskGPS,        /* Function to implement the task */
      "taskGPS",      /* Name of the task */
      4096,           /* Stack size in words */
      NULL,           /* Task input parameter */
      2,              /* Priority of the task */
      &taskGpsHandle, /* Task handle. */
      0);             /* Core where the task should run */

  if (config.wifi_mode != 0)
  {
    // Task 2
    xTaskCreatePinnedToCore(
        taskNetwork,        /* Function to implement the task */
        "taskNetwork",      /* Name of the task */
        8192,               /* Stack size in words */
        NULL,               /* Task input parameter */
        2,                  /* Priority of the task */
        &taskNetworkHandle, /* Task handle. */
        1);                 /* Core where the task should run */
  }
  // Task 3
  xTaskCreatePinnedToCore(
      mainDisp,           /* Function to implement the task */
      "mainDisplay",      /* Name of the task */
      16384,              /* Stack size in words */
      NULL,               /* Task input parameter */
      1,                  /* Priority of the task */
      &mainDisplayHandle, /* Task handle. */
      1);                 /* Core where the task should run */
}

int pkgCount = 0;

float conv_coords(float in_coords)
{
  // Initialize the location.
  float f = in_coords;
  // Get the first two digits by turning f into an integer, then doing an integer divide by 100;
  // firsttowdigits should be 77 at this point.
  int firsttwodigits = ((int)f) / 100; // This assumes that f < 10000.
  float nexttwodigits = f - (float)(firsttwodigits * 100);
  float theFinalAnswer = (float)(firsttwodigits + nexttwodigits / 60.0);
  return theFinalAnswer;
}

void DD_DDDDDtoDDMMSS(float DD_DDDDD, int *DD, int *MM, int *SS)
{

  *DD = (int)DD_DDDDD;                       // сделали из 37.45545 это 37 т.е. Градусы
  *MM = (int)((DD_DDDDD - *DD) * 60);        // получили минуты
  *SS = ((DD_DDDDD - *DD) * 60 - *MM) * 100; // получили секунды
}

String compress_position(double nowLat, double nowLng, int alt_feed, double course, uint16_t spdKnot, char table, char symbol, bool gps)
{
  String str_comp = "";
  String lat, lon;
  lat = deg2lat(nowLat);
  lon = deg2lon(nowLng);
  ESP_LOGE("GPS", "Aprs Compress");
  // Translate from semicircles to Base91 format
  char aprs_position[13];
  long latitude = semicircles((char *)lat.c_str(), (nowLat < 0));
  long longitude = semicircles((char *)lon.c_str(), (nowLat < 0));
  long ltemp = 1073741824L - latitude; // 90 degrees - latitude
  ESP_LOGE("GPS", "lat=%u lon=%u", latitude, longitude);
  memset(aprs_position, 0, sizeof(aprs_position));

  base91encode(ltemp, aprs_position);
  ltemp = 1073741824L + (longitude >> 1); // 180 degrees + longitude
  base91encode(ltemp, aprs_position + 4);
  // Encode heading
  uint8_t c = (uint8_t)(course / 4);
  // Scan lookup table to encode speed
  uint8_t s = (uint8_t)(log(spdKnot + 1) / log(1.08));
  if ((spdKnot <= 0.1) && (alt_feed > 0))
  {
    if (gps)
    {
      // Send Altitude
      aprs_position[11] = '!' + 0x30; // t current,GGA
      int alt = (int)alt_feed;
      int cs = (int)(log(alt) / log(1.002));
      c = (uint8_t)cs / 91;
      // c+='!';
      s = (uint8_t)(cs - ((int)c * 91));
      if (s > 91)
        s = 91;
      // s+='!';
      aprs_position[9] = '!' + c;  // c
      aprs_position[10] = '!' + s; // s
    }
    else
    {
      // Send Range
      aprs_position[11] = '!' + 0x00; //
      aprs_position[9] = '{';         // c = {
      if (!config.rf_power)
      {
        s = 10;
      }
      else
      {
        s = 30;
      }
      aprs_position[10] = '!' + s; // s
    }
  }
  else
  {
    aprs_position[9] = '!' + c;  // c
    aprs_position[10] = '!' + s; // s

    if (gps)
    {
      aprs_position[11] = '!' + 0x20 + 0x18 + 0x06; // t 0x20 1=current,0x18 11=RMC,0x06 110=Other tracker
    }
    else
    {
      aprs_position[11] = '!' + 0x00 + 0x18 + 0x06; // t
    }
  }
  aprs_position[12] = 0;
  // waveFlag = false;
  aprs_position[8] = symbol; // Symbol
  str_comp = String(table) + String(aprs_position);
  return str_comp;
}

String send_fix_location()
{
  String tnc2Raw = "";
  int lat_dd, lat_mm, lat_ss, lon_dd, lon_mm, lon_ss;
  char strtmp[300], loc[30];
  memset(strtmp, 0, 300);
  DD_DDDDDtoDDMMSS(config.igate_lat, &lat_dd, &lat_mm, &lat_ss);
  DD_DDDDDtoDDMMSS(config.igate_lon, &lon_dd, &lon_mm, &lon_ss);
  // sprintf(loc, "!%02d%02d.%02dN%c%03d%02d.%02dE%c", lat_dd, lat_mm, lat_ss, config.aprs_table, lon_dd, lon_mm, lon_ss, config.aprs_symbol);
  if (strlen(config.igate_object) >= 3)
  {
    sprintf(loc, ")%s!%02d%02d.%02dN%c%03d%02d.%02dE%c", config.igate_object, lat_dd, lat_mm, lat_ss, config.igate_symbol[0], lon_dd, lon_mm, lon_ss, config.igate_symbol[1]);
  }
  else
  {
    sprintf(loc, "!%02d%02d.%02dN%c%03d%02d.%02dE%c", lat_dd, lat_mm, lat_ss, config.igate_symbol[0], lon_dd, lon_mm, lon_ss, config.igate_symbol[1]);
  }
  if (config.aprs_ssid == 0)
    sprintf(strtmp, "%s>APTWR", config.aprs_mycall);
  else
    sprintf(strtmp, "%s-%d>APTWR", config.aprs_mycall, config.aprs_ssid);
  tnc2Raw = String(strtmp);
  if (config.igate_path[0] != 0)
  {
    tnc2Raw += ",";
    tnc2Raw += String(config.igate_path);
  }
  tnc2Raw += ":";
  char object[10];
  memset(object, 0, 10);
  memcpy(object, config.igate_object, strlen(config.igate_object));
  object[9] = 0;
  if (strlen(object) > 3)
  {
    tnc2Raw += ")" + String(object);
  }
  tnc2Raw += String(loc);
  tnc2Raw += String(config.igate_comment);
  return tnc2Raw;
}

String trk_gps_postion(String comment)
{
  String rawData = "";
  String lat, lon;
  double nowLat, nowLng;
  char rawTNC[300];
  char aprs_table, aprs_symbol;
  char timestamp[10];
  struct tm tmstruct;
  double dist, course, speed;
  time_t nowTime;

  memset(rawTNC, 0, sizeof(rawTNC));
  getLocalTime(&tmstruct, 5000);
  // if (config.mygps==true) {
  // if (gps.location.isValid())
  // {
  sprintf(timestamp, "%02d%02d%02d%02d", (tmstruct.tm_mon + 1), tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min);
  time(&nowTime);
  // nowTime = gps.time.value();
  if (lastTimeStamp == 0)
    lastTimeStamp = nowTime;

  // lastTimeStamp=10000;
  // nowTime=lastTimeStamp+1800;
  time_t tdiff = nowTime - lastTimeStamp;

  // aprs_table = config.aprs_table;
  // aprs_symbol = config.aprs_symbol;
  if (config.trk_smartbeacon)
  {
    if (SB_SPEED < config.trk_lspeed)
    {
      aprs_table = config.trk_symstop[0];
      aprs_symbol = config.trk_symstop[1];
      SB_SPEED = 0;
    }
    else
    {
      aprs_table = config.trk_symmove[0];
      aprs_symbol = config.trk_symmove[1];
    }
  }
  else
  {
    aprs_table = config.trk_symbol[0];
    aprs_symbol = config.trk_symbol[1];
  }

  char object[10];
  memset(object, 0, 10);
  memcpy(object, config.trk_object, strlen(config.trk_object));
  object[9] = 0;

  if (gps.location.isValid())
  {
    nowLat = gps.location.lat();
    nowLng = gps.location.lng();

    uint16_t spdKnot;
    if (LastLng == 0 || LastLat == 0)
    {
      course = gps.course.deg();
      spdKnot = (uint16_t)gps.speed.knots();
    }
    else
    {
      dist = distance(LastLng, LastLat, nowLng, nowLat);
      course = direction(LastLng, LastLat, nowLng, nowLat);
      if (dist > 50.0F)
        dist = 0;
      if (tdiff > 10 && (nowTime > lastTimeStamp))
        speed = dist / ((double)tdiff / 3600);
      else
        speed = 0.0F;

      if (speed > 999)
        speed = 999.0F;
      // uint16_t spdMph=(uint16_t)(speed / 1.609344);
      spdKnot = (uint16_t)(speed * 0.53996F);
      if (spdKnot > 94)
        spdKnot = 0;
    }

    // lat = deg2lat(nowLat);
    // lon = deg2lon(nowLng);

    LastLat = nowLat;
    LastLng = nowLng;
    lastTimeStamp = nowTime;

    char strAltitude[10];
    memset(strAltitude, 0, sizeof(strAltitude));
    if (config.trk_altitude)
    {
      sprintf(strAltitude, "/A=%06d", (int)gps.altitude.feet());
    }

    if (config.trk_compress)
    { // Compress DATA

      String compPosition = compress_position(nowLat, nowLng, 0, course, spdKnot, aprs_table, aprs_symbol, gps.location.isValid());
      // ESP_LOGE("GPS", "Compress=%s", aprs_position);
      if (strlen(object) >= 3)
      {
        // sprintf(rawTNC, "%s-%d>APTWR1,%s:)%s!%c%s", config.aprs_mycall, config.aprs_ssid, Path.c_str(), object, aprs_table, aprs_position);
        sprintf(rawTNC, ")%s!%s%s", object, compPosition.c_str(), strAltitude);
      }
      else
      {
        sprintf(rawTNC, "!%s%s", compPosition.c_str(), strAltitude);
      }
    }
    else
    { // None compress DATA
      int lat_dd, lat_mm, lat_ss, lon_dd, lon_mm, lon_ss;
      DD_DDDDDtoDDMMSS(nowLat, &lat_dd, &lat_mm, &lat_ss);
      DD_DDDDDtoDDMMSS(nowLng, &lon_dd, &lon_mm, &lon_ss);
      char csd_spd[8];
      memset(csd_spd, 0, sizeof(csd_spd));
      if (config.trk_speed)
      {
        sprintf(csd_spd, "%03d/%03d", (int)gps.course.deg(), (int)gps.speed.knots());
      }
      if (strlen(object) >= 3)
      {
        char object[10];
        memset(object, 0, 10);
        memcpy(object, config.trk_object, strlen(config.trk_object));
        object[9] = 0;
        sprintf(rawTNC, ")%s!%02d%02d.%02dN%c%03d%02d.%02dE%c%s%s", object, object, lat_dd, lat_mm, lat_ss, aprs_table, lon_dd, lon_mm, lon_ss, aprs_symbol, csd_spd, strAltitude);
      }
      else
      {
        sprintf(rawTNC, "!%02d%02d.%02dN%c%03d%02d.%02dE%c%s%s", lat_dd, lat_mm, lat_ss, aprs_table, lon_dd, lon_mm, lon_ss, aprs_symbol, csd_spd, strAltitude);
      }
    }
  }
  else
  {
    sprintf(rawTNC, ">%s", object);
    // sprintf(rawTNC, "%s-%d>APDRH1,%s:)%s_%s", config.aprs_mycall, config.aprs_ssid, Path.c_str(), object, timestamp);
    //  sprintf(rawTNC, "%s-%d>APBT01-1%s:>%s_%s", config.aprs_mycall, config.aprs_ssid, config.aprs_path, object, timestamp);
  }

  String tnc2Raw = "";
  char strtmp[300];
  if (config.trk_ssid == 0)
    sprintf(strtmp, "%s>APTWR", config.trk_mycall);
  else
    sprintf(strtmp, "%s-%d>APTWR", config.trk_mycall, config.trk_ssid);
  tnc2Raw = String(strtmp);
  if (config.trk_path[0] != 0)
  {
    tnc2Raw += ",";
    tnc2Raw += String(config.trk_path);
  }
  tnc2Raw += ":";
  tnc2Raw += String(rawTNC);
  tnc2Raw += comment + " " + String(config.trk_comment);
  return tnc2Raw;
}

String trk_fix_position(String comment)
{
  String tnc2Raw = "";
  int lat_dd, lat_mm, lat_ss, lon_dd, lon_mm, lon_ss;
  char strtmp[300], loc[30];
  memset(strtmp, 0, 300);
  DD_DDDDDtoDDMMSS(config.trk_lat, &lat_dd, &lat_mm, &lat_ss);
  DD_DDDDDtoDDMMSS(config.trk_lon, &lon_dd, &lon_mm, &lon_ss);
  if (strlen(config.trk_object) >= 3)
  {
    char object[10];
    memset(object, 0, 10);
    memcpy(object, config.trk_object, strlen(config.trk_object));
    object[9] = 0;
    sprintf(loc, ")%s!%02d%02d.%02dN%c%03d%02d.%02dE%c", object, lat_dd, lat_mm, lat_ss, config.trk_symbol[0], lon_dd, lon_mm, lon_ss, config.trk_symbol[1]);
  }
  else
  {
    sprintf(loc, "!%02d%02d.%02dN%c%03d%02d.%02dE%c", lat_dd, lat_mm, lat_ss, config.trk_symbol[0], lon_dd, lon_mm, lon_ss, config.trk_symbol[1]);
  }
  if (config.trk_ssid == 0)
    sprintf(strtmp, "%s>APTWR", config.trk_mycall);
  else
    sprintf(strtmp, "%s-%d>APTWR", config.trk_mycall, config.trk_ssid);
  tnc2Raw = String(strtmp);
  if (config.trk_path[0] != 0)
  {
    tnc2Raw += ",";
    tnc2Raw += String(config.trk_path);
  }
  tnc2Raw += ":";
  tnc2Raw += String(loc);
  tnc2Raw += comment + " " + String(config.trk_comment);
  return tnc2Raw;
}

String igate_position(double lat, double lon, String comment)
{
  String tnc2Raw = "";
  int lat_dd, lat_mm, lat_ss, lon_dd, lon_mm, lon_ss;
  char strtmp[300], loc[30];
  memset(strtmp, 0, 300);
  DD_DDDDDtoDDMMSS(lat, &lat_dd, &lat_mm, &lat_ss);
  DD_DDDDDtoDDMMSS(lon, &lon_dd, &lon_mm, &lon_ss);
  if (strlen(config.igate_object) >= 3)
  {
    char object[10];
    memset(object, 0, 10);
    memcpy(object, config.igate_object, strlen(config.igate_object));
    object[9] = 0;
    sprintf(loc, ")%s!%02d%02d.%02dN%c%03d%02d.%02dE%c", object, lat_dd, lat_mm, lat_ss, config.igate_symbol[0], lon_dd, lon_mm, lon_ss, config.igate_symbol[1]);
  }
  else
  {
    sprintf(loc, "!%02d%02d.%02dN%c%03d%02d.%02dE%c", lat_dd, lat_mm, lat_ss, config.igate_symbol[0], lon_dd, lon_mm, lon_ss, config.igate_symbol[1]);
  }
  if (config.aprs_ssid == 0)
    sprintf(strtmp, "%s>APTWR", config.aprs_mycall);
  else
    sprintf(strtmp, "%s-%d>APTWR", config.aprs_mycall, config.aprs_ssid);
  tnc2Raw = String(strtmp);
  if (config.igate_path[0] != 0)
  {
    tnc2Raw += ",";
    tnc2Raw += String(config.igate_path);
  }
  tnc2Raw += ":";
  tnc2Raw += String(loc);
  tnc2Raw += comment + " " + String(config.igate_comment);
  return tnc2Raw;
}

String digi_position(double lat, double lon, String comment)
{
  String tnc2Raw = "";
  int lat_dd, lat_mm, lat_ss, lon_dd, lon_mm, lon_ss;
  char strtmp[300], loc[30];
  memset(strtmp, 0, 300);
  DD_DDDDDtoDDMMSS(lat, &lat_dd, &lat_mm, &lat_ss);
  DD_DDDDDtoDDMMSS(lon, &lon_dd, &lon_mm, &lon_ss);
  // if (strlen(config.igate_object) >= 3)
  // {
  //   char object[10];
  //   memset(object, 0, 10);
  //   memcpy(object, config.digi_object, strlen(config.igate_object));
  //   object[9] = 0;
  //   sprintf(loc, ")%s!%02d%02d.%02dN%c%03d%02d.%02dE%c", object, lat_dd, lat_mm, lat_ss, config.igate_symbol[0], lon_dd, lon_mm, lon_ss, config.igate_symbol[1]);
  // }
  // else
  // {
  sprintf(loc, "!%02d%02d.%02dN%c%03d%02d.%02dE%c", lat_dd, lat_mm, lat_ss, config.digi_symbol[0], lon_dd, lon_mm, lon_ss, config.digi_symbol[1]);
  //}
  if (config.digi_ssid == 0)
    sprintf(strtmp, "%s>APTWR", config.digi_mycall);
  else
    sprintf(strtmp, "%s-%d>APTWR", config.digi_mycall, config.digi_ssid);
  tnc2Raw = String(strtmp);
  if (config.digi_path[0] != 0)
  {
    tnc2Raw += ",";
    tnc2Raw += String(config.digi_path);
  }
  tnc2Raw += ":";
  tnc2Raw += String(loc);
  tnc2Raw += comment + " " + String(config.digi_comment);
  return tnc2Raw;
}

int packet2Raw(String &tnc2, AX25Msg &Packet)
{
  if (Packet.len < 5)
    return 0;
  tnc2 = String(Packet.src.call);
  if (Packet.src.ssid > 0)
  {
    tnc2 += String(F("-"));
    tnc2 += String(Packet.src.ssid);
  }
  tnc2 += String(F(">"));
  tnc2 += String(Packet.dst.call);
  if (Packet.dst.ssid > 0)
  {
    tnc2 += String(F("-"));
    tnc2 += String(Packet.dst.ssid);
  }
  for (int i = 0; i < Packet.rpt_count; i++)
  {
    tnc2 += String(",");
    tnc2 += String(Packet.rpt_list[i].call);
    if (Packet.rpt_list[i].ssid > 0)
    {
      tnc2 += String("-");
      tnc2 += String(Packet.rpt_list[i].ssid);
    }
    if (Packet.rpt_flags & (1 << i))
      tnc2 += "*";
  }
  tnc2 += String(F(":"));
  tnc2 += String((const char *)Packet.info);
  tnc2 += String("\n");

  // #ifdef DEBUG_TNC
  //     Serial.printf("[%d] ", ++pkgTNC_count);
  //     Serial.print(tnc2);
  // #endif
  return tnc2.length();
}

long sendTimer = 0;
bool AFSKInitAct = false;
int btn_count = 0;
long timeCheck = 0;

void loop()
{
  vTaskDelay(10 / portTICK_PERIOD_MS);
  if (ESP.getFreeHeap() < 60000)
    ESP.restart();

  if (config.bt_master)
  {
    //   if (BTdeviceConnected) {
    //       pTxCharacteristic->setValue(&BTtxValue, 1);
    //       pTxCharacteristic->notify();
    //       BTtxValue++;
    // 	delay(10); // bluetooth stack will go into congestion, if too many packets are sent
    // }

    // disconnecting
    if (!BTdeviceConnected && BToldDeviceConnected)
    {
      delay(500);                  // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      BToldDeviceConnected = BTdeviceConnected;
    }
    // connecting
    if (BTdeviceConnected && !BToldDeviceConnected)
    {
      // do stuff here on connecting
      BToldDeviceConnected = BTdeviceConnected;
    }
  }

  if (digitalRead(0) == LOW)
  {
    btn_count++;
    if (btn_count > 1000) // Push BOOT 10sec
    {
      // digitalWrite(LED_PIN, HIGH);
      //  digitalWrite(LED_TX_PIN, HIGH);
    }
  }
  else
  {
    if (btn_count > 0)
    {
      // Serial.printf("btn_count=%dms\n", btn_count * 10);
      if (btn_count > 1000) // Push BOOT 10sec to Factory Default
      {
        // digitalWrite(LED_RX, LOW);
        // digitalWrite(LED_TX, LOW);
        // defaultConfig();
        log_d("SYSTEM REBOOT NOW!");
        // esp_restart();
      }
      else
      {
        EVENT_TX_POSITION = 1;
        //     showDisp=true;
        //     if( oledSleepTimeout>0){
        //     curTab++;
        //     if(curTab>3) curTab=0;
        //     }
        //     log_d("curTab: %d",curTab);
      }
      btn_count = 0;
    }
  }

  if (millis() > timeCheck)
  {
    // log_d("taskAPRS: %d mS\ttaskNetwork: %d mS\ttaskGUI: %d mS\n", timeAprs, timeNetwork, timeGui);
    timeCheck = millis() + 1000;
    // if (ESP.getFreeHeap() < 60000)
    //     esp_restart();
    // Serial.println(String(ESP.getFreeHeap()));
    // Popup Display
    // if (dispBuffer.getCount() > 0)
    // {
    //     if (millis() > timeHalfSec)
    //     {
    //         char tnc2[300];
    //         dispBuffer.pop(&tnc2);
    //         dispWindow(String(tnc2), 0, false);
    //     }
    // }
    // else
    // {
    //     // Sleep display
    //     if (millis() > timeHalfSec)
    //     {
    //         if (timeHalfSec > 0)
    //         {
    //             timeHalfSec = 0;
    //             oledSleepTimeout = millis() + (config.oled_timeout * 1000);
    //         }
    //         else
    //         {
    //             if (millis() > oledSleepTimeout && oledSleepTimeout > 0)
    //             {
    //                 oledSleepTimeout = 0;
    //                 display.clearDisplay();
    //                 display.display();
    //             }
    //         }
    //     }
    // }
  }
#ifdef SA818
// if (SerialRF.available())
// {
//     Serial.print(Serial.readString());
// }
#endif
}

String sendIsAckMsg(String toCallSign, int msgId)
{
  char str[300];
  char call[11];
  int i;
  memset(&call[0], 0, 11);
  sprintf(call, "%s-%d", config.aprs_mycall, config.aprs_ssid);
  strcpy(&call[0], toCallSign.c_str());
  i = strlen(call);
  for (; i < 9; i++)
    call[i] = 0x20;
  memset(&str[0], 0, 300);

  sprintf(str, "%s-%d>APE32I%s::%s:ack%d", config.aprs_mycall, config.aprs_ssid, VERSION, call, msgId);
  //	client.println(str);
  return String(str);
}

void sendIsPkg(char *raw)
{
  char str[300];
  sprintf(str, "%s-%d>APE32I%s:%s", config.aprs_mycall, config.aprs_ssid, VERSION, raw);
  // client.println(str);
  String tnc2Raw = String(str);
  if (aprsClient.connected())
    aprsClient.println(tnc2Raw); // Send packet to Inet
  if (config.rf_en && config.digi_en)
    pkgTxPush(str, 0);
}

void sendIsPkgMsg(char *raw)
{
  char str[300];
  char call[11];
  int i;
  memset(&call[0], 0, 11);
  if (config.aprs_ssid == 0)
    sprintf(call, "%s", config.aprs_mycall);
  else
    sprintf(call, "%s-%d", config.aprs_mycall, config.aprs_ssid);
  i = strlen(call);
  for (; i < 9; i++)
    call[i] = 0x20;

  if (config.aprs_ssid == 0)
    sprintf(str, "%s>APE32I::%s:%s", config.aprs_mycall, call, raw);
  else
    sprintf(str, "%s-%d>APE32I::%s:%s", config.aprs_mycall, config.aprs_ssid, call, raw);

  String tnc2Raw = String(str);
  if (aprsClient.connected())
    aprsClient.println(tnc2Raw); // Send packet to Inet
  if (config.rf_en && config.digi_en)
    pkgTxPush(str, 0);
  // APRS_sendTNC2Pkt(tnc2Raw); // Send packet to RF
}

void taskGPS(void *pvParameters)
{
  unsigned long gpsTickInterval;
  GPS_INIT();
  for (;;)
  {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // if (millis() > gpsTickInterval)
    // {
    //     gpsTickInterval = millis() + 1000;
    // ESP_LOGE("FreeHEAP", "%d", esp_get_free_heap_size());
    while (SerialGPS.available())
      gps.encode(SerialGPS.read());

    if (firstGpsTime && gps.time.isValid())
    {
      if (gps.time.isUpdated())
      {

        // if (Config.wifi_enable == false ) {
        time_t timeGps = getGpsTime();
        if (timeGps > 1653152400 && timeGps < 2347462800)
        {
          setTime(timeGps);
          time_t rtc = timeGps;
          timeval tv = {rtc, 0};
          timezone tz = {TZ_SEC + DST_MN, 0};
          settimeofday(&tv, &tz);
// setSyncProvider(getGpsTime);
// setSyncInterval(600);
#ifdef DEBUG
          log_d("\nSET GPS Timestamp = %u Year=%d\n", timeGps, year());
#endif
          // firstGpsTime = false;
          firstGpsTime = false;
          if (startTime == 0)
            startTime = now();
        }
        else
        {
          startTime = 0;
        }
        //}
      }
    }
    //}
  }
}

long timeSlot;
void taskAPRS(void *pvParameters)
{
  //	long start, stop;
  char *raw;
  char *str;
  // bool firstGpsTime = true;
  unsigned long tickInterval;
  unsigned long iGatetickInterval;
  unsigned long DiGiInterval;

  log_d("Task APRS has been start");
  PacketBuffer.clean();

  APRS_init();
  APRS_setCallsign(config.aprs_mycall, config.aprs_ssid);
  APRS_setPath1(config.igate_path, 1);
  APRS_setPreamble(300);
  APRS_setTail(0);
  sendTimer = millis() - (config.igate_interval * 1000) + 30000;
  igateTLM.TeleTimeout = millis() + 60000; // 1Min
  AFSKInitAct = true;
  timeSlot = millis();
  timeAprs = 0;
  // AFSK_TimerEnable(true);

  unsigned long timeAprsOld = millis();
  unsigned long gpsTimeout;

  // tx_counter = igate_tx_counter = digi_tx_counter = millis();
  // tx_interval = igate_tx_interval = digi_tx_interval = millis() + 5000;
  DiGiInterval = iGatetickInterval = millis() + 15000;
  tx_interval = config.trk_interval;
  tx_counter = tx_interval - 10;
  for (;;)
  {
    unsigned long now = millis();
    timeAprs = now - timeAprsOld;
    timeAprsOld = now;
    // wdtSensorTimer = now;
    time_t timeStamp;
    time(&timeStamp);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    // serviceHandle();

    if (AFSKInitAct == true)
    {
#ifdef SA818
      AFSK_Poll(true, config.rf_power);
#else
      AFSK_Poll(false, LOW);
#endif
    }

    if (config.rf_en)
    { // RF Module enable
      // SEND RF in time slot
      if (now > (timeSlot + 10))
      {
        // Transmit in timeslot if enabled
        if (config.gpio_sql_pin > -1)
        { // Set SQL pin
          if (!digitalRead(config.gpio_sql_pin))
          { // RX State Fail
            if (pkgTxSend())
              timeSlot = millis() + config.tx_timeslot; // Tx Time Slot = 5sec.
            else
              timeSlot = millis();
          }
          else
          {
            timeSlot = millis() + 500;
          }
        }
        else
        {
          if (pkgTxSend())
            timeSlot = millis() + config.tx_timeslot; // Tx Time Slot = 5sec.
          else
            timeSlot = millis();
        }
      }
    }

    if (config.trk_en)
    { // TRACKER MODE

      if (millis() > tickInterval)
      {
        tickInterval = millis() + 1000;

        tx_counter++;
        // Check interval timeout
        if (config.trk_smartbeacon && config.trk_gps)
        {
          if (tx_counter > tx_interval)
          {
            if (tx_counter > config.trk_mininterval)
              EVENT_TX_POSITION = 4;
          }
          else
          {
            if (tx_counter >= (tx_interval + 5))
            {
              EVENT_TX_POSITION = 5;
            }
          }
        }
        else if (tx_counter > tx_interval)
        {
          EVENT_TX_POSITION = 6;
          tx_interval = config.trk_interval;
        }

        if (config.trk_gps && gps.speed.isValid())
        {
          // if ((SB_SPEED_OLD == 0 && gps.speed.kmph() > 150.0F)) gps.speed.kmph() = 0.0F;
          SB_SPEED_OLD = SB_SPEED;
          SB_SPEED = (unsigned char)gps.speed.kmph();
          // SB_HEADING=(unsigned char)((fixdata.velocity >> 16) & 0xff);
          SB_HEADING = (int16_t)gps.course.deg();
          if (config.trk_smartbeacon) // SMART BEACON CAL
          {
            if (SB_SPEED < config.trk_lspeed && SB_SPEED_OLD > config.trk_lspeed)
            { // STOPING
              SB_SPEED_OLD = 0;
              // tx_counter=tx_interval;
              if (tx_counter > config.trk_mininterval)
                EVENT_TX_POSITION = 7;
              tx_interval = config.trk_slowinterval;
            }
            else
            {
              smartbeacon();
            }
          }
          else if (tx_counter > tx_interval)
          { // send gps location
            EVENT_TX_POSITION = 8;
            tx_interval = config.trk_interval;
          }
        }
        // else // send fix location
        // {
        //   tx_interval = 900;
        // }
      }

      if (EVENT_TX_POSITION > 0)
      {
        powerWakeup();
        String rawData;
        String cmn = "";
        if (config.trk_sat)
          cmn += "SAT:" + String(gps.satellites.value());
        if (config.trk_bat)
        {
          if (config.trk_sat)
            cmn += ",";
          // cmn += "BAT:" + String((float)PMU.getBattVoltage() / 1000, 1) + "V";
          cmn += "BAT:" + String(vbat, 1) + "V";
        }
        if (config.trk_gps) // TRACKER by GPS
        {
          if (!gps.location.isValid())
          {
            continue;
          }
          // rawData = myBeacon(String(config.trk_path));
          rawData = trk_gps_postion(cmn);
        }
        else // TRACKER by FIX position
        {
          rawData = trk_fix_position(cmn);
          // tx_interval = config.trk_interval;
        }

        log_d("TRACKER RAW: %s\n", rawData.c_str());
        log_d("TRACKER EVENT_TX_POSITION=%d\t INTERVAL=%d\n", EVENT_TX_POSITION, tx_interval);
        tx_counter = 0;
        EVENT_TX_POSITION = 0;

        dispFlagTX = 1;
        if (config.trk_loc2rf)
        { // TRACKER SEND TO RF
          pkgTxPush(rawData.c_str(), 0);
          // digitalWrite(POWER_PIN, RF_PWR); // set RF Power H/L
          // AFSK_TimerEnable(true);
          // APRS_sendTNC2Pkt(rawData);
          // for (int i = 0; i < 100; i++)
          // {
          //   if (digitalRead(PTT_PIN))
          //     break;
          //   delay(50); // TOT 5sec
          // }
          // LED_Color(0, 0, 0);
          // digitalWrite(POWER_PIN, 0); // set RF Power Low
        }
        if (config.trk_loc2inet)
        { // TRACKER SEND TO APRS-IS
          if (aprsClient.connected())
          {
            aprsClient.println(rawData); // Send packet to Inet
            // dispTX(0);
          }
        }
      }
    }

    // if (config.tnc_telemetry)
    // {
    //     if (igateTLM.TeleTimeout < millis())
    //     {
    //         igateTLM.TeleTimeout = millis() + 600000; // 10Min
    //         if ((igateTLM.Sequence % 6) == 0)
    //         {
    //             sendIsPkgMsg((char *)&PARM[0]);
    //             sendIsPkgMsg((char *)&UNIT[0]);
    //             sendIsPkgMsg((char *)&EQNS[0]);
    //         }
    //         char rawTlm[100];
    //         if (config.aprs_ssid == 0)
    //             sprintf(rawTlm, "%s>APE32I:T#%03d,%d,%d,%d,%d,%d,00000000", config.aprs_mycall, igateTLM.Sequence, igateTLM.RF2INET, igateTLM.INET2RF, igateTLM.RX, igateTLM.TX, igateTLM.DROP);
    //         else
    //             sprintf(rawTlm, "%s-%d>APE32I:T#%03d,%d,%d,%d,%d,%d,00000000", config.aprs_mycall, config.aprs_ssid, igateTLM.Sequence, igateTLM.RF2INET, igateTLM.INET2RF, igateTLM.RX, igateTLM.TX, igateTLM.DROP);

    //         if (aprsClient.connected())
    //             aprsClient.println(String(rawTlm)); // Send packet to Inet
    //         if (config.tnc && config.tnc_digi)
    //             pkgTxPush(rawTlm, 0);
    //         // APRS_sendTNC2Pkt(String(rawTlm)); // Send packet to RF
    //         igateTLM.Sequence++;
    //         if (igateTLM.Sequence > 999)
    //             igateTLM.Sequence = 0;
    //         igateTLM.DROP = 0;
    //         igateTLM.INET2RF = 0;
    //         igateTLM.RF2INET = 0;
    //         igateTLM.RX = 0;
    //         igateTLM.TX = 0;
    //         // client.println(raw);
    //     }
    // }

    // LOAD DATA incomming
    bool newIGatePkg = false;
    bool newDigiPkg = false;
    if (PacketBuffer.getCount() > 0)
    {
      String tnc2;
      // นำข้อมูลแพ็จเกจจาก TNC ออกจากคิว
      PacketBuffer.pop(&incomingPacket);
      // igateProcess(incomingPacket);
      packet2Raw(tnc2, incomingPacket);
      newIGatePkg = true;
      newDigiPkg = true;
      if (config.bt_master)
      { // Output TNC2RAW to BT Serial
        // SerialBT.println(tnc2);
        if ((config.bt_mode == 1) && BTdeviceConnected)
        {
          pTxCharacteristic->setValue((uint8_t *)tnc2.c_str(), tnc2.length());
          pTxCharacteristic->notify();
        }
      }
      // if (config.dispTNC == true){
      //     if(!dispBuffer.isFull()) dispBuffer.push(tnc2.c_str());
      // }

      log_d("RX: %s", tnc2.c_str());

      // SerialBT.println(tnc2);
      uint8_t type = pkgType((char *)incomingPacket.info);
      char call[11];
      if (incomingPacket.src.ssid > 0)
        sprintf(call, "%s-%d", incomingPacket.src.call, incomingPacket.src.ssid);
      else
        sprintf(call, "%s", incomingPacket.src.call);

      int idx = pkgListUpdate(call, (char *)tnc2.c_str(), type);
      pushTNC2Raw(idx);

      lastPkg = true;
      lastPkgRaw = tnc2;
      // ESP_BT.println(tnc2);
      status.allCount++;
    }

    // IGate Process
    if (config.igate_en)
    {
      // IGATE Position
      if (config.igate_bcn)
      {
        if (millis() > iGatetickInterval)
        {
          
          String rawData = "";
          if (config.igate_gps)
          { // IGATE Send GPS position
            if (gps.location.isValid())
              rawData = igate_position(gps.location.lat(), gps.location.lng(), "");
          }
          else
          { // IGATE Send fix position
            rawData = igate_position(config.igate_lat, config.igate_lon, "");
          }
          // dispFlagTX = 1;
          if (rawData != "")
          {
            iGatetickInterval = millis() + (config.igate_interval * 1000);
            log_d("IGATE_POSITION: %s", rawData.c_str());
            if (config.igate_loc2rf)
            { // IGATE SEND POSITION TO RF
              pkgTxPush(rawData.c_str(), 0);
            }
            if (config.igate_loc2inet)
            { // IGATE SEND TO APRS-IS
              if (aprsClient.connected())
              {
                aprsClient.println(rawData); // Send packet to Inet
              }
            }
          }
        }
      }
      // IGATE send to inet
      if (newIGatePkg)
      {
        newIGatePkg = false;
        if (config.rf2inet && aprsClient.connected())
        {
          int ret = igateProcess(incomingPacket);
          if (ret == 0)
          {
            status.dropCount++;
            igateTLM.DROP++;
          }
          else
          {
            status.rf2inet++;
            igateTLM.RF2INET++;
            igateTLM.TX++;
#ifdef DEBUG
            // printTime();
            // log_d("RF->INET: ");
            // log_d("%s\n", tnc2.c_str());
#endif
            // char call[11];
            // if (incomingPacket.src.ssid > 0)
            //     sprintf(call, "%s-%d", incomingPacket.src.call, incomingPacket.src.ssid);
            // else
            //     sprintf(call, "%s", incomingPacket.src.call);

            // uint8_t type = pkgType((char*)incomingPacket.info);
            // pkgListUpdate(call,(char *)tnc2.c_str(), type);
          }
        }
      }
    }

    // Digi Repeater Process
    if (config.digi_en)
    {
      // DIGI Position
      if (config.digi_bcn)
      {
        if (millis() > DiGiInterval)
        {
          
          String rawData;
          if (config.digi_gps)
          { // DIGI Send GPS position
            if (gps.location.isValid())
              rawData = digi_position(gps.location.lat(), gps.location.lng(), "");
          }
          else
          { // DIGI Send fix position
            rawData = digi_position(config.digi_lat, config.digi_lon, "");
          }
          if (rawData != "")
          {
            DiGiInterval = millis() + (config.digi_interval * 1000);
            log_d("DIGI_POSITION: %s", rawData.c_str());
            // dispFlagTX = 1;
            if (config.digi_loc2rf)
            { // DIGI SEND POSITION TO RF
              pkgTxPush(rawData.c_str(), 0);
            }
            if (config.digi_loc2inet)
            { // DIGI SEND TO APRS-IS
              if (aprsClient.connected())
              {
                aprsClient.println(rawData); // Send packet to Inet
              }
            }
          }
        }
      }

      // Repeater packet
      if (newDigiPkg)
      {
        newDigiPkg = false;
        int dlyFlag = digiProcess(incomingPacket);
        if (dlyFlag > 0)
        {
          int digiDelay;
          status.digiCount++;
          if (dlyFlag == 1)
          {
            digiDelay = 0;
          }
          else
          {
            if (config.digi_delay == 0)
            { // Auto mode
              if (digiCount > 20)
                digiDelay = random(5000);
              else if (digiCount > 10)
                digiDelay = random(3000);
              else if (digiCount > 0)
                digiDelay = random(1500);
              else
                digiDelay = random(500);
            }
            else
            {
              digiDelay = random(config.digi_delay);
            }
          }

          String digiPkg;
          packet2Raw(digiPkg, incomingPacket);
          log_d("DIGI_REPEAT: %s", digiPkg.c_str());
          log_d("DIGI delay=%d ms.", digiDelay);
          pkgTxPush(digiPkg.c_str(), digiDelay);
        }
      }
    }
  }
}

int mqttRetry = 0;
long wifiTTL = 0;
WiFiMulti wifiMulti;

void taskNetwork(void *pvParameters)
{
  int c = 0;
  log_d("Task Network has been start");
  //     pinMode(MODEM_PWRKEY, OUTPUT);
  //         // Pull down PWRKEY for more than 1 second according to manual requirements
  //     digitalWrite(MODEM_PWRKEY, HIGH);
  //     delay(100);
  //     digitalWrite(MODEM_PWRKEY, LOW);
  //     delay(1000);
  //     digitalWrite(MODEM_PWRKEY, HIGH);

  // Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  //   Serial1.setTimeout(10);
  //   Serial1.setRxBufferSize(2048);
  //   ppp.begin(&Serial1);

  //   Serial.print("Connecting PPPoS");
  //   ppp.connect(PPP_APN, PPP_USER, PPP_PASS);
  //   while (!ppp.status()) {
  //     delay(500);
  //     Serial.print(".");
  //   }
  //   Serial.println("OK");

  // if (config.wifi_mode == WIFI_AP_STA_FIX || config.wifi_mode == WIFI_AP_FIX)
  // { // AP=false
  //     // WiFi.mode(config.wifi_mode);
  //     if (config.wifi_mode == WIFI_AP_STA_FIX)
  //     {
  //         WiFi.mode(WIFI_AP_STA);
  //     }
  //     else if (config.wifi_mode == WIFI_AP_FIX)
  //     {
  //         WiFi.mode(WIFI_AP);
  //     }
  //     //กำหนดค่าการทำงานไวไฟเป็นแอสเซสพ้อย
  //     WiFi.softAP(config.wifi_ap_ssid, config.wifi_ap_pass); // Start HOTspot removing password will disable security
  //     WiFi.softAPConfig(local_IP, gateway, subnet);
  //     Serial.print("Access point running. IP address: ");
  //     Serial.print(WiFi.softAPIP());
  //     Serial.println("");
  // }
  // else if (config.wifi_mode == WIFI_STA_FIX)
  // {
  //     WiFi.mode(WIFI_STA);
  //     WiFi.disconnect();
  //     delay(100);
  //     Serial.println(F("WiFi Station Only mode."));
  // }
  // else
  // {
  //     WiFi.mode(WIFI_OFF);
  //     WiFi.disconnect(true);
  //     delay(100);
  //     Serial.println(F("WiFi OFF All mode."));
  //     SerialBT.begin("ESP32TNC");
  // }

  // webService();
  if (config.wifi_mode == WIFI_STA)
  { /**< WiFi station mode */
    WiFi.mode(WIFI_MODE_STA);
  }
  else if (config.wifi_mode == WIFI_AP)
  { /**< WiFi soft-AP mode */
    WiFi.mode(WIFI_MODE_AP);
  }
  if (config.wifi_mode == WIFI_AP_STA)
  { /**< WiFi station + soft-AP mode */
    WiFi.mode(WIFI_MODE_APSTA);
  }
  else
  {
    WiFi.mode(WIFI_MODE_NULL);
  }

  if (config.wifi_mode & WIFI_STA)
  {
    wifiMulti.addAP(config.wifi_ssid, config.wifi_pass);
    wifiMulti.addAP("APRSTH", "aprsthnetwork");
    WiFi.setTxPower((wifi_power_t)config.wifi_power);
    WiFi.setHostname("ESP32APRS_T-TWR");
  }

  if (config.wifi_mode & WIFI_AP)
  {
    // กำหนดค่าการทำงานไวไฟเป็นแอสเซสพ้อย
    WiFi.softAP(config.wifi_ap_ssid, config.wifi_ap_pass); // Start HOTspot removing password will disable security
    WiFi.softAPConfig(local_IP, gateway, subnet);
    Serial.print("Access point running. IP address: ");
    Serial.print(WiFi.softAPIP());
    Serial.println("");
    webService();
  }

  if (wifiMulti.run() == WL_CONNECTED)
  {
    // Serial.println("");
    log_d("Wi-Fi CONNECTED!");
    log_d("IP address: %s", WiFi.localIP().toString().c_str());
    webService();
    NTP_Timeout = millis() + 2000;
  }

  pingTimeout = millis() + 10000;
  unsigned long timeNetworkOld = millis();
  timeNetwork = 0;
  for (;;)
  {
    unsigned long now = millis();
    timeNetwork = now - timeNetworkOld;
    timeNetworkOld = now;
    // wdtNetworkTimer = millis();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    {
      if (WiFi.status() == WL_CONNECTED)
      {
        serviceHandle();

        //         if (config.wifi_mode == WIFI_AP_trk_FIX || config.wifi_mode == WIFI_trk_FIX)
        //         {
        //             if (WiFi.status() != WL_CONNECTED)
        //             {
        //                 unsigned long int tw = millis();
        //                 if (tw > wifiTTL)
        //                 {
        // #ifndef I2S_INTERNAL
        //                     AFSK_TimerEnable(false);
        // #endif
        //                     wifiTTL = tw + 60000;
        //                     Serial.println("WiFi connecting..");
        //                     // udp.endPacket();
        //                     WiFi.disconnect();
        //                     WiFi.setTxPower((wifi_power_t)config.wifi_power);
        //                     WiFi.setHostname("ESP32IGate");
        //                     WiFi.begin(config.wifi_ssid, config.wifi_pass);
        //                     // Wait up to 1 minute for connection...
        //                     for (c = 0; (c < 30) && (WiFi.status() != WL_CONNECTED); c++)
        //                     {
        //                         // Serial.write('.');
        //                         vTaskDelay(1000 / portTICK_PERIOD_MS);
        //                         // for (t = millis(); (millis() - t) < 1000; refresh());
        //                     }
        //                     if (c >= 30)
        //                     { // If it didn't connect within 1 min
        //                         Serial.println("Failed. Will retry...");
        //                         WiFi.disconnect();
        //                         // WiFi.mode(WIFI_OFF);
        //                         delay(3000);
        //                         // WiFi.mode(WIFI_STA);
        //                         WiFi.reconnect();
        //                         continue;
        //                     }

        //                     Serial.println("WiFi connected");
        //                     Serial.print("IP address: ");
        //                     Serial.println(WiFi.localIP());

        //                     vTaskDelay(1000 / portTICK_PERIOD_MS);
        //                     NTP_Timeout = millis() + 5000;
        // // Serial.println("Contacting Time Server");
        // // configTime(3600 * timeZone, 0, "aprs.dprns.com", "1.pool.ntp.org");
        // // vTaskDelay(3000 / portTICK_PERIOD_MS);
        // #ifndef I2S_INTERNAL
        //                     AFSK_TimerEnable(true);
        // #endif
        //                 }
        //             }
        //             else
        //             {

        if (millis() > NTP_Timeout)
        {
          NTP_Timeout = millis() + 86400000;
          // Serial.println("Config NTP");
          // setSyncProvider(getNtpTime);
          log_d("Contacting Time Server\n");
          configTime(3600 * config.timeZone, 0, "203.150.19.26", "110.170.126.101", "77.68.122.252");
          vTaskDelay(1000 / portTICK_PERIOD_MS);
          time_t systemTime;
          time(&systemTime);
          setTime(systemTime);
          if (systemUptime == 0)
          {
            systemUptime = time(NULL);
          }
          pingTimeout = millis() + 2000;
        }

        if (config.igate_en)
        {
          if (aprsClient.connected() == false)
          {
            APRSConnect();
          }
          else
          {
            if (aprsClient.available())
            {
              pingTimeout = millis() + 300000;                // Reset ping timout
              String line = aprsClient.readStringUntil('\n'); // อ่านค่าที่ Server ตอบหลับมาทีละบรรทัด
#ifdef DEBUG_IS
              printTime();
              Serial.print("APRS-IS ");
              Serial.println(line);
#endif
              status.isCount++;
              int start_val = line.indexOf(">", 0); // หาตำแหน่งแรกของ >
              if (start_val > 3)
              {
                // if (config.dispINET == true){
                //     if(!dispBuffer.isFull()) dispBuffer.push(line.c_str());
                // }

                char *raw = (char *)malloc(line.length() + 1);
                String src_call = line.substring(0, start_val);
                String msg_call = "::" + src_call;

                status.allCount++;
                igateTLM.RX++;
                if (config.rf_en && config.inet2rf)
                {
                  if (line.indexOf(msg_call) <= 0) // src callsign = msg callsign ไม่ใช่หัวข้อโทรมาตร
                  {
                    if (line.indexOf(":T#") < 0) // ไม่ใช่ข้อความโทรมาตร
                    {
                      if (line.indexOf("::") > 0) // ข้อความเท่านั้น
                      {                           // message only
                        // raw[0] = '}';
                        // line.toCharArray(&raw[1], line.length());
                        // tncTxEnable = false;
                        // SerialTNC.flush();
                        // SerialTNC.println(raw);
                        pkgTxPush(line.c_str(), 0);
                        // APRS_sendTNC2Pkt(line); // Send out RF by TNC build in
                        //  tncTxEnable = true;
                        status.inet2rf++;
                        igateTLM.INET2RF++;
#ifdef DEBUG
                        // printTime();
                        log_d("INET->RF ");
                        log_d("%s\n", line.c_str());
#endif
                      }
                    }
                  }
                  else
                  {
                    igateTLM.DROP++;
                    log_d("INET Message TELEMETRY from ");
                    log_d("%s\n", src_call.c_str());
                    ;
                  }
                }

                memset(&raw[0], 0, sizeof(raw));
                line.toCharArray(&raw[0], start_val + 1);
                raw[start_val + 1] = 0;
                // pkgListUpdate(&raw[0], 0);
                uint8_t type = pkgType(&raw[0]);
                int idx = pkgListUpdate((char *)src_call.c_str(), &raw[0], type);
                pushTNC2Raw(idx);
                free(raw);
              }
            }
          }
        }

        // if (millis() > pingTimeout)
        //                 {
        //                     pingTimeout = millis() + 3000;
        //                     Serial.print("Ping to " + vpn_IP.toString());
        //                     if (ping_start(vpn_IP, 3, 0, 0, 10) == true)
        //                     {
        //                         Serial.println("VPN Ping Success!!");
        //                     }
        //                     else
        //                     {
        //                         Serial.println("VPN Ping Fail!");
        //                     }
        //                 }

        if (millis() > pingTimeout)
        {
          pingTimeout = millis() + 300000;
          log_d("Ping GW to %s\n", WiFi.gatewayIP().toString().c_str());
          if (ping_start(WiFi.gatewayIP(), 3, 0, 0, 5) == true)
          {
            log_d("GW Success!!\n");
          }
          else
          {
            log_d("GW Fail!\n");
            WiFi.disconnect();
            wifiTTL = 0;
          }
        }
      }
    }
  }
}
