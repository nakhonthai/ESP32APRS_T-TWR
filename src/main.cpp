/*
 Name:		ESP32APRS T-TWR Plus
 Created:	13-10-2023 14:27:23
 Author:	HS5TQA/Atten
 Github:	https://github.com/nakhonthai
 Facebook:	https://www.facebook.com/atten
 Support IS: host:aprs.dprns.com port:14580 or aprs.hs5tqa.ampr.org:14580
 Support IS monitor: http://aprs.dprns.com:14501 or http://aprs.hs5tqa.ampr.org:14501
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
#include <BLESecurity.h>
#include "XPowersLib.h"
#include "cppQueue.h"
#include "digirepeater.h"
#include "igate.h"
#include "wireguardif.h"
#include "wireguard.h"
#include "wireguard_vpn.h"

#include "time.h"

#include <TinyGPSPlus.h>
#include <pbuf.h>
#include <parse_aprs.h>

#include <WiFiUdp.h>

#include <WiFiClientSecure.h>

#include "AFSK.h"

#include "gui_lcd.h"

#define DEBUG_TNC

#define EEPROM_SIZE 2048

#include <Wire.h>
#include "Adafruit_SSD1306.h"
#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

struct pbuf_t aprs;
ParseAPRS aprsParse;

TinyGPSPlus gps;

#define VBAT_PIN 36
#define POWER_PIN SA868_PWR_PIN
#define PULLDOWN_PIN SA868_PD_PIN
#define SQL_PIN -1
HardwareSerial SerialRF(1);

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

RTC_DATA_ATTR time_t systemUptime;
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

long timeNetwork, timeAprs, timeGui;

unsigned long previousMillis = 0; // กำหนดตัวแปรเก็บค่า เวลาสุดท้ายที่ทำงาน
long interval = 10000;            // กำหนดค่าตัวแปร ให้ทำงานทุกๆ 10 วินาที
int conStat = 0;
int conStatNetwork = 0;

cppQueue PacketBuffer(sizeof(AX25Msg), 10, IMPLEMENTATION); // Instantiate queue

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

String RF_VERSION;

XPowersAXP2101 PMU;

Configuration config;

pkgListType *pkgList;

TelemetryType *Telemetry;

TaskHandle_t taskNetworkHandle;
TaskHandle_t taskAPRSHandle;
TaskHandle_t mainDisplayHandle;
TaskHandle_t taskGpsHandle;
TaskHandle_t taskTNCHandle;

bool firstGpsTime = true;
time_t startTime = 0;
HardwareSerial SerialGPS(2);
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool BTdeviceConnected = false;
bool BToldDeviceConnected = false;
uint8_t BTtxValue = 0;

// class MySecurity : public BLESecurityCallbacks
// {

//   uint32_t onPassKeyRequest()
//   {
//     ESP_LOGI(LOG_TAG, "PassKeyRequest");
//     return passkey;
//   }
//   void onPassKeyNotify(uint32_t pass_key)
//   {
//     ESP_LOGI(LOG_TAG, "The passkey Notify number:%d", pass_key);
//   }
//   bool onConfirmPIN(uint32_t pass_key)
//   {
//     ESP_LOGI(LOG_TAG, "The passkey YES/NO number:%d", pass_key);
//     vTaskDelay(5000);
//     return true;
//   }
//   bool onSecurityRequest()
//   {
//     ESP_LOGI(LOG_TAG, "SecurityRequest");
//     return true;
//   }

//   void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl)
//   {
//     ESP_LOGI(LOG_TAG, "Starting BLE!");
//   }
// };

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
      char raw[500];
      int i = 0;
      memset(raw, 0, sizeof(raw));
      for (i = 0; i < rxValue.length(); i++)
      {
        if (i > sizeof(raw))
          break;
        raw[i] = (char)rxValue[i];
        if (raw[i] == 0)
          break;
      }
      if (config.bt_mode == 1)
      { // TNC2RAW MODE
        pkgTxPush(raw, strlen(raw), 1);
      }
      else if (config.bt_mode == 2)
      {
        // KISS MODE
        for (int n = 0; n < i; n++)
          kiss_serial((uint8_t)raw[n]);
      }
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
      if (SB_SPEED <= 0)
        SB_SPEED = 1;
      tx_interval = (trk_interval * config.trk_hspeed) / SB_SPEED;
      if (tx_interval < 5)
        tx_interval = 5;
      if (tx_interval > config.trk_maxinterval)
        tx_interval = (uint16_t)config.trk_maxinterval;

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
    time = timeStamp + config.timeZone * SECS_PER_HOUR;
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
    time = timeStamp + config.timeZone * SECS_PER_HOUR;
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
  config.wifi_sta[0].enable = true;
  sprintf(config.wifi_sta[0].wifi_ssid, "APRSTH");
  sprintf(config.wifi_sta[0].wifi_pass, "aprsthnetwork");
  for (int i = 1; i < 5; i++)
  {
    config.wifi_sta[i].enable = false;
    config.wifi_sta[i].wifi_ssid[0] = 0;
    config.wifi_sta[i].wifi_pass[0] = 0;
  }
  sprintf(config.wifi_ap_ssid, "ESP32APRS");
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
  config.bt_pin = 0;

  //--RF Module
  config.rf_en = true;
  config.rf_type = RF_SA868_VHF;
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

  // IGATE
  config.igate_bcn = false;
  config.igate_en = false;
  config.rf2inet = true;
  config.inet2rf = false;
  config.igate_loc2rf = false;
  config.igate_loc2inet = true;
  config.rf2inetFilter=0xFFF;//All
  config.inet2rfFilter=config.digiFilter=FILTER_OBJECT|FILTER_ITEM|FILTER_MESSAGE|FILTER_MICE|FILTER_POSITION|FILTER_WX;
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
  config.igate_alt = 0;
  config.igate_interval = 600;
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
  config.digi_gps = false;
  config.digi_lat = 13.7555;
  config.digi_lon = 100.4930;
  config.digi_alt = 0;
  config.digi_interval = 600;
  config.digi_delay = 0;
  config.digiFilter=FILTER_OBJECT|FILTER_ITEM|FILTER_MESSAGE|FILTER_MICE|FILTER_POSITION|FILTER_WX;
  sprintf(config.digi_symbol, "/#");
  sprintf(config.digi_phg, "");
  sprintf(config.digi_comment, "DIGI MODE");

  // Tracker
  config.trk_en = false;
  config.trk_loc2rf = true;
  config.trk_loc2inet = false;
  config.trk_bat = false;
  config.trk_sat = false;
  config.trk_dx = false;
  config.trk_ssid = 9;
  sprintf(config.trk_mycall, "NOCALL");
  sprintf(config.trk_path, "WIDE1-1,WIDE2-1");

  //--Position
  config.trk_gps = false;
  config.trk_lat = 13.7555;
  config.trk_lon = 100.4930;
  config.trk_alt = 0;
  config.trk_interval = 600;
  // Smart beacon
  config.trk_smartbeacon = true;
  config.trk_compress = true;
  config.trk_altitude = true;
  config.trk_cst = true;
  config.trk_hspeed = 120;
  config.trk_lspeed = 5;
  config.trk_maxinterval = 30;
  config.trk_mininterval = 5;
  config.trk_minangle = 25;
  config.trk_slowinterval = 600;

  sprintf(config.trk_symbol, "/[");
  sprintf(config.trk_symmove, "/>");
  sprintf(config.trk_symstop, "\\>");
  // sprintf(config.trk_btext, "");
  sprintf(config.trk_mycall, "NOCALL");
  sprintf(config.trk_comment, "TRACKER MODE");
  sprintf(config.trk_item, "");
  // sprintf(config.trk_item, "");

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
  config.tx_display = true;
  config.rx_display = true;
  config.audio_hpf = false;
  config.audio_bpf = false;
  config.preamble = 3;
  sprintf(config.ntp_host, "ntp.dprns.com");

  sprintf(config.path[0], "WIDE1-1");
  sprintf(config.path[1], "WIDE1-1,WIDE2-1");
  sprintf(config.path[2], "TRACK3-3");
  sprintf(config.path[3], "RS0ISS");

  // VPN Wireguard
  config.vpn = false;
  config.wg_port = 51820;
  sprintf(config.wg_peer_address, "vpn.dprns.com");
  sprintf(config.wg_local_address, "192.168.1.2");
  sprintf(config.wg_netmask_address, "255.255.255.0");
  sprintf(config.wg_gw_address, "192.168.1.1");
  sprintf(config.wg_public_key, "");
  sprintf(config.wg_private_key, "");

  sprintf(config.http_username, "admin");
  sprintf(config.http_password, "admin");

  config.gpio_sql_pin = -1;

  saveEEPROM();
}

unsigned long NTP_Timeout;
unsigned long pingTimeout;

bool psramBusy = false;

// const char *lastTitle = "LAST HEARD";

int tlmList_Find(char *call)
{
  int i;
  // while (psramBusy) delay(1);
  // psramBusy = true;
  for (i = 0; i < TLMLISTSIZE; i++)
  {
    if (strstr(Telemetry[i].callsign, call) != NULL)
    {
      psramBusy = false;
      return i;
    }
  }
  // psramBusy = false;
  return -1;
}

int tlmListOld()
{
  // while (psramBusy) delay(1);
  // psramBusy = true;
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
  // psramBusy = false;
  return ret;
}

TelemetryType getTlmList(int idx)
{
  TelemetryType ret;
  while (psramBusy)
    delay(1);
  psramBusy = true;
  memcpy(&ret, &Telemetry[idx], sizeof(TelemetryType));
  psramBusy = false;
  return ret;
}

int pkgList_Find(char *call)
{
  int i;
  for (i = 0; i < PKGLISTSIZE; i++)
  {
    if (strstr(pkgList[(int)i].calsign, call) != NULL)
      return i;
  }
  return -1;
}

int pkgList_Find(char *call, uint16_t type)
{
  int i;
  for (i = 0; i < PKGLISTSIZE; i++)
  {
    if ((strstr(pkgList[i].calsign, call) != NULL) && (pkgList[i].type == type))
      return i;
  }
  return -1;
}

int pkgListOld()
{
  int i, ret = -1;
  time_t minimum = time(NULL) + 86400; // pkgList[0].time;
  for (i = 0; i < PKGLISTSIZE; i++)
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
  while (psramBusy)
    delay(1);
  psramBusy = true;
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
  psramBusy = false;
}

void sortPkgDesc(pkgListType a[], int size)
{
  pkgListType t;
  char *ptr1;
  char *ptr2;
  char *ptr3;
  ptr1 = (char *)&t;
  while (psramBusy)
    delay(1);
  psramBusy = true;
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
  psramBusy = false;
}

uint16_t pkgType(const char *raw)
{
  uint16_t type = 0;
  char packettype = 0;
  const char *info_start, *body;
  int paclen = strlen(raw);
  char *ptr;

  if (*raw == 0)
    return 0;

  packettype = (char)raw[0];
  body = &raw[1];

  switch (packettype)
  {
  case '$': // NMEA
    type |= FILTER_POSITION;
    break;
  case 0x27: /* ' */
  case 0x60: /* ` */
    type |= FILTER_POSITION;
    type |= FILTER_MICE;
    break;
  case '!':
  case '=':
    type |= FILTER_POSITION;
    if (body[18] == '_' || body[10] == '_')
    {
      type |= FILTER_WX;
      break;
    }
  case '/':
  case '@':
    type |= FILTER_POSITION;
    if (body[25] == '_' || body[16] == '_')
    {
      type |= FILTER_WX;
      break;
    }
    if (strchr(body, 'r') != NULL)
    {
      if (strchr(body, 'g') != NULL)
      {
        if (strchr(body, 't') != NULL)
        {
          if (strchr(body, 'P') != NULL)
          {
            type |= FILTER_WX;
          }
        }
      }
    }
    break;
  case ':':
    if (body[9] == ':' &&
        (memcmp(body + 10, "PARM", 4) == 0 ||
         memcmp(body + 10, "UNIT", 4) == 0 ||
         memcmp(body + 10, "EQNS", 4) == 0 ||
         memcmp(body + 10, "BITS", 4) == 0))
    {
      type |= FILTER_TELEMETRY;
    }
    else
    {
      type |= FILTER_MESSAGE;
    }
    break;
  case '{': // User defind
  case '<': // statcapa
  case '>':
    type |= FILTER_STATUS;
    break;
  case '?':
    type |= FILTER_QUERY;
    break;
  case ';':
    if (body[28] == '_')
      type |= FILTER_WX;
    else
      type |= FILTER_OBJECT;
    break;
  case ')':
    type |= FILTER_ITEM;
    break;
  case '}':
    type |= FILTER_THIRDPARTY;
    ptr=strchr(raw,':');
    if(ptr!=NULL){
        ptr++;
        type|=pkgType(ptr);
    }
    break;
  case 'T':
    type |= FILTER_TELEMETRY;
    break;
  case '#': /* Peet Bros U-II Weather Station */
  case '*': /* Peet Bros U-I  Weather Station */
  case '_': /* Weather report without position */
    type |= FILTER_WX;
    break;
  default:
    type = 0;
    break;
  }
  return type;
}

uint16_t TNC2Raw[PKGLISTSIZE];
int raw_count = 0, raw_idx_rd = 0, raw_idx_rw = 0;

int pushTNC2Raw(int raw)
{
  if (raw < 0)
    return -1;
  if (raw_count > PKGLISTSIZE)
    return -1;
  if (++raw_idx_rw >= PKGLISTSIZE)
    raw_idx_rw = 0;
  TNC2Raw[raw_idx_rw] = raw;
  raw_count++;
  return raw_count;
}

int popTNC2Raw(int &ret)
{
  String raw = "";
  int idx = 0;
  if (raw_count <= 0)
    return -1;
  if (++raw_idx_rd >= PKGLISTSIZE)
    raw_idx_rd = 0;
  idx = TNC2Raw[raw_idx_rd];
  if (idx < PKGLISTSIZE)
    ret = idx;
  if (raw_count > 0)
    raw_count--;
  return raw_count;
}

pkgListType getPkgList(int idx)
{
    pkgListType ret;
#ifdef BOARD_HAS_PSRAM
    while (psramBusy)
        delay(1);
    psramBusy = true;
#endif
    memset(&ret,0,sizeof(pkgListType));
    if(idx<PKGLISTSIZE) memcpy(&ret, &pkgList[idx], sizeof(pkgListType));
    psramBusy = false;
    return ret;
}

int pkgListUpdate(char *call, char *raw, uint16_t type, bool channel)
{
  size_t len;
  if (*call == 0)
    return -1;
  if (*raw == 0)
    return -1;

  char callsign[11];
  size_t sz = strlen(call);
  memset(callsign, 0, 11);
  if (sz > 10)
    sz = 10;
  // strncpy(callsign, call, sz);
  memcpy(callsign, call, sz);
#ifdef BOARD_HAS_PSRAM
    while (psramBusy)
        delay(1);
    psramBusy = true;
#endif
  int i = pkgList_Find(callsign, type);
  if (i > PKGLISTSIZE)
  {
    psramBusy = false;
    return -1;
  }
  if (i > -1)
  { // Found call in old pkg
    if ((channel==1)||(channel == pkgList[i].channel))
    {
      pkgList[i].time = time(NULL);
      pkgList[i].pkg++;
      pkgList[i].type = type;
      if (channel == 0)
        pkgList[i].audio_level = (int16_t)mVrms;
      else
        pkgList[i].audio_level = 0;
      len = strlen(raw);
      if (len > 500)
        len = 500;
      memcpy(pkgList[i].raw, raw, len);
      // SerialLOG.print("Update: ");
    }
  }
  else
  {
    i = pkgListOld(); // Search free in array
    if (i > PKGLISTSIZE || i < 0)
    {
      psramBusy = false;
      return -1;
    }
    // memset(&pkgList[i], 0, sizeof(pkgListType));
    pkgList[i].channel = channel;
    pkgList[i].time = time(NULL);
    pkgList[i].pkg = 1;
    pkgList[i].type = type;
    if (channel == 0)
      pkgList[i].audio_level = (int16_t)mVrms;
    else
      pkgList[i].audio_level = 0;
    // strcpy(pkgList[i].calsign, callsign);
    memcpy(pkgList[i].calsign, callsign, strlen(callsign));
    len = strlen(raw);
    if (len > 500)
      len = 500;
    memcpy(pkgList[i].raw, raw, len);
    // strcpy(pkgList[i].raw, raw);
    pkgList[i].calsign[10] = 0;
    // SerialLOG.print("NEW: ");
  }
  psramBusy = false;
  return i;
}

bool pkgTxDuplicate(AX25Msg ax25)
{
  while (psramBusy)
    delay(1);
  psramBusy = true;
  char callsign[12];
  for (int i = 0; i < PKGTXSIZE; i++)
  {
    if (txQueue[i].Active)
    {
      if (ax25.src.ssid > 0)
        sprintf(callsign, "%s-%d", ax25.src.call, ax25.src.ssid);
      else
        sprintf(callsign, "%s", ax25.src.call);
      if (strncmp(&txQueue[i].Info[0], callsign, strlen(callsign)) >= 0) // Check duplicate src callsign
      {
        char *ecs1 = strstr(txQueue[i].Info, ":");
        if (ecs1 == NULL)
          continue;
        ;
        if (strncmp(ecs1, (const char *)ax25.info, strlen(ecs1)) >= 0)
        { // Check duplicate aprs info
          txQueue[i].Active = false;
          psramBusy = false;
          return true;
        }
      }
    }
  }

  psramBusy = false;
  return false;
}

bool pkgTxPush(const char *info, size_t len, int dly)
{
  char *ecs = strstr(info, ">");
  if (ecs == NULL)
    return false;

  while (psramBusy)
    delay(1);
  psramBusy = true;
  // for (int i = 0; i < PKGTXSIZE; i++)
  // {
  //   if (txQueue[i].Active)
  //   {
  //     if ((strncmp(&txQueue[i].Info[0], info, info - ecs)==0)) //Check src callsign
  //     {
  //       // strcpy(&txQueue[i].Info[0], info);
  //       memset(txQueue[i].Info, 0, sizeof(txQueue[i].Info));
  //       memcpy(&txQueue[i].Info[0], info, len);
  //       txQueue[i].Delay = dly;
  //       txQueue[i].timeStamp = millis();
  //       psramBusy = false;
  //       return true;
  //     }
  //   }
  // }

  // Add
  for (int i = 0; i < PKGTXSIZE; i++)
  {
    if (txQueue[i].Active == false)
    {
      memset(txQueue[i].Info, 0, sizeof(txQueue[i].Info));
      memcpy(&txQueue[i].Info[0], info, len);
      txQueue[i].Delay = dly;
      txQueue[i].Active = true;
      txQueue[i].timeStamp = millis();
      break;
    }
  }
  psramBusy = false;
  return true;
}

bool pkgTxSend()
{
  if (getReceive())
    return false;
  while (psramBusy)
    delay(1);
  psramBusy = true;
  char info[500];
  for (int i = 0; i < PKGTXSIZE; i++)
  {
    if (txQueue[i].Active)
    {
      int decTime = millis() - txQueue[i].timeStamp;
      if (decTime > txQueue[i].Delay)
      {
        txQueue[i].Active = false;
        memset(info, 0, sizeof(info));
        strcpy(info, txQueue[i].Info);
        psramBusy = false;
        digitalWrite(POWER_PIN, config.rf_power); // RF Power LOW
        status.txCount++;
        LED_Color(255, 0, 0);
        adcActive(false);
        setOLEDLock(true);

        APRS_setPreamble(config.preamble * 100);
        APRS_sendTNC2Pkt(String(info)); // Send packet to RF
        log_d("TX->RF: %s\n", info);

        for (int i = 0; i < 100; i++)
        {
          if (digitalRead(PTT_PIN))
            break;
          delay(50); // TOT 5sec
        }

        // delay(2000);
        LED_Color(0, 0, 0);
        digitalWrite(POWER_PIN, 0); // set RF Power Low
        adcActive(true);
        setOLEDLock(false);
        return true;
      }
    }
  }
  psramBusy = false;
  return false;
}

uint8_t *packetData;
// ฟังชั่นถูกเรียกมาจาก ax25_decode
void aprs_msg_callback(struct AX25Msg *msg)
{
  AX25Msg pkg;

  memcpy(&pkg, msg, sizeof(AX25Msg));
  PacketBuffer.push(&pkg); // ใส่แพ็จเก็จจาก TNC ลงคิวบัพเฟอร์
  status.rxCount++;
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

void setupPowerRF()
{
  // bool result = PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
  // if (result == false) {
  //     while (1) {
  //         Serial.println("PMU is not online...");
  //         delay(500);
  //     }
  // }
  //! DC3 Radio Pixels VDD , Don't change
  PMU.setDC3Voltage(3400);
  PMU.enableDC3();
}

bool SA868_waitResponse(String &data, String rsp, uint32_t timeout)
{
  uint32_t startMillis = millis();
  data.clear();
  do
  {
    while (SerialRF.available() > 0)
    {
      int8_t ch = SerialRF.read();
      data += static_cast<char>(ch);
      if (rsp.length() && data.endsWith(rsp))
      {
        return true;
      }
    }
  } while (millis() - startMillis < timeout);
  return false;
}

int SA868_getRSSI()
{
  String data;
  int rssi;

  // Serial.printf("AT+RSSI?\r\n");
  SerialRF.printf("AT+RSSI?\r\n");
  if (SA868_waitResponse(data, "\r\n", 1000))
  {
    // Serial.println(INFO + data);
    String rssi = data.substring(data.indexOf("RSSI=") + strlen("RSSI="), data.indexOf("\r\n"));
    rssi = rssi.toInt();
    return rssi.toInt();
  }
  else
  {
    // timeout or error
    return 0;
  }
}

String SA868_getVERSION()
{
  String data;
  int rssi;

  SerialRF.printf("AT+VERSION\r\n");
  if (SA868_waitResponse(data, "\r\n", 1000))
  {
    String version = data.substring(0, data.indexOf("\r\n"));
    return version;
  }
  else
  {
    // timeout or error
    return "-";
  }
}

String FRS_getVERSION()
{
    String data;
    String version;

    SerialRF.printf("AT+DMOVER\r\n");
    if (SA868_waitResponse(data, "\r\n", 1000))
    {
        int st=data.indexOf("DMOVER:");
        if(st>0){
            version = data.substring(st+7, data.indexOf("\r\n"));
        }else{
            version="Not found";
        }
        return version;
    }
    else
    {
        // timeout or error
        return "-";
    }
}

unsigned long SA818_Timeout = 0;

void RF_MODULE_SLEEP()
{
  digitalWrite(POWER_PIN, LOW);
  digitalWrite(PULLDOWN_PIN, LOW);
  // PMU.disableDC3();
}


String hexToString(String hex) {  //for String to HEX conversion
  String text = "";    
  for(int k=0;k< hex.length();k++) {
    if(k%2!=0) {
      char temp[3];
      sprintf(temp,"%c%c",hex[k-1],hex[k]);
      int number = (int)strtol(temp, NULL, 16);
      text+=char(number);
    }
  }  
  return text;
}

String ctcssToHex(unsigned int decValue, int section){  //use to convert the CTCSS reading which RF module needed
  if(decValue == 7777){
    return hexToString("FF");
  }
  String d1d0 = String(decValue);
  if (decValue < 1000){
    d1d0 = "0" + d1d0;
  }
  if (section == 1) {
    return hexToString(d1d0.substring(2,4));
  }else{
    return hexToString(d1d0.substring(0,2));
  }
}

void RF_MODULE(bool boot)
{
  String data;
  if (config.rf_en == false)
  {
    RF_MODULE_SLEEP();
    return;
  }
  if (config.rf_type == RF_NONE)
    return;
  log_d("RF Module %s Init", RF_TYPE[config.rf_type]);
  //! DC3 Radio & Pixels VDD , Don't change
  // PMU.setDC3Voltage(3400);
  // PMU.disableDC3();

  pinMode(SA868_PD_PIN, OUTPUT);
  digitalWrite(SA868_PD_PIN, HIGH); // PWR HIGH

  pinMode(SA868_PWR_PIN, OUTPUT);
  digitalWrite(SA868_PWR_PIN, LOW); // RF POWER LOW

  pinMode(SA868_PTT_PIN, OUTPUT);
  digitalWrite(SA868_PTT_PIN, HIGH); // PTT HIGH

  setupPowerRF();
  // pinMode(POWER_PIN, OUTPUT);
  // pinMode(PULLDOWN_PIN, OUTPUT);
  // // pinMode(SQL_PIN, INPUT);
  // pinMode(PTT_PIN, OUTPUT);

  // digitalWrite(PTT_PIN, LOW);
  // digitalWrite(POWER_PIN, LOW);
  // digitalWrite(PULLDOWN_PIN, LOW);
  // delay(1000);
  // digitalWrite(PTT_PIN, HIGH);
  // digitalWrite(PULLDOWN_PIN, HIGH);
  // delay(500);
  if (boot)
  {
    SerialRF.begin(9600, SERIAL_8N1, SA868_RX_PIN, SA868_TX_PIN);
  }

  delay(1500);
  SerialRF.write("\r\n");

  char str[200];
  String rsp = "\r\n";
  if (config.sql_level > 8)
    config.sql_level = 8;
  if ((config.rf_type == RF_SR_1WV) || (config.rf_type == RF_SR_1WU) || (config.rf_type == RF_SR_1W350))
  {
    SerialRF.printf("AT+DMOCONNECT\r\n");
    if (SA868_waitResponse(data, rsp, 1000))
      log_d("%s", data.c_str());
    sprintf(str, "AT+DMOSETGROUP=%01d,%0.4f,%0.4f,%d,%01d,%d,0", config.band, config.freq_tx + ((float)config.offset_tx / 1000000), config.freq_rx + ((float)config.offset_rx / 1000000), config.tone_rx, config.sql_level, config.tone_tx);
    SerialRF.println(str);
    log_d("Write to SR_FRS: %s", str);
    if (SA868_waitResponse(data, rsp, 1000))
      log_d("%s", data.c_str());
    // Module auto power save setting
    SerialRF.printf("AT+DMOAUTOPOWCONTR=1\r\n");
    if (SA868_waitResponse(data, rsp, 1000))
      log_d("%s", data.c_str());
    SerialRF.printf("AT+DMOSETVOX=0\r\n");
    if (SA868_waitResponse(data, rsp, 1000))
      log_d("%s", data.c_str());
    SerialRF.printf("AT+DMOSETMIC=6,0\r\n");
    if (SA868_waitResponse(data, rsp, 1000))
      log_d("%s", data.c_str());
    SerialRF.printf("AT+DMOSETVOLUME=%d\r\n", config.volume);
    if (SA868_waitResponse(data, rsp, 1000))
      log_d("%s", data.c_str());
  }
  else if ((config.rf_type == RF_SA868_VHF) || (config.rf_type == RF_SA868_UHF) || (config.rf_type == RF_SA868_350))
  {
    SerialRF.printf("AT+DMOCONNECT\r\n");
    if (SA868_waitResponse(data, rsp, 1000))
      log_d("%s", data.c_str());
    SerialRF.printf("AT+DMOCONNECT\r\n");
    if (SA868_waitResponse(data, rsp, 1000))
      log_d("%s", data.c_str());
    sprintf(str, "AT+DMOSETGROUP=%01d,%0.4f,%0.4f,%04d,%01d,%04d\r\n", config.band, config.freq_tx, config.freq_rx, config.tone_tx, config.sql_level, config.tone_rx);
    SerialRF.print(str);
    log_d("Write to SA868: %s", str);
    if (SA868_waitResponse(data, rsp, 2000))
      log_d("%s", data.c_str());
    SerialRF.printf("AT+SETTAIL=0\r\n");
    if (SA868_waitResponse(data, rsp, 1000))
      log_d("%s", data.c_str());
    SerialRF.printf("AT+SETFILTER=1,1,1\r\n");
    if (SA868_waitResponse(data, rsp, 1000))
      log_d("%s", data.c_str());
    SerialRF.printf("AT+DMOSETVOLUME=%d\r\n", config.volume);
    if (SA868_waitResponse(data, rsp, 1000))
      log_d("%s", data.c_str());
  }
  else if ((config.rf_type == RF_SR_2WVS) || (config.rf_type == RF_SR_2WUS))
  {
    uint8_t flag = 0;  // Bit0:busy lock 0:OFF,1:ON Bit1:band 0:Wide,1:Narrow
    uint8_t flag1 = 0; // Bit0: Hi/Lo 0:2W,1:0.5W  Bit1:Middle 0:2W/0.5W 1:1W
    RF_VERSION = FRS_getVERSION();
    log_d("RF Module Version %s", RF_VERSION.c_str());

    String tone_rx, tone_tx;

    if (config.tone_rx > 0)
    {
      int idx = config.tone_rx;
      if (idx < sizeof(ctcss))
      {
        int tone = (int)(ctcss[config.tone_rx] * 10.0F);
        tone_rx = ctcssToHex(tone, 1);
        tone_rx += ctcssToHex(tone, 2);
        log_d("Tone RX[%d] rx=%0X %0X", tone, tone_rx.charAt(0), tone_rx.charAt(1));
      }
    }
    else
    {
      tone_rx = ctcssToHex(7777, 1);
      tone_rx += ctcssToHex(7777, 2);
    }

    if (config.tone_tx > 0)
    {
      int idx = config.tone_tx;
      if (idx < sizeof(ctcss))
      {
        int tone = (int)(ctcss[config.tone_tx] * 10.0F);
        tone_tx = ctcssToHex(tone, 1);
        tone_tx += ctcssToHex(tone, 2);
        log_d("Tone RX[%d] tx=%0X %0X", tone, tone_tx.charAt(0), tone_tx.charAt(1));
      }
    }
    else
    {
      tone_tx = ctcssToHex(7777, 1);
      tone_tx += ctcssToHex(7777, 2);
    }

    if (config.band == 0)
      flag |= 0x02;
    if (config.rf_power == 0)
      flag1 |= 0x02;
    sprintf(str, "AT+DMOGRP=%0.5f,%0.5f,%s,%s,%d,%d\r\n", config.freq_rx, config.freq_tx, tone_rx.c_str(), tone_tx.c_str(), flag, flag1);
    log_d("Write to SR_FRS_2W: %s", str);
    SerialRF.print(str);

    if (SA868_waitResponse(data, rsp, 1000))
      log_d("%s", data.c_str());
    SerialRF.printf("AT+DMOSAV=1\r\n");
    if (SA868_waitResponse(data, rsp, 1000))
      log_d("%s", data.c_str());
    SerialRF.printf("AT+DMOVOL=%d\r\n", config.volume);
    if (SA868_waitResponse(data, rsp, 1000))
      log_d("%s", data.c_str());
    SerialRF.printf("AT+DMOVOX=0\r\n");
    if (SA868_waitResponse(data, rsp, 1000))
      log_d("%s", data.c_str());
    SerialRF.printf("AT+DMOFUN=%d,5,1,0,0\r\n", config.sql_level);
    if (SA868_waitResponse(data, rsp, 1000))
      log_d("%s", data.c_str());
  }
}

void RF_MODULE_CHECK()
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
      // Serial.println(SerialRF.readString());
      log_d("RF Module %s Activate", RF_TYPE[config.rf_type]);
    }
  }
  else
  {
    log_d("RF Module %s sleep", RF_TYPE[config.rf_type]);
    digitalWrite(POWER_PIN, LOW);
    digitalWrite(PULLDOWN_PIN, LOW);
    delay(500);
    RF_MODULE(true);
  }
}

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
      login = "user " + String(config.igate_object) + " pass " + String(passcode, DEC) + " vers ESP32APRS T-TWR+ " + String(VERSION) + String(VERSION_BUILD) + " filter " + String(config.aprs_filter);
    }
    else
    {
      uint16_t passcode = aprsParse.passCode(config.aprs_mycall);
      if (config.aprs_ssid == 0)
        login = "user " + String(config.aprs_mycall) + " pass " + String(passcode, DEC) + " vers ESP32APRS T-TWR+ " + String(VERSION) + String(VERSION_BUILD) + " filter " + String(config.aprs_filter);
      else
        login = "user " + String(config.aprs_mycall) + "-" + String(config.aprs_ssid) + " pass " + String(passcode, DEC) + " vers ESP32APRS T-TWR+ " + String(VERSION) + String(VERSION_BUILD) + " filter " + String(config.aprs_filter);
    }
    aprsClient.println(login);
    // Serial.println(login);
    // Serial.println("Success");
    delay(500);
  }
  return true;
}

String NMEA;

bool powerEvent = true;

void powerSave()
{
  if (config.oled_enable)
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
  if (config.oled_enable)
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
      log_d("PMU is not online...");
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
  log_d("->  getSysPowerDownVoltage:%u\n", vol);

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

  log_d("DCDC=======================================================================");
  log_d("DC1  : %s   Voltage:%u mV \n", PMU.isEnableDC1() ? "+" : "-", PMU.getDC1Voltage());
  log_d("DC2  : %s   Voltage:%u mV \n", PMU.isEnableDC2() ? "+" : "-", PMU.getDC2Voltage());
  log_d("DC3  : %s   Voltage:%u mV \n", PMU.isEnableDC3() ? "+" : "-", PMU.getDC3Voltage());
  log_d("DC4  : %s   Voltage:%u mV \n", PMU.isEnableDC4() ? "+" : "-", PMU.getDC4Voltage());
  log_d("DC5  : %s   Voltage:%u mV \n", PMU.isEnableDC5() ? "+" : "-", PMU.getDC5Voltage());
  log_d("ALDO=======================================================================");
  log_d("ALDO1: %s   Voltage:%u mV\n", PMU.isEnableALDO1() ? "+" : "-", PMU.getALDO1Voltage());
  log_d("ALDO2: %s   Voltage:%u mV\n", PMU.isEnableALDO2() ? "+" : "-", PMU.getALDO2Voltage());
  log_d("ALDO3: %s   Voltage:%u mV\n", PMU.isEnableALDO3() ? "+" : "-", PMU.getALDO3Voltage());
  log_d("ALDO4: %s   Voltage:%u mV\n", PMU.isEnableALDO4() ? "+" : "-", PMU.getALDO4Voltage());
  log_d("BLDO=======================================================================");
  log_d("BLDO1: %s   Voltage:%u mV\n", PMU.isEnableBLDO1() ? "+" : "-", PMU.getBLDO1Voltage());
  log_d("BLDO2: %s   Voltage:%u mV\n", PMU.isEnableBLDO2() ? "+" : "-", PMU.getBLDO2Voltage());
  log_d("===========================================================================");

  // Set the time of pressing the button to turn off
  PMU.setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
  uint8_t opt = PMU.getPowerKeyPressOffTime();
  log_d("PowerKeyPressOffTime:");
  switch (opt)
  {
  case XPOWERS_POWEROFF_4S:
    log_d("4 Second");
    break;
  case XPOWERS_POWEROFF_6S:
    log_d("6 Second");
    break;
  case XPOWERS_POWEROFF_8S:
    log_d("8 Second");
    break;
  case XPOWERS_POWEROFF_10S:
    log_d("10 Second");
    break;
  default:
    break;
  }
  // Set the button power-on press time
  PMU.setPowerKeyPressOnTime(XPOWERS_POWERON_128MS);
  opt = PMU.getPowerKeyPressOnTime();
  log_d("PowerKeyPressOnTime:");
  switch (opt)
  {
  case XPOWERS_POWERON_128MS:
    log_d("128 Ms");
    break;
  case XPOWERS_POWERON_512MS:
    log_d("512 Ms");
    break;
  case XPOWERS_POWERON_1S:
    log_d("1 Second");
    break;
  case XPOWERS_POWERON_2S:
    log_d("2 Second");
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
  //PMU.disableLongPressShutdown();
  PMU.enableLongPressShutdown();

  // Get charging target current
  const uint16_t currTable[] = {
      0, 0, 0, 0, 100, 125, 150, 175, 200, 300, 400, 500, 600, 700, 800, 900, 1000};
  uint8_t val = PMU.getChargerConstantCurr();
  log_d("Val = %d", val);
  log_d("Setting Charge Target Current : %d", currTable[val]);

  // Get charging target voltage
  const uint16_t tableVoltage[] = {
      0, 4000, 4100, 4200, 4350, 4400, 255};
  val = PMU.getChargeTargetVoltage();
  log_d("Setting Charge Target Voltage : %d", tableVoltage[val]);
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
      log_d("No SD_MMC card attached");
      return;
    }
    else
    {
      log_d("SD_MMC Card Type: ");
      if (cardType == CARD_MMC)
      {
        log_d("MMC");
      }
      else if (cardType == CARD_SD)
      {
        log_d("SDSC");
      }
      else if (cardType == CARD_SDHC)
      {
        log_d("SDHC");
      }
      else
      {
        log_d("UNKNOWN");
      }
      uint32_t cardSize = SD.cardSize() / (1024 * 1024);
      uint32_t cardTotal = SD.totalBytes() / (1024 * 1024);
      uint32_t cardUsed = SD.usedBytes() / (1024 * 1024);
      log_d("SD Card Size: %lu MB\n", cardSize);
      log_d("Total space: %lu MB\n", cardTotal);
      log_d("Used space: %lu MB\n", cardUsed);
    }
  }
}

long oledSleepTimeout = 0;
bool showDisp = false;

char *htmlBuffer;
void setup()
{
  byte *ptr;
#ifdef BOARD_HAS_PSRAM
  pkgList = (pkgListType *)ps_malloc(sizeof(pkgListType) * PKGLISTSIZE);
  Telemetry = (TelemetryType *)malloc(sizeof(TelemetryType) * TLMLISTSIZE);
  txQueue = (txQueueType *)ps_malloc(sizeof(txQueueType) * PKGTXSIZE);
  // TNC2Raw = (int *)ps_malloc(sizeof(int) * PKGTXSIZE);
#else
  pkgList = (pkgListType *)malloc(sizeof(pkgListType) * PKGLISTSIZE);
  Telemetry = (TelemetryType *)malloc(sizeof(TelemetryType) * TLMLISTSIZE);
  txQueue = (txQueueType *)malloc(sizeof(txQueueType) * PKGTXSIZE);
  // TNC2Raw = (int *)malloc(sizeof(int) * PKGTXSIZE);
#endif

  memset(pkgList, 0, sizeof(pkgListType) * PKGLISTSIZE);
  memset(Telemetry, 0, sizeof(TelemetryType) * TLMLISTSIZE);
  memset(txQueue, 0, sizeof(txQueueType) * PKGTXSIZE);
  // memset(TNC2Raw, 0, sizeof(TNC2Raw) * PKGTXSIZE);

  pinMode(ENCODER_OK_PIN, INPUT_PULLUP);

  // Set up serial port
  Serial.begin(115200); // debug

  // Serial.println();
  log_d("Start ESP32IGate V%s", String(VERSION).c_str());
  // log_d("Push BOOT after 3 sec for Factory Default config.");
  log_d("Total heap: %d", ESP.getHeapSize());

  Wire.begin(I2C_SDA, I2C_SCL, 400000L);

  // Setup Power PMU AXP2101
  setupPower();
  setupSDCard();

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

  display.setCursor(60, 40);
  display.printf("FW Ver %s%c", VERSION, VERSION_BUILD);
  display.setCursor(60, 55);
  display.print("Copy@2023");
  display.display();

  if (!EEPROM.begin(EEPROM_SIZE))
  {
    log_d("failed to initialise EEPROM"); // delay(100000);
  }

  delay(1000);

  if (digitalRead(ENCODER_OK_PIN) == LOW)
  {
    defaultConfig();
    log_d("Manual Default configure!");
    display.clearDisplay();
    display.setCursor(10, 22);
    display.print("Factory Reset!");
    display.display();
  }

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

  if (config.bt_master == true)
  {
    // Create the BLE Device
    BLEDevice::init(config.bt_name);
    // BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_MITM);  // The line you told me to add
    // BLESecurity *pSecurity = new BLESecurity();
    // pSecurity->setStaticPIN(config.bt_pin);

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

    // pRxCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM);
    pRxCharacteristic->setCallbacks(new MyCallbacks());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
  }

  RF_MODULE(true);

  display.clearDisplay();
  display.setTextSize(1);
  display.display();

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
      3,               /* Priority of the task */
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

  xTaskCreatePinnedToCore(
      taskTNC,        /* Function to implement the task */
      "taskTNC",      /* Name of the task */
      8192,           /* Stack size in words */
      NULL,           /* Task input parameter */
      1,              /* Priority of the task */
      &taskTNCHandle, /* Task handle. */
      0);             /* Core where the task should run */
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
  DD_DDDDD = abs(DD_DDDDD);
  *DD = (int)DD_DDDDD;
  *MM = (int)((DD_DDDDD - *DD) * 60);
  *SS = ((DD_DDDDD - *DD) * 60 - *MM) * 100;
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
  long longitude = semicircles((char *)lon.c_str(), (nowLng < 0));
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
  if ((spdKnot <= 5) && (alt_feed > 0) && config.trk_altitude)
  {
    if (gps && config.trk_cst)
    {
      // Send Altitude
      aprs_position[11] = '!' + 0x30; // t current,GGA
      int alt = (int)alt_feed;
      int cs = (int)(log(alt) / log(1.002));
      c = (uint8_t)(cs / 91);
      s = (uint8_t)(cs - ((int)c * 91));
      if (s > 91)
        s = 91;
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
    // Send course and speed
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
  memcpy(object, config.trk_item, strlen(config.trk_item));
  object[9] = 0;

  if (gps.location.isValid() && (gps.hdop.hdop()<10.0))
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

    LastLat = nowLat;
    LastLng = nowLng;
    lastTimeStamp = nowTime;

    if (config.trk_compress)
    { // Compress DATA

      String compPosition = compress_position(nowLat, nowLng, gps.altitude.feet(), course, spdKnot, aprs_table, aprs_symbol, (gps.satellites.value()>3));
      // ESP_LOGE("GPS", "Compress=%s", aprs_position);
      if (strlen(object) >= 3)
      {
        // sprintf(rawTNC, "%s-%d>APTWR1,%s:)%s!%c%s", config.aprs_mycall, config.aprs_ssid, Path.c_str(), object, aprs_table, aprs_position);
        sprintf(rawTNC, ")%s!%s", object, compPosition.c_str());
      }
      else
      {
        sprintf(rawTNC, "!%s", compPosition.c_str());
      }
    }
    else
    { // None compress DATA
      int lat_dd, lat_mm, lat_ss, lon_dd, lon_mm, lon_ss;
      char lon_ew = 'E';
      char lat_ns = 'N';
      if (nowLat < 0)
        lat_ns = 'S';
      if (nowLng < 0)
        lon_ew = 'W';
      DD_DDDDDtoDDMMSS(nowLat, &lat_dd, &lat_mm, &lat_ss);
      DD_DDDDDtoDDMMSS(nowLng, &lon_dd, &lon_mm, &lon_ss);
      char csd_spd[8];
      memset(csd_spd, 0, sizeof(csd_spd));
      if (config.trk_cst)
      {
        sprintf(csd_spd, "%03d/%03d", (int)gps.course.deg(), (int)gps.speed.knots());
      }
      if (strlen(object) >= 3)
      {
        char object[10];
        memset(object, 0, 10);
        strcpy(object, config.trk_item);
        object[9] = 0;
        sprintf(rawTNC, ")%s!%02d%02d.%02d%c%c%03d%02d.%02d%c%c%s", object, lat_dd, lat_mm, lat_ss, lat_ns, aprs_table, lon_dd, lon_mm, lon_ss, lon_ew, aprs_symbol, csd_spd);
      }
      else
      {
        sprintf(rawTNC, "!%02d%02d.%02d%c%c%03d%02d.%02d%c%c%s", lat_dd, lat_mm, lat_ss, lat_ns, aprs_table, lon_dd, lon_mm, lon_ss, lon_ew, aprs_symbol, csd_spd);
      }
      if (config.trk_altitude)
      {

        if (gps.altitude.isValid())
        {
          char strAltitude[10];
          memset(strAltitude, 0, sizeof(strAltitude));
          sprintf(strAltitude, "/A=%06d", (int)gps.altitude.feet());
          strcat(rawTNC, strAltitude);
        }
      }
    }
  }
  else
  {
    sprintf(rawTNC, ">%s ", object);
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
  char strtmp[500], loc[100];
  String tnc2Raw = "";
  memset(strtmp, 0, sizeof(strtmp));
  memset(loc, 0, sizeof(loc));
  if (config.trk_compress)
  { // Compress DATA

    String compPosition = compress_position(config.trk_lat, config.trk_lon, (int)(config.trk_alt * 3.28F), 0, 0, config.trk_symbol[0], config.trk_symbol[1], true);
    // ESP_LOGE("GPS", "Compress=%s", aprs_position);
    if (strlen(config.trk_item) >= 3)
    {
      sprintf(loc, ")%s!%s", config.trk_item, compPosition.c_str());
    }
    else
    {
      sprintf(loc, "!%s", compPosition.c_str());
    }
  }
  else
  {
    int lat_dd, lat_mm, lat_ss, lon_dd, lon_mm, lon_ss;

    char lon_ew = 'E';
    char lat_ns = 'N';
    if (config.trk_lat < 0)
      lat_ns = 'S';
    if (config.trk_lon < 0)
      lon_ew = 'W';

    DD_DDDDDtoDDMMSS(config.trk_lat, &lat_dd, &lat_mm, &lat_ss);
    DD_DDDDDtoDDMMSS(config.trk_lon, &lon_dd, &lon_mm, &lon_ss);

    if (strlen(config.trk_item) >= 3)
    {
      char object[10];
      memset(object, 0, 10);
      strcpy(object, config.trk_item);
      object[9] = 0;
      sprintf(loc, ")%s!%02d%02d.%02d%c%c%03d%02d.%02d%c%c", object, lat_dd, lat_mm, lat_ss, lat_ns, config.trk_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.trk_symbol[1]);
    }
    else
    {
      sprintf(loc, "!%02d%02d.%02d%c%c%03d%02d.%02d%c%c", lat_dd, lat_mm, lat_ss, lat_ns, config.trk_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.trk_symbol[1]);
    }

    if (config.trk_alt > 0)
    {
      char strAltitude[12];
      memset(strAltitude, 0, sizeof(strAltitude));
      sprintf(strAltitude, "/A=%06d", (int)(config.trk_alt * 3.28F));
      strcat(loc, strAltitude);
    }
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

String igate_position(double lat, double lon, double alt, String comment)
{
  String tnc2Raw = "";
  int lat_dd, lat_mm, lat_ss, lon_dd, lon_mm, lon_ss;
  char strtmp[500], loc[100];
  char lon_ew = 'E';
  char lat_ns = 'N';
  if (lat < 0)
    lat_ns = 'S';
  if (lon < 0)
    lon_ew = 'W';
  memset(strtmp, 0, sizeof(strtmp));
  memset(loc, 0, sizeof(loc));
  DD_DDDDDtoDDMMSS(lat, &lat_dd, &lat_mm, &lat_ss);
  DD_DDDDDtoDDMMSS(lon, &lon_dd, &lon_mm, &lon_ss);
  char strAltitude[12];
  memset(strAltitude, 0, sizeof(strAltitude));
  if (alt > 0)
  {
    sprintf(strAltitude, "/A=%06d", (int)(alt * 3.28F));
  }
  if (strlen(config.igate_object) >= 3)
  {
    char object[10];
    memset(object, 0, 10);
    strcpy(object, config.igate_object);
    object[9] = 0;
    sprintf(loc, ")%s!%02d%02d.%02d%c%c%03d%02d.%02d%c%c", object, lat_dd, lat_mm, lat_ss, lat_ns, config.igate_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.igate_symbol[1]);
  }
  else
  {
    sprintf(loc, "!%02d%02d.%02d%c%c%03d%02d.%02d%c%c", lat_dd, lat_mm, lat_ss, lat_ns, config.igate_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.igate_symbol[1]);
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
  tnc2Raw += String(config.igate_phg) + String(strAltitude);
  tnc2Raw += comment + " " + String(config.igate_comment);
  return tnc2Raw;
}

String digi_position(double lat, double lon, double alt, String comment)
{
  String tnc2Raw = "";
  int lat_dd, lat_mm, lat_ss, lon_dd, lon_mm, lon_ss;
  char strtmp[500], loc[100];
  char lon_ew = 'E';
  char lat_ns = 'N';
  if (lat < 0)
    lat_ns = 'S';
  if (lon < 0)
    lon_ew = 'W';
  memset(strtmp, 0, sizeof(strtmp));
  memset(loc, 0, sizeof(loc));
  DD_DDDDDtoDDMMSS(lat, &lat_dd, &lat_mm, &lat_ss);
  DD_DDDDDtoDDMMSS(lon, &lon_dd, &lon_mm, &lon_ss);
  char strAltitude[12];
  memset(strAltitude, 0, sizeof(strAltitude));
  if (alt > 0)
  {
    sprintf(strAltitude, "/A=%06d", (int)(alt * 3.28F));
  }
  sprintf(loc, "!%02d%02d.%02d%c%c%03d%02d.%02d%c%c", lat_dd, lat_mm, lat_ss, lat_ns, config.digi_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.digi_symbol[1]);
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
  tnc2Raw += String(config.digi_phg) + String(strAltitude);
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
}

String sendIsAckMsg(String toCallSign, char *msgId)
{
  char str[300];
  char call[11];
  int i;
  memset(&call[0], 0, 11);
  strcpy(&call[0], toCallSign.c_str());
  i = strlen(call);
  for (; i < 9; i++)
    call[i] = 0x20;
  memset(&str[0], 0, 300);

  if (config.aprs_ssid > 0)
    sprintf(str, "%s-%d>APTWR,%s::%s:ack%s", config.aprs_mycall, config.aprs_ssid, config.igate_path, call, msgId);
  else
    sprintf(str, "%s>APTWR,%s::%s:ack%s", config.aprs_mycall, config.igate_path, call, msgId);
  return String(str);
}

void sendIsPkg(char *raw)
{
  char str[500];
  sprintf(str, "%s-%d>APTWR%s:%s", config.aprs_mycall, config.aprs_ssid, VERSION, raw);
  String tnc2Raw = String(str);
  if (aprsClient.connected())
    aprsClient.println(tnc2Raw); // Send packet to Inet
  if (config.rf_en && config.digi_en)
    pkgTxPush(str, strlen(str), 0);
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
    sprintf(str, "%s>APTWR::%s:%s", config.aprs_mycall, call, raw);
  else
    sprintf(str, "%s-%d>APTWR::%s:%s", config.aprs_mycall, config.aprs_ssid, call, raw);

  String tnc2Raw = String(str);
  if (aprsClient.connected())
    aprsClient.println(tnc2Raw); // Send packet to Inet
  if (config.rf_en && config.digi_en)
    pkgTxPush(str, strlen(str), 0);
  // APRS_sendTNC2Pkt(tnc2Raw); // Send packet to RF
}

void taskGPS(void *pvParameters)
{
  unsigned long gpsTickInterval;
  GPS_INIT();
  for (;;)
  {
    vTaskDelay(100 / portTICK_PERIOD_MS);

    while (SerialGPS.available())
      gps.encode(SerialGPS.read());

    if (firstGpsTime && gps.time.isValid())
    {
      if (gps.time.isUpdated())
      {
        time_t timeGps = getGpsTime(); // Local gps time
        if (timeGps > 1653152400 && timeGps < 2347462800)
        {
          setTime(timeGps);
          time_t rtc = timeGps;
          timeval tv = {rtc, 0};
          timezone tz = {TZ_SEC + DST_MN, 0};
          settimeofday(&tv, &tz);
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
      }
    }
  }
}

extern cppQueue adcq;
void taskTNC(void *pvParameters)
{
  unsigned long timer, result;
  for (;;)
  {
    // timer=micros();
    if (AFSKInitAct == true)
      AFSK_Poll(true, config.rf_power);
    // result=micros()-timer;
    // log_d("AFSK_Poll timer = %.2f mS",(float)result/1000);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

long timeSlot;
bool initInterval=true;
void taskAPRS(void *pvParameters)
{
  char sts[50];
  //	long start, stop;
  char *raw;
  char *str;
  // bool firstGpsTime = true;
  unsigned long tickInterval;
  unsigned long iGatetickInterval;
  unsigned long DiGiInterval;

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
  afskSetHPF(config.audio_hpf);
  afskSetBPF(config.audio_bpf);
  // AFSK_TimerEnable(true);

  unsigned long timeAprsOld = millis();
  unsigned long gpsTimeout;

  initInterval=true;
  tx_interval = config.trk_interval;
  tx_counter = tx_interval - 10;
  log_d("Task APRS has been start");
  int rxPerMin = 0;
  for (;;)
  {
    unsigned long now = millis();
    if(initInterval){
      tickInterval = DiGiInterval = iGatetickInterval = millis() + 15000;
      initInterval=false;
    }
    timeAprs = now - timeAprsOld;
    timeAprsOld = now;
    // wdtSensorTimer = now;
    time_t timeStamp;
    time(&timeStamp);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // serviceHandle();

    if (AFSKInitAct == true)
    {
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
              timeSlot = millis() + config.tx_timeslot; // Tx Time Slot >2sec.
            else
              timeSlot = millis() + 3000;
          }
          else
          {
            timeSlot = millis() + 1500;
          }
        }
        else
        {
          if (pkgTxSend())
            timeSlot = millis() + config.tx_timeslot; // Tx Time Slot > 2sec.
          else
            timeSlot = millis() + 3000;
        }
      }
    }

    if (config.trk_en)
    { // TRACKER MODE

      if (millis() > tickInterval)
      {
        tickInterval = millis() + 1000;

        tx_counter++;
        //log_d("TRACKER tx_counter=%d\t INTERVAL=%d\n", tx_counter, tx_interval);
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

        if (config.trk_gps && gps.speed.isValid() && gps.location.isValid() && gps.course.isValid() && (gps.hdop.hdop()<10.0) && (gps.satellites.value()>3))
        {
          SB_SPEED_OLD = SB_SPEED;
          if (gps.speed.isUpdated())
            SB_SPEED = (unsigned char)gps.speed.kmph();
          if (gps.course.isUpdated())
            SB_HEADING = (int16_t)gps.course.deg();
          if (config.trk_smartbeacon) // SMART BEACON CAL
          {
            if (SB_SPEED < config.trk_lspeed && SB_SPEED_OLD > config.trk_lspeed) // Speed slow down to STOP
            {                                                                     // STOPING
              SB_SPEED_OLD = 0;
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
      }

      if (EVENT_TX_POSITION > 0)
      {
        String rawData;
        String cmn = "";
        if (config.trk_sat)
          cmn += "SAT:" + String(gps.satellites.value())+",HDOP:"+String(gps.hdop.hdop(),1);
        if (config.trk_bat)
        {
          if (config.trk_sat)
            cmn += ",";
          // cmn += "BAT:" + String((float)PMU.getBattVoltage() / 1000, 1) + "V";
          cmn += "BAT:" + String(vbat, 1) + "V";
        }
        if (config.trk_gps) // TRACKER by GPS
        {
          rawData = trk_gps_postion(cmn);
        }
        else // TRACKER by FIX position
        {
          rawData = trk_fix_position(cmn);
        }

        log_d("TRACKER RAW: %s\n", rawData.c_str());
        log_d("TRACKER EVENT_TX_POSITION=%d\t INTERVAL=%d\n", EVENT_TX_POSITION, tx_interval);
        tx_counter = 0;
        EVENT_TX_POSITION = 0;
        last_heading = SB_HEADING;

        if (config.trk_gps)
        {
          if (gps.location.isValid() && (gps.hdop.hdop()<10.0))
            sprintf(sts, "POSITION GPS\nSPD %dkPh/%d\nINTERVAL %ds", SB_SPEED, SB_HEADING, tx_interval);
          else
            sprintf(sts, "POSITION GPS\nGPS INVALID\nINTERVAL %ds", tx_interval);
        }
        else
        {
          sprintf(sts, "POSITION FIX\nINTERVAL %ds", tx_interval);
        }

        if (config.trk_loc2rf)
        { // TRACKER SEND TO RF
          char *rawP = (char *)malloc(rawData.length());
          memcpy(rawP, rawData.c_str(), rawData.length());
          // rawData.toCharArray(rawP, rawData.length());
          pkgTxPush(rawP, rawData.length(), 0);
          pushTxDisp(TXCH_RF, "TX TRACKER", sts);
          free(rawP);
        }
        if (config.trk_loc2inet)
        { // TRACKER SEND TO APRS-IS
          if (aprsClient.connected())
          {
            aprsClient.println(rawData); // Send packet to Inet
            pushTxDisp(TXCH_TCP, "TX TRACKER", sts);
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
    //             sprintf(rawTlm, "%s>APTWR:T#%03d,%d,%d,%d,%d,%d,00000000", config.aprs_mycall, igateTLM.Sequence, igateTLM.RF2INET, igateTLM.INET2RF, igateTLM.RX, igateTLM.TX, igateTLM.DROP);
    //         else
    //             sprintf(rawTlm, "%s-%d>APTWR:T#%03d,%d,%d,%d,%d,%d,00000000", config.aprs_mycall, config.aprs_ssid, igateTLM.Sequence, igateTLM.RF2INET, igateTLM.INET2RF, igateTLM.RX, igateTLM.TX, igateTLM.DROP);

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
        if (BTdeviceConnected)
        {
          if (config.bt_mode == 1)
          {
            char *rawP = (char *)malloc(tnc2.length());
            memcpy(rawP, tnc2.c_str(), tnc2.length());
            pTxCharacteristic->setValue((uint8_t *)rawP, tnc2.length());
            pTxCharacteristic->notify();
            free(rawP);
          }
          else if (config.bt_mode == 2)
          { // KISS
            uint8_t pkg[500];
            int sz = kiss_wrapper(pkg);
            pTxCharacteristic->setValue(pkg, sz);
            pTxCharacteristic->notify();
          }
        }
      }

      log_d("RX: %s", tnc2.c_str());

      // SerialBT.println(tnc2);
      uint16_t type = pkgType((char *)incomingPacket.info);
      char call[11];
      if (incomingPacket.src.ssid > 0)
        sprintf(call, "%s-%d", incomingPacket.src.call, incomingPacket.src.ssid);
      else
        sprintf(call, "%s", incomingPacket.src.call);

      char *rawP = (char *)malloc(tnc2.length());
      memcpy(rawP, tnc2.c_str(), tnc2.length());
      int idx = pkgListUpdate(call, rawP, type, 0);
      free(rawP);
      if (idx > -1)
      {

        if (config.rx_display && config.dispRF && (type & config.dispFilter))
        {
          pushTNC2Raw(idx);
          log_d("RF_putQueueDisp:[pkgList_idx=%d,Type=%d] %s\n", idx, type, call);
        }
      }

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
            if (gps.location.isValid() && (gps.hdop.hdop()<10.0))
              rawData = igate_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters(), "");
          }
          else
          { // IGATE Send fix position
            rawData = igate_position(config.igate_lat, config.igate_lon, config.igate_alt, "");
          }
          if (rawData != "")
          {
            iGatetickInterval = millis() + (config.igate_interval * 1000);
            log_d("IGATE_POSITION: %s", rawData.c_str());

            if (config.igate_gps)
              sprintf(sts, "POSITION GPS\nINTERVAL %ds", tx_interval);
            else
              sprintf(sts, "POSITION FIX\nINTERVAL %ds", tx_interval);
            if (config.igate_loc2rf)
            { // IGATE SEND POSITION TO RF
              char *rawP = (char *)malloc(rawData.length());
              // rawData.toCharArray(rawP, rawData.length());
              memcpy(rawP, rawData.c_str(), rawData.length());
              pkgTxPush(rawP, rawData.length(), 0);
              pushTxDisp(TXCH_RF, "TX IGATE", sts);
              free(rawP);
            }
            if (config.igate_loc2inet)
            { // IGATE SEND TO APRS-IS
              if (aprsClient.connected())
              {
                status.txCount++;
                aprsClient.println(rawData); // Send packet to Inet
                pushTxDisp(TXCH_TCP, "TX IGATE", sts);
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
          int ret = 0;
          uint16_t type = pkgType((const char *)&incomingPacket.info[0]);
          // IGate Filter RF->INET
          if ((type & config.rf2inetFilter))
            ret = igateProcess(incomingPacket);
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
            if (gps.location.isValid() && (gps.hdop.hdop()<10.0))
              rawData = digi_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters(), "");
          }
          else
          { // DIGI Send fix position
            rawData = digi_position(config.digi_lat, config.digi_lon, config.digi_alt, "");
          }
          if (rawData != "")
          {
            DiGiInterval = millis() + (config.digi_interval * 1000);
            log_d("DIGI_POSITION: %s", rawData.c_str());

            if (config.digi_gps)
              sprintf(sts, "POSITION GPS\nINTERVAL %ds", tx_interval);
            else
              sprintf(sts, "POSITION FIX\nINTERVAL %ds", tx_interval);
            if (config.digi_loc2rf)
            { // DIGI SEND POSITION TO RF
              char *rawP = (char *)malloc(rawData.length());
              // rawData.toCharArray(rawP, rawData.length());
              memcpy(rawP, rawData.c_str(), rawData.length());
              pkgTxPush(rawP, rawData.length(), 0);
              pushTxDisp(TXCH_RF, "TX DIGI POS", sts);
              free(rawP);
            }
            if (config.digi_loc2inet)
            { // DIGI SEND TO APRS-IS
              if (aprsClient.connected())
              {
                status.txCount++;
                aprsClient.println(rawData); // Send packet to Inet
                pushTxDisp(TXCH_TCP, "TX DIGI POS", sts);
              }
            }
          }
        }
      }

      // Repeater packet
      if (newDigiPkg)
      {
        newDigiPkg = false;
        uint16_t type = pkgType((const char *)&incomingPacket.info[0]);
        // Digi repeater filter
        if ((type & config.digiFilter))
        {
          // Packet recheck
          pkgTxDuplicate(incomingPacket); // Search duplicate in tx and drop packet for renew
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
                // if (digiCount > 20)
                //   digiDelay = random(5000);
                // else if (digiCount > 10)
                //   digiDelay = random(3000);
                // else if (digiCount > 0)
                //   digiDelay = random(1500);
                // else
                  digiDelay = random(100);
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
            char *rawP = (char *)malloc(digiPkg.length());
            // digiPkg.toCharArray(rawP, digiPkg.length());
            memcpy(rawP, digiPkg.c_str(), digiPkg.length());
            pkgTxPush(rawP, digiPkg.length(), digiDelay);
            sprintf(sts, "--src call--\n%s\nDelay: %dms.", incomingPacket.src.call, digiDelay);
            pushTxDisp(TXCH_DIGI, "DIGI REPEAT", sts);
            free(rawP);
          }
        }
      }
    }
  }
}

int mqttRetry = 0;
long wifiTTL = 0;
WiFiMulti wifiMulti;

// WiFi connect timeout per AP. Increase when connecting takes longer.
const uint32_t connectTimeoutMs = 10000;

// void Wifi_connected(WiFiEvent_t event, WiFiEventInfo_t info){
//   log_d("Successfully connected to Access Point");
// }

// void Get_IPAddress(WiFiEvent_t event, WiFiEventInfo_t info){
//   log_d("WIFI is connected!");
//   log_d("IP address: %s",WiFi.localIP().toString().c_str());
// }

// void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info){
//   log_d("Disconnected from WIFI access point\n");
//   log_d("WiFi lost connection. Reason: ");
//   log_d("%s\n",info.wifi_sta_disconnected.reason);
//   log_d("Reconnecting...");
// }

void taskNetwork(void *pvParameters)
{
  int c = 0;
  log_d("Task Network has been start");

  // WiFi.onEvent(Wifi_connected,SYSTEM_EVENT_STA_CONNECTED);
  // WiFi.onEvent(Get_IPAddress, SYSTEM_EVENT_STA_GOT_IP);
  // WiFi.onEvent(Wifi_disconnected, SYSTEM_EVENT_STA_DISCONNECTED);

  if (config.wifi_mode == WIFI_STA_FIX)
  { /**< WiFi station mode */
    WiFi.mode(WIFI_MODE_STA);
  }
  else if (config.wifi_mode == WIFI_AP_FIX)
  { /**< WiFi soft-AP mode */
    WiFi.mode(WIFI_MODE_AP);
  }
  if (config.wifi_mode == WIFI_AP_STA_FIX)
  { /**< WiFi station + soft-AP mode */
    WiFi.mode(WIFI_MODE_APSTA);
  }
  else
  {
    WiFi.mode(WIFI_MODE_NULL);
  }

  if (config.wifi_mode & WIFI_STA_FIX)
  {
    for (int i = 0; i < 5; i++)
    {
      if (config.wifi_sta[i].enable)
      {
        wifiMulti.addAP(config.wifi_sta[i].wifi_ssid, config.wifi_sta[i].wifi_pass);
      }
    }
    WiFi.setTxPower((wifi_power_t)config.wifi_power);
    WiFi.setHostname("ESP32APRS_T-TWR");
  }

  if (config.wifi_mode & WIFI_AP_FIX)
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
    // if (WiFi.status() == WL_CONNECTED)
    if (wifiMulti.run(connectTimeoutMs) == WL_CONNECTED)
    {
      webService();
      serviceHandle();

      if (millis() > NTP_Timeout)
      {
        NTP_Timeout = millis() + 86400000;
        // Serial.println("Config NTP");
        // setSyncProvider(getNtpTime);
        log_d("Contacting Time Server\n");
        configTime(3600 * config.timeZone, 0, config.ntp_host);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        time_t systemTime;
        time(&systemTime);
        setTime(systemTime);
        if (systemUptime == 0)
        {
          systemUptime = time(NULL);
        }
        pingTimeout = millis() + 2000;
        if (config.vpn)
        {
          if (!wireguard_active())
          {
            log_d("Setup Wiregurad VPN!");
            wireguard_setup();
          }
        }
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
#ifdef BOARD_HAS_PSRAM
              // char *raw = (char *)malloc(line.length() + 1);
#else
              char *raw = (char *)malloc(line.length() + 1);
#endif
              String src_call = line.substring(0, start_val);
              String msg_call = "::" + src_call;

              status.allCount++;
              status.rxCount++;
              igateTLM.RX++;

              log_d("INET: %s\n", line.c_str());
              char raw[500];
              memset(&raw[0], 0, sizeof(raw));
              start_val = line.indexOf(":", 10); // Search of info in ax25
              if (start_val > 5)
              {
                String info = line.substring(start_val + 1);
                // info.toCharArray(&raw[0], info.length(), 0);
                memcpy(raw, info.c_str(), info.length());

                uint16_t type = pkgType(&raw[0]);
                int start_dstssid = line.indexOf("-", 1); // get SSID -
                if (start_dstssid < 0)
                  start_dstssid = line.indexOf(" ", 1); // get ssid space
                char ssid = 0;
                if (start_dstssid > 0)
                  ssid = line.charAt(start_dstssid + 1);

                if (ssid > 47 && ssid < 58)
                {
                  size_t len = src_call.length();
                  char call[15];
                  memset(call, 0, sizeof(call));
                  if (len > 15)
                    len = 15;
                  memcpy(call, src_call.c_str(), len);
                  call[14] = 0;
                  memset(raw, 0, sizeof(raw));
                  memcpy(raw, line.c_str(), line.length());
                  raw[sizeof(raw) - 1] = 0;
                  int idx = pkgListUpdate(call, raw, type, 1);
                  int cnt = 0;
                  if (idx > -1)
                  {
                    // Put queue affter filter for display popup
                    if (config.rx_display && config.dispINET && (type & config.dispFilter))
                    {
                      cnt = pushTNC2Raw(idx);
                      log_d("INET_putQueueDisp:[pkgList_idx=%d/queue=%d,Type=%d] %s\n", idx, cnt, type, call);
                    }
                  }

                  // INET2RF affter filter
                  if (config.rf_en && config.inet2rf)
                  {
                    if (type & config.inet2rfFilter)
                    {
                      char strtmp[300];
                      String tnc2Raw = "";
                      if (config.aprs_ssid == 0)
                        sprintf(strtmp, "%s>APTWR", config.aprs_mycall);
                      else
                        sprintf(strtmp, "%s-%d>APTWR", config.aprs_mycall, config.aprs_ssid);
                      tnc2Raw = String(strtmp);
                      tnc2Raw += ",RFONLY"; // fix path to rf only not send loop to inet
                      tnc2Raw += ":}";      // 3rd-party frame
                      tnc2Raw += line;
                      pkgTxPush(tnc2Raw.c_str(), tnc2Raw.length(), 0);
                      char sts[50];
                      sprintf(sts, "--SRC CALL--\n%s\n", src_call.c_str());
                      pushTxDisp(TXCH_3PTY, "TX INET->RF", sts);
                      status.inet2rf++;
                      igateTLM.INET2RF++;
                      log_d("INET2RF: %s\n", line);
                    }
                  }
                }
              }
            }
            // free(raw);
          }
        }
      }

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
          WiFi.reconnect();
          wifiTTL = 0;
        }
        if (config.vpn)
        {
          IPAddress vpnIP;
          vpnIP.fromString(String(config.wg_gw_address));
          log_d("Ping VPN to %s", vpnIP.toString().c_str());
          if (ping_start(vpnIP, 2, 0, 0, 10) == true)
          {
            log_d("VPN Ping Success!!");
          }
          else
          {
            log_d("VPN Ping Fail!");
            wireguard_remove();
            delay(3000);
            wireguard_setup();
          }
        }
      }
    }else if (config.wifi_mode & WIFI_AP_FIX){ // WiFi connected
      serviceHandle();
    }
  }   // for loop
}
