/*
 Name:		ESP32APRS T-TWR Plus
 Created:	13-10-2023 14:27:23
 Author:	HS5TQA/Atten
 Github:	https://github.com/nakhonthai/ESP32APRS_T-TWR
 Facebook:	https://www.facebook.com/atten
 Support IS: host:aprs.dprns.com port:14580 or aprs.hs5tqa.ampr.org:14580
 Support IS monitor: http://aprs.dprns.com:14501 or http://aprs.hs5tqa.ampr.org:14501
*/

#include <Arduino.h>
#include <task.h>
#include <limits.h>
#include <KISS.h>
#include <WiFiUdp.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClient.h>
#include <LITTLEFS.h>
#include <esp_task_wdt.h>
#include "main.h"
#include "XPowersLib.h"
//#include "cppQueue.h"
#include "digirepeater.h"
#include "igate.h"
#include "wireguardif.h"
#include "wireguard.h"
#include "wireguard_vpn.h"
#include "weather.h"
#include "sensor.h"
#include "ESP32Ping.h"
#include "webservice.h"
#include "LibAPRSesp.h"
#include "modem.h"

#include "time.h"

#include <TinyGPSPlus.h>
#include <pbuf.h>
#include <parse_aprs.h>

#include <WiFiUdp.h>

#include <WiFiClientSecure.h>

#include "AFSK.h"

#include "gui_lcd.h"

#include <NimBLEDevice.h>

#define DEBUG_TNC

#define EEPROM_SIZE 2048

#define FORMAT_LITTLEFS_IF_FAILED true
extern fs::LITTLEFSFS LITTLEFS;

bool i2c_busy = false;

#include <Wire.h>
#include "Adafruit_SSD1306.h"
#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>
#include "sa868.h"

#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel *strip;

//#define BOOT_PIN 0

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

struct pbuf_t aprs;
ParseAPRS aprsParse;

TinyGPSPlus gps;

#include <ModbusMaster.h>
// instantiate ModbusMaster object
ModbusMaster modbus;

#define VBAT_PIN 36
#define POWER_PIN SA868_PWR_PIN
#define PULLDOWN_PIN SA868_PD_PIN
#define SQL_PIN -1
HardwareSerial SerialRF(2); // RX, TX

extern bool pttON;
extern bool pttOFF;

SA868 sa868(&SerialRF, SA868_RX_PIN, SA868_TX_PIN);

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

long timeNetwork, timeAprs, timeGui;

unsigned long previousMillis = 0; // กำหนดตัวแปรเก็บค่า เวลาสุดท้ายที่ทำงาน
long interval = 10000;            // กำหนดค่าตัวแปร ให้ทำงานทุกๆ 10 วินาที
int conStat = 0;
int conStatNetwork = 0;

//cppQueue dispBuffer(300, 5, IMPLEMENTATION, true);

statusType status;
RTC_DATA_ATTR igateTLMType igateTLM;
RTC_DATA_ATTR dataTLMType systemTLM;
#ifdef BOARD_HAS_PSRAM
txQueueType *txQueue;
#else
RTC_DATA_ATTR txQueueType txQueue[PKGTXSIZE];
#endif

RTC_DATA_ATTR double LastLat, LastLng;
RTC_DATA_ATTR time_t lastTimeStamp;
// RTC_DATA_ATTR uint32_t COUNTER0_RAW;
// RTC_DATA_ATTR uint32_t COUNTER1_RAW;

extern RTC_DATA_ATTR uint8_t digiCount;

uint8_t Sleep_Activate = 0;
unsigned int StandByTick = 0;

bool lastHeard_Flag = 0;

String RF_VERSION;

XPowersAXP2101 PMU;

Configuration config;

pkgListType *pkgList;

TelemetryType *Telemetry;

long timeSleep = 0;

RTC_DATA_ATTR double VBat;
// RTC_DATA_ATTR double TempNTC;

RTC_NOINIT_ATTR uint16_t TLM_SEQ;
RTC_NOINIT_ATTR uint16_t IGATE_TLM_SEQ;
RTC_NOINIT_ATTR uint16_t DIGI_TLM_SEQ;

TaskHandle_t taskNetworkHandle;
TaskHandle_t taskAPRSHandle;
TaskHandle_t taskAPRSPollHandle;
TaskHandle_t mainDisplayHandle;
TaskHandle_t taskGPSHandle;
TaskHandle_t taskSerialHandle;
TaskHandle_t taskSensorHandle;

unsigned long timerNetwork, timerNetwork_old;
unsigned long timerAPRS, timerAPRS_old;
unsigned long timerGPS, timerGPS_old;
unsigned long timerSerial, timerSerial_old;

bool firstGpsTime = true;
time_t startTime = 0;
// HardwareSerial SerialGPS(2);
NimBLEServer *pServer = NULL;
NimBLECharacteristic *pTxCharacteristic,**pGATTCharacteristic;
bool BTdeviceConnected = false;
bool BToldDeviceConnected = false;
uint8_t BTtxValue = 0;

/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class ServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override
  {
    log_d("Client connect address: %s\n", connInfo.getAddress().toString().c_str());

    /**
     *  We can use the connection handle here to ask for different connection parameters.
     *  Args: connection handle, min connection interval, max connection interval
     *  latency, supervision timeout.
     *  Units; Min/Max Intervals: 1.25 millisecond increments.
     *  Latency: number of intervals allowed to skip.
     *  Timeout: 10 millisecond increments.
     */
    BTdeviceConnected = true;
    pServer->updateConnParams(connInfo.getConnHandle(), 48, 96, 1, 360);
  }

  void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override
  {
    log_d("Client disconnected - start advertising\n");
    NimBLEDevice::startAdvertising();
    BTdeviceConnected = false;
  }

  void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo) override
  {
    log_d("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
  }

  /********************* Security handled here *********************/
  uint32_t onPassKeyDisplay() override
  {
    log_d("Server Passkey Display\n");
    /**
     * This should return a random 6 digit number for security
     *  or make your own static passkey as done here.
     */
    return 123456;
  }

  void onConfirmPassKey(NimBLEConnInfo &connInfo, uint32_t pass_key) override
  {
    log_d("The passkey YES/NO number: %" PRIu32 "\n", pass_key);
    /** Inject false if passkeys don't match. */
    NimBLEDevice::injectConfirmPasskey(connInfo, true);
  }

  void onAuthenticationComplete(NimBLEConnInfo &connInfo) override
  {
    /** Check that encryption was successful, if not we disconnect the client */
    if (!connInfo.isEncrypted())
    {
      NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
      log_d("Encrypt connection failed - disconnecting client\n");
      return;
    }

    log_d("Secured connection to: %s\n", connInfo.getAddress().toString().c_str());
  }
} serverCallbacks;

/** Handler class for characteristic actions */
class CharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
  /**
   *  The value returned in code is the NimBLE host return code.
   */
  void onStatus(NimBLECharacteristic *pCharacteristic, int code) override
  {
    log_d("Notification/Indication return code: %d, %s\n", code, NimBLEUtils::returnCodeToString(code));
  }

  /** Peer subscribed to notifications/indications */
  void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue) override
  {
    std::string str = "Client ID: ";
    str += connInfo.getConnHandle();
    str += " Address: ";
    str += connInfo.getAddress().toString();
    if (subValue == 0)
    {
      str += " Unsubscribed to ";
    }
    else if (subValue == 1)
    {
      str += " Subscribed to notifications for ";
    }
    else if (subValue == 2)
    {
      str += " Subscribed to indications for ";
    }
    else if (subValue == 3)
    {
      str += " Subscribed to notifications and indications for ";
    }
    str += std::string(pCharacteristic->getUUID());
    log_d("%s\n", str.c_str());
  }
} chrCallbacks;

/** Handler class for descriptor actions */
class DescriptorCallbacks : public NimBLEDescriptorCallbacks
{
  void onWrite(NimBLEDescriptor *pDescriptor, NimBLEConnInfo &connInfo) override
  {
    std::string dscVal = pDescriptor->getValue();
    log_d("Descriptor written value: %s\n", dscVal.c_str());
  }

  void onRead(NimBLEDescriptor *pDescriptor, NimBLEConnInfo &connInfo) override
  {
    log_d("%s Descriptor read\n", pDescriptor->getUUID().toString().c_str());
  }
} dscCallbacks;

extern SensorData sen[SENSOR_NUMBER];

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
unsigned long upTimeStamp = 0;
void convertSecondsToDHMS(char *dmhs, unsigned long totalSeconds)
{
  // Calculate days, hours, minutes, and seconds
  unsigned int days = totalSeconds / 86400;           // 86400 seconds in a day
  unsigned int hours = (totalSeconds % 86400) / 3600; // 3600 seconds in an hour
  unsigned int minutes = (totalSeconds % 3600) / 60;  // 60 seconds in a minute
  unsigned int seconds = totalSeconds % 60;

  sprintf(dmhs, "%dD[%d:%d:%d]", days, hours, minutes, seconds);
  // Print the result in D:H:M:S format
  // Serial.print("Time: ");
  // Serial.print(days);
  // Serial.print("D ");
  // Serial.print(hours);
  // Serial.print("H ");
  // Serial.print(minutes);
  // Serial.print("M ");
  // Serial.print(seconds);
  // Serial.println("S");
}

class MyServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer)
  {
    BTdeviceConnected = true;
    log_d("BLE Connected");
  };

  void onDisconnect(NimBLEServer *pServer)
  {
    BTdeviceConnected = false;
    log_d("BLE Disconnect");
  }
};

class MyCallbacks : public NimBLECharacteristicCallbacks
{
  void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
  {
    log_d("%s : onRead(), value: %s\n",
          pCharacteristic->getUUID().toString().c_str(),
          pCharacteristic->getValue().c_str());
  }
  // void onWrite(NimBLECharacteristic *pCharacteristic)
  //{
  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
  {
    log_d("%s : onWrite(), Length: %d\n",
          pCharacteristic->getUUID().toString().c_str(),
          pCharacteristic->getLength());    

    if (pCharacteristic->getLength() > 0)
    {
      // Serial.println("*********");
      // Serial.print("Received Value: ");
      // char raw[500];
      // int i = 0;
      // memset(raw, 0, sizeof(raw));
      // for (i = 0; i < rxValue.length(); i++)
      // {
      //   if (i > sizeof(raw))
      //     break;
      //   raw[i] = (char)rxValue[i];
      //   if (raw[i] == 0)
      //     break;
      // }
      if (config.bt_mode == 1)
      { // TNC2RAW MODE
        String rxValue = pCharacteristic->getValue();
        uint8_t SendMode = TNC_CHANNEL;
        if (config.igate_loc2rf)
          SendMode |= RF_CHANNEL;
        if (config.igate_loc2inet)
          SendMode |= INET_CHANNEL;
        pkgTxPush(rxValue.c_str(), rxValue.length(), 1, SendMode);
      }
      else if (config.bt_mode == 2)
      {
        // KISS MODE   
        for(int i=0;i<pCharacteristic->getLength();i++)
          kiss_serial((uint8_t)pCharacteristic->getValue().c_str()[i]);
        //char raw[500];   
         
        // int len = kiss_serial((uint8_t *)raw,(uint8_t *)pCharacteristic->getValue().c_str(),pCharacteristic->getLength());
        // log_d("Parse KISS len=%d",len);
        // if(len>0){          
        //   // AX25Msg pkg;
        //   // String tnc2;
        //   // ax25_decode(raw, len, mV, &pkg);
        //   // packet2Raw(tnc2, pkg);
        //   // uint8_t SendMode = TNC_CHANNEL;
        //   // SendMode |= RF_CHANNEL;
        //   // pkgTxPush(tnc2.c_str(), tnc2.length(), 1, SendMode);
        //   APRS_sendRawPkt((uint8_t *)raw, len);
        // }
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

void PowerOn()
{
#ifdef ST7735_LED_K_Pin
    ledcWrite(0, (uint32_t)config.disp_brightness);
#endif
    // Power ON
    if(config.pwr_gpio>-1){
      if (config.pwr_active)
      {
          pinMode(config.pwr_gpio, OUTPUT);
          digitalWrite(config.pwr_gpio, HIGH);
      }
      else
      {
          pinMode(config.pwr_gpio, OUTPUT_OPEN_DRAIN);
          digitalWrite(config.pwr_gpio, LOW);
      }
    }
}

void PowerOff()
{
#ifdef ST7735_LED_K_Pin
    ledcWrite(0, 0);
#endif
    // Power OFF
    if(config.pwr_gpio>-1){
      if (config.pwr_active)
      {
          pinMode(config.pwr_gpio, OUTPUT);
          digitalWrite(config.pwr_gpio, LOW);
      }
      else
      {
          pinMode(config.pwr_gpio, OUTPUT_OPEN_DRAIN);
          digitalWrite(config.pwr_gpio, HIGH);
      }
    }
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

void logTracker(double lat, double lon, double speed, double course)
{
  char data[200];
  char datetime[30];
  char dfName[30];
  bool header = false;
  double dist;
  time_t nowTime;

  if (gps.time.isValid())
  {
    time_t timeGps = getGpsTime(); // Local gps time
    time(&nowTime);
    int tdiff = abs(timeGps - nowTime);
    if (timeGps > 1700000000 && tdiff > 2) // && timeGps < 2347462800)
    {
      nowTime = timeGps;
      setTime(nowTime);
      time_t rtc = nowTime - (time_t)(config.timeZone * (float)SECS_PER_HOUR);
      timeval tv = {rtc, 0};
      timezone tz = {(int)(config.timeZone * (float)SECS_PER_HOUR), 0};
      settimeofday(&tv, &tz);
    }
  }
  else
  {
    time(&nowTime);
  }

  String col;
  struct tm tmstruct;
  getLocalTime(&tmstruct, 100);
  sprintf(dfName, "/trk_%02d%d.csv", (tmstruct.tm_mon) + 1, (tmstruct.tm_year) + 1900);

  if (lastTimeStamp == 0)
    lastTimeStamp = nowTime;
  time_t tdiff = nowTime - lastTimeStamp;
  double nowLat = gps.location.lat();
  double nowLng = gps.location.lng();
  double spd = gps.speed.kmph();

  // dist = distance(LastLng, LastLat, nowLng, nowLat);

  if (spd < 5)
  {

    dist = gps.distanceBetween(LastLng, LastLat, nowLng, nowLat);
    course = direction(LastLng, LastLat, nowLng, nowLat);
    if (dist > 50.0F)
      dist = 0;
    if (tdiff > 10 && (nowTime > lastTimeStamp))
      speed = dist / ((double)tdiff / 3600);
    else
      speed = 0.0F;

    if (speed > 5)
    {
      speed = gps.speed.kmph();
      course = gps.course.deg();
    }
  }
  LastLat = nowLat;
  LastLng = nowLng;
  lastTimeStamp = nowTime;
  // if (!waterTempFlag)
  // 		{
  // ds18b20.setWaitForConversion(false); // makes it async
  // ds18b20.requestTemperatures();
  // ds18b20.setWaitForConversion(true);
  // waterTemperature = ds18b20.getTempCByIndex(0);
  // if (waterTemperature > 0)
  // 	waterTempFlag = true;
  //}

  if (!LITTLEFS.exists(dfName))
  {
    header = true;
  }

  File f = LITTLEFS.open(dfName, "a");
#ifdef DEBUG
  log_d("====== Writing to data file =========");
  log_d("Open File %s", dfName);
#endif
  if (!f)
  {
#ifdef DEBUG
    log_d("Data file open failed");
#endif
  }
  else
  {
    if (header)
    {
      // col = "\"dd/mm/yyyy h:m\",\"Weight Origin(Kg)\",\"Weight(Kg)\",\"Water Drain(L)\",\"Water Height(mm)\",\"Temperature(C)\",\"Raw\"";
      col = "dd/mm/yyyy h:m:s";
      col += ",Latitude";
      col += ",Longitude";
      col += ",Speed(kph)";
      col += ",Course(°)";
      for (int i = 0; i < 5; i++)
      {
        if (config.trk_tlm_sensor[i] > 0)
        {
          col += "," + String(config.trk_tlm_PARM[i]) + "(" + String(config.trk_tlm_UNIT[i]) + ")";
        }
      }
      f.println(col);
    }
    sprintf(datetime, "%d-%02d-%02d %02d:%02d:%02d", (tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
    sprintf(data, "%s,%0.5f,%0.5f,%0.2f,%d", datetime, lat, lon, speed, (int)course);
    for (int s = 0; s < 5; s++)
    {
      if (config.trk_tlm_sensor[s] == 0)
      {
        continue;
        // strcat(tlm_data, "0");
      }
      else
      {
        strcat(data, ",");
        int sen_idx = config.trk_tlm_sensor[s] - 1;
        double val = 0;
        if (sen[sen_idx].visable)
        {
          if (config.trk_tlm_avg[s] && (sen[sen_idx].timeAvg > 0))
            val = sen[sen_idx].average;
          else
            val = sen[sen_idx].sample;
        }
        strcat(data, String(val, 2).c_str());
      }
    }
    f.println(data);
    f.close();
#ifdef DEBUG
    log_d("Data file updated");
#endif
  }
}

void logIGate(double lat, double lon, double speed, double course)
{
  char data[200];
  char datetime[30];
  char dfName[30];
  bool header = false;
  double dist;
  time_t nowTime;

  if (gps.time.isValid())
  {
    time_t timeGps = getGpsTime(); // Local gps time
    time(&nowTime);
    int tdiff = abs(timeGps - nowTime);
    if (timeGps > 1700000000 && tdiff > 2) // && timeGps < 2347462800)
    {
      nowTime = timeGps;
      setTime(nowTime);
      time_t rtc = nowTime - (time_t)(config.timeZone * (float)SECS_PER_HOUR);
      timeval tv = {rtc, 0};
      timezone tz = {(int)(config.timeZone * (float)SECS_PER_HOUR), 0};
      settimeofday(&tv, &tz);
    }
  }
  else
  {
    time(&nowTime);
  }

  String col;
  struct tm tmstruct;
  getLocalTime(&tmstruct, 100);
  sprintf(dfName, "/igate_%02d%d.csv", (tmstruct.tm_mon) + 1, (tmstruct.tm_year) + 1900);

  if (lastTimeStamp == 0)
    lastTimeStamp = nowTime;
  time_t tdiff = nowTime - lastTimeStamp;
  double nowLat = gps.location.lat();
  double nowLng = gps.location.lng();
  double spd = gps.speed.kmph();

  // dist = distance(LastLng, LastLat, nowLng, nowLat);

  if (spd < 5)
  {

    dist = gps.distanceBetween(LastLng, LastLat, nowLng, nowLat);
    course = direction(LastLng, LastLat, nowLng, nowLat);
    if (dist > 50.0F)
      dist = 0;
    if (tdiff > 10 && (nowTime > lastTimeStamp))
      speed = dist / ((double)tdiff / 3600);
    else
      speed = 0.0F;

    if (speed > 5)
    {
      speed = gps.speed.kmph();
      course = gps.course.deg();
    }
  }
  LastLat = nowLat;
  LastLng = nowLng;
  lastTimeStamp = nowTime;

  if (!LITTLEFS.exists(dfName))
  {
    header = true;
  }

  File f = LITTLEFS.open(dfName, "a");
#ifdef DEBUG
  log_d("====== Writing to data file =========");
  log_d("Open File %s", dfName);
#endif
  if (!f)
  {
#ifdef DEBUG
    log_d("Data file open failed");
#endif
  }
  else
  {
    if (header)
    {
      // col = "\"dd/mm/yyyy h:m\",\"Weight Origin(Kg)\",\"Weight(Kg)\",\"Water Drain(L)\",\"Water Height(mm)\",\"Temperature(C)\",\"Raw\"";
      col = "dd/mm/yyyy h:m:s";
      col += ",Latitude";
      col += ",Longitude";
      col += ",Speed(kph)";
      col += ",Course(°)";
      for (int i = 0; i < 5; i++)
      {
        if (config.igate_tlm_sensor[i] > 0)
        {
          col += "," + String(config.igate_tlm_PARM[i]) + "(" + String(config.igate_tlm_UNIT[i]) + ")";
        }
      }
      f.println(col);
    }
    sprintf(datetime, "%d-%02d-%02d %02d:%02d:%02d", (tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
    sprintf(data, "%s,%0.5f,%0.5f,%0.2f,%d", datetime, lat, lon, speed, (int)course);
    for (int s = 0; s < 5; s++)
    {
      if (config.igate_tlm_sensor[s] == 0)
      {
        continue;
        // strcat(tlm_data, "0");
      }
      else
      {
        strcat(data, ",");
        int sen_idx = config.igate_tlm_sensor[s] - 1;
        double val = 0;
        if (sen[sen_idx].visable)
        {
          if (config.igate_tlm_avg[s] && (sen[sen_idx].timeAvg > 0))
            val = sen[sen_idx].average;
          else
            val = sen[sen_idx].sample;
        }
        strcat(data, String(val, 2).c_str());
      }
    }
    f.println(data);
    f.close();
#ifdef DEBUG
    log_d("Data file updated");
#endif
  }
}

void logDigi(double lat, double lon, double speed, double course)
{
  char data[200];
  char datetime[30];
  char dfName[30];
  bool header = false;
  double dist;
  time_t nowTime;

  if (gps.time.isValid())
  {
    time_t timeGps = getGpsTime(); // Local gps time
    time(&nowTime);
    int tdiff = abs(timeGps - nowTime);
    if (timeGps > 1700000000 && tdiff > 2) // && timeGps < 2347462800)
    {
      nowTime = timeGps;
      setTime(nowTime);
      time_t rtc = nowTime - (time_t)(config.timeZone * (float)SECS_PER_HOUR);
      timeval tv = {rtc, 0};
      timezone tz = {(int)(config.timeZone * (float)SECS_PER_HOUR), 0};
      settimeofday(&tv, &tz);
    }
  }
  else
  {
    time(&nowTime);
  }

  String col;
  struct tm tmstruct;
  getLocalTime(&tmstruct, 100);
  sprintf(dfName, "/digi_%02d%d.csv", (tmstruct.tm_mon) + 1, (tmstruct.tm_year) + 1900);

  if (lastTimeStamp == 0)
    lastTimeStamp = nowTime;
  time_t tdiff = nowTime - lastTimeStamp;
  double nowLat = gps.location.lat();
  double nowLng = gps.location.lng();
  double spd = gps.speed.kmph();

  // dist = distance(LastLng, LastLat, nowLng, nowLat);

  if (spd < 5)
  {

    dist = gps.distanceBetween(LastLng, LastLat, nowLng, nowLat);
    course = direction(LastLng, LastLat, nowLng, nowLat);
    if (dist > 50.0F)
      dist = 0;
    if (tdiff > 10 && (nowTime > lastTimeStamp))
      speed = dist / ((double)tdiff / 3600);
    else
      speed = 0.0F;

    if (speed > 5)
    {
      speed = gps.speed.kmph();
      course = gps.course.deg();
    }
  }
  LastLat = nowLat;
  LastLng = nowLng;
  lastTimeStamp = nowTime;

  if (!LITTLEFS.exists(dfName))
  {
    header = true;
  }

  File f = LITTLEFS.open(dfName, "a");
#ifdef DEBUG
  log_d("====== Writing to data file =========");
  log_d("Open File %s", dfName);
#endif
  if (!f)
  {
#ifdef DEBUG
    log_d("Data file open failed");
#endif
  }
  else
  {
    if (header)
    {
      // col = "\"dd/mm/yyyy h:m\",\"Weight Origin(Kg)\",\"Weight(Kg)\",\"Water Drain(L)\",\"Water Height(mm)\",\"Temperature(C)\",\"Raw\"";
      col = "dd/mm/yyyy h:m:s";
      col += ",Latitude";
      col += ",Longitude";
      col += ",Speed(kph)";
      col += ",Course(°)";
      for (int i = 0; i < 5; i++)
      {
        if (config.digi_tlm_sensor[i] > 0)
        {
          col += "," + String(config.digi_tlm_PARM[i]) + "(" + String(config.digi_tlm_UNIT[i]) + ")";
        }
      }
      f.println(col);
    }
    sprintf(datetime, "%d-%02d-%02d %02d:%02d:%02d", (tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
    sprintf(data, "%s,%0.5f,%0.5f,%0.2f,%d", datetime, lat, lon, speed, (int)course);
    for (int s = 0; s < 5; s++)
    {
      if (config.digi_tlm_sensor[s] == 0)
      {
        continue;
        // strcat(tlm_data, "0");
      }
      else
      {
        strcat(data, ",");
        int sen_idx = config.digi_tlm_sensor[s] - 1;
        double val = 0;
        if (sen[sen_idx].visable)
        {
          if (config.digi_tlm_avg[s] && (sen[sen_idx].timeAvg > 0))
            val = sen[sen_idx].average;
          else
            val = sen[sen_idx].sample;
        }
        strcat(data, String(val, 2).c_str());
      }
    }
    f.println(data);
    f.close();
#ifdef DEBUG
    log_d("Data file updated");
#endif
  }
}

void logWeather(double lat, double lon, double speed, double course)
{
  char data[500];
  char datetime[30];
  char dfName[30];
  bool header = false;
  double dist;
  time_t nowTime;

  if (gps.time.isValid())
  {
    time_t timeGps = getGpsTime(); // Local gps time
    time(&nowTime);
    int tdiff = abs(timeGps - nowTime);
    if (timeGps > 1700000000 && tdiff > 2) // && timeGps < 2347462800)
    {
      nowTime = timeGps;
      setTime(nowTime);
      time_t rtc = nowTime - (time_t)(config.timeZone * (float)SECS_PER_HOUR);
      timeval tv = {rtc, 0};
      timezone tz = {(int)(config.timeZone * (float)SECS_PER_HOUR), 0};
      settimeofday(&tv, &tz);
    }
  }
  else
  {
    time(&nowTime);
  }

  String col;
  struct tm tmstruct;
  getLocalTime(&tmstruct, 100);
  sprintf(dfName, "/wx_%02d%d.csv", (tmstruct.tm_mon) + 1, (tmstruct.tm_year) + 1900);

  if (lastTimeStamp == 0)
    lastTimeStamp = nowTime;
  time_t tdiff = nowTime - lastTimeStamp;
  double nowLat = gps.location.lat();
  double nowLng = gps.location.lng();
  double spd = gps.speed.kmph();

  // dist = distance(LastLng, LastLat, nowLng, nowLat);

  if (spd < 5)
  {

    dist = gps.distanceBetween(LastLng, LastLat, nowLng, nowLat);
    course = direction(LastLng, LastLat, nowLng, nowLat);
    if (dist > 50.0F)
      dist = 0;
    if (tdiff > 10 && (nowTime > lastTimeStamp))
      speed = dist / ((double)tdiff / 3600);
    else
      speed = 0.0F;

    if (speed > 5)
    {
      speed = gps.speed.kmph();
      course = gps.course.deg();
    }
  }
  LastLat = nowLat;
  LastLng = nowLng;
  lastTimeStamp = nowTime;

  if (!LITTLEFS.exists(dfName))
  {
    header = true;
  }

  File f = LITTLEFS.open(dfName, "a");
#ifdef DEBUG
  log_d("====== Writing to data file =========");
  log_d("Open File %s", dfName);
#endif
  if (!f)
  {
#ifdef DEBUG
    log_d("Data file open failed");
#endif
  }
  else
  {
    if (header)
    {
      // col = "\"dd/mm/yyyy h:m\",\"Weight Origin(Kg)\",\"Weight(Kg)\",\"Water Drain(L)\",\"Water Height(mm)\",\"Temperature(C)\",\"Raw\"";
      col = "dd/mm/yyyy h:m:s";
      col += ",Latitude";
      col += ",Longitude";
      col += ",Speed(kph)";
      col += ",Course(°)";
      for (int s = 0; s < WX_SENSOR_NUM; s++)
      {
        int senIdx = config.wx_sensor_ch[s];
        if ((config.wx_sensor_enable[s] == 0) || (senIdx == 0))
        {
          continue;
        }
        else
        {
          senIdx -= 1;
          col += "," + String(String(WX_SENSOR[s])) + "(" + String(config.sensor[senIdx].unit) + ")";
        }
      }
      f.println(col);
    }
    sprintf(datetime, "%d-%02d-%02d %02d:%02d:%02d", (tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
    sprintf(data, "%s,%0.5f,%0.5f,%0.2f,%d", datetime, lat, lon, speed, (int)course);
    for (int s = 0; s < WX_SENSOR_NUM; s++)
    {
      int senIdx = config.wx_sensor_ch[s];
      if ((config.wx_sensor_enable[s] == 0) || (senIdx == 0))
      {
        continue;
      }
      else
      {
        strcat(data, ",");
        senIdx -= 1;
        double val = 0;
        if (sen[senIdx].visable)
        {
          if ((config.wx_sensor_avg[s] && (config.sensor[senIdx].averagerate <= config.sensor[senIdx].samplerate)) || (sen[senIdx].timeAvg == 0))
            val = sen[senIdx].sample;
          else
            val = sen[senIdx].average;
        }
        strcat(data, String(val, 2).c_str());
      }
    }
    f.println(data);
    f.close();
#ifdef DEBUG
    log_d("Data file updated");
#endif
  }
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
  sprintf(config.wifi_ap_ssid, "ESP32APRS_TWR");
  sprintf(config.wifi_ap_pass, "aprsthnetwork");
  // Blutooth
  config.bt_slave = false;
  config.bt_master = false;
  config.bt_mode = 1; // 0-None,1-TNC2RAW,2-KISS
  config.bt_power = 3;
  //Nordic UART UUID
  // sprintf(config.bt_uuid, "6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
  // sprintf(config.bt_uuid_rx, "6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
  // sprintf(config.bt_uuid_tx, "6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
  //GATT UUID Support APRSDroid
  sprintf(config.bt_uuid, "00000001-ba2a-46c9-ae49-01b0961f68bb");
  sprintf(config.bt_uuid_rx, "00000002-ba2a-46c9-ae49-01b0961f68bb");
  sprintf(config.bt_uuid_tx, "00000003-ba2a-46c9-ae49-01b0961f68bb");
  sprintf(config.bt_name, "ESP32APRS_TWR");
  config.bt_pin = 123456;

  //--RF Module
  config.rf_en = true;
  config.rf_type = RF_SA8x8_OpenEdit;
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
  config.modem_type = 1;

  config.rf_tx_gpio = 39;
  config.rf_rx_gpio = 48;
  config.rf_sql_gpio = -1;
  config.rf_pd_gpio = 40;
  config.rf_pwr_gpio = 38;
  config.rf_ptt_gpio = 41;
  config.rf_sql_active = 0;
  config.rf_pd_active = 1;
  config.rf_pwr_active = 1;
  config.rf_ptt_active = 1;
  config.adc_atten = 4;
  config.adc_dc_offset = 600;
  config.rf_baudrate = 9600;

  // IGATE
  config.igate_bcn = false;
  config.igate_en = false;
  config.rf2inet = true;
  config.inet2rf = false;
  config.igate_loc2rf = false;
  config.igate_loc2inet = true;
  config.rf2inetFilter = 0xFFF; // All
  config.inet2rfFilter = config.digiFilter = FILTER_OBJECT | FILTER_ITEM | FILTER_MESSAGE | FILTER_MICE | FILTER_POSITION | FILTER_WX;
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
  sprintf(config.igate_symbol, "Lz");
  memset(config.igate_object, 0, sizeof(config.igate_object));
  memset(config.igate_phg, 0, sizeof(config.igate_phg));
  config.igate_path = 8;
  sprintf(config.igate_comment, "");
  sprintf(config.igate_status, "IGATE MODE");
  config.igate_sts_interval = 1800;

  // DIGI REPEATER
  config.digi_en = false;
  config.digi_loc2rf = true;
  config.digi_loc2inet = false;
  config.digi_ssid = 3;
  config.digi_timestamp = false;
  sprintf(config.digi_mycall, "NOCALL");
  config.digi_path = 8;
  //--Position
  config.digi_gps = false;
  config.digi_lat = 13.7555;
  config.digi_lon = 100.4930;
  config.digi_alt = 0;
  config.digi_interval = 600;
  config.igate_timestamp = false;
  config.digi_delay = 0;
  config.digiFilter = FILTER_OBJECT | FILTER_ITEM | FILTER_MESSAGE | FILTER_MICE | FILTER_POSITION | FILTER_WX;

  sprintf(config.digi_symbol, "L#");
  memset(config.digi_phg, 0, sizeof(config.digi_phg));
  sprintf(config.digi_comment, "");
  sprintf(config.digi_status, "DIGI MODE");
  config.digi_sts_interval = 0;

  // Tracker
  config.trk_en = false;
  config.trk_loc2rf = true;
  config.trk_loc2inet = false;
  config.trk_bat = false;
  config.trk_sat = false;
  config.trk_dx = false;
  config.trk_ssid = 7;
  config.trk_timestamp = false;
  sprintf(config.trk_mycall, "NOCALL");
  config.trk_path = 2;

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
  config.trk_sts_interval = 0;

  // WX
  config.wx_en = false;
  config.wx_2rf = true;
  config.wx_2inet = true;
  // config.wx_channel = 0;
  config.wx_ssid = 13;
  sprintf(config.wx_mycall, "NOCALL");
  config.wx_path = 8;
  sprintf(config.wx_comment, "WX MODE");
  memset(config.wx_object, 0, sizeof(config.wx_object));
  config.wx_gps = false;
  config.wx_lat = 13.7555;
  config.wx_lon = 100.4930;
  config.wx_alt = 0;
  config.wx_interval = 600;
  config.wx_flage = 0;

  // Telemetry_0
  config.tlm0_en = false;
  config.tlm0_2rf = true;
  config.tlm0_2inet = true;
  config.tlm0_ssid = 0;
  sprintf(config.tlm0_mycall, "NOCALL");
  config.tlm0_path = 0;
  config.tlm0_data_interval = 600;
  config.tlm0_info_interval = 3600;
  sprintf(config.tlm0_PARM[0], "RF->INET");
  sprintf(config.tlm0_PARM[1], "INET->RF");
  sprintf(config.tlm0_PARM[2], "Repeater");
  sprintf(config.tlm0_PARM[3], "AllCount");
  sprintf(config.tlm0_PARM[4], "AllDrop");
  sprintf(config.tlm0_PARM[5], "IGATE");
  sprintf(config.tlm0_PARM[6], "DIGI");
  sprintf(config.tlm0_PARM[7], "WX");
  sprintf(config.tlm0_PARM[8], "SAT");
  sprintf(config.tlm0_PARM[9], "INET");
  sprintf(config.tlm0_PARM[10], "VPN");
  sprintf(config.tlm0_PARM[11], "4G");
  sprintf(config.tlm0_PARM[12], "FX25");

  for (int i = 0; i < 5; i++)
  {
    sprintf(config.tlm0_UNIT[i], "Pkts");
    config.tlm0_EQNS[i][0] = 0; // a av2 + bv + c
    config.tlm0_EQNS[i][1] = 1; // b
    config.tlm0_EQNS[i][2] = 0; // c
  }
  sprintf(config.tlm0_UNIT[5], "En");
  sprintf(config.tlm0_UNIT[6], "En");
  sprintf(config.tlm0_UNIT[7], "En");
  sprintf(config.tlm0_UNIT[8], "En");
  sprintf(config.tlm0_UNIT[9], "ON");
  sprintf(config.tlm0_UNIT[10], "ON");
  sprintf(config.tlm0_UNIT[11], "ON");
  sprintf(config.tlm0_UNIT[12], "ON");
  config.tlm0_BITS_Active = 0xFF;
  config.tml0_data_channel[0] = 2;
  config.tml0_data_channel[1] = 3;
  config.tml0_data_channel[2] = 4;
  config.tml0_data_channel[3] = 1;
  config.tml0_data_channel[4] = 5;
  config.tml0_data_channel[5] = 1;
  config.tml0_data_channel[6] = 2;
  config.tml0_data_channel[7] = 3;
  config.tml0_data_channel[8] = 4;
  config.tml0_data_channel[9] = 5;
  config.tml0_data_channel[10] = 6;
  config.tml0_data_channel[11] = 7;
  config.tml0_data_channel[12] = 8;
  sprintf(config.tlm0_comment, "SYSTEM STATUS");

  // OLED DISPLAY
  config.oled_enable = true;
  config.oled_timeout = 60;
  config.dim = 0;
  config.contrast = 0;
  config.startup = 0;

  // Display
  config.disp_brightness = 250;
  config.disp_flip = false;
  config.dispDelay = 3; // Popup display 3 sec
  config.dispRF = true;
  config.dispINET = false;
  config.filterDistant = 0;
  config.dispFilter = FILTER_OBJECT | FILTER_ITEM | FILTER_MESSAGE | FILTER_MICE | FILTER_POSITION | FILTER_WX | FILTER_STATUS | FILTER_BUOY | FILTER_QUERY;
  config.h_up = true;
  config.tx_display = true;
  config.rx_display = true;
  config.audio_hpf = false;
  config.audio_lpf = false;
  config.preamble = 3;
  sprintf(config.ntp_host, "ntp.dprns.com");

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

  // config.gpio_sql_pin = -1;

  config.gnss_enable = true;
  config.gnss_channel = 2;
  config.gnss_tcp_port = 8080;
  sprintf(config.gnss_tcp_host, "192.168.0.1");
  memset(config.gnss_at_command, 0, sizeof(config.gnss_at_command));

  config.sensor[0].enable = false;
  config.sensor[0].port = 0;
  config.sensor[0].address = 2;
  config.sensor[0].samplerate = 10;
  config.sensor[0].averagerate = 600;
  config.sensor[0].eqns[0] = 0; // a
  config.sensor[0].eqns[1] = 1; // b
  config.sensor[0].eqns[2] = 0; // c
  config.sensor[0].type = SENSOR_TEMPERATURE;
  sprintf(config.sensor[0].parm, "Temperature");
  sprintf(config.sensor[0].unit, "°C");

  config.sensor[1].enable = false;
  config.sensor[1].port = 0;
  config.sensor[1].address = 2;
  config.sensor[1].samplerate = 10;
  config.sensor[1].averagerate = 600;
  config.sensor[1].eqns[0] = 0; // a
  config.sensor[1].eqns[1] = 1; // b
  config.sensor[1].eqns[2] = 0; // c
  config.sensor[1].type = SENSOR_HUMIDITY;
  sprintf(config.sensor[1].parm, "Humidity");
  sprintf(config.sensor[1].unit, "%%RH");

  config.sensor[2].enable = false;
  config.sensor[2].port = 0;
  config.sensor[2].address = 2;
  config.sensor[2].samplerate = 10;
  config.sensor[2].averagerate = 600;
  config.sensor[2].eqns[0] = 0; // a
  config.sensor[2].eqns[1] = 1; // b
  config.sensor[2].eqns[2] = 0; // c
  config.sensor[2].type = SENSOR_PM25;
  sprintf(config.sensor[2].parm, "PM2.5");
  sprintf(config.sensor[2].unit, "μg/m³");

  config.sensor[3].enable = false;
  config.sensor[3].port = 0;
  config.sensor[3].address = 2;
  config.sensor[3].samplerate = 10;
  config.sensor[3].averagerate = 600;
  config.sensor[3].eqns[0] = 0; // a
  config.sensor[3].eqns[1] = 1; // b
  config.sensor[3].eqns[2] = 0; // c
  config.sensor[3].type = SENSOR_PM100;
  sprintf(config.sensor[3].parm, "PM10.0");
  sprintf(config.sensor[3].unit, "μg/m³");

  config.sensor[4].enable = false;
  config.sensor[4].port = 0;
  config.sensor[4].address = 2;
  config.sensor[4].samplerate = 10;
  config.sensor[4].averagerate = 600;
  config.sensor[4].eqns[0] = 0; // a
  config.sensor[4].eqns[1] = 1; // b
  config.sensor[4].eqns[2] = 0; // c
  config.sensor[4].type = SENSOR_CO2;
  sprintf(config.sensor[4].parm, "CO2");
  sprintf(config.sensor[4].unit, "ppm");

  config.sensor[5].enable = false;
  config.sensor[5].port = 0;
  config.sensor[5].address = 2;
  config.sensor[5].samplerate = 10;
  config.sensor[5].averagerate = 600;
  config.sensor[5].eqns[0] = 0; // a
  config.sensor[5].eqns[1] = 1; // b
  config.sensor[5].eqns[2] = 0; // c
  config.sensor[5].type = SENSOR_CH2O;
  sprintf(config.sensor[5].parm, "CH2O");
  sprintf(config.sensor[5].unit, "μg/m³");

  config.sensor[6].enable = false;
  config.sensor[6].port = 0;
  config.sensor[6].address = 2;
  config.sensor[6].samplerate = 10;
  config.sensor[6].averagerate = 600;
  config.sensor[6].eqns[0] = 0; // a
  config.sensor[6].eqns[1] = 1; // b
  config.sensor[6].eqns[2] = 0; // c
  config.sensor[6].type = SENSOR_TVOC;
  sprintf(config.sensor[6].parm, "TVOC");
  sprintf(config.sensor[6].unit, "μg/m³");

  config.sensor[7].enable = false;
  config.sensor[7].port = 0;
  config.sensor[7].address = 2;
  config.sensor[7].samplerate = 10;
  config.sensor[7].averagerate = 600;
  config.sensor[7].eqns[0] = 0; // a
  config.sensor[7].eqns[1] = 1; // b
  config.sensor[7].eqns[2] = 0; // c
  config.sensor[7].type = SENSOR_PRESSURE;
  sprintf(config.sensor[7].parm, "Pressure");
  sprintf(config.sensor[7].unit, "hPa(mBar)");

  config.sensor[8].enable = false;
  config.sensor[8].port = 0;
  config.sensor[8].address = 2;
  config.sensor[8].samplerate = 10;
  config.sensor[8].averagerate = 600;
  config.sensor[8].eqns[0] = 0;   // a
  config.sensor[8].eqns[1] = 0.2; // b
  config.sensor[8].eqns[2] = 0;   // c
  config.sensor[8].type = SENSOR_RAIN;
  sprintf(config.sensor[8].parm, "Rain");
  sprintf(config.sensor[8].unit, "mm.");

  config.sensor[9].enable = false;
  config.sensor[9].port = 0;
  config.sensor[9].address = 2;
  config.sensor[9].samplerate = 10;
  config.sensor[9].averagerate = 600;
  config.sensor[9].eqns[0] = 0; // a
  config.sensor[9].eqns[1] = 1; // b
  config.sensor[9].eqns[2] = 0; // c
  config.sensor[9].type = SENSOR_WIND_SPD;
  sprintf(config.sensor[9].parm, "Wind Speed");
  sprintf(config.sensor[9].unit, "kPh");

#ifdef CORE_DEBUG_LEVEL
  config.uart0_enable = false;
  config.uart0_baudrate = 115200;
  config.uart0_rx_gpio = 44;
  config.uart0_tx_gpio = 43;
  config.uart0_rts_gpio = -1;
#else
  config.uart0_enable = false;
  config.uart0_baudrate = 9600;
  config.uart0_rx_gpio = 3;
  config.uart0_tx_gpio = 1;
  config.uart0_rts_gpio = -1;
#endif

  config.uart1_enable = true;
  config.uart1_baudrate = 9600;
  config.uart1_rx_gpio = 5;
  config.uart1_tx_gpio = 6;
  config.uart1_rts_gpio = -1;

  // config.uart2_enable = false;
  // config.uart2_baudrate = 9600;
  // config.uart2_rx_gpio = 16;
  // config.uart2_tx_gpio = 17;
  // config.uart2_rts_gpio = -1;

  config.modbus_enable = false;
  config.modbus_de_gpio = -1;
  config.modbus_address = 0;
  config.modbus_channel = 0;

  config.onewire_enable = false;
  config.onewire_gpio = -1;

  config.pwr_en = false;
  config.pwr_mode = MODE_A;        // A=Continue,B=Wait for receive,C=Send and sleep
  config.pwr_sleep_interval = 600; // sec
  config.pwr_stanby_delay = 300;   // sec
  config.pwr_sleep_activate = ACTIVATE_TRACKER | ACTIVATE_WIFI;
  config.pwr_gpio = -1;
  config.pwr_active = 1;

  for (int i = 0; i < 5; i++)
  {
    config.trk_tlm_avg[i] = false;
    config.trk_tlm_sensor[i] = 0;
    config.trk_tlm_precision[i] = 0;
    config.trk_tlm_offset[i] = 0;
    memset(config.trk_tlm_UNIT[i], 0, 8);
    memset(config.trk_tlm_PARM[i], 0, 10);
    config.trk_tlm_EQNS[i][0] = 0; // a av2 + bv + c
    config.trk_tlm_EQNS[i][1] = 1; // b
    config.trk_tlm_EQNS[i][2] = 0; // c
  }

  for (int i = 0; i < 5; i++)
  {
    config.digi_tlm_avg[i] = false;
    config.digi_tlm_sensor[i] = 0;
    config.digi_tlm_precision[i] = 0;
    config.digi_tlm_offset[i] = 0;
    memset(config.digi_tlm_UNIT[i], 0, 8);
    memset(config.digi_tlm_PARM[i], 0, 10);
    config.digi_tlm_EQNS[i][0] = 0; // a av2 + bv + c
    config.digi_tlm_EQNS[i][1] = 1; // b
    config.digi_tlm_EQNS[i][2] = 0; // c
  }

  for (int i = 0; i < 5; i++)
  {
    config.igate_tlm_avg[i] = false;
    config.igate_tlm_sensor[i] = 0;
    config.igate_tlm_precision[i] = 0;
    config.igate_tlm_offset[i] = 0;
    memset(config.igate_tlm_UNIT[i], 0, 8);
    memset(config.igate_tlm_PARM[i], 0, 10);
    config.igate_tlm_EQNS[i][0] = 0; // a av2 + bv + c
    config.igate_tlm_EQNS[i][1] = 1; // b
    config.igate_tlm_EQNS[i][2] = 0; // c
  }

  for (int i = 0; i < WX_SENSOR_NUM; i++)
  {
    config.wx_sensor_enable[i] = false;
    config.wx_sensor_avg[i] = true;
    config.wx_sensor_ch[i] = 0;
  }

#ifdef OLED
  config.i2c_enable = true;
  config.i2c_sda_pin = I2C_SDA_SYS;
  config.i2c_sck_pin = I2C_SCL_SYS;
#else
  config.i2c_enable = false;
  config.i2c_sda_pin = -1;
  config.i2c_sck_pin = -1;
#endif
  config.i2c_freq = 400000;
  config.i2c1_enable = false;
  config.i2c1_sda_pin = -1;
  config.i2c1_sck_pin = -1;
  config.i2c1_freq = 100000;

  config.counter0_enable = false;
  config.counter0_active = 0;
  config.counter0_gpio = -1;

  config.counter1_enable = false;
  config.counter1_active = 0;
  config.counter1_gpio = -1;

  config.ext_tnc_enable = false;
  config.ext_tnc_channel = 0;
  config.ext_tnc_mode = 2;

  sprintf(config.path[0], "TRACE2-2");
  sprintf(config.path[1], "WIDE1-1");
  sprintf(config.path[2], "WIDE1-1,WIDE2-1");
  sprintf(config.path[3], "RFONLY");

  config.log = 0;
}

unsigned long NTP_Timeout;
unsigned long pingTimeout;

bool psramBusy = false;

bool waitPSRAM(bool state)
{
#ifdef BOARD_HAS_PSRAM
  if (state)
  {
    int i = 0;
    while (psramBusy)
    {
      delay(1);
      if (i++ > 1000)
      {
        log_e("PSRAM Busy Timeout");
        return false;
      }
    }
    return true;
  }
  else
  {
    psramBusy = false;
    return true;
  }
#else
  return true;
#endif
}

// const char *lastTitle = "LAST HEARD";

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
    if (Telemetry[i].time > now())
      Telemetry[i].time = 0;
  }
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

int pkgList_Find(char *call, char *object, uint16_t type)
{
  int i;
  for (i = 0; i < PKGLISTSIZE; i++)
  {
    if (pkgList[i].type == type)
    {
      if (strnstr(pkgList[i].calsign, call, strlen(call)) != NULL)
      {
        if (strnstr(pkgList[i].object, object, strlen(object)) != NULL)
          return i;
      }
    }
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
  if (waitPSRAM(true))
  {
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
    waitPSRAM(false);
  }
}

void sortPkgDesc(pkgListType a[], int size)
{
  pkgListType t;
  char *ptr1;
  char *ptr2;
  char *ptr3;
  ptr1 = (char *)&t;
  if (waitPSRAM(true))
  {
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
    waitPSRAM(false);
  }
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
    // ptr = strchr(raw, ':');
    // if (ptr != NULL)
    // {
    //   ptr++;
    //   type |= pkgType(ptr);
    // }
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
  if (waitPSRAM(true))
  {
    memset(&ret, 0, sizeof(pkgListType));
    if (idx < PKGLISTSIZE)
      memcpy(&ret, &pkgList[idx], sizeof(pkgListType));
    waitPSRAM(false);
  }
  return ret;
}

int pkgListUpdate(char *call, char *raw, uint16_t type, bool channel, uint16_t audioLvl)
{
  size_t len;
  if (*call == 0)
    return -1;
  if (*raw == 0)
    return -1;

  // int start_info = strchr(':',0);

  char callsign[11];
  char object[10];
  size_t sz = strlen(call);
  memset(callsign, 0, 11);
  if (sz > 10)
    sz = 10;
  // strncpy(callsign, call, sz);
  memcpy(callsign, call, sz);

  waitPSRAM(true);
  int i = -1;

  memset(object, 0, sizeof(object));
  if (type & FILTER_ITEM)
  {
    int x = 0;
    char *body = strchr(raw, ':');
    if (body != NULL)
    {
      body += 2;

      for (int z = 0; z < 9 && body[z] != '!' && body[z] != '_'; z++)
      {
        if (body[z] < 0x20 || body[z] > 0x7e)
        {
          log_d("\titem name has unprintable characters");
          break; /* non-printable */
        }
        object[x++] = body[z];
      }
    }
    i = pkgList_Find(callsign, object, type);
  }
  else if (type & FILTER_OBJECT)
  {
    int x = 0;
    char *body = strchr(raw, ':');
    // log_d("body=%s",body);
    if (body != NULL)
    {
      body += 2;
      for (int z = 0; z < 9; z++)
      {
        if (body[z] < 0x20 || body[z] > 0x7e)
        {
          log_d("\tobject name has unprintable characters");
          break; // non-printable
        }
        object[x++] = body[z];
        // if (raw[i] != ' ')
        //     namelen = i;
      }
    }
    i = pkgList_Find(callsign, object, type);
  }
  else
  {
    i = pkgList_Find(callsign, type);
  }

  if (i > PKGLISTSIZE)
  {
    psramBusy = false;
    return -1;
  }
  if (i > -1)
  { // Found call in old pkg
    if ((channel == 1) || (channel == pkgList[i].channel))
    {
      pkgList[i].time = time(NULL);
      pkgList[i].pkg++;
      pkgList[i].type = type;
      // memcpy(pkgList[i].object,object,sizeof(object));
      if (channel == 0)
      {
        pkgList[i].audio_level = (int16_t)audioLvl;
        //     pkgList[i].rssi = rssi;
        //     pkgList[i].snr = snr;
        //     pkgList[i].freqErr = freqErr;
      }
      else
      {
        pkgList[i].rssi = -140;
        pkgList[i].snr = 0;
        pkgList[i].freqErr = 0;
        pkgList[i].audio_level = 0;
      }
      len = strlen(raw);
      pkgList[i].length = len + 1;
#ifdef BOARD_HAS_PSRAM
      if (pkgList[i].raw != NULL)
      {
        pkgList[i].raw = (char *)ps_realloc(pkgList[i].raw, pkgList[i].length);
      }
      else
      {
        pkgList[i].raw = (char *)ps_calloc(pkgList[i].length, sizeof(char));
      }
#else
      if (pkgList[i].raw != NULL)
      {
        pkgList[i].raw = (char *)realloc(pkgList[i].raw, pkgList[i].length);
      }
      else
      {
        pkgList[i].raw = (char *)calloc(pkgList[i].length, sizeof(char));
      }
#endif
      if (pkgList[i].raw)
      {
        memcpy(pkgList[i].raw, raw, len);
        pkgList[i].raw[len] = 0;
        log_d("Update: pkgList_idx=%d callsign:%s object:%s", i, callsign, object);
      }
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
    if (strlen(object) > 3)
    {
      memcpy(pkgList[i].object, object, 9);
      pkgList[i].object[9] = 0;
    }
    else
    {
      memset(pkgList[i].object, 0, sizeof(pkgList[i].object));
    }
    if (channel == 0)
    {
      pkgList[i].audio_level = (int16_t)audioLvl;
      //     pkgList[i].rssi = rssi;
      //     pkgList[i].snr = snr;
      //     pkgList[i].freqErr = freqErr;
    }
    else
    {
      pkgList[i].rssi = -140;
      pkgList[i].snr = 0;
      pkgList[i].freqErr = 0;
      pkgList[i].audio_level = 0;
    }
    // strcpy(pkgList[i].calsign, callsign);
    memcpy(pkgList[i].calsign, callsign, strlen(callsign));
    len = strlen(raw);
    pkgList[i].length = len + 1;
#ifdef BOARD_HAS_PSRAM
    if (pkgList[i].raw != NULL)
    {
      pkgList[i].raw = (char *)ps_realloc(pkgList[i].raw, pkgList[i].length);
    }
    else
    {
      pkgList[i].raw = (char *)ps_calloc(pkgList[i].length, sizeof(char));
    }
#else
    if (pkgList[i].raw != NULL)
    {
      pkgList[i].raw = (char *)realloc(pkgList[i].raw, pkgList[i].length);
    }
    else
    {
      pkgList[i].raw = (char *)calloc(pkgList[i].length, sizeof(char));
    }
#endif
    if (pkgList[i].raw)
    {
      memcpy(pkgList[i].raw, raw, len);
      pkgList[i].raw[len] = 0;
      log_d("New: pkgList_idx=%d callsign:%s object:%s", i, callsign, object);
    }
  }
  waitPSRAM(false);
  event_lastHeard();
  lastHeard_Flag = true;
  return i;
}

bool pkgTxDuplicate(AX25Msg ax25)
{
  waitPSRAM(true);
  char callsign[12];
  for (int i = 0; i < PKGTXSIZE; i++)
  {
    if (txQueue[i].Active)
    {
      if (txQueue[i].Channel & INET_CHANNEL)
        continue;

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

int pkgTxCount()
{
    int count = 0;
    for (int i = 0; i < PKGTXSIZE; i++)
    {
        if (txQueue[i].Active)
        {
            count++;
        }
    }
    return count;
}

bool pkgTxPush(const char *info, size_t len, int dly, uint8_t Ch)
{
  char *ecs = strstr(info, ">");
  if (ecs == NULL)
    return false;
  waitPSRAM(true);

  // Add
  for (int i = 0; i < PKGTXSIZE; i++)
  {
    if (txQueue[i].Active == false)
    {
      if (len > sizeof(txQueue[i].Info))
        len = sizeof(txQueue[i].Info);
      memset(txQueue[i].Info, 0, sizeof(txQueue[i].Info));
      memcpy(&txQueue[i].Info[0], info, len);
      txQueue[i].length = len;
      txQueue[i].Delay = dly;
      txQueue[i].Active = true;
      txQueue[i].timeStamp = millis();
      txQueue[i].Channel = Ch;
      break;
    }
  }
  psramBusy = false;
  return true;
}

void burstAfterVoice()
{
  String rawData;
  String cmn = "";
  if (gps.location.isValid()) // TRACKER by GPS
  {
    rawData = trk_gps_postion(cmn);
  }
  else // TRACKER by FIX position
  {
    rawData = trk_fix_position(cmn);
  }
  digitalWrite(POWER_PIN, config.rf_power); // RF Power LOW
  digitalWrite(SA868_MIC_SEL, HIGH);        // Select = ESP2MIC
  status.txCount++;
  log_d("Burst TX->RF: %s\n", rawData.c_str());
  pkgTxPush(rawData.c_str(), rawData.length(), 0, RF_CHANNEL);
  // APRS_sendTNC2Pkt(rawData); // Send packet to RF

  // for (int i = 0; i < 100; i++)
  // {
  //   if (digitalRead(SA868_PTT_PIN))
  //     break;
  //   delay(50); // TOT 5sec
  // }
  digitalWrite(SA868_PWR_PIN, 0); // set RF Power Low
}

bool pkgTxSend()
{
  waitPSRAM(true);
  // char info[300];
  for (int i = 0; i < PKGTXSIZE; i++)
  {
    if (txQueue[i].Active)
    {
      int decTime = millis() - txQueue[i].timeStamp;
      if (txQueue[i].Channel & INET_CHANNEL)
      {
        if (config.igate_en == false)
        {
          txQueue[i].Channel &= ~INET_CHANNEL;
        }
        else
        {
          if (aprsClient.connected())
          {
            // status.txCount++;
            // aprsClient.printf("%s\r\n", txQueue[i].Info); // Send packet to Inet
            aprsClient.write(txQueue[i].Info, txQueue[i].length); // Send binary frame packet to APRS-IS (aprsc)
            aprsClient.write("\r\n");                             // Send CR LF the end frame packet
            txQueue[i].Channel &= ~INET_CHANNEL;
            log_d("TX->INET: %s", txQueue[i].Info);
            continue;
          }
        }
      }
      if (decTime > txQueue[i].Delay)
      {
        if (txQueue[i].Channel & RF_CHANNEL)
        {
          psramBusy = false;
          // if ((config.rf_type == RF_SR_1WV) || (config.rf_type == RF_SR_1WU) || (config.rf_type == RF_SR_1W350))
          // {
          //   digitalWrite(config.rf_pwr_gpio, LOW);
          //   if (config.rf_power ^ !config.rf_pwr_active)
          //     pinMode(config.rf_pwr_gpio, OPEN_DRAIN);
          //   else
          //     pinMode(config.rf_pwr_gpio, OUTPUT);
          // }
          // else
          // {
          //   digitalWrite(config.rf_pwr_gpio, config.rf_power ^ !config.rf_pwr_active); // ON RF Power H/L
          // }
          status.txCount++;
          log_d("TX->RF[%i]: %s\n", txQueue[i].length, txQueue[i].Info);
          APRS_setPreamble(config.preamble * 100); // Send packet to RF
          APRS_sendTNC2Pkt((uint8_t *)txQueue[i].Info, txQueue[i].length);
          igateTLM.TX++;

          // digitalWrite(config.rf_pwr_gpio, !config.rf_pwr_active); // OFF RF Power H/L
          // pinMode(config.rf_pwr_gpio, OUTPUT);
          txQueue[i].Channel &= ~RF_CHANNEL;
        }
      }

      if ((txQueue[i].Channel == 0) || (decTime > 60000))
      {
        txQueue[i].Channel = 0;
        txQueue[i].Active = false;
      }
    }
  }
  psramBusy = false;
  return true;
}

uint8_t *packetData;
// ฟังชั่นถูกเรียกมาจาก ax25_decode

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

void setupPowerRF(bool sts)
{
  // bool result = PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
  // if (result == false) {
  //     while (1) {
  //         Serial.println("PMU is not online...");
  //         delay(500);
  //     }
  // }
  //! DC3 Radio Pixels VDD , Don't change
  if(sts){
    PMU.setDC3Voltage(3400);
    PMU.enableDC3();
  }else{
    PMU.disableDC3();
  }
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
  return sa868.getRSSI();
}

String SA868_getVERSION()
{
  String data;
  int rssi;

  if (config.rf_type == RF_SA8x8_OpenEdit)
  {
    SA868_Version version = sa868.Version();
    char str[15];
    sprintf(str, "%d.%d.%d.%d", version.major, version.minor, version.patch, version.revision);
    return String(str);
  }
  else
  {
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
  return "-";
}

String FRS_getVERSION()
{
  String data;
  String version;

  SerialRF.printf("AT+DMOVER\r\n");
  if (SA868_waitResponse(data, "\r\n", 1000))
  {
    int st = data.indexOf("DMOVER:");
    if (st > 0)
    {
      version = data.substring(st + 7, data.indexOf("\r\n"));
    }
    else
    {
      version = "Not found";
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
  if (config.rf_type == RF_SA8x8_OpenEdit)
    sa868.setLowPower();
  else
    digitalWrite(POWER_PIN, LOW);
  digitalWrite(PULLDOWN_PIN, LOW);
  // PMU.disableDC3();
}

String hexToString(String hex)
{ // for String to HEX conversion
  String text = "";
  for (int k = 0; k < hex.length(); k++)
  {
    if (k % 2 != 0)
    {
      char temp[3];
      sprintf(temp, "%c%c", hex[k - 1], hex[k]);
      int number = (int)strtol(temp, NULL, 16);
      text += char(number);
    }
  }
  return text;
}

String ctcssToHex(unsigned int decValue, int section)
{ // use to convert the CTCSS reading which RF module needed
  if (decValue == 7777)
  {
    return hexToString("FF");
  }
  String d1d0 = String(decValue);
  if (decValue < 1000)
  {
    d1d0 = "0" + d1d0;
  }
  if (section == 1)
  {
    return hexToString(d1d0.substring(2, 4));
  }
  else
  {
    return hexToString(d1d0.substring(0, 2));
  }
}

void RF_MODULE(bool boot)
{
  String data;
  // todo find out what is config.rf_en
  if (config.rf_en == false)
  {
    RF_MODULE_SLEEP();
    setupPowerRF(false);
    return;
  }
  // if (config.rf_type == RF_NONE)
  //   return;
  log_d("RF Module %s Init", RF_TYPE[config.rf_type]);
  //! DC3 Radio & Pixels VDD , Don't change
  // PMU.setDC3Voltage(3400);
  // PMU.disableDC3();

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
    //SerialRF.begin(9600, SERIAL_8N1, SA868_RX_PIN, SA868_TX_PIN);
    //pinMode(BUTTON_PTT_PIN, INPUT_PULLUP); // PTT BUTTON
    pinMode(SA868_PWR_PIN, OUTPUT);
    digitalWrite(SA868_PWR_PIN, LOW); // RF POWER LOW

    pinMode(SA868_MIC_SEL, OUTPUT); // MIC_SEL
    digitalWrite(SA868_MIC_SEL, LOW);

    pinMode(SA868_PD_PIN, OUTPUT);
    digitalWrite(SA868_PD_PIN, LOW); // PWR OFF
    setupPowerRF(false);

    digitalWrite(SA868_PTT_PIN,LOW);
    
    // pinMode(config.rf_tx_gpio,OUTPUT);
    // pinMode(config.rf_rx_gpio,OUTPUT);
    // digitalWrite(config.rf_tx_gpio, LOW);
    // digitalWrite(config.rf_rx_gpio, LOW);
    

    //pinMode(SA868_PTT_PIN, OUTPUT);
    //digitalWrite(SA868_PTT_PIN, HIGH); // PTT HIGH

    delay(1000);
    setupPowerRF(true);
    delay(100);
    digitalWrite(SA868_PD_PIN, HIGH); // PWR ON
    //SerialRF.begin(config.rf_baudrate,SERIAL_8N1,config.rf_rx_gpio,config.rf_tx_gpio);
    delay(1000);
    if (config.rf_type == RF_SA8x8_OpenEdit)
    {
      sa868.init();
    }
  }
  // else
  // {
  //   digitalWrite(SA868_PD_PIN, LOW); // PWR LOW
  //   delay(1000);
  // }
  //digitalWrite(SA868_PD_PIN, HIGH); // PWR ON

  if (config.rf_type == RF_SA8x8_OpenEdit)
  {
      sa868.setBandwidth(config.band);
      sa868.setRxFrequency((uint32_t)(config.freq_rx * 1000000));
      sa868.setTxFrequency((uint32_t)(config.freq_tx * 1000000));
      sa868.setVolume(config.volume);
      sa868.setSqlThresh(config.sql_level);
      delay(100);
      sa868.TxOff();
      sa868.setLowPower();
      delay(100);
      sa868.RxOn();
  }
  else
  {
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
    if (config.rf_type == RF_SA8x8_OpenEdit)
      sa868.setLowPower();
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
      login = "user " + String(config.igate_object) + " pass " + String(passcode, DEC) + " vers ESP32APRS_T-TWR V" + String(VERSION) + String(VERSION_BUILD) + " filter " + String(config.aprs_filter);
    }
    else
    {
      uint16_t passcode = aprsParse.passCode(config.aprs_mycall);
      if (config.aprs_ssid == 0)
        login = "user " + String(config.aprs_mycall) + " pass " + String(passcode, DEC) + " vers ESP32APRS_T-TWR V" + String(VERSION) + String(VERSION_BUILD) + " filter " + String(config.aprs_filter);
      else
        login = "user " + String(config.aprs_mycall) + "-" + String(config.aprs_ssid) + " pass " + String(passcode, DEC) + " vers ESP32APRS_T-TWR V" + String(VERSION) + String(VERSION_BUILD) + " filter " + String(config.aprs_filter);
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
  bool result = false;
  int c = 0;
  while (result == false)
  {
    log_d("PMU is not online...");
    delay(500);
    result = PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA_SYS, I2C_SCL_SYS);
    if (result)
      break;
    c++;
    if (c > 10)
      return;
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

  log_d("===========================================================================");
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
  // PMU.disableLongPressShutdown();
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
bool AFSKInitAct = false;

void preTransmission()
{
  digitalWrite(config.modbus_de_gpio, 1);
}

void postTransmission()
{
  digitalWrite(config.modbus_de_gpio, 0);
}

char *htmlBuffer;
void setup()
{
  byte *ptr;
#ifdef BOARD_HAS_PSRAM
  pkgList = (pkgListType *)ps_malloc(sizeof(pkgListType) * PKGLISTSIZE);
  Telemetry = (TelemetryType *)malloc(sizeof(TelemetryType) * TLMLISTSIZE);
  txQueue = (txQueueType *)ps_malloc(sizeof(txQueueType) * PKGTXSIZE);
  // TNC2Raw = (int *)ps_malloc(sizeof(int) * PKGTXSIZE);
  heap_caps_malloc_extmem_enable(1000000);
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

  pinMode(BOOT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_OK_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PTT_PIN, INPUT_PULLUP); // PTT BUTTON
  LED_init(-1,-1,42);

  // setCpuFrequencyMhz(160);
  Serial.begin(115200);
  
  if (!LITTLEFS.begin(FORMAT_LITTLEFS_IF_FAILED))
  {
    log_d("LITTLEFS Mount Failed");
  }
  else
  {
    log_d("File system mounted");
    log_d("Total space: %lu\n", LITTLEFS.totalBytes());
    log_d("Use space:  %lu\n\n", LITTLEFS.usedBytes());
  }

  if (!LITTLEFS.exists("/default.cfg"))
  {
    log_d("Factory Default");
    defaultConfig();
    saveConfiguration("/default.cfg", config);
  }
  else
  {
    if (!loadConfiguration("/default.cfg", config))
      defaultConfig();
  }

  log_d("Start ESP32APRS_T-TWR V%s", String(VERSION).c_str());
  // log_d("Push BOOT after 3 sec for Factory Default config.");
  log_d("Total heap: %d", ESP.getHeapSize());

  afskSetSQL(config.rf_sql_gpio, config.rf_sql_active);
  afskSetPTT(config.rf_ptt_gpio, config.rf_ptt_active);
  afskSetPWR(config.rf_pwr_gpio, config.rf_pwr_active);

  if(config.rf_baudrate<4800) config.rf_baudrate = 9600;
  SerialRF.begin(config.rf_baudrate,SERIAL_8N1,config.rf_rx_gpio,config.rf_tx_gpio);

  config.i2c_enable = true;
  Wire.begin(config.i2c_sda_pin, config.i2c_sck_pin, config.i2c_freq);

  // Setup Power PMU AXP2101
  setupPower();
  //delay(500);
  // setupSDCard();
  //strip = new Adafruit_NeoPixel(1, PIXELS_PIN, NEO_GRB + NEO_KHZ800);
  //strip->begin();


  i2c_busy = true;
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

  delay(1000);
  LED_Status(250, 0, 0);
  display.fillRect(49, 49, 75, 14, 0);
  display.setCursor(70, 52);
  display.print("3 Sec");
  display.display();
  delay(1000);
  LED_Status(0, 250, 0);
  display.fillRect(49, 49, 75, 14, 0);
  display.setCursor(70, 52);
  display.print("2 Sec");
  display.display();
  delay(1000);
  display.fillRect(49, 49, 75, 14, 0);
  display.setCursor(70, 52);
  display.print("1 Sec");
  display.display();
  delay(1000);
  LED_Status(0, 0, 250);

  if (digitalRead(BOOT_PIN) == LOW)
  {
    defaultConfig();
    log_d("Manual Default configure!");
#ifdef OLED
    display.clearDisplay();
    display.setCursor(10, 22);
    display.print("Factory Reset!");
    display.display();
#endif
    while (digitalRead(BOOT_PIN) == LOW)
    {
      delay(500);
      LED_Status(0, 0, 0);
      delay(500);
      LED_Status(200, 200, 0);
    }
  }

  i2c_busy = false;
  LED_Status(0, 0, 0);

  log_d("UART config");
  if (config.uart0_enable)
  {
//#ifndef CORE_DEBUG_LEVEL
    Serial0.begin(config.uart0_baudrate, SERIAL_8N1, config.uart0_rx_gpio, config.uart0_tx_gpio);
    Serial0.setTimeout(50);
//#endif
  }
  if (config.uart1_enable)
  {
    Serial1.begin(config.uart1_baudrate, SERIAL_8N1, config.uart1_rx_gpio, config.uart1_tx_gpio);
    Serial1.setTimeout(50);
  }
  // if (config.uart2_enable)
  // {
  //     Serial2.begin(config.uart2_baudrate, SERIAL_8N1, config.uart2_rx_gpio, config.uart2_tx_gpio);
  // }
  log_d("MODBUS config");
  if (config.modbus_enable)
  {
    if (config.modbus_channel == 1)
    {
      modbus.begin(config.modbus_address, Serial);
    }
    else if (config.modbus_channel == 2)
    {
      modbus.begin(config.modbus_address, Serial1);
    }
// #ifdef __XTENSA__
//     else if (config.modbus_channel == 3)
//     {
//       modbus.begin(config.modbus_address, Serial2);
//     }
// #endif
    if (config.modbus_channel > 0 && config.modbus_channel < 4)
    {
      // Modbus slave ID 1
      if (config.modbus_de_gpio > -1)
      {
        pinMode(config.modbus_de_gpio, OUTPUT);
        // Callbacks allow us to configure the RS485 transceiver correctly
        modbus.preTransmission(preTransmission);
        modbus.postTransmission(postTransmission);
      }
    }
  }

  if (config.bt_master == true)
  {
    // Create the BLE Device
    NimBLEDevice::init(config.bt_name);
    NimBLEDevice::setPower(config.bt_power); /** +3db */
    // BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_MITM);  // The line you told me to add
    // BLESecurity *pSecurity = new BLESecurity();
    // pSecurity->setStaticPIN(config.bt_pin);

    // Create the BLE Server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(&serverCallbacks);

    //BLE Auth
    if(config.bt_pin>0){
     NimBLEDevice::setSecurityAuth(true, true, true); /** bonding, MITM, BLE secure connections */
     NimBLEDevice::setSecurityPasskey(config.bt_pin);
     NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); /** Display only passkey */
    }

    //pServer->getAdvertising()->addServiceUUID(config.bt_uuid);

    // Create the BLE Service as Nordic UART Service
    NimBLEService *pService = pServer->createService(config.bt_uuid);
    // Create a BLE TX Characteristic
    pTxCharacteristic = pService->createCharacteristic(config.bt_uuid_tx, NIMBLE_PROPERTY::NOTIFY);
    pTxCharacteristic->setCallbacks(&chrCallbacks);
    // Create a BLE RX Characteristic
    static NimBLECharacteristic *pRxCharacteristic = pService->createCharacteristic(config.bt_uuid_rx, NIMBLE_PROPERTY::WRITE);
    pRxCharacteristic->setCallbacks(new MyCallbacks());   

    // Start the service
    pService->start();
    pServer->startAdvertising();
  }

  i2c_busy = true;
  display.clearDisplay();
  display.setTextSize(1);
  display.display();
  i2c_busy = false;

  oledSleepTimeout = millis() + (config.oled_timeout * 1000);
  AFSKInitAct = false;
  log_d("GNSS config");
  if (config.gnss_enable)
  {
    //! ALDO4 GNSS VDD
    PMU.enableALDO4();
  }
  else
  {
    //! ALDO4 GNSS VDD
    PMU.disableALDO4();
  }

  // log_d("Free heap: %d", ESP.getFreeHeap());
  // log_d("Total PSRAM: %d", ESP.getPsramSize());
  // log_d("Free PSRAM: %d", ESP.getFreePsram());
  // log_d("Used PSRAM: %d", ESP.getPsramSize() - ESP.getFreePsram());

  // enableLoopWDT();
  // enableCore0WDT();
  // enableCore1WDT();
  firstGpsTime = true;
  // xTaskCreateStaticPinnedToCore(taskAPRS,"taskAPRS",8192,NULL,1,)
  //  Task 1
  xTaskCreatePinnedToCore(
      taskAPRS,        /* Function to implement the task */
      "taskAPRS",      /* Name of the task */
      8192,            /* Stack size in words */
      NULL,            /* Task input parameter */
      1,               /* Priority of the task */
      &taskAPRSHandle, /* Task handle. */
      0);              /* Core where the task should run */

  xTaskCreatePinnedToCore(
      taskAPRSPoll,        /* Function to implement the task */
      "taskAPRSPoll",      /* Name of the task */
      4096,                /* Stack size in words */
      NULL,                /* Task input parameter */
      2,                   /* Priority of the task */
      &taskAPRSPollHandle, /* Task handle. */
      0);                  /* Core where the task should run */

  if (config.gnss_enable)
  {
    xTaskCreatePinnedToCore(
        taskGPS,        /* Function to implement the task */
        "taskGPS",      /* Name of the task */
        3072,           /* Stack size in words */
        NULL,           /* Task input parameter */
        1,              /* Priority of the task */
        &taskGPSHandle, /* Task handle. */
        1);             /* Core where the task should run */
  }

  if (config.ext_tnc_enable)
  {
    xTaskCreatePinnedToCore(
        taskSerial,        /* Function to implement the task */
        "taskSerial",      /* Name of the task */
        2048,              /* Stack size in words */
        NULL,              /* Task input parameter */
        3,                 /* Priority of the task */
        &taskSerialHandle, /* Task handle. */
        1);                /* Core where the task should run */
  }

  if (config.wifi_mode != 0)
  {
    // Task 2
    xTaskCreatePinnedToCore(
        taskNetwork,        /* Function to implement the task */
        "taskNetwork",      /* Name of the task */
        12384,               /* Stack size in words */
        NULL,               /* Task input parameter */
        2,                  /* Priority of the task */
        &taskNetworkHandle, /* Task handle. */
        1);                 /* Core where the task should run */
  }
  // Task 3
  if (config.oled_enable)
  {
    xTaskCreatePinnedToCore(
        mainDisp,           /* Function to implement the task */
        "mainDisplay",      /* Name of the task */
        8192,               /* Stack size in words */
        NULL,               /* Task input parameter */
        1,                  /* Priority of the task */
        &mainDisplayHandle, /* Task handle. */
        1);                 /* Core where the task should run */
  }

  bool SensorEn = false;
  for (int i = 0; i < SENSOR_NUMBER; i++)
  {
    if (config.sensor[i].enable)
    {
      SensorEn = true;
      break;
    }
  }

  if (SensorEn == true)
  {
    xTaskCreatePinnedToCore(
        taskSensor,        /* Function to implement the task */
        "taskSensor",      /* Name of the task */
        4096,              /* Stack size in words */
        NULL,              /* Task input parameter */
        1,                 /* Priority of the task */
        &taskSensorHandle, /* Task handle. */
        1);                /* Core where the task should run */
  }

  delay(1000);
  upTimeStamp = millis() / 1000;
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 30000, // 30 seconds
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,    // Bitmask of all cores
    .trigger_panic = false,
  };
esp_task_wdt_init(&twdt_config);
printf("TWDT initialized\n");
esp_task_wdt_add(NULL);
esp_task_wdt_status(NULL);
StandByTick = millis() + (config.pwr_stanby_delay * 1000);
}

int pkgCount = 0;

String getTimeStamp()
{
  char strtmp[50];
  time_t now;
  time(&now);
  struct tm *info = gmtime(&now);
  sprintf(strtmp, "%02d%02d%02dz", info->tm_mday, info->tm_hour, info->tm_min);
  return String(strtmp);
}

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
  // ESP_LOGE("GPS", "Aprs Compress");
  //  Translate from semicircles to Base91 format
  char aprs_position[13];
  long latitude = semicircles((char *)lat.c_str(), (nowLat < 0));
  long longitude = semicircles((char *)lon.c_str(), (nowLng < 0));
  long ltemp = 1073741824L - latitude; // 90 degrees - latitude
  // ESP_LOGE("GPS", "lat=%u lon=%u", latitude, longitude);
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
    if (gps)
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

// String compress_position(double nowLat, double nowLng, int alt_feed, double course, uint16_t spdKnot, char table, char symbol, bool gps)

// String compressMicE(float lat, float lon, uint16_t heading, uint16_t speed, uint8_t type, uint8_t *telem, size_t telemLen, char *grid, char *status, int32_t alt, char table, char symbol)
// {
//     String strRet = "";
//     // sanity checks first
//     if (((telemLen == 0) && (telem != NULL)) || ((telemLen != 0) && (telem == NULL)))
//     {
//         return strRet;
//     }

//     if ((telemLen != 0) && (telemLen != 2) && (telemLen != 5))
//     {
//         return strRet;
//     }

//     if ((telemLen > 0) && ((grid != NULL) || (status != NULL) || (alt != RADIOLIB_APRS_MIC_E_ALTITUDE_UNUSED)))
//     {
//         // can't have both telemetry and status
//         return strRet;
//     }

//     // prepare buffers
//     char destCallsign[7];
// #if !RADIOLIB_STATIC_ONLY
//     size_t infoLen = 10;
//     if (telemLen > 0)
//     {
//         infoLen += 1 + telemLen;
//     }
//     else
//     {
//         if (grid != NULL)
//         {
//             infoLen += strlen(grid) + 2;
//         }
//         if (status != NULL)
//         {
//             infoLen += strlen(status);
//         }
//         if (alt > RADIOLIB_APRS_MIC_E_ALTITUDE_UNUSED)
//         {
//             infoLen += 4;
//         }
//     }
//     char *info = new char[infoLen];
// #else
//     char info[RADIOLIB_STATIC_ARRAY_SIZE];
// #endif
//     size_t infoPos = 0;

//     // the following is based on APRS Mic-E implementation by https://github.com/omegat
//     // as discussed in https://github.com/jgromes/RadioLib/issues/430

//     // latitude first, because that is in the destination field
//     float lat_abs = RADIOLIB_ABS(lat);
//     int lat_deg = (int)lat_abs;
//     int lat_min = (lat_abs - (float)lat_deg) * 60.0f;
//     int lat_hun = (((lat_abs - (float)lat_deg) * 60.0f) - lat_min) * 100.0f;
//     destCallsign[0] = lat_deg / 10;
//     destCallsign[1] = lat_deg % 10;
//     destCallsign[2] = lat_min / 10;
//     destCallsign[3] = lat_min % 10;
//     destCallsign[4] = lat_hun / 10;
//     destCallsign[5] = lat_hun % 10;

//     // next, add the extra bits
//     if (type & 0x04)
//     {
//         destCallsign[0] += RADIOLIB_APRS_MIC_E_DEST_BIT_OFFSET;
//     }
//     if (type & 0x02)
//     {
//         destCallsign[1] += RADIOLIB_APRS_MIC_E_DEST_BIT_OFFSET;
//     }
//     if (type & 0x01)
//     {
//         destCallsign[2] += RADIOLIB_APRS_MIC_E_DEST_BIT_OFFSET;
//     }
//     if (lat >= 0)
//     {
//         destCallsign[3] += RADIOLIB_APRS_MIC_E_DEST_BIT_OFFSET;
//     }
//     if (lon >= 100 || lon <= -100)
//     {
//         destCallsign[4] += RADIOLIB_APRS_MIC_E_DEST_BIT_OFFSET;
//     }
//     if (lon < 0)
//     {
//         destCallsign[5] += RADIOLIB_APRS_MIC_E_DEST_BIT_OFFSET;
//     }
//     destCallsign[6] = '\0';

//     // now convert to Mic-E characters to get the "callsign"
//     for (uint8_t i = 0; i < 6; i++)
//     {
//         if (destCallsign[i] <= 9)
//         {
//             destCallsign[i] += '0';
//         }
//         else
//         {
//             destCallsign[i] += ('A' - 10);
//         }
//     }

//     // setup the information field
//     info[infoPos++] = RADIOLIB_APRS_MIC_E_GPS_DATA_CURRENT;

//     // encode the longtitude
//     float lon_abs = RADIOLIB_ABS(lon);
//     int32_t lon_deg = (int32_t)lon_abs;
//     int32_t lon_min = (lon_abs - (float)lon_deg) * 60.0f;
//     int32_t lon_hun = (((lon_abs - (float)lon_deg) * 60.0f) - lon_min) * 100.0f;

//     if (lon_deg <= 9)
//     {
//         info[infoPos++] = lon_deg + 118;
//     }
//     else if (lon_deg <= 99)
//     {
//         info[infoPos++] = lon_deg + 28;
//     }
//     else if (lon_deg <= 109)
//     {
//         info[infoPos++] = lon_deg + 8;
//     }
//     else
//     {
//         info[infoPos++] = lon_deg - 72;
//     }

//     if (lon_min <= 9)
//     {
//         info[infoPos++] = lon_min + 88;
//     }
//     else
//     {
//         info[infoPos++] = lon_min + 28;
//     }

//     info[infoPos++] = lon_hun + 28;

//     // now the speed and heading - this gets really weird
//     int32_t speed_hun_ten = speed / 10;
//     int32_t speed_uni = speed % 10;
//     int32_t head_hun = heading / 100;
//     int32_t head_ten_uni = heading % 100;

//     if (speed <= 199)
//     {
//         info[infoPos++] = speed_hun_ten + 'l';
//     }
//     else
//     {
//         info[infoPos++] = speed_hun_ten + '0';
//     }

//     info[infoPos++] = speed_uni * 10 + head_hun + 32;
//     info[infoPos++] = head_ten_uni + 28;
//     info[infoPos++] = symbol;
//     info[infoPos++] = table;

//     // onto the optional stuff - check telemetry first
//     if (telemLen > 0)
//     {
//         if (telemLen == 2)
//         {
//             info[infoPos++] = RADIOLIB_APRS_MIC_E_TELEMETRY_LEN_2;
//         }
//         else
//         {
//             info[infoPos++] = RADIOLIB_APRS_MIC_E_TELEMETRY_LEN_5;
//         }
//         for (uint8_t i = 0; i < telemLen; i++)
//         {
//             sprintf(&(info[infoPos]), "%02X", telem[i]);
//             infoPos += 2;
//         }
//     }
//     else
//     {
//         if (grid != NULL)
//         {
//             memcpy(&(info[infoPos]), grid, strlen(grid));
//             infoPos += strlen(grid);
//             info[infoPos++] = '/';
//             info[infoPos++] = 'G';
//         }
//         if (status != NULL)
//         {
//             info[infoPos++] = ' ';
//             memcpy(&(info[infoPos]), status, strlen(status));
//             infoPos += strlen(status);
//         }
//         if (alt > RADIOLIB_APRS_MIC_E_ALTITUDE_UNUSED)
//         {
//             // altitude is offset by -10 km
//             int32_t alt_val = alt + 10000;

//             // ... and encoded in base 91 for some reason
//             info[infoPos++] = (alt_val / 8281) + 33;
//             info[infoPos++] = ((alt_val % 8281) / 91) + 33;
//             info[infoPos++] = ((alt_val % 8281) % 91) + 33;
//             info[infoPos++] = '}';
//         }
//     }
//     info[infoPos++] = '\0';

//     strRet = String(info);
//     return strRet;
// }

String trk_gps_postion(String comment)
{
  String rawData = "";
  String lat, lon;
  double nowLat, nowLng;
  char rawTNC[300];
  char aprs_table, aprs_symbol;
  // char timestamp[10];
  struct tm tmstruct;
  double dist, course, speed;
  time_t nowTime;

  memset(rawTNC, 0, sizeof(rawTNC));
  // getLocalTime(&tmstruct, 5000);
  // sprintf(timestamp, "%02d%02d%02d%02d", (tmstruct.tm_mon + 1), tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min);
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

  if (gps.location.isValid()) // && (gps.hdop.hdop() < 10.0))
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

      String compPosition = compress_position(nowLat, nowLng, gps.altitude.feet(), course, spdKnot, aprs_table, aprs_symbol, (gps.satellites.value() > 3));
      // ESP_LOGE("GPS", "Compress=%s", aprs_position);
      if (strlen(config.trk_item) >= 3)
      {
        char object[10];
        memset(object, 0x20, 10);
        memcpy(object, config.trk_item, strlen(config.trk_item));
        object[9] = 0;
        if (config.trk_timestamp)
        {
          String timeStamp = getTimeStamp();
          sprintf(rawTNC, ";%s*%s%s", object, timeStamp, compPosition.c_str());
        }
        else
        {
          sprintf(rawTNC, ")%s!%s", config.trk_item, compPosition.c_str());
        }
      }
      else
      {
        if (config.trk_timestamp)
        {
          String timeStamp = getTimeStamp();
          sprintf(rawTNC, "/%s%s", timeStamp, compPosition.c_str());
        }
        else
        {
          sprintf(rawTNC, "!%s", compPosition.c_str());
        }
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
      sprintf(csd_spd, "%03d/%03d", (int)gps.course.deg(), (int)gps.speed.knots());
      if (strlen(config.trk_item) >= 3)
      {
        char object[10];
        memset(object, 0x20, 10);
        memcpy(object, config.trk_item, strlen(config.trk_item));
        object[9] = 0;
        if (config.trk_timestamp)
        {
          String timeStamp = getTimeStamp();
          sprintf(rawTNC, ";%s*%s%02d%02d.%02d%c%c%03d%02d.%02d%c%c%s", object, timeStamp, lat_dd, lat_mm, lat_ss, lat_ns, aprs_table, lon_dd, lon_mm, lon_ss, lon_ew, aprs_symbol, csd_spd);
        }
        else
        {
          sprintf(rawTNC, ")%s!%02d%02d.%02d%c%c%03d%02d.%02d%c%c%s", config.trk_item, lat_dd, lat_mm, lat_ss, lat_ns, aprs_table, lon_dd, lon_mm, lon_ss, lon_ew, aprs_symbol, csd_spd);
        }
      }
      else
      {
        if (config.trk_timestamp)
        {
          String timeStamp = getTimeStamp();
          sprintf(rawTNC, "/%s%02d%02d.%02d%c%c%03d%02d.%02d%c%c%s", timeStamp, lat_dd, lat_mm, lat_ss, lat_ns, aprs_table, lon_dd, lon_mm, lon_ss, lon_ew, aprs_symbol, csd_spd);
        }
        else
        {
          sprintf(rawTNC, "!%02d%02d.%02d%c%c%03d%02d.%02d%c%c%s", lat_dd, lat_mm, lat_ss, lat_ns, aprs_table, lon_dd, lon_mm, lon_ss, lon_ew, aprs_symbol, csd_spd);
        }
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
    sprintf(rawTNC, ">%s ", config.trk_item);
  }

  String tnc2Raw = "";
  char strtmp[300];
  if (config.trk_ssid == 0)
    sprintf(strtmp, "%s>APE32T", config.trk_mycall);
  else
    sprintf(strtmp, "%s-%d>APE32T", config.trk_mycall, config.trk_ssid);
  tnc2Raw = String(strtmp);
  if (config.trk_path < 5)
  {
    if (config.trk_path > 0)
      tnc2Raw += "-" + String(config.trk_path);
  }
  else
  {
    tnc2Raw += ",";
    tnc2Raw += getPath(config.trk_path);
  }
  tnc2Raw += ":";
  tnc2Raw += String(rawTNC);
  tnc2Raw += comment + String(config.trk_comment);
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
      char object[10];
      memset(object, 0x20, 10);
      memcpy(object, config.trk_item, strlen(config.trk_item));
      object[9] = 0;
      if (config.trk_timestamp)
      {
        String timeStamp = getTimeStamp();
        sprintf(loc, ";%s*%s%s", object, timeStamp, compPosition.c_str());
      }
      else
      {
        sprintf(loc, ")%s!%s", config.trk_item, compPosition.c_str());
      }
    }
    else
    {
      if (config.trk_timestamp)
      {
        String timeStamp = getTimeStamp();
        sprintf(loc, "/%s%s", timeStamp, compPosition.c_str());
      }
      else
      {
        sprintf(loc, "!%s", compPosition.c_str());
      }
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
      memset(object, 0x20, 10);
      memcpy(object, config.trk_item, strlen(config.trk_item));
      object[9] = 0;
      if (config.trk_timestamp)
      {
        String timeStamp = getTimeStamp();
        sprintf(loc, ";%s*%s%02d%02d.%02d%c%c%03d%02d.%02d%c%c", object, timeStamp, lat_dd, lat_mm, lat_ss, lat_ns, config.trk_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.trk_symbol[1]);
      }
      else
      {
        sprintf(loc, ")%s!%02d%02d.%02d%c%c%03d%02d.%02d%c%c", config.trk_item, lat_dd, lat_mm, lat_ss, lat_ns, config.trk_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.trk_symbol[1]);
      }
    }
    else
    {
      if (config.trk_timestamp)
      {
        String timeStamp = getTimeStamp();
        sprintf(loc, "/%s%02d%02d.%02d%c%c%03d%02d.%02d%c%c", timeStamp, lat_dd, lat_mm, lat_ss, lat_ns, config.trk_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.trk_symbol[1]);
      }
      else
      {
        sprintf(loc, "!%02d%02d.%02d%c%c%03d%02d.%02d%c%c", lat_dd, lat_mm, lat_ss, lat_ns, config.trk_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.trk_symbol[1]);
      }
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
    sprintf(strtmp, "%s>APE32T", config.trk_mycall);
  else
    sprintf(strtmp, "%s-%d>APE32T", config.trk_mycall, config.trk_ssid);
  tnc2Raw = String(strtmp);
  if (config.trk_path < 5)
  {
    if (config.trk_path > 0)
      tnc2Raw += "-" + String(config.trk_path);
  }
  else
  {
    tnc2Raw += ",";
    tnc2Raw += getPath(config.trk_path);
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
    memset(object, 0x20, 10);
    memcpy(object, config.igate_object, strlen(config.igate_object));
    object[9] = 0;
    if (config.igate_timestamp)
    {
      String timeStamp = getTimeStamp();
      sprintf(loc, ";%s*%s%02d%02d.%02d%c%c%03d%02d.%02d%c%c", object, timeStamp, lat_dd, lat_mm, lat_ss, lat_ns, config.igate_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.igate_symbol[1]);
    }
    else
    {
      sprintf(loc, ")%s!%02d%02d.%02d%c%c%03d%02d.%02d%c%c", config.igate_object, lat_dd, lat_mm, lat_ss, lat_ns, config.igate_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.igate_symbol[1]);
    }
  }
  else
  {
    if (config.igate_timestamp)
    {
      String timeStamp = getTimeStamp();
      sprintf(loc, "/%s%02d%02d.%02d%c%c%03d%02d.%02d%c%c", timeStamp.c_str(), lat_dd, lat_mm, lat_ss, lat_ns, config.igate_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.igate_symbol[1]);
    }
    else
    {
      sprintf(loc, "!%02d%02d.%02d%c%c%03d%02d.%02d%c%c", lat_dd, lat_mm, lat_ss, lat_ns, config.igate_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.igate_symbol[1]);
    }
  }
  if (config.aprs_ssid == 0)
    sprintf(strtmp, "%s>APE32T", config.aprs_mycall);
  else
    sprintf(strtmp, "%s-%d>APE32T", config.aprs_mycall, config.aprs_ssid);
  tnc2Raw = String(strtmp);
  if (config.igate_path < 5)
  {
    if (config.igate_path > 0)
      tnc2Raw += "-" + String(config.igate_path);
  }
  else
  {
    tnc2Raw += ",";
    tnc2Raw += getPath(config.igate_path);
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
  if (config.digi_timestamp)
  {
    String timeStamp = getTimeStamp();
    sprintf(loc, "/%s%02d%02d.%02d%c%c%03d%02d.%02d%c%c", timeStamp, lat_dd, lat_mm, lat_ss, lat_ns, config.digi_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.digi_symbol[1]);
  }
  else
  {
    sprintf(loc, "!%02d%02d.%02d%c%c%03d%02d.%02d%c%c", lat_dd, lat_mm, lat_ss, lat_ns, config.digi_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.digi_symbol[1]);
  }
  if (config.digi_ssid == 0)
    sprintf(strtmp, "%s>APE32T", config.digi_mycall);
  else
    sprintf(strtmp, "%s-%d>APE32T", config.digi_mycall, config.digi_ssid);
  tnc2Raw = String(strtmp);
  if (config.digi_path < 5)
  {
    if (config.digi_path > 0)
      tnc2Raw += "-" + String(config.digi_path);
  }
  else
  {
    tnc2Raw += ",";
    tnc2Raw += getPath(config.digi_path);
  }
  tnc2Raw += ":";
  tnc2Raw += String(loc);
  tnc2Raw += String(config.digi_phg) + String(strAltitude);
  tnc2Raw += comment + " " + String(config.digi_comment);
  return tnc2Raw;
}

void tracker_status(char *text)
{
  char name[50];

  if (config.trk_ssid > 0)
    sprintf(name, "%s-%d>APE32T", config.trk_mycall, config.trk_ssid);
  else
    sprintf(name, "%s>APE32T", config.trk_mycall);

  String tnc2Raw = String(name);
  if (config.trk_path < 5)
  {
    if (config.trk_path > 0)
      tnc2Raw += "-" + String(config.trk_path);
  }
  else
  {
    tnc2Raw += ",";
    tnc2Raw += getPath(config.trk_path);
  }
  tnc2Raw += ":>";
  tnc2Raw += String(text);

  uint8_t SendMode = 0;
  if (config.trk_loc2rf)
    SendMode |= RF_CHANNEL;
  if (config.trk_loc2inet)
    SendMode |= INET_CHANNEL;
  pkgTxPush(tnc2Raw.c_str(), tnc2Raw.length(), 0, SendMode);
}

void igate_status(char *text)
{
  char name[50];

  if (config.aprs_ssid > 0)
    sprintf(name, "%s-%d>APE32T", config.aprs_mycall, config.aprs_ssid);
  else
    sprintf(name, "%s>APE32T", config.aprs_mycall);

  String tnc2Raw = String(name);
  if (config.igate_path < 5)
  {
    if (config.igate_path > 0)
      tnc2Raw += "-" + String(config.igate_path);
  }
  else
  {
    tnc2Raw += ",";
    tnc2Raw += getPath(config.igate_path);
  }
  tnc2Raw += ":>";
  tnc2Raw += String(text);

  uint8_t SendMode = 0;
  if (config.igate_loc2rf)
    SendMode |= RF_CHANNEL;
  if (config.igate_loc2inet)
    SendMode |= INET_CHANNEL;
  pkgTxPush(tnc2Raw.c_str(), tnc2Raw.length(), 0, SendMode);
}

void digi_status(char *text)
{
  char name[50];

  if (config.digi_ssid > 0)
    sprintf(name, "%s-%d>APE32T", config.digi_mycall, config.digi_ssid);
  else
    sprintf(name, "%s>APE32T", config.digi_mycall);

  String tnc2Raw = String(name);
  if (config.digi_path < 5)
  {
    if (config.digi_path > 0)
      tnc2Raw += "-" + String(config.digi_path);
  }
  else
  {
    tnc2Raw += ",";
    tnc2Raw += getPath(config.digi_path);
  }
  tnc2Raw += ":>";
  tnc2Raw += String(text);

  uint8_t SendMode = 0;
  if (config.digi_loc2rf)
    SendMode |= RF_CHANNEL;
  if (config.digi_loc2inet)
    SendMode |= INET_CHANNEL;
  pkgTxPush(tnc2Raw.c_str(), tnc2Raw.length(), 0, SendMode);
}

extern float mslAltitude;
String wx_report(double lat, double lon, double alt, String comment)
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
  // char strAltitude[12];
  // memset(strAltitude, 0, sizeof(strAltitude));
  // if (alt > 0)
  // {
  //     sprintf(strAltitude, "/A=%06d", (int)(alt * 3.28F));
  // }
  if (config.wx_gps)
  {
    if (gps.satellites.value() > 5 && gps.hdop.hdop() < 5)
      mslAltitude = alt;
  }
  else
  {
    mslAltitude = alt;
  }
  if (strlen(config.wx_object) >= 3)
  {
    char object[10];
    memset(object, 0x20, 10);
    memcpy(object, config.wx_object, strlen(config.wx_object));
    object[9] = 0;
    if (config.wx_timestamp)
    {
      String timeStamp = getTimeStamp();
      sprintf(loc, ";%s*%s%02d%02d.%02d%c/%03d%02d.%02d%c_", object, timeStamp.c_str(), lat_dd, lat_mm, lat_ss, lat_ns, lon_dd, lon_mm, lon_ss, lon_ew);
    }
    else
    {
      sprintf(loc, ")%s!%02d%02d.%02d%c/%03d%02d.%02d%c_", config.wx_object, lat_dd, lat_mm, lat_ss, lat_ns, lon_dd, lon_mm, lon_ss, lon_ew);
    }
  }
  else
  {
    if (config.wx_timestamp)
    {
      String timeStamp = getTimeStamp();
      sprintf(loc, "@%s%02d%02d.%02d%c/%03d%02d.%02d%c_", timeStamp.c_str(), lat_dd, lat_mm, lat_ss, lat_ns, lon_dd, lon_mm, lon_ss, lon_ew);
    }
    else
    {
      sprintf(loc, "!%02d%02d.%02d%c/%03d%02d.%02d%c_", lat_dd, lat_mm, lat_ss, lat_ns, lon_dd, lon_mm, lon_ss, lon_ew);
    }
  }
  if (config.wx_ssid == 0)
    sprintf(strtmp, "%s>APE32T", config.wx_mycall);
  else
    sprintf(strtmp, "%s-%d>APE32T", config.wx_mycall, config.wx_ssid);
  tnc2Raw = String(strtmp);
  if (config.wx_path < 5)
  {
    if (config.wx_path > 0)
      tnc2Raw += "-" + String(config.wx_path);
  }
  else
  {
    tnc2Raw += ",";
    tnc2Raw += getPath(config.wx_path);
  }
  tnc2Raw += ":";
  tnc2Raw += String(loc);
  char WxRaw[500];
  memset(WxRaw, 0, 500);
  getRawWx(&WxRaw[0]);
  tnc2Raw += String(WxRaw);
  tnc2Raw += comment + String(config.wx_comment);
  return tnc2Raw;
}

int packet2Raw(String &tnc2, AX25Msg &Packet)
{
  if (Packet.len < 1)
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
    if (i >= 8)
      break;
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
int btn_count = 0;
long timeCheck = 0;
bool afterVoice = false;
long int sleep_timer = 0;
bool save_mode = false;
bool save_act = false;

bool ptt_stat_old = false;

void loop()
{
  vTaskDelay(10 / portTICK_PERIOD_MS);
  if(getDCD())
  {
    StandByTick = millis() + (config.pwr_stanby_delay * 1000);
    if (save_mode)
      save_act = true;
  }
  // if (ESP.getFreeHeap() < 60000)
  //   ESP.restart();

  // if (config.bt_master)
  // {
  //   // disconnecting
  //   if (!BTdeviceConnected && BToldDeviceConnected)
  //   {
  //     delay(500);                  // give the bluetooth stack the chance to get things ready
  //     pServer->startAdvertising(); // restart advertising
  //     log_d("start advertising");
  //     BToldDeviceConnected = BTdeviceConnected;
  //   }
  //   // connecting
  //   if (BTdeviceConnected && !BToldDeviceConnected)
  //   {
  //     // do stuff here on connecting
  //     BToldDeviceConnected = BTdeviceConnected;
  //   }
  // }
  // if (pServer->getConnectedCount())
  // {
  //   NimBLEService *pSvc = pServer->getServiceByUUID(config.bt_name);
  //   if (pSvc)
  //   {
  //     NimBLECharacteristic *pChr = pSvc->getCharacteristic(config.bt_uuid_tx);
  //     if (pChr)
  //     {
  //       pChr->notify();
  //     }
  //   }
  // }

  // PTT push to FM Voice
  if (digitalRead(BUTTON_PTT_PIN) == LOW)
  {
    pttStat = 1;
    // setTransmit(true);
    AFSKInitAct = false;
    delay(50);

    digitalWrite(POWER_PIN, config.rf_power); // RF Power
    // sa868.setTxPower(sa868._config.rf_power);
    digitalWrite(SA868_MIC_SEL, LOW); // Select = MIC
    digitalWrite(SA868_PTT_PIN, LOW); // PTT RF
    if (config.rf_type == RF_SA8x8_OpenEdit)
    {
      if(config.rf_power)
        sa868.setHighPower();
      else
        sa868.setLowPower();
      sa868.TxOn();
    }
    delay(500);
    LED_Status(255, 0, 0);
    while (digitalRead(BUTTON_PTT_PIN) == LOW)
    {
      pttStat++;
      delay(100);
    }
    pttStat = 0;
    // burstAfterVoice();
    // char sts[50];
    // if (gps.location.isValid() && (gps.hdop.hdop() < 10.0))
    //   sprintf(sts, "POSITION GPS\nSPD %dkPh/%d\n", SB_SPEED, SB_HEADING);
    // else
    //   sprintf(sts, "POSITION GPS\nGPS INVALID\n");
    // pushTxDisp(TXCH_RF, "After Voice", sts);
    LED_Status(0, 0, 0);
    AFSKInitAct = true;
    if (config.rf_type == RF_SA8x8_OpenEdit)
    {
      //delay(200);
      sa868.TxOff();
      //sa868.setLowPower();
      //delay(1000);
      sa868.RxOn();
    }
    // setTransmit(false);
  }

  if (digitalRead(BOOT_PIN) == LOW)
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

  if (config.rf_type == RF_SA8x8_OpenEdit)
  {
    // bool ptt_stat = getTransmit();
    // if (ptt_stat == true && ptt_stat_old == false)
    // {
    //   ptt_stat_old = ptt_stat;
    if (pttON)
    {
      if (config.rf_type == RF_SA8x8_OpenEdit)
      {
        if(config.rf_power)
          sa868.setHighPower();
        else
          sa868.setLowPower();
        sa868.TxOn();
      }
      digitalWrite(POWER_PIN, config.rf_power); // RF Power
      setTransmit(true);
      log_d("PTT ON");
      pttON = false;
      LED_Status(255, 0, 0);
    }
    // else if (ptt_stat == false && ptt_stat_old == true)
    // {
    if (pttOFF)
    {
      digitalWrite(POWER_PIN, LOW); // RF Power
      //delay(100);
      sa868.TxOff();
      //sa868.setLowPower();
      //delay(100);
      sa868.RxOn();
      pttOFF = false;
      log_d("PTT OFF");
      LED_Status(0, 0, 0);
    }
  }

  if (millis() > timeCheck)
  {
    // log_d("taskAPRS: %d mS\ttaskNetwork: %d mS\ttaskGUI: %d mS\n", timeAprs, timeNetwork, timeGui);
    timeCheck = millis() + 10000;
    VBat = (float)PMU.getBattVoltage() / 1000;
    if (!((WiFi.isConnected() == true) || (WiFi.softAPgetStationNum() > 0)))
        {
            Sleep_Activate &= ~ACTIVATE_WIFI;
        }

        if ((config.pwr_mode == MODE_B) && config.gnss_enable) // expand sleep delay from GPS moving
        {
            if ((gps.satellites.value() > 3) && (gps.hdop.hdop() < 5) && (gps.speed.kmph() > 10))
            {
                if (config.trk_en)
                {
                    if (config.trk_gps)
                    {
                        if (SB_SPEED > 10)
                            StandByTick = millis() + (config.trk_slowinterval * 1000);
                    }
                    else
                    {
                        StandByTick = millis() + (config.trk_interval * 1000) + 1000;
                        if (save_mode)
                            save_act = true;
                    }
                }
                else if (config.igate_en)
                {
                    if (config.igate_gps)
                    {
                        StandByTick = millis() + (config.igate_interval * 1000) + 1000;
                        if (save_mode)
                            save_act = true;
                    }
                }
                else if (config.digi_en)
                {
                    if (config.digi_gps)
                    {
                        StandByTick = millis() + (config.digi_interval * 1000) + 1000;
                        if (save_mode)
                            save_act = true;
                    }
                }
            }
        }
    // log_d( "Task Name\tStatus\tPrio\tHWM\tTask\tAffinity");
    // char stats_buffer[1024];
    // vTaskList(stats_buffer);
    // log_d("%s", stats_buffer);
    // vTaskGetRunTimeStats(stats_buffer);
    // log_d("%s", stats_buffer);
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
  if (millis() > timeSleep)
    {
        esp_task_wdt_reset();
        timeSleep = millis() + 1000;

        if (config.pwr_en)
        {
            if (config.pwr_mode == MODE_A) // CPU and Radio active, power down control
            {
#if defined(XPOWERS_CHIP_AXP2101)
                if (PMU.isCharging())
                {
                    StandByTick = millis() + (config.pwr_stanby_delay * 1000);
                }
#endif
                if (((millis() > StandByTick) || (save_act)) && (pkgTxCount() == 0))
                {
                    save_act = false;
                    if (save_mode == false)
                    {
                        save_mode = true;
                        log_d("System to save mode A %d Sec", config.pwr_sleep_interval);
                        StandByTick = millis() + (config.pwr_sleep_interval * 1000);
                        vTaskSuspend(taskSensorHandle);
                        display.clearDisplay();
                        display.display();
                        // Power OFF
                        PowerOff();

#if defined(XPOWERS_CHIP_AXP2101)
                        PMU.disableDC5();
                        PMU.disableALDO1(); // QMC6310,BME280,OLED
                        // PMU.disableALDO3(); //LoRa
                        PMU.disableBLDO2();
                        PMU.disableALDO2();
                        // PMU.disableALDO4(); //GNSS,
                        PMU.disableBLDO1(); // TF Card
                        PMU.disableDC3();
#endif
                        setCpuFrequencyMhz(80);
                        esp_task_wdt_reset();
                        delay(100);
                    }
                    else
                    {
                        // Wakeup
                        save_mode = false;
                        log_d("System to Wakeup save mode A %d Sec", config.pwr_sleep_interval);
                        // sleep_timer = millis() + (config.pwr_sleep_interval * 1000);
                        StandByTick = millis() + (config.pwr_stanby_delay * 1000);
#if defined(__XTENSA__)
                        setCpuFrequencyMhz(240);
#else
                        setCpuFrequencyMhz(160);
#endif
                        PowerOn();
                        sensorInit(false);
                        delay(100);
                        vTaskResume(taskSensorHandle);
#if defined(XPOWERS_CHIP_AXP2101)
                        PMU.enableDC5();
                        PMU.enableALDO1();
                        PMU.enableALDO3();
                        PMU.enableBLDO2();
                        PMU.enableALDO2();
                        PMU.enableALDO4();
                        PMU.enableBLDO1();
                        PMU.enableDC3();
#endif
                    }
                }
            }
            else if (config.pwr_mode == MODE_B) // Wake up and wait for delay time to sleepp
            {
              #if defined(XPOWERS_CHIP_AXP2101)
                if (PMU.isCharging())
                {
                    StandByTick = millis() + (config.pwr_stanby_delay * 1000);
                }
#endif
                if (((millis() > StandByTick) || (save_act)) && (pkgTxCount() == 0))
                {
                    save_act = false;
                    if (!save_mode)
                    {
                        save_mode = true;
                        log_d("System to light sleep Mode B %d Sec", config.pwr_sleep_interval);
                        // sleep_timer = millis() + (config.pwr_sleep_interval * 1000);
                        StandByTick = millis() + (config.pwr_sleep_interval * 1000);
                        vTaskDelete(taskSensorHandle);
                        display.clearDisplay();
                        display.display();
                        PowerOff();
                        // adc_power_off();
                        vTaskDelete(taskNetworkHandle);
                        WiFi.disconnect(true); // Disconnect from the network
                        WiFi.persistent(false);
                        WiFi.mode(WIFI_OFF); // Switch WiFi off

                        // vTaskSuspend(taskNetworkHandle);
                        // delay(100);
                        setCpuFrequencyMhz(80);
                        // esp_task_wdt_deinit();
                        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
                        #if defined(XPOWERS_CHIP_AXP2101)
                        PMU.disableDC5();
                        PMU.disableALDO1(); // QMC6310,BME280,OLED
                        // PMU.disableALDO3(); //LoRa
                        PMU.disableBLDO2();
                        PMU.disableALDO2();
                        PMU.disableALDO4(); // GNSS,
                        PMU.disableBLDO1(); // TF Card
                        PMU.disableDC3();
                        // esp_sleep_enable_ext0_wakeup((gpio_num_t)PMU_IRQ, LOW);
                        gpio_wakeup_enable((gpio_num_t)PMU_IRQ, GPIO_INTR_LOW_LEVEL);
#else
#if defined(__XTENSA__)
                        esp_sleep_enable_ext0_wakeup((gpio_num_t)config.rf_dio1_gpio, HIGH);
                        // gpio_wakeup_enable ((gpio_num_t)config.rf_dio1_gpio, GPIO_INTR_HIGH_LEVEL);
#else
                        // esp_sleep_enable_ext1_wakeup(0x200, ESP_EXT1_WAKEUP_ALL_LOW);
                        esp_deep_sleep_enable_gpio_wakeup((1 << config.rf_dio1_gpio), ESP_GPIO_WAKEUP_GPIO_HIGH);
#endif
#endif

                        delay(100);
#ifdef __XTENSA__
                        //esp_sleep_enable_ext1_wakeup(0x1, ESP_EXT1_WAKEUP_ALL_LOW);
                        gpio_wakeup_enable ((gpio_num_t)0, GPIO_INTR_LOW_LEVEL);
#else
                        // esp_deep_sleep_enable_gpio_wakeup((1<<9), ESP_GPIO_WAKEUP_GPIO_LOW);
                        gpio_wakeup_enable((gpio_num_t)9, GPIO_INTR_LOW_LEVEL);
#endif
                        esp_sleep_enable_timer_wakeup((uint64_t)config.pwr_sleep_interval * uS_TO_S_FACTOR);
                        esp_sleep_enable_gpio_wakeup();
                        esp_light_sleep_start();
                        save_mode = true;
                        save_act = true;
                        StandByTick = millis() + (config.pwr_stanby_delay * 1000);
                    }
                    else
                    {
                        // Wakeup
                        save_mode = false;
                        log_d("System to wakeup sleep Mode B %d Sec", config.pwr_sleep_interval);
                        // sleep_timer = millis() + (config.pwr_sleep_interval * 1000);
                        StandByTick = millis() + (config.pwr_stanby_delay * 1000);
#if defined(__XTENSA__)
                        setCpuFrequencyMhz(240);
#else
                        setCpuFrequencyMhz(160);
#endif
                        esp_task_wdt_reset();
                        // esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
                        // esp_task_wdt_add(NULL);               // add current thread to WDT watch
                        // esp_task_wdt_reset();
                        // adc_power_on();
                        // WiFi.disconnect(false);  // Reconnect the network
                        // WiFi.mode(WIFI_STA);    // Switch WiFi off
                        PowerOn();
                        sensorInit(false);
                        delay(100);
                        vTaskResume(taskSensorHandle);
#ifdef __XTENSA__
                        xTaskCreatePinnedToCore(
                            taskNetwork,        /* Function to implement the task */
                            "taskNetwork",      /* Name of the task */
                            12000,              /* Stack size in words */
                            NULL,               /* Task input parameter */
                            0,                  /* Priority of the task */
                            &taskNetworkHandle, /* Task handle. */
                            1);                 /* Core where the task should run */
                        xTaskCreatePinnedToCore(
                            taskSensor,        /* Function to implement the task */
                            "taskSensor",      /* Name of the task */
                            4096,              /* Stack size in words */
                            NULL,              /* Task input parameter */
                            1,                 /* Priority of the task */
                            &taskSensorHandle, /* Task handle. */
                            1);                /* Core where the task should run */
#else
                        xTaskCreatePinnedToCore(
                            taskNetwork,        /* Function to implement the task */
                            "taskNetwork",      /* Name of the task */
                            12000,              /* Stack size in words */
                            NULL,               /* Task input parameter */
                            1,                  /* Priority of the task */
                            &taskNetworkHandle, /* Task handle. */
                            0);                 /* Core where the task should run */
                        xTaskCreatePinnedToCore(
                            taskSensor,        /* Function to implement the task */
                            "taskSensor",      /* Name of the task */
                            4096,              /* Stack size in words */
                            NULL,              /* Task input parameter */
                            4,                 /* Priority of the task */
                            &taskSensorHandle, /* Task handle. */
                            0);                /* Core where the task should run */
#endif
                        // vTaskResume(taskNetworkHandle);
                        #if defined(XPOWERS_CHIP_AXP2101)
                        PMU.enableDC5();
                        PMU.enableALDO1();
                        PMU.enableALDO3();
                        PMU.enableBLDO2();
                        PMU.enableALDO2();
                        PMU.enableALDO4();
                        PMU.enableBLDO1();
                        PMU.enableDC3();
#endif
                    }
                }
            }
            else if (config.pwr_mode == MODE_C) // Wake up and wait for event to sleep
            {
                if ((Sleep_Activate == ACTIVATE_OFF) && (millis() > StandByTick) && (pkgTxCount() == 0))
                {
                    log_d("System to SLEEP Mode %d Sec", config.pwr_sleep_interval);
                    // radioSleep();
                    // esp_deep_sleep_enable_gpio_wakeup(BIT(DEFAULT_WAKEUP_PIN), DEFAULT_WAKEUP_LEVEL));
                    display.clearDisplay();
                    display.display();
                    PowerOff();
                    // delay(100);
                    // esp_sleep_enable_ext0_wakeup(GPIO_NUM_14,LOW);
                    if(config.rf_en)
                    {
                      digitalWrite(config.rf_pd_gpio, LOW); // RF Power
                    }
                    //radioSleep();
                    #if defined(XPOWERS_CHIP_AXP2101)
                    PMU.disableDC5();
                    PMU.disableALDO1();
                    PMU.disableALDO3();
                    PMU.disableBLDO2();
                    PMU.disableALDO2();
                    PMU.disableALDO4();
                    PMU.disableBLDO1();
                    PMU.disableDC3();
#endif

                    delay(100);
#ifdef __XTENSA__
                    //esp_sleep_enable_ext1_wakeup(0x1, ESP_EXT1_WAKEUP_ALL_LOW);
                    esp_sleep_enable_ext1_wakeup(0x0, ESP_EXT1_WAKEUP_ALL_LOW);
#else
                    esp_deep_sleep_enable_gpio_wakeup((1 << 9), ESP_GPIO_WAKEUP_GPIO_LOW);
#endif
                    esp_sleep_enable_timer_wakeup((uint64_t)config.pwr_sleep_interval * uS_TO_S_FACTOR);
                    esp_deep_sleep_start();
                }
            }
        }
        // if (ESP.getFreeHeap() < 60000)
        //     esp_restart();
        // Serial.println(String(ESP.getFreeHeap()));
    }

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

  sprintf(str, "%s-%d>APE32T%s::%s:ack%d", config.aprs_mycall, config.aprs_ssid, VERSION, call, msgId);
  //	client.println(str);
  return String(str);
}

void sendIsPkg(char *raw)
{
  char str[300];
  sprintf(str, "%s-%d>APE32T%s:%s", config.aprs_mycall, config.aprs_ssid, VERSION, raw);
  // client.println(str);
  String tnc2Raw = String(str);
  if (aprsClient.connected())
    aprsClient.println(tnc2Raw); // Send packet to Inet
  if (config.digi_en)
    pkgTxPush(str, strlen(str), 0, RF_CHANNEL);
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
    sprintf(str, "%s>APE32T::%s:%s", config.aprs_mycall, call, raw);
  else
    sprintf(str, "%s-%d>APE32T::%s:%s", config.aprs_mycall, config.aprs_ssid, call, raw);

  String tnc2Raw = String(str);
  if (aprsClient.connected())
    aprsClient.println(tnc2Raw); // Send packet to Inet
                                 // if (config.tnc && config.tnc_digi)
                                 //     pkgTxUpdate(str, 0);
                                 // APRS_sendTNC2Pkt(tnc2Raw); // Send packet to RF
}

void sendTelemetry_0(char *raw, bool header)
{
  char str[300];
  char call[11];
  int i;
  memset(&call[0], 0, 11);
  if (strlen(config.trk_item) > 3)
  {
    sprintf(call, "%s", config.trk_item);
  }
  else
  {
    if (config.tlm0_ssid == 0)
      sprintf(call, "%s", config.tlm0_mycall);
    else
      sprintf(call, "%s-%d", config.tlm0_mycall, config.tlm0_ssid);
  }
  i = strlen(call);
  for (; i < 9; i++)
    call[i] = 0x20;

  if (header)
  {
    if (config.tlm0_ssid == 0)
    {
      if (config.tlm0_path < 5)
      {
        if (config.tlm0_path > 0)
          sprintf(str, "%s>APE32T-%d::%s:%s ", config.tlm0_mycall, config.tlm0_path, call, raw);
        else
          sprintf(str, "%s>APE32T::%s:%s ", config.tlm0_mycall, call, raw);
      }
      else
      {
        sprintf(str, "%s>APE32T,%s::%s:%s ", config.tlm0_mycall, getPath(config.tlm0_path).c_str(), call, raw);
      }
    }
    else
    {
      if (config.tlm0_path < 5)
      {
        if (config.tlm0_path > 0)
          sprintf(str, "%s-%d>APE32T-%d::%s:%s ", config.tlm0_mycall, config.tlm0_ssid, config.tlm0_path, call, raw);
        else
          sprintf(str, "%s-%d>APE32T::%s:%s ", config.tlm0_mycall, config.tlm0_ssid, call, raw);
      }
      else
      {
        sprintf(str, "%s-%d>APE32T,%s::%s:%s ", config.tlm0_mycall, config.tlm0_ssid, getPath(config.tlm0_path).c_str(), call, raw);
      }
    }
  }
  else
  {
    if (config.tlm0_ssid == 0)
    {
      if (config.tlm0_path < 5)
      {
        if (config.tlm0_path > 0)
          sprintf(str, "%s>APE32T-%d:%s", config.tlm0_mycall, config.tlm0_path, raw);
        else
          sprintf(str, "%s>APE32T:%s", config.tlm0_mycall, raw);
      }
      else
      {
        sprintf(str, "%s>APE32T,%s:%s", config.tlm0_mycall, getPath(config.tlm0_path).c_str(), raw);
      }
    }
    else
    {
      if (config.tlm0_path < 5)
      {
        if (config.tlm0_path > 0)
          sprintf(str, "%s-%d>APE32T-%d:%s", config.tlm0_mycall, config.tlm0_ssid, config.tlm0_path, raw);
        else
          sprintf(str, "%s-%d>APE32T:%s", config.tlm0_mycall, config.tlm0_ssid, raw);
      }
      else
      {
        sprintf(str, "%s-%d>APE32T,%s:%s", config.tlm0_mycall, config.tlm0_ssid, getPath(config.tlm0_path).c_str(), raw);
      }
    }
  }

  uint8_t SendMode = 0;
  if (config.tlm0_2rf)
    SendMode |= RF_CHANNEL;
  if (config.tlm0_2inet)
    SendMode |= INET_CHANNEL;
  pkgTxPush(str, strlen(str), 0, SendMode);

#if 1
  if (config.tlm0_2rf)
  { // TLM SEND TO RF
    SendMode |= RF_CHANNEL;
    // char *rawP = (char *)malloc(rawData.length());
    //  rawData.toCharArray(rawP, rawData.length());
    // memcpy(rawP, rawData.c_str(), rawData.length());
    pkgTxPush(str, strlen(str), 0, RF_CHANNEL);
    // pushTxDisp(TXCH_RF, "TX DIGI POS", sts);
    // free(rawP);
  }
  if (config.tlm0_2inet)
  { // TLM SEND TO APRS-IS

    if (aprsClient.connected())
    {
      status.txCount++;
      aprsClient.printf("%s\r\n", str); // Send packet to Inet
                                        // pushTxDisp(TXCH_TCP, "TX DIGI POS", sts);
    }
  }
#endif
}

void sendTelemetry_trk(char *raw)
{
  char str[300];
  char call[11];
  int i;
  memset(&call[0], 0, 11);
  if (strlen(config.trk_item) > 3)
  {
    sprintf(call, "%s", config.trk_item);
  }
  else
  {
    if (config.trk_ssid == 0)
      sprintf(call, "%s", config.trk_mycall);
    else
      sprintf(call, "%s-%d", config.trk_mycall, config.trk_ssid);
  }
  i = strlen(call);
  for (; i < 9; i++)
    call[i] = 0x20;

  if (config.trk_ssid == 0)
  {
    if (config.trk_path < 5)
    {
      if (config.trk_path > 0)
        sprintf(str, "%s>APE32T-%d::%s:%s ", config.trk_mycall, config.trk_path, call, raw);
      else
        sprintf(str, "%s>APE32T::%s:%s ", config.trk_mycall, call, raw);
    }
    else
    {
      sprintf(str, "%s>APE32T,%s::%s:%s ", config.trk_mycall, getPath(config.trk_path).c_str(), call, raw);
    }
  }
  else
  {
    if (config.trk_path < 5)
    {
      if (config.trk_path > 0)
        sprintf(str, "%s-%d>APE32T-%d::%s:%s ", config.trk_mycall, config.trk_ssid, config.trk_path, call, raw);
      else
        sprintf(str, "%s-%d>APE32T::%s:%s ", config.trk_mycall, config.trk_ssid, call, raw);
    }
    else
    {
      sprintf(str, "%s-%d>APE32T,%s::%s:%s ", config.trk_mycall, config.trk_ssid, getPath(config.trk_path).c_str(), call, raw);
    }
  }

  uint8_t SendMode = 0;
  if (config.trk_loc2rf)
    SendMode |= RF_CHANNEL;
  if (config.trk_loc2inet)
    SendMode |= INET_CHANNEL;
  pkgTxPush(str, strlen(str), 0, SendMode);

  // if (config.trk_loc2rf)
  // { // TLM SEND TO RF
  //     pkgTxPush(str, strlen(str), 0);
  // }
  // if (config.trk_loc2inet)
  // { // TLM SEND TO APRS-IS
  //     if (aprsClient.connected())
  //     {
  //         status.txCount++;
  //         aprsClient.printf("%s\r\n", str); // Send packet to Inet
  //         delay(2000);
  //     }
  // }
}

void sendTelemetry_igate(char *raw)
{
  char str[300];
  char call[11];
  int i;
  memset(&call[0], 0, 11);
  if (strlen(config.igate_object) > 3)
  {
    sprintf(call, "%s", config.igate_object);
  }
  else
  {
    if (config.aprs_ssid == 0)
      sprintf(call, "%s", config.aprs_mycall);
    else
      sprintf(call, "%s-%d", config.aprs_mycall, config.aprs_ssid);
  }
  i = strlen(call);
  for (; i < 9; i++)
    call[i] = 0x20;

  if (config.aprs_ssid == 0)
  {
    if (config.igate_path < 5)
    {
      if (config.igate_path > 0)
        sprintf(str, "%s>APE32T-%d::%s:%s ", config.aprs_mycall, config.igate_path, call, raw);
      else
        sprintf(str, "%s>APE32T::%s:%s ", config.aprs_mycall, call, raw);
    }
    else
    {
      sprintf(str, "%s>APE32T,%s::%s:%s ", config.aprs_mycall, getPath(config.igate_path).c_str(), call, raw);
    }
  }
  else
  {
    if (config.igate_path < 5)
    {
      if (config.igate_path > 0)
        sprintf(str, "%s-%d>APE32T-%d::%s:%s ", config.aprs_mycall, config.aprs_ssid, config.igate_path, call, raw);
      else
        sprintf(str, "%s-%d>APE32T::%s:%s ", config.aprs_mycall, config.aprs_ssid, call, raw);
    }
    else
    {
      sprintf(str, "%s-%d>APE32T,%s::%s:%s ", config.aprs_mycall, config.aprs_ssid, getPath(config.igate_path).c_str(), call, raw);
    }
  }

  uint8_t SendMode = 0;
  if (config.igate_loc2rf)
    SendMode |= RF_CHANNEL;
  if (config.igate_loc2inet)
    SendMode |= INET_CHANNEL;
  pkgTxPush(str, strlen(str), 0, SendMode);
  // if (config.igate_loc2rf)
  // { // TLM SEND TO RF
  //     pkgTxPush(str, strlen(str), 0);
  // }
  // if (config.igate_loc2inet)
  // { // TLM SEND TO APRS-IS
  //     if (aprsClient.connected())
  //     {
  //         status.txCount++;
  //         aprsClient.printf("%s\r\n", str); // Send packet to Inet
  //         delay(2000);
  //     }
  // }
}

void sendTelemetry_digi(char *raw)
{
  char str[300];
  char call[11];
  int i;
  memset(&call[0], 0, 11);

  if (config.digi_ssid == 0)
    sprintf(call, "%s", config.digi_mycall);
  else
    sprintf(call, "%s-%d", config.digi_mycall, config.digi_ssid);
  i = strlen(call);
  for (; i < 9; i++)
    call[i] = 0x20;

  if (config.digi_ssid == 0)
  {
    if (config.digi_path < 5)
    {
      if (config.digi_path > 0)
        sprintf(str, "%s>APTWR-%d::%s:%s ", config.digi_mycall, config.digi_path, call, raw);
      else
        sprintf(str, "%s>APTWR::%s:%s ", config.digi_mycall, call, raw);
    }
    else
    {
      sprintf(str, "%s>APTWR,%s::%s:%s ", config.digi_mycall, getPath(config.digi_path).c_str(), call, raw);
    }
  }
  else
  {
    if (config.digi_path < 5)
    {
      if (config.digi_path > 0)
        sprintf(str, "%s-%d>APTWR-%d::%s:%s ", config.digi_mycall, config.digi_ssid, config.digi_path, call, raw);
      else
        sprintf(str, "%s-%d>APTWR::%s:%s ", config.digi_mycall, config.digi_ssid, call, raw);
    }
    else
    {
      sprintf(str, "%s-%d>APTWR,%s::%s:%s ", config.digi_mycall, config.digi_ssid, getPath(config.digi_path).c_str(), call, raw);
    }
  }

  uint8_t SendMode = 0;
  if (config.digi_loc2rf)
    SendMode |= RF_CHANNEL;
  if (config.digi_loc2inet)
    SendMode |= INET_CHANNEL;
  pkgTxPush(str, strlen(str), 0, SendMode);
  // if (config.digi_loc2rf)
  // { // TLM SEND TO RF
  //     pkgTxPush(str, strlen(str), 0);
  // }
  // if (config.digi_loc2inet)
  // { // TLM SEND TO APRS-IS
  //     if (aprsClient.connected())
  //     {
  //         status.txCount++;
  //         aprsClient.printf("%s\r\n", str); // Send packet to Inet
  //         delay(2000);
  //     }
  // }
}

RTC_DATA_ATTR statusType statOld;
uint8_t getState(int ch)
{
  int val = 0;
  switch (ch)
  {
  case 0: // none
    val = 0;
    break;
  case 1: // allCount
    val = status.allCount - statOld.allCount;
    statOld.allCount = status.allCount;
    break;
  case 2: // rf2inet
    val = status.rf2inet - statOld.rf2inet;
    statOld.rf2inet = status.rf2inet;
    break;
  case 3: // inet2rf
    val = status.inet2rf - statOld.inet2rf;
    statOld.inet2rf = status.inet2rf;
    break;
  case 4: // digi
    val = status.digiCount - statOld.digiCount;
    statOld.digiCount = status.digiCount;
    break;
  case 5: // Drop
    val = status.dropCount - statOld.dropCount;
    statOld.dropCount = status.dropCount;
    break;
  }
  if (val < 0)
    val = 0;
  return val;
}

bool getBits(int ch)
{
  bool val = false;
  switch (ch)
  {
  case 0: // none
    val = 0;
    break;
  case 1: // IGATE Enable
    val = config.igate_en;
    break;
  case 2: // DIGI Enable
    val = config.digi_en;
    break;
  case 3: // WX Enable
    val = config.wx_en;
    break;
  case 4: // Sat Enable
    val = 0;
    break;
  case 5: // APRS-IS Status
    if (aprsClient.connected())
      val = 1;
    break;
  case 6: // VPN Status
    val = wireguard_active();
    break;
  case 7: // 4G LTE
    val = 0;
    break;
  case 8: // FX.25
    val = (config.fx25_mode>0)?1:0;
    break;
  }
  return val;
}

void getTelemetry_0()
{
  systemTLM.A1 = getState(config.tml0_data_channel[0]);
  systemTLM.A2 = getState(config.tml0_data_channel[1]);
  systemTLM.A3 = getState(config.tml0_data_channel[2]);
  systemTLM.A4 = getState(config.tml0_data_channel[3]);
  systemTLM.A5 = getState(config.tml0_data_channel[4]);
  systemTLM.BITS = 0;
  if (getBits(config.tml0_data_channel[5]))
    systemTLM.BITS |= 0x01;
  if (getBits(config.tml0_data_channel[6]))
    systemTLM.BITS |= 0x02;
  if (getBits(config.tml0_data_channel[7]))
    systemTLM.BITS |= 0x04;
  if (getBits(config.tml0_data_channel[8]))
    systemTLM.BITS |= 0x08;
  if (getBits(config.tml0_data_channel[9]))
    systemTLM.BITS |= 0x10;
  if (getBits(config.tml0_data_channel[10]))
    systemTLM.BITS |= 0x20;
  if (getBits(config.tml0_data_channel[11]))
    systemTLM.BITS |= 0x40;
  if (getBits(config.tml0_data_channel[12]))
    systemTLM.BITS |= 0x80;
}

WiFiClient gnssClient;
extern AsyncWebSocket ws_gnss;
char nmea[100];
int nmea_idx = 0;

void taskGPS(void *pvParameters)
{
  int c;
  log_d("GNSS Init");
  nmea_idx = 0;

  if (config.gnss_enable)
  {
    if ((config.gnss_channel > 0) && (config.gnss_channel < 4))
    {
      if (strstr("AT", config.gnss_at_command) != NULL)
      {
        if (config.gnss_channel == 1)
        {
          Serial0.println(config.gnss_at_command);
        }
        else if (config.gnss_channel == 2)
        {
          Serial1.println(config.gnss_at_command);
        }
// #ifdef __XTENSA__
//         else if (config.gnss_channel == 3)
//         {
//           Serial2.println(config.gnss_at_command);
//         }
// #endif
      }
    }
  }
  for (;;)
  {
    timerGPS = micros() - timerGPS_old;
    vTaskDelay(10 / portTICK_PERIOD_MS);
    timerGPS_old = micros();

    if (config.gnss_enable)
    {
      if ((config.gnss_channel > 0) && (config.gnss_channel < 4))
      {
        do
        {
          c = -1;
          if (config.gnss_channel == 1)
          {
            c = Serial0.read();
          }
          else if (config.gnss_channel == 2)
          {
            c = Serial1.read();
          }
// #ifdef __XTENSA__
//           else if (config.gnss_channel == 3)
//           {
//             c = Serial2.read();
//           }
// #endif
          if (c > -1)
          {
            gps.encode((char)c);
            if (webServiceBegin == false)
            {
              if (nmea_idx > 99)
              {
                nmea_idx = 0;
                memset(nmea, 0, sizeof(nmea));
                // SerialGNSS->flush();
              }
              else
              {
                nmea[nmea_idx++] = (char)c;
                if ((char)c == '\r' || (char)c == '\n')
                {
                  // nmea[nmea_idx++] = 0;
                  if (nmea_idx > 10)
                  {
                    if (ws_gnss.enabled() && !ws_gnss.getClients().isEmpty())
                    // if (webServiceBegin == false)
                    {
                      handle_ws_gnss(nmea, nmea_idx);
                    }
                    // log_d("%s",nmea);
                  }
                  nmea_idx = 0;
                  memset(nmea, 0, sizeof(nmea));
                  vTaskDelay(50 / portTICK_PERIOD_MS);
                  break;
                }
              }
            }
            //}
          }
          else
          {
            break;
          }
        } while (1);
      }
      else if (config.gnss_channel == 4)
      { // TCP
        if (WiFi.isConnected())
        {
          if (!gnssClient.connected())
          {
            gnssClient.connect(config.gnss_tcp_host, config.gnss_tcp_port);
            log_d("GNSS TCP ReConnect to %s:%d", config.gnss_tcp_host, config.gnss_tcp_port);
            delay(3000);
          }
          else
          {
            while (gnssClient.available())
            {
              c = (char)gnssClient.read();
              // Serial.print(c);
              gps.encode(c);
              if (webServiceBegin == false)
              {
                if (nmea_idx > 99)
                {
                  nmea_idx = 0;
                  memset(nmea, 0, sizeof(nmea));
                }
                else
                {
                  // nmea[nmea_idx++] = c;
                  if (c == '\r' || c == '\n')
                  {
                    nmea[nmea_idx++] = 0;
                    if (nmea_idx > 10)
                    {
                      if (ws_gnss.enabled() && !ws_gnss.getClients().isEmpty())
                      {
                        handle_ws_gnss(nmea, nmea_idx);
                      }
                      // log_d("%s",nmea);
                    }
                    nmea_idx = 0;
                    memset(nmea, 0, sizeof(nmea));
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                  }
                }
              }
            }
          }
        }
      }

      if (firstGpsTime && gps.time.isValid())
      {
        if (gps.time.isUpdated())
        {
          time_t timeGps = getGpsTime(); // Local gps time
          if (timeGps > 1700000000 && timeGps < 2347462800)
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
}

void taskSerial(void *pvParameters)
{
  String raw;
  int c;
  char rawP[500];
  char call[11];
  log_d("Serial task Init");
  nmea_idx = 0;
  if (config.ext_tnc_enable)
  {
    if (config.ext_tnc_channel == 1)
    {

#if ARDUINO_USB_CDC_ON_BOOT
      Serial0.setTimeout(50);
      Serial0.setRxBufferSize(350);
#else
      Serial.setTimeout(10);
#endif
    }
    else if (config.ext_tnc_channel == 2)
    {
      Serial1.setTimeout(50);
      Serial1.setRxBufferSize(350);
    }
    else if (config.ext_tnc_channel == 3)
    {
      Serial.setTimeout(25);
      Serial.setRxBufferSize(350);
      // Serial2.setTimeout(10);
    }
  }

  for (;;)
  {
    timerSerial = millis() - timerSerial_old;
    vTaskDelay(10 / portTICK_PERIOD_MS);
    timerSerial_old = micros();

    if (config.ext_tnc_enable && (config.ext_tnc_mode > 0 && config.ext_tnc_mode < 4))
    {
      if (config.ext_tnc_mode == 1)
      { // KISS
        // KISS MODE
        do
        {
          c = -1;
          if (config.ext_tnc_channel == 1)
          {
#if ARDUINO_USB_CDC_ON_BOOT
            c = Serial0.read();
#else
            c = Serial.read();
#endif
          }
          else if (config.ext_tnc_channel == 2)
          {
            c = Serial1.read();
          }
          else if (config.ext_tnc_channel == 3)
          {
            c = Serial.read();
          }

          if (c > -1)
           kiss_serial((uint8_t)c);
          else
           break;
        } while (c > -1);
      }
      else if (config.ext_tnc_mode == 2)
      { // TNC2RAW
        raw.clear();
        if (config.ext_tnc_channel == 1)
        {
#if ARDUINO_USB_CDC_ON_BOOT
          raw = Serial0.readStringUntil(0x0D);
#else
          raw = Serial.readStringUntil(0x0D);
#endif
        }
        else if (config.ext_tnc_channel == 2)
        {
          raw = Serial1.readStringUntil(0x0D);
        }
        else if (config.ext_tnc_channel == 3)
        {
          raw = Serial.readStringUntil(0x0D);
        }

        log_d("Ext TNC2RAW RX:%s", raw.c_str());
        if(raw.length()>5){
          String src_call = raw.substring(0, raw.indexOf('>'));
          if ((src_call != "") && (src_call.length() < 10) && (raw.length() < sizeof(rawP)))
          {
            memset(call, 0, sizeof(call));
            strcpy(call, src_call.c_str());
            strcpy(rawP, raw.c_str());
            uint16_t type = pkgType((const char *)rawP);
            pkgListUpdate(call, rawP, type, 1, 0);
            if (config.rf2inet && aprsClient.connected())
            {
              // RF->INET
              aprsClient.write(&rawP[0], strlen(rawP)); // Send binary frame packet to APRS-IS (aprsc)
              aprsClient.write("\r\n");                 // Send CR LF the end frame packet
              status.rf2inet++;
              igateTLM.RF2INET++;
              igateTLM.RX++;
            }
          }
        }
      }
      else if (config.ext_tnc_mode == 3)
      { // YAESU FTM-350,FTM-400
        String info = "";
        if (config.ext_tnc_channel == 1)
        {
#if ARDUINO_USB_CDC_ON_BOOT
          info = Serial0.readString();
#else
          info = Serial.readString();
#endif
        }
        else if (config.ext_tnc_channel == 2)
        {
          info = Serial1.readString();
        }
        else if (config.ext_tnc_channel == 3)
        {
          info = Serial.readString();
        }

        //  log_d("Ext Yaesu Packet >> %s",info.c_str());
        int ed = info.indexOf(" [");
        if (info != "" && ed > 10)
        {
          raw.clear();
          raw = info.substring(0, ed);
          int st = info.indexOf(">:");
          if (st > ed)
          {
            int idx = 0;
            st += 2;
            for (int i = 0; i < 5; i++)
            {
              if (info.charAt(st + i) == 0x0A || info.charAt(st + i) == 0x0D)
              {
                idx++;
              }
              else
              {
                break;
              }
            }
            st += idx;
            ed = info.indexOf(0x0D, st + 1);
            if (ed > info.length())
              ed = info.length();
            if (ed > st)
            {
              raw += ":" + info.substring(st, ed);

              String src_call = raw.substring(0, raw.indexOf('>'));
              if ((src_call != "") && (src_call.length() < 11) && (raw.length() < sizeof(rawP)))
              {
                memset(call, 0, sizeof(call));
                strcpy(call, src_call.c_str());
                memset(rawP, 0, sizeof(rawP));
                strcpy(rawP, raw.c_str());
                log_d("Yaesu Packet: CallSign:%s RAW:%s", call, rawP);
                // String hstr="";
                // for(int i=0;i<raw.length();i++){
                //     hstr+=" "+String(rawP[i],HEX);
                // }
                // log_d("HEX: %s",hstr.c_str());
                uint16_t type = pkgType((const char *)rawP);
                pkgListUpdate(call, rawP, type, 1, 0);
                if (config.rf2inet && aprsClient.connected())
                {
                  // RF->INET
                  aprsClient.write(&rawP[0], strlen(rawP)); // Send binary frame packet to APRS-IS (aprsc)
                  aprsClient.write("\r\n");                 // Send CR LF the end frame packet
                  status.rf2inet++;
                  igateTLM.RF2INET++;
                  igateTLM.RX++;
                }
              }
            }
          }
        }
      }
      //}
    }
  }
}

void taskTNC(void *pvParameters)
{
  unsigned long timer, result;
  for (;;)
  {
    // timer=micros();
    if (AFSKInitAct == true)
      // AFSK_Poll(true, config.rf_power);
      // result=micros()-timer;
      // log_d("AFSK_Poll timer = %.2f mS",(float)result/1000);
      vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

String getPath(int idx)
{
  String ret = "";
  switch (idx)
  {
  case 0: // OFF
    ret = "";
    break;
  case 1: // DST-TRACE1
  case 2: // DST-TRACE2
  case 3: // DST-TRACE3
  case 4: // DST-TRACE4
    ret = "DST" + String(idx);
    break;
  case 5: // TRACE1-1
    ret = "TRACE1-1";
    break;
  case 6:
    ret = "TRACE2-2";
    break;
  case 7:
    ret = "TRACE3-3";
    break;
  case 8:
    ret = "WIDE1-1";
    break;
  case 9:
    ret = "RFONLY";
    break;
  case 10:
    ret = "RELAY";
    break;
  case 11:
    ret = "GATE";
    break;
  case 12:
    ret = "ECHO";
    break;
  case 13: // UserDefine1
    ret = String(config.path[0]);
    break;
  case 14: // UserDefine2
    ret = String(config.path[1]);
    break;
  case 15: // UserDefine3
    ret = String(config.path[2]);
    break;
  case 16: // UserDefine4
    ret = String(config.path[3]);
    break;
  default:
    ret = "WIDE1-1";
    break;
  }
  return ret;
}

extern int8_t adcEn;
extern int8_t dacEn;
long timeSlot;
unsigned long iGatetickInterval;
unsigned long WxInterval;
bool initInterval = true;
bool RF_INIT = true;
void taskAPRS(void *pvParameters)
{
  //	long start, stop;
  char sts[50];
  unsigned long tickInterval = 0;
  unsigned long DiGiInterval = 0;

  unsigned long igateSTSInterval = 0;
  unsigned long digiSTSInterval = 0;
  unsigned long trkSTSInterval = 0;

  //delay(2000);
  log_d("Task APRS has been start");
  log_d("RF Module config");
 
  afskSetModem(config.modem_type, config.audio_lpf, config.tx_timeslot, config.preamble * 100,config.fx25_mode);
   
  // afskSetDCOffset(config.adc_dc_offset);
  afskSetADCAtten(config.adc_atten);
  APRS_setCallsign(config.aprs_mycall, config.aprs_ssid);
  // APRS_setPath1("WIDE1-1", 1);
  // APRS_setPreamble(300);
  // APRS_setTail(0);
  AFSK_init(config.adc_gpio, config.dac_gpio, config.rf_ptt_gpio, config.rf_sql_gpio, config.rf_pwr_gpio, -1, -1, 42, config.rf_ptt_active, config.rf_sql_active, config.rf_pwr_active);
  sendTimer = millis() - (config.igate_interval * 1000) + 30000;
  igateTLM.TeleTimeout = millis() + 60000; // 1Min

  timeSlot = millis();
  timeAprs = 0;
  // afskSetHPF(config.audio_hpf);
  // afskSetBPF(config.audio_bpf);
  tx_interval = config.trk_interval;
  tx_counter = tx_interval - 10;

  initInterval = true;
  AFSKInitAct = true;

  if (config.rf_en){
    RF_MODULE(true);
    setPtt(false);        
  }
  RF_INIT = false;
  for (;;)
  {
    if(RF_INIT){
      if (config.rf_en){
        RF_MODULE(false);
        setPtt(false);        
      }
      RF_INIT=false;
    }

    if (adcEn == 1)
    {
      AFSK_TimerEnable(true);
      adcEn = 0;
    }
    else if (adcEn == -1)
    {
      AFSK_TimerEnable(false);
      adcEn = 0;
    }

    if (dacEn == 1)
    {
      DAC_TimerEnable(true);
      dacEn = 0;
    }
    else if (dacEn == -1)
    {
      DAC_TimerEnable(false);
      dacEn = 0;
    }
    long now = millis();
    // wdtSensorTimer = now;
    // time_t timeStamp;
    // time(&timeStamp);
    if (initInterval)
    {
      tickInterval = WxInterval = DiGiInterval = igateSTSInterval = iGatetickInterval = digiSTSInterval =trkSTSInterval= millis() + 10000;
      systemTLM.ParmTimeout = millis() + 20000;
      systemTLM.TeleTimeout = millis() + 30000;
      initInterval = false;
      tx_interval = config.trk_interval;
      tx_counter = tx_interval - 10;
    }
    timerAPRS = micros() - timerAPRS_old;
    vTaskDelay(10 / portTICK_PERIOD_MS);
    timerAPRS_old = micros();

    // SEND RF in time slot
    // if (now > timeSlot)
    //{
    // Transmit in timeslot if enabled
    pkgTxSend();
    // timeSlot = millis() + 100;
    //}
    Ax25TransmitBuffer(); // transmit buffer (will return if nothing to be transmitted)
    Ax25TransmitCheck();  // check for pending transmission request
    // if (config.rf_type == RF_SA8x8_OpenEdit)
    // {
    //   // bool ptt_stat = getTransmit();
    //   // if (ptt_stat == true && ptt_stat_old == false)
    //   // {
    //   //   ptt_stat_old = ptt_stat;
    //   if(pttON)
    //     pttON=false;
    //     sa868.TxOn();
    //     setTransmit(true);
    //   }
    //   // else if (ptt_stat == false && ptt_stat_old == true)
    //   // {
    //   //   ptt_stat_old = ptt_stat;
    //   //   sa868.TxOff();
    //   //   sa868.setLowPower();
    //   //   delay(100);
    //   //   sa868.RxOn();
    //   //   setTransmit(false);
    //   // }
    // }

    if (config.trk_en)
    { // TRACKER MODE
      if (config.trk_sts_interval > 10)
      {
        if (millis() > trkSTSInterval)
        {
          trkSTSInterval = millis() + (config.trk_sts_interval * 1000);
          tracker_status(config.trk_status);
        }
      }
      if (millis() > tickInterval)
      {
        tickInterval = millis() + 1000;

        tx_counter++;
        // log_d("TRACKER tx_counter=%d\t INTERVAL=%d\n", tx_counter, tx_interval);
        //   Check interval timeout
        if (config.trk_smartbeacon && config.trk_gps)
        {
          if ((gps.satellites.value() > 3) && (gps.hdop.hdop() < 10))
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
        }
        else if (tx_counter > tx_interval)
        {
          // if (tx_counter > config.trk_slowinterval)
          //{
          EVENT_TX_POSITION = 6;
          tx_interval = config.trk_interval;
          //}
        }

        // if (config.trk_gps && gps.speed.isValid() && gps.location.isValid() && gps.course.isValid() && (gps.hdop.hdop() < 10.0) && (gps.satellites.value() > 3))
        // if (config.trk_gps && gps.speed.isValid() && gps.location.isValid() && gps.course.isValid())
        if (config.trk_gps)
        {
          SB_SPEED_OLD = SB_SPEED;
          if (gps.satellites.value() > 3 && gps.hdop.hdop() < 10)
          {
            SB_SPEED = (unsigned char)gps.speed.kmph();
            // if (gps.course.isUpdated())
            if (gps.speed.kmph() > config.trk_lspeed)
              SB_HEADING = (int16_t)gps.course.deg();
          }
          else
          {
            if (SB_SPEED > 0)
              SB_SPEED--;
          }
          if (config.trk_smartbeacon) // SMART BEACON CAL
          {
            if (SB_SPEED < config.trk_lspeed && SB_SPEED_OLD > config.trk_lspeed) // Speed slow down to STOP
            {                                                                     // STOPING
              SB_SPEED_OLD = 0;
              if (tx_counter > config.trk_mininterval)
              {
                EVENT_TX_POSITION = 7;
                tx_interval = config.trk_slowinterval;
              }
            }
            else
            {
              smartbeacon();
            }
          }
          else if (tx_counter > tx_interval)
          { // send gps location
            if (gps.location.isValid() && gps.hdop.hdop() < 10)
            {
              EVENT_TX_POSITION = 8;
              tx_interval = config.trk_interval;
            }
          }
        }
      }

      if (EVENT_TX_POSITION > 0)
      {
        String rawData;
        String cmn = "";
        Sleep_Activate &= ~ACTIVATE_TRACKER;
        StandByTick = millis() + (5000);
        if (config.trk_tlm_sensor[0] | config.trk_tlm_sensor[1] | config.trk_tlm_sensor[2] | config.trk_tlm_sensor[3] | config.trk_tlm_sensor[4])
        {
          char tlm_result[100];
          char tlm_data[200];
          size_t tlm_sz;
          if ((TLM_SEQ % 100) == 0)
          {
            char rawInfo[100];
            char name[10];
            sprintf(rawInfo, "PARM.");
            int i, c = 0;
            for (i = 0; i < 5; i++)
            {
              if (config.trk_tlm_sensor[i] == 0)
              {
                c++;
                continue;
              }
              else
              {
                if (i > 0)
                  strcat(rawInfo, ",");
                sprintf(name, "%s", config.trk_tlm_PARM[i]);
                strcat(rawInfo, name);
              }
            }
            for (int n = c + 8; n > 0; n--)
            {
              strcat(rawInfo, ",");
            }
            sendTelemetry_trk(rawInfo);
            memset(rawInfo, 0, sizeof(rawInfo));
            sprintf(rawInfo, "UNIT.");
            c = 0;
            for (i = 0; i < 5; i++)
            {
              if (config.trk_tlm_sensor[i] == 0)
              {
                c++;
                continue;
              }
              else
              {
                if (i > 0)
                  strcat(rawInfo, ",");
                sprintf(name, "%s", config.trk_tlm_UNIT[i]);
                strcat(rawInfo, name);
              }
            }
            for (int n = c + 8; n > 0; n--)
            {
              strcat(rawInfo, ",");
            }
            sendTelemetry_trk(rawInfo);
            memset(rawInfo, 0, sizeof(rawInfo));
            sprintf(rawInfo, "EQNS.");
            c = 0;
            for (i = 0; i < 5; i++)
            {
              if (config.trk_tlm_sensor[i] == 0)
              {
                c++;
                continue;
              }
              else
              {
                if (i > 0)
                  strcat(rawInfo, ",");
                if (fmod(config.trk_tlm_EQNS[i][0], 1) == 0)
                  sprintf(name, "%0.f", config.trk_tlm_EQNS[i][0]);
                else
                  sprintf(name, "%.3f", config.trk_tlm_EQNS[i][0]);
                strcat(rawInfo, name);
                if (fmod(config.trk_tlm_EQNS[i][1], 1) == 0)
                  sprintf(name, ",%0.f", config.trk_tlm_EQNS[i][1]);
                else
                  sprintf(name, ",%.3f", config.trk_tlm_EQNS[i][1]);
                strcat(rawInfo, name);
                if (fmod(config.trk_tlm_EQNS[i][2], 1) == 0)
                  sprintf(name, ",%0.f", config.trk_tlm_EQNS[i][2]);
                else
                  sprintf(name, ",%.3f", config.trk_tlm_EQNS[i][2]);
                strcat(rawInfo, name);
              }
            }
            for (int n = c; n > 0; n--)
            {
              strcat(rawInfo, ",");
              sprintf(name, "0");
              strcat(rawInfo, name);
              sprintf(name, ",1");
              strcat(rawInfo, name);
              sprintf(name, ",0");
              strcat(rawInfo, name);
            }
            // strcat(rawInfo, ",");
            sendTelemetry_trk(rawInfo);
          }

          if (++TLM_SEQ > 8279)
            TLM_SEQ = 0;
          memset(tlm_data, 0, 200);
          memset(tlm_result, 0, 100);
          int n = 0;
          sprintf(tlm_data, "%i", TLM_SEQ);
          for (int s = 0; s < 5; s++)
          {
            if (config.trk_tlm_sensor[s] == 0)
            {
              continue;
              // strcat(tlm_data, "0");
            }
            else
            {
              strcat(tlm_data, ",");
              int sen_idx = config.trk_tlm_sensor[s] - 1;
              double data = 0;
              if (sen[sen_idx].visable)
                data = sen[sen_idx].sample;
              double precision = pow(10.0f, (double)config.trk_tlm_precision[s]);
              int val = (int)((data + config.trk_tlm_offset[s]) * precision);
              // log_d("s:%d Data:%.2f /tPresion:%.5f /tOffset:%.5f/t Val:%d",s,sen[sen_idx].sample,precision,config.trk_tlm_offset[s],val);
              if (val > 8280)
                val = 8280;
              if (val < 0)
                val = 0;
              char strVal[10];
              sprintf(strVal, "%i", val);
              strcat(tlm_data, strVal);
            }
          }
          // log_d("TLM_DATA:%s",tlm_data);
          //  sprintf(tlm_data, "%i,%i,%i,%i", TLM_SEQ, (int)(VBat * 100), int(TempNTC * 100), gps.satellites.value());
          telemetry_base91(tlm_data, tlm_result, tlm_sz);
          cmn = String(tlm_result);
        }

        if (config.trk_rssi)
        {
          cmn += " ?RSSI";
        }

        if (config.trk_gps) // TRACKER by GPS
        {
          rawData = trk_gps_postion(cmn);
          if (config.log & LOG_TRACKER)
          {
            logTracker(gps.location.lat(), gps.location.lng(), gps.speed.kmph(), gps.course.deg());
          }
        }
        else // TRACKER by FIX position
        {
          rawData = trk_fix_position(cmn);
          if (config.log & LOG_TRACKER)
          {
            logTracker(config.trk_lat, config.trk_lon, 0, 0);
          }
        }

        log_d("TRACKER RAW: %s\n", rawData.c_str());
        log_d("TRACKER EVENT_TX_POSITION=%d\t INTERVAL=%d\n", EVENT_TX_POSITION, tx_interval);
        tx_counter = 0;
        EVENT_TX_POSITION = 0;
        last_heading = SB_HEADING;
#if defined OLED || defined ST7735_160x80
        if (config.trk_gps)
        {
          // if (gps.location.isValid() && (gps.hdop.hdop() < 10.0))
          sprintf(sts, "POSITION GPS\nSPD %dkPh/%d\nINTERVAL %ds", SB_SPEED, SB_HEADING, tx_interval);
          // else
          //     sprintf(sts, "POSITION GPS\nGPS INVALID\nINTERVAL %ds", tx_interval);
        }
        else
        {
          sprintf(sts, "POSITION FIX\nINTERVAL %ds", tx_interval);
        }
        char name[12];
        if (strlen(config.trk_item) > 3)
        {
          sprintf(name, "%s", config.trk_item);
        }
        else
        {
          if (config.trk_ssid > 0)
            sprintf(name, "%s-%d", config.trk_mycall, config.trk_ssid);
          else
            sprintf(name, "%s", config.trk_mycall);
        }
#endif
        uint8_t SendMode = 0;
        if (config.trk_loc2rf)
          SendMode |= RF_CHANNEL;
        if (config.trk_loc2inet)
          SendMode |= INET_CHANNEL;
        pkgTxPush(rawData.c_str(), rawData.length(), 0, SendMode);

        //                 if (config.trk_loc2rf)
        //                 { // TRACKER SEND TO RF
        //                     char *rawP = (char *)calloc(rawData.length(), sizeof(char));
        //                     memcpy(rawP, rawData.c_str(), rawData.length());
        //                     // rawData.toCharArray(rawP, rawData.length());
        //                     pkgTxPush(rawP, rawData.length(), 0);

#if defined OLED || defined ST7735_160x80
        pushTxDisp(TXCH_RF, name, sts);
#endif
        //                     free(rawP);
        //                 }
        //                 if (config.trk_loc2inet)
        //                 { // TRACKER SEND TO APRS-IS
        //                     if (aprsClient.connected())
        //                     {
        //                         aprsClient.println(rawData); // Send packet to Inet
        // #if defined OLED || defined ST7735_160x80
        //                         // pushTxDisp(TXCH_TCP, "TX TRACKER", sts);
        // #endif
        //                     }
        //                 }
        rawData.clear();
        cmn.clear();
      }
    }

    // LOAD DATA incomming
    bool newIGatePkg = false;
    bool newDigiPkg = false;
    uint8_t *buf;
    uint16_t size = 0;
    int8_t peak = 0;
    int8_t valley = 0;
    uint8_t signalLevel = 0;
    uint8_t fixed = 0;
    uint16_t mV = 0;
    if (Ax25ReadNextRxFrame(&buf, &size, &peak, &valley, &signalLevel, &fixed, &mV))
    {
      String tnc2 = "";
      // นำข้อมูลแพ็จเกจจาก TNC ออกจากคิว
      ax25_decode(buf, size, mV, &incomingPacket);
      status.rxCount++;
      packet2Raw(tnc2, incomingPacket);
      newIGatePkg = true;
      newDigiPkg = true;
      if (config.ext_tnc_enable)
      {
        if (config.ext_tnc_channel > 0 && config.ext_tnc_channel < 4)
        {
          if (config.ext_tnc_mode == 1)
          {
            // KISS MODE
            uint8_t pkg[500];
            int sz = kiss_wrapper(pkg,buf,size);
            if (config.ext_tnc_channel == 1)
            {
#if ARDUINO_USB_CDC_ON_BOOT
              Serial0.write(pkg, sz);
#else
              Serial.write(pkg, sz);
#endif
            }
            else if (config.ext_tnc_channel == 2)
            {
              Serial1.write(pkg, sz);
            }
            else if (config.ext_tnc_channel == 3)
            {
               Serial.write(pkg, sz);
            }
          }
          else if (config.ext_tnc_mode == 2)
          {
            // TNC2
            if (config.ext_tnc_channel == 1)
            {
#if ARDUINO_USB_CDC_ON_BOOT
              Serial0.println(tnc2);
#else
              Serial.println(tnc2);
#endif
            }
            else if (config.ext_tnc_channel == 2)
            {
              Serial1.println(tnc2);
            }
            else if (config.ext_tnc_channel == 3)
            {
              Serial.println(tnc2);
            }
          }
        }
      }
#ifdef BLUETOOTH
      if (config.bt_master)
      { // Output TNC2RAW to BT Serial
        // SerialBT.println(tnc2);
        if (config.bt_mode == 1)
        {
          char *rawP = (char *)ps_malloc(tnc2.length());
          memcpy(rawP, tnc2.c_str(), tnc2.length());
          // SerialBT.write((uint8_t *)rawP, tnc2.length());
          pTxCharacteristic->setValue((uint8_t *)rawP, tnc2.length());
          pTxCharacteristic->notify();
          free(rawP);
        }
        else if (config.bt_mode == 2)
        { // KISS
          //uint8_t pkg[500];
          uint8_t *pkg = (uint8_t *)ps_malloc(500);
          if(pkg){
            int sz = kiss_wrapper(pkg,buf,size);
            // SerialBT.write(pkg, sz);
            pTxCharacteristic->setValue((const uint8_t *)pkg, sz);
            pTxCharacteristic->notify();
            free(pkg);
          }
        }
      }
#endif
      log_d("RX: %s", tnc2.c_str());

      // SerialBT.println(tnc2);
      uint16_t type = pkgType((char *)incomingPacket.info);
      if (!(type & FILTER_THIRDPARTY))
      {
        char call[11];
        if (incomingPacket.src.ssid > 0)
          sprintf(call, "%s-%d", incomingPacket.src.call, incomingPacket.src.ssid);
        else
          sprintf(call, "%s", incomingPacket.src.call);

        char *rawP = (char *)calloc(tnc2.length(), sizeof(char));
        if (rawP)
        {
          memset(rawP, 0, tnc2.length());
          tnc2.toCharArray(rawP, tnc2.length(), 0);
          // memcpy(rawP, tnc2.c_str(), tnc2.length());
          int idx = pkgListUpdate(call, rawP, type, 0, incomingPacket.mVrms);

#if defined OLED || defined ST7735_160x80
          if (idx > -1)
          {

            if (config.rx_display && config.dispRF && (type & config.dispFilter))
            {
              //dispBuffer.push(tnc2.c_str());
              pushTNC2Raw(idx);
              log_d("RF_putQueueDisp:[pkgList_idx=%d,Type=%d RAW:%s] %s\n", idx, type, call, tnc2.c_str());
            }
          }
#endif
          free(rawP);
        }
      }
      lastPkg = true;
      handle_ws(tnc2, incomingPacket.mVrms);
      status.allCount++;
      tnc2.clear();
    }

    // IGate Process
    if (config.igate_en)
    {
      if (config.igate_sts_interval > 10)
      {
        if (millis() > igateSTSInterval)
        {
          igateSTSInterval = millis() + (config.igate_sts_interval * 1000);
          igate_status(config.igate_status);
        }
      }
      // IGATE Position
      if (config.igate_bcn)
      {
        if (millis() > iGatetickInterval)
        {

          String rawData = "";
          if (config.igate_gps)
          { // IGATE Send GPS position
            if (gps.location.isValid())
            {
              rawData = igate_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters(), "");
              if (config.log & LOG_IGATE)
              {
                logIGate(gps.location.lat(), gps.location.lng(), gps.speed.kmph(), gps.course.deg());
              }
            }
          }
          else
          { // IGATE Send fix position
            rawData = igate_position(config.igate_lat, config.igate_lon, config.igate_alt, "");
            if (config.log & LOG_TRACKER)
            {
              logIGate(config.igate_lat, config.igate_lon, 0, 0);
            }
          }
          if (rawData != "")
          {
            iGatetickInterval = millis() + (config.igate_interval * 1000);
            Sleep_Activate &= ~ACTIVATE_IGATE;
            StandByTick = millis() + (5000);

            if (config.igate_tlm_sensor[0] | config.igate_tlm_sensor[1] | config.igate_tlm_sensor[2] | config.igate_tlm_sensor[3] | config.igate_tlm_sensor[4])
            {
              char tlm_result[100];
              char tlm_data[200];
              size_t tlm_sz;
              if ((IGATE_TLM_SEQ % 100) == 0)
              {
                char rawInfo[100];
                char name[10];
                sprintf(rawInfo, "PARM.");
                int i, c = 0;
                for (i = 0; i < 5; i++)
                {
                  if (config.igate_tlm_sensor[i] == 0)
                  {
                    c++;
                    continue;
                  }
                  else
                  {
                    if (i > 0)
                      strcat(rawInfo, ",");
                    sprintf(name, "%s", config.igate_tlm_PARM[i]);
                    strcat(rawInfo, name);
                  }
                }
                for (int n = c + 8; n > 0; n--)
                {
                  strcat(rawInfo, ",");
                }
                sendTelemetry_igate(rawInfo);
                memset(rawInfo, 0, sizeof(rawInfo));
                sprintf(rawInfo, "UNIT.");
                c = 0;
                for (i = 0; i < 5; i++)
                {
                  if (config.igate_tlm_sensor[i] == 0)
                  {
                    c++;
                    continue;
                  }
                  else
                  {
                    if (i > 0)
                      strcat(rawInfo, ",");
                    sprintf(name, "%s", config.igate_tlm_UNIT[i]);
                    strcat(rawInfo, name);
                  }
                }
                for (int n = c + 8; n > 0; n--)
                {
                  strcat(rawInfo, ",");
                }
                sendTelemetry_igate(rawInfo);
                memset(rawInfo, 0, sizeof(rawInfo));
                sprintf(rawInfo, "EQNS.");
                c = 0;
                for (i = 0; i < 5; i++)
                {
                  if (config.igate_tlm_sensor[i] == 0)
                  {
                    c++;
                    continue;
                  }
                  else
                  {
                    if (i > 0)
                      strcat(rawInfo, ",");
                    if (fmod(config.igate_tlm_EQNS[i][0], 1) == 0)
                      sprintf(name, "%0.f", config.igate_tlm_EQNS[i][0]);
                    else
                      sprintf(name, "%.3f", config.igate_tlm_EQNS[i][0]);
                    strcat(rawInfo, name);
                    if (fmod(config.igate_tlm_EQNS[i][1], 1) == 0)
                      sprintf(name, ",%0.f", config.igate_tlm_EQNS[i][1]);
                    else
                      sprintf(name, ",%.3f", config.igate_tlm_EQNS[i][1]);
                    strcat(rawInfo, name);
                    if (fmod(config.igate_tlm_EQNS[i][2], 1) == 0)
                      sprintf(name, ",%0.f", config.igate_tlm_EQNS[i][2]);
                    else
                      sprintf(name, ",%.3f", config.igate_tlm_EQNS[i][2]);
                    strcat(rawInfo, name);
                  }
                }
                for (int n = c; n > 0; n--)
                {
                  strcat(rawInfo, ",");
                  sprintf(name, "0");
                  strcat(rawInfo, name);
                  sprintf(name, ",1");
                  strcat(rawInfo, name);
                  sprintf(name, ",0");
                  strcat(rawInfo, name);
                }
                // strcat(rawInfo, ",");
                sendTelemetry_igate(rawInfo);
              }

              if (++IGATE_TLM_SEQ > 8279)
                IGATE_TLM_SEQ = 0;
              memset(tlm_data, 0, 200);
              memset(tlm_result, 0, 100);
              int n = 0;
              sprintf(tlm_data, "%i", IGATE_TLM_SEQ);
              for (int s = 0; s < 5; s++)
              {
                if (config.igate_tlm_sensor[s] == 0)
                {
                  continue;
                  // strcat(tlm_data, "0");
                }
                else
                {
                  strcat(tlm_data, ",");
                  int sen_idx = config.igate_tlm_sensor[s] - 1;
                  double data = 0;
                  if (sen[sen_idx].visable)
                    data = sen[sen_idx].sample;
                  double precision = pow(10.0f, (double)config.igate_tlm_precision[s]);
                  int val = (int)((data + config.igate_tlm_offset[s]) * precision);
                  // log_d("s:%d Data:%.2f /tPresion:%.5f /tOffset:%.5f/t Val:%d",s,sen[sen_idx].sample,precision,config.trk_tlm_offset[s],val);
                  if (val > 8280)
                    val = 8280;
                  if (val < 0)
                    val = 0;
                  char strVal[10];
                  sprintf(strVal, "%i", val);
                  strcat(tlm_data, strVal);
                }
              }
              // log_d("TLM_DATA:%s",tlm_data);
              //  sprintf(tlm_data, "%i,%i,%i,%i", TLM_SEQ, (int)(VBat * 100), int(TempNTC * 100), gps.satellites.value());
              telemetry_base91(tlm_data, tlm_result, tlm_sz);
              rawData += String(tlm_result);
            }
            if (strlen(config.igate_comment) > 0)
            {
              rawData += String(config.igate_comment);
            }

            log_d("IGATE_POSITION: %s", rawData.c_str());

            if (config.igate_gps)
              sprintf(sts, "POSITION GPS\nINTERVAL %ds", tx_interval);
            else
              sprintf(sts, "POSITION FIX\nINTERVAL %ds", tx_interval);

            uint8_t SendMode = 0;
            if (config.igate_loc2rf)
              SendMode |= RF_CHANNEL;
            if (config.igate_loc2inet)
              SendMode |= INET_CHANNEL;
            pkgTxPush(rawData.c_str(), rawData.length(), 0, SendMode);
//                         if (config.igate_loc2rf)
//                         { // IGATE SEND POSITION TO RF
//                             char *rawP = (char *)calloc(rawData.length(), sizeof(char));
//                             // rawData.toCharArray(rawP, rawData.length());
//                             memcpy(rawP, rawData.c_str(), rawData.length());
//                             pkgTxPush(rawP, rawData.length(), 0);
#if defined OLED || defined ST7735_160x80
            pushTxDisp(TXCH_RF, "TX IGATE", sts);
#endif
            //                             free(rawP);
            //                         }
            //                         if (config.igate_loc2inet)
            //                         { // IGATE SEND TO APRS-IS
            //                             if (aprsClient.connected())
            //                             {
            //                                 status.txCount++;
            //                                 aprsClient.println(rawData); // Send packet to Inet
            // #if defined OLED || defined ST7735_160x80
            //                                 pushTxDisp(TXCH_TCP, "TX IGATE", sts);
            // #endif
            //                             }
            //                         }
          }
        }
      }
      // IGATE send to inet
      if (newIGatePkg)
      {
        newIGatePkg = false;
        // if (config.rf2inet && aprsClient.connected())
        if (config.rf2inet)
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
      if (config.digi_sts_interval > 10)
      {
        if (millis() > digiSTSInterval)
        {
          digiSTSInterval = millis() + (config.digi_sts_interval * 1000);
          digi_status(config.digi_status);
        }
      }
      // DIGI Position
      if (config.digi_bcn)
      {
        if (millis() > DiGiInterval)
        {

          String rawData;
          if (config.digi_gps)
          { // DIGI Send GPS position
            if (gps.location.isValid())
            {
              rawData = digi_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters(), "");
              if (config.log & LOG_DIGI)
              {
                logDigi(gps.location.lat(), gps.location.lng(), gps.speed.kmph(), gps.course.deg());
              }
            }
          }
          else
          { // DIGI Send fix position
            rawData = digi_position(config.digi_lat, config.digi_lon, config.digi_alt, "");
            if (config.log & LOG_DIGI)
            {
              logDigi(config.digi_lat, config.digi_lon, 0, 0);
            }
          }
          if (rawData != "")
          {
            DiGiInterval = millis() + (config.digi_interval * 1000);
            Sleep_Activate &= ~ACTIVATE_DIGI;
            StandByTick = millis() + (5000);

            if (config.digi_tlm_sensor[0] | config.digi_tlm_sensor[1] | config.digi_tlm_sensor[2] | config.digi_tlm_sensor[3] | config.digi_tlm_sensor[4])
            {
              char tlm_result[100];
              char tlm_data[200];
              size_t tlm_sz;
              if ((DIGI_TLM_SEQ % 100) == 0)
              {
                char rawInfo[100];
                char name[10];
                sprintf(rawInfo, "PARM.");
                int i, c = 0;
                for (i = 0; i < 5; i++)
                {
                  if (config.digi_tlm_sensor[i] == 0)
                  {
                    c++;
                    continue;
                  }
                  else
                  {
                    if (i > 0)
                      strcat(rawInfo, ",");
                    sprintf(name, "%s", config.digi_tlm_PARM[i]);
                    strcat(rawInfo, name);
                  }
                }
                for (int n = c + 8; n > 0; n--)
                {
                  strcat(rawInfo, ",");
                }
                sendTelemetry_digi(rawInfo);
                memset(rawInfo, 0, sizeof(rawInfo));
                sprintf(rawInfo, "UNIT.");
                c = 0;
                for (i = 0; i < 5; i++)
                {
                  if (config.digi_tlm_sensor[i] == 0)
                  {
                    c++;
                    continue;
                  }
                  else
                  {
                    if (i > 0)
                      strcat(rawInfo, ",");
                    sprintf(name, "%s", config.digi_tlm_UNIT[i]);
                    strcat(rawInfo, name);
                  }
                }
                for (int n = c + 8; n > 0; n--)
                {
                  strcat(rawInfo, ",");
                }
                sendTelemetry_digi(rawInfo);
                memset(rawInfo, 0, sizeof(rawInfo));
                sprintf(rawInfo, "EQNS.");
                c = 0;
                for (i = 0; i < 5; i++)
                {
                  if (config.digi_tlm_sensor[i] == 0)
                  {
                    c++;
                    continue;
                  }
                  else
                  {
                    if (i > 0)
                      strcat(rawInfo, ",");
                    if (fmod(config.digi_tlm_EQNS[i][0], 1) == 0)
                      sprintf(name, "%0.f", config.digi_tlm_EQNS[i][0]);
                    else
                      sprintf(name, "%.3f", config.digi_tlm_EQNS[i][0]);
                    strcat(rawInfo, name);
                    if (fmod(config.digi_tlm_EQNS[i][1], 1) == 0)
                      sprintf(name, ",%0.f", config.digi_tlm_EQNS[i][1]);
                    else
                      sprintf(name, ",%.3f", config.digi_tlm_EQNS[i][1]);
                    strcat(rawInfo, name);
                    if (fmod(config.digi_tlm_EQNS[i][2], 1) == 0)
                      sprintf(name, ",%0.f", config.digi_tlm_EQNS[i][2]);
                    else
                      sprintf(name, ",%.3f", config.digi_tlm_EQNS[i][2]);
                    strcat(rawInfo, name);
                  }
                }
                for (int n = c; n > 0; n--)
                {
                  strcat(rawInfo, ",");
                  sprintf(name, "0");
                  strcat(rawInfo, name);
                  sprintf(name, ",1");
                  strcat(rawInfo, name);
                  sprintf(name, ",0");
                  strcat(rawInfo, name);
                }
                // strcat(rawInfo, ",");
                sendTelemetry_digi(rawInfo);
              }

              if (++DIGI_TLM_SEQ > 8279)
                DIGI_TLM_SEQ = 0;
              memset(tlm_data, 0, 200);
              memset(tlm_result, 0, 100);
              int n = 0;
              sprintf(tlm_data, "%i", DIGI_TLM_SEQ);
              for (int s = 0; s < 5; s++)
              {
                if (config.digi_tlm_sensor[s] == 0)
                {
                  continue;
                  // strcat(tlm_data, "0");
                }
                else
                {
                  strcat(tlm_data, ",");
                  int sen_idx = config.digi_tlm_sensor[s] - 1;
                  double data = 0;
                  if (sen[sen_idx].visable)
                    data = sen[sen_idx].sample;
                  double precision = pow(10.0f, (double)config.digi_tlm_precision[s]);
                  int val = (int)((data + config.digi_tlm_offset[s]) * precision);
                  // log_d("s:%d Data:%.2f /tPresion:%.5f /tOffset:%.5f/t Val:%d",s,sen[sen_idx].sample,precision,config.trk_tlm_offset[s],val);
                  if (val > 8280)
                    val = 8280;
                  if (val < 0)
                    val = 0;
                  char strVal[10];
                  sprintf(strVal, "%i", val);
                  strcat(tlm_data, strVal);
                }
              }
              // log_d("TLM_DATA:%s",tlm_data);
              //  sprintf(tlm_data, "%i,%i,%i,%i", TLM_SEQ, (int)(VBat * 100), int(TempNTC * 100), gps.satellites.value());
              telemetry_base91(tlm_data, tlm_result, tlm_sz);
              rawData += String(tlm_result);
            }
            if (strlen(config.digi_comment) > 0)
            {
              rawData += String(config.digi_comment);
            }

            log_d("DIGI_POSITION: %s", rawData.c_str());

            if (config.digi_gps)
              sprintf(sts, "POSITION GPS\nINTERVAL %ds", tx_interval);
            else
              sprintf(sts, "POSITION FIX\nINTERVAL %ds", tx_interval);

            uint8_t SendMode = 0;
            if (config.digi_loc2rf)
              SendMode |= RF_CHANNEL;
            if (config.digi_loc2inet)
              SendMode |= INET_CHANNEL;
            pkgTxPush(rawData.c_str(), rawData.length(), 0, SendMode);
//                         if (config.digi_loc2rf)
//                         { // DIGI SEND POSITION TO RF
//                             char *rawP = (char *)calloc(rawData.length(), sizeof(char));
//                             // rawData.toCharArray(rawP, rawData.length());
//                             memcpy(rawP, rawData.c_str(), rawData.length());
//                             pkgTxPush(rawP, rawData.length(), 0);
#if defined OLED || defined ST7735_160x80
            pushTxDisp(TXCH_RF, "TX DIGI POS", sts);
#endif
            //                             free(rawP);
            //                         }
            //                         if (config.digi_loc2inet)
            //                         { // DIGI SEND TO APRS-IS
            //                             if (aprsClient.connected())
            //                             {
            //                                 status.txCount++;
            //                                 aprsClient.println(rawData); // Send packet to Inet
            // #if defined OLED || defined ST7735_160x80
            //                                 pushTxDisp(TXCH_TCP, "TX DIGI POS", sts);
            // #endif
            //                             }
            //                         }
          }
          rawData.clear();
        }
      }

      // Repeater packet
      if (newDigiPkg)
      {
        newDigiPkg = false;
        uint16_t type = pkgType((const char *)&incomingPacket.info[0]);
        Sleep_Activate &= ~ACTIVATE_DIGI;
        StandByTick = millis() + (config.pwr_stanby_delay * 1000);
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
            // char *rawP = (char *)calloc(digiPkg.length()+1, sizeof(char));
            //  digiPkg.toCharArray(rawP, digiPkg.length());
            // memcpy(rawP, digiPkg.c_str(), digiPkg.length());
            pkgTxPush(digiPkg.c_str(), digiPkg.length(), digiDelay, RF_CHANNEL);
            digiPkg.clear();
            // pkgTxPush(rawP, digiPkg.length(), digiDelay, RF_CHANNEL);
            sprintf(sts, "--src call--\n%s\nDelay: %dms.", incomingPacket.src.call, digiDelay);
#if defined OLED || defined ST7735_160x80
            pushTxDisp(TXCH_DIGI, "DIGI REPEAT", sts);
#endif
            // free(rawP);
          }
        }
      }
    }

    // Weather
    if (config.wx_en)
    {
      if (millis() > WxInterval)
      {

        String rawData = "";
        if (config.wx_gps)
        { // Wx Send GPS position
          if (gps.location.isValid())
          {
            rawData = wx_report(gps.location.lat(), gps.location.lng(), gps.altitude.meters(), "");
            if (config.log & LOG_WX)
            {
              logWeather(gps.location.lat(), gps.location.lng(), gps.speed.kmph(), gps.course.deg());
            }
          }
        }
        else
        { // Wx Send fix position
          rawData = wx_report(config.wx_lat, config.wx_lon, config.wx_alt, "");
          if (config.log & LOG_WX)
          {
            logWeather(config.wx_lat, config.wx_lon, 0, 0);
          }
        }
        if (rawData != "")
        {
          WxInterval = millis() + (config.wx_interval * 1000);
          Sleep_Activate &= ~ACTIVATE_WX;
          StandByTick = millis() + (5000);
          log_d("WX_REPORT: %s", rawData.c_str());
          uint8_t SendMode = 0;
          if (config.wx_2rf)
            SendMode |= RF_CHANNEL;
          if (config.wx_2inet)
            SendMode |= INET_CHANNEL;
          pkgTxPush(rawData.c_str(), rawData.length(), 0, SendMode);
//                     if (config.wx_2rf)
//                     { // WX SEND POSITION TO RF
//                         char *rawP = (char *)calloc(rawData.length(), sizeof(char));
//                         // rawData.toCharArray(rawP, rawData.length());
//                         memcpy(rawP, rawData.c_str(), rawData.length());
//                         pkgTxPush(rawP, rawData.length(), 0);
#ifdef OLED
          sprintf(sts, "--src call--\n%s\nDelay: %dms.", config.wx_mycall, (config.wx_interval * 1000));
          pushTxDisp(TXCH_RF, "WX REPORT", sts);
#endif
          //                         free(rawP);
          //                     }
          //                     if (config.wx_2inet)
          //                     { // WX SEND TO APRS-IS
          //                         if (aprsClient.connected())
          //                         {
          //                             status.txCount++;
          //                             aprsClient.println(rawData); // Send packet to Inet
          // #ifdef OLED
          //                             // pushTxDisp(TXCH_TCP, "WX REPORT", sts);
          // #endif
          //                         }
          //                     }
        }
        else
        {
          WxInterval = millis() + (10 * 1000);
        }
      }
    }

    if (config.tlm0_en)
    {
      if (systemTLM.ParmTimeout < millis())
      {
        systemTLM.ParmTimeout = millis() + (config.tlm0_info_interval * 1000);
        char rawInfo[100];
        char name[10];
        sprintf(rawInfo, "PARM.");
        for (int i = 0; i < 13; i++)
        {
          if (i > 0)
            strcat(rawInfo, ",");
          sprintf(name, "%s", config.tlm0_PARM[i]);
          strcat(rawInfo, name);
        }
        sendTelemetry_0(rawInfo, true);
        memset(rawInfo, 0, sizeof(rawInfo));
        sprintf(rawInfo, "UNIT.");
        for (int i = 0; i < 13; i++)
        {
          if (i > 0)
            strcat(rawInfo, ",");
          sprintf(name, "%s", config.tlm0_UNIT[i]);
          strcat(rawInfo, name);
        }
        sendTelemetry_0(rawInfo, true);
        memset(rawInfo, 0, sizeof(rawInfo));
        sprintf(rawInfo, "EQNS.");
        for (int i = 0; i < 5; i++)
        {
          if (i > 0)
            strcat(rawInfo, ",");
          if (fmod(config.tlm0_EQNS[i][0], 1) == 0)
            sprintf(name, "%0.f", config.tlm0_EQNS[i][0]);
          else
            sprintf(name, "%.3f", config.tlm0_EQNS[i][0]);
          strcat(rawInfo, name);
          if (fmod(config.tlm0_EQNS[i][1], 1) == 0)
            sprintf(name, ",%0.f", config.tlm0_EQNS[i][1]);
          else
            sprintf(name, ",%.3f", config.tlm0_EQNS[i][1]);
          strcat(rawInfo, name);
          if (fmod(config.tlm0_EQNS[i][2], 1) == 0)
            sprintf(name, ",%0.f", config.tlm0_EQNS[i][2]);
          else
            sprintf(name, ",%.3f", config.tlm0_EQNS[i][2]);
          strcat(rawInfo, name);
        }
        sendTelemetry_0(rawInfo, true);
        memset(rawInfo, 0, sizeof(rawInfo));
        sprintf(rawInfo, "BITS.");
        uint8_t b = 1;
        for (int i = 0; i < 8; i++)
        {
          if (config.tlm0_BITS_Active & b)
          {
            strcat(rawInfo, "1");
          }
          else
          {
            strcat(rawInfo, "0");
          }
          b <<= 1;
        }
        strcat(rawInfo, ",");
        strcat(rawInfo, config.tlm0_comment);
        sendTelemetry_0(rawInfo, true);
      }

      if (systemTLM.TeleTimeout < millis())
      {
        systemTLM.TeleTimeout = millis() + (config.tlm0_data_interval * 1000);
        char rawTlm[100];
        if (systemTLM.Sequence > 999)
          systemTLM.Sequence = 0;
        else
          systemTLM.Sequence++;
        getTelemetry_0();
        sprintf(rawTlm, "T#%03d,%03d,%03d,%03d,%03d,%03d,", systemTLM.Sequence, systemTLM.A1, systemTLM.A2, systemTLM.A3, systemTLM.A4, systemTLM.A5);
        uint8_t b = 1;
        for (int i = 0; i < 8; i++)
        {
          if (!((systemTLM.BITS & b) ^ (config.tlm0_BITS_Active & b)))
          {
            strcat(rawTlm, "1");
          }
          else
          {
            strcat(rawTlm, "0");
          }
          b <<= 1;
        }
        sendTelemetry_0(rawTlm, false);
      }
    }
  }
}

void taskAPRSPoll(void *pvParameters)
{
  for (;;)
  {
    if (config.modem_type == 3)
      vTaskDelay(2 / portTICK_PERIOD_MS);
    else
      vTaskDelay(5 / portTICK_PERIOD_MS);

    if (AFSKInitAct == true)
    {
      AFSK_Poll(false, LOW);
    }
  }
}

int mqttRetry = 0;
long wifiTTL = 0;
WiFiMulti wifiMulti;

// WiFi connect timeout per AP. Increase when connecting takes longer.
const uint32_t connectTimeoutMs = 10000;
uint8_t APStationNum = 0;

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
    log_d("Access point running. IP address: ");
    log_d("%s",WiFi.softAPIP().toString().c_str());
    // webService();
  }

  if (wifiMulti.run() == WL_CONNECTED)
  {
    log_d("Wi-Fi CONNECTED!");
    log_d("IP address: %s", WiFi.localIP().toString().c_str());
    webService();
    NTP_Timeout = millis() + 2000;
  }

  pingTimeout = millis() + 10000;
  unsigned long timeNetworkOld = millis();
  timeNetwork = 0;
  if (config.wifi_mode & WIFI_AP_STA_FIX)
    webService();
  for (;;)
  {
    unsigned long now = millis();
    timeNetwork = now - timeNetworkOld;
    timeNetworkOld = now;
    // wdtNetworkTimer = millis();
    serviceHandle();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    if (config.wifi_mode & WIFI_AP_FIX)
    {
      APStationNum = WiFi.softAPgetStationNum();
      if (APStationNum > 0)
      {
        if (WiFi.isConnected() == false)
        {
          vTaskDelay(9 / portTICK_PERIOD_MS);
          continue;
        }
      }
    }

    // if (WiFi.status() == WL_CONNECTED)
    if (wifiMulti.run(connectTimeoutMs) == WL_CONNECTED)
    {
      // webService();
      // serviceHandle();

      if (millis() > NTP_Timeout)
      {
        NTP_Timeout = millis() + 86400000;
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
            status.isCount++;
            int start_val = line.indexOf(">", 0); // หาตำแหน่งแรกของ >
            if (start_val > 3)
            {
              // if (config.dispINET == true){
              //     if(!dispBuffer.isFull()) dispBuffer.push(line.c_str());
              // }
              // #ifdef BOARD_HAS_PSRAM
              //               // char *raw = (char *)malloc(line.length() + 1);
              // #else
              //               char *raw = (char *)malloc(line.length() + 1);
              // #endif
              String src_call = line.substring(0, start_val);
              String msg_call = "::" + src_call;

              status.allCount++;
              igateTLM.RX++;

              log_d("INET: %s\n", line.c_str());
              // char raw[500];
              // memset(&raw[0], 0, sizeof(raw));
              start_val = line.indexOf(":", 10); // Search of info in ax25
              if (start_val > 5)
              {
                String info = line.substring(start_val + 1);
                // info.toCharArray(&raw[0], info.length(), 0);
                size_t rawSize = line.length();
                char *raw = (char *)calloc(rawSize, sizeof(char));
                if (raw)
                {
                  memset(raw, 0, rawSize);
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
                    raw[line.length() - 1] = 0;
                    // int idx = pkgListUpdate(call, raw, type, 1, 0);
                    // int cnt = 0;
                    // if (idx > -1)
                    // {
                    //   // Put queue affter filter for display popup
                    //   if (config.rx_display && config.dispINET && (type & config.dispFilter))
                    //   {
                    //     cnt = pushTNC2Raw(idx);
                    //     log_d("INET_putQueueDisp:[pkgList_idx=%d/queue=%d,Type=%d] %s\n", idx, cnt, type, call);
                    //   }
                    // }
                    if (type & config.dispFilter)
                    {
                      int idx = pkgListUpdate(call, raw, type, 1, 0);
                      int cnt = 0;
                      if (idx > -1)
                      {
                        // Put queue affter filter for display popup
                        if (config.rx_display && config.dispINET)
                        {
                          cnt = pushTNC2Raw(idx);
                          log_d("INET_putQueueDisp:[pkgList_idx=%d/queue=%d,Type=%d] %s\n", idx, cnt, type, call);
                        }
                      }
                    }

                    // INET2RF affter filter
                    if (config.inet2rf)
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
                        pkgTxPush(tnc2Raw.c_str(), tnc2Raw.length(), 0, RF_CHANNEL);
                        char sts[50];
                        sprintf(sts, "--SRC CALL--\n%s\n", src_call.c_str());
                        pushTxDisp(TXCH_3PTY, "TX INET->RF", sts);
                        status.inet2rf++;
                        igateTLM.INET2RF++;
                        log_d("INET2RF: %s\n", line);
                      }
                    }
                  }
                  free(raw);
                }
              }
            }
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
    }
    else if (config.wifi_mode & WIFI_AP_FIX)
    { // WiFi connected
      serviceHandle();
    }
  } // for loop
}