/*
 Name:		ESP32APRS T-TWR Plus
 Created:	13-10-2023 14:27:23
 Author:	HS5TQA/Atten
 Github:	https://github.com/nakhonthai
 Facebook:	https://www.facebook.com/atten
 Support IS: host:aprs.dprns.com port:14580 or aprs.hs5tqa.ampr.org:14580
 Support IS monitor: http://aprs.dprns.com:14501 or http://aprs.hs5tqa.ampr.org:14501
*/

#ifndef MAIN_H
#define MAIN_H

#define VERSION "0.5"
#define VERSION_BUILD 'a'

// #define DEBUG

#define SA868_TX_PIN (39)
#define SA868_RX_PIN (48)
#define SA868_PTT_PIN (41)
#define SA868_PD_PIN (40)
#define SA868_PWR_PIN (38)
#define SA868_MIC_SEL (17)
#define SA868_MIC (18)
#define SA868_AU (1)

#define MIC_CTRL_PIN (17)

#define BUTTON_PTT_PIN (3)
#define BUTTON_DOWN_PIN (0)

#define ENCODER_A_PIN (47)
#define ENCODER_B_PIN (46)
#define ENCODER_OK_PIN (21)

#define BATTERY_ADC_PIN (-1)
#define OLED_POWER_PIN (-1)
#define I2C_SDA_SYS (8)
#define I2C_SCL_SYS (9)
#define PMU_IRQ (4)

#define SPI_MOSI (11)
#define SPI_MISO (13)
#define SPI_SCK (12)
#define SD_CS (10)
#define USER_CS (14)

#define GNSS_TX (6)
#define GNSS_RX (5)
#define GNSS_PPS (7)

#define PIXELS_PIN (42)

#define ESP2SA868_MIC (18)
#define SA8682ESP_AUDIO (1)

#define BLUETOOTH

#define OLED
#define SDCARD

#define WIFI_OFF_FIX 0
#define WIFI_AP_FIX 1
#define WIFI_STA_FIX 2
#define WIFI_AP_STA_FIX 3

#define IMPLEMENTATION FIFO

#define TZ 7	 // (utc+) TZ in hours
#define DST_MN 0 // use 60mn for summer time in some countries
#define TZ_MN ((TZ) * 60)
#define TZ_SEC ((TZ) * 3600)
#define DST_SEC ((DST_MN) * 60)

#define FORMAT_SPIFFS_IF_FAILED true

#ifdef BOARD_HAS_PSRAM
#define TLMLISTSIZE 10
#define PKGLISTSIZE 100
#define PKGTXSIZE 100
#else
#define TLMLISTSIZE 10
#define PKGLISTSIZE 10
#define PKGTXSIZE 10
#endif

#define LOG_NONE 0
#define LOG_TRACKER (1 << 0)
#define LOG_IGATE (1 << 1)
#define LOG_DIGI (1 << 2)
#define LOG_WX (1 << 3)
#define LOG_STATUS (1 << 4)

#define FILTER_ALL 0				// Packet is disable all packet
#define FILTER_OBJECT (1 << 0)		// packet is an object
#define FILTER_ITEM (1 << 1)		// packet is an item
#define FILTER_MESSAGE (1 << 2)		// packet is a message
#define FILTER_WX (1 << 3)			// packet is WX data
#define FILTER_TELEMETRY (1 << 4)	// packet is telemetry
#define FILTER_QUERY (1 << 5)		// packet is a query
#define FILTER_STATUS (1 << 6)		// packet is status
#define FILTER_POSITION (1 << 7)	// packet is postion
#define FILTER_BUOY (1 << 8)		// packet is buoy
#define FILTER_MICE (1 << 9)		// packet is MIC-E
#define FILTER_THIRDPARTY (1 << 10) // packet is 3rd-party packet from INET2RF

#define RF_NONE 0
#define RF_SA868_VHF 1 // G-NiceRF SA818,SA868 VHF band 134~174 MHz
#define RF_SA868_UHF 2 // G-NiceRF SA818,SA868 UHF band 400~470 MHz
#define RF_SA868_350 3 // G-NiceRF SA818,SA868 350 band frequency：320-400MHz
#define RF_SR_1WV 4	   // SUNRISE SR110V,FRS-1WV VHF band 136~174 MHz
#define RF_SR_1WU 5	   // SUNRISE SR110U,FRS-1WU UHF band 400~470 MHz
#define RF_SR_1W350 6  // SUNRISE SR350P 350 band frequency：350-390MHz
#define RF_SR_2WVS 7   // SUNRISE SR120V,SR_2WVS VHF band 136~174 MHz
#define RF_SR_2WUS 8   // SUNRISE SR120U,SR_2WUS UHF band 400~470 MHz
#define RF_SA8x8_OpenEdit 9

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */

#define TXCH_TCP 0
#define TXCH_RF 1
#define TXCH_DIGI 2
#define TXCH_3PTY 3

#define RF_CHANNEL	(1<<0)
#define INET_CHANNEL	(1<<1)
#define TNC_CHANNEL	(1<<2)

#define XPOWERS_CHIP_AXP2101

#include <Arduino.h>
#include "config.h"
#include <FS.h>
#include <SD.h>
#include <SPIFFS.h>
#include <AX25.h>

#include "HardwareSerial.h"
#include "EEPROM.h"


typedef struct igateTLM_struct
{
	uint16_t Sequence;
	unsigned long ParmTimeout;
	unsigned long TeleTimeout;
	uint8_t RF2INET;
	uint8_t INET2RF;
	uint8_t RX;
	uint8_t TX;
	uint8_t DROP;
} igateTLMType;

typedef struct
{
	time_t time;
	char calsign[11];
	char object[10];
	char ssid[5];
	bool channel;
	unsigned int pkg;
	uint16_t type;
	uint8_t symbol;
	int16_t audio_level;
	float rssi;
	float snr;
	float freqErr;
	char *raw;
	size_t length;
	//char raw[256];
} pkgListType;

typedef struct statisticStruct
{
	uint32_t allCount;
	uint32_t tncCount;
	uint32_t isCount;
	uint32_t locationCount;
	uint32_t wxCount;
	uint32_t digiCount;
	uint32_t errorCount;
	uint32_t dropCount;
	uint32_t rf2inet;
	uint32_t inet2rf;
	uint32_t txCount;
	uint32_t rxCount;
} statusType;

typedef struct digiTLM_struct
{
	unsigned int Sequence;
	unsigned int ParmTimeout;
	unsigned int TeleTimeout;
	unsigned char RxPkts;
	unsigned char TxPkts;
	unsigned char DropRx;
	unsigned char ErPkts;
} digiTLMType;

typedef struct dataTLM_struct
{
	unsigned int Sequence;
	unsigned long ParmTimeout;
	unsigned long TeleTimeout;
	uint8_t A1;
	uint8_t A2;
	uint8_t A3;
	uint8_t A4;
	uint8_t A5;
	uint8_t BITS;
} dataTLMType;

typedef struct Telemetry_struct
{
	time_t time;
	char callsign[10];
	char PARM[5][10];
	char UNIT[5][10];
	float VAL[5];
	float RAW[5];
	float EQNS[15];
	uint8_t BITS;
	uint8_t BITS_FLAG;
	bool EQNS_FLAG;
} TelemetryType;

typedef struct txQueue_struct
{
	bool Active;
	uint8_t Channel;
	long timeStamp;
	int Delay;
	size_t length;
	char Info[500];
} txQueueType;

const char PARM[] = {"PARM.RF->INET,INET->RF,DigiRpt,TX2RF,DropRx"};
const char UNIT[] = {"UNIT.Pkts,Pkts,Pkts,Pkts,Pkts"};
const char EQNS[] = {"EQNS.0,1,0,0,1,0,0,1,0,0,1,0,0,1,0"};

const float ctcss[] = {0, 67, 71.9, 74.4, 77, 79.7, 82.5, 85.4, 88.5, 91.5, 94.8, 97.4, 100, 103.5, 107.2, 110.9, 114.8, 118.8, 123, 127.3, 131.8, 136.5, 141.3, 146.2, 151.4, 156.7, 162.2, 167.9, 173.8, 179.9, 186.2, 192.8, 203.5, 210.7, 218.1, 225.7, 233.6, 241.8, 250.3};
const float wifiPwr[12][2] = {{-4, -1}, {8, 2}, {20, 5}, {28, 7}, {34, 8.5}, {44, 11}, {52, 13}, {60, 15}, {68, 17}, {74, 18.5}, {76, 19}, {78, 19.5}};
const char RF_TYPE[10][11] = {"NONE", "SA868_VHF", "SA868_UHF", "SA868_350", "SR110V_VHF", "SR110U_UHF", "SR350P", "SR120V_VHF", "SR120U_UHF","SA8x8_Open"};
const unsigned long baudrate[] = {2400, 4800, 9600, 19200, 2880, 38400, 57600, 76800, 115200, 230400, 460800, 576000, 921600};
const char GNSS_PORT[5][6] = {"NONE", "UART0", "UART1", "UART2", "TCP"};
const char TNC_PORT[4][6] = {"NONE", "UART0", "UART1", "USB"};
const char TNC_MODE[4][6] = {"NONE", "KISS", "TNC2", "YAESU"};
const char WX_PORT[7][11] = {"NONE", "UART0_CSV", "UART1_CSV", "UART2_CSV", "MODBUS","SENSOR","TCP/UDP"};
const char MODEM_TYPE[4][17] = {"AFSK_300", "AFSK_1200","AFSK_1200v23","GFSK9600(G3RUH)"};
const char FX25_MODE[3][6] = {"NONE","RX","RX+TX"};
const char PWR_MODE[3][10] = {"MODE A", "MODE B","MODE C"};
const char WX_SENSOR[23][19]={"Wind Course","Wind Speed","Wind Gust","Temperature","Rain 1hr","Rain 24hr","Rain Midnight","Humidity","Barometric","Luminosity","Snow","Soil Temperature","Soil Humidity","Water Temperature","Water TDS","Water Level","PM 2.5","PM 10","Co2","CH2O","TVOC","UV","SOUND"};

uint8_t checkSum(uint8_t *ptr, size_t count);
void saveEEPROM();
void defaultConfig();
String getValue(String data, char separator, int index);
boolean isValidNumber(String str);
void taskSerial(void *pvParameters);
void taskGPS(void *pvParameters);
void taskAPRS(void *pvParameters);
void taskAPRSPoll(void *pvParameters);
void taskNetwork(void *pvParameters);
void taskTNC(void *pvParameters);
void sort(pkgListType a[], int size);
void sortPkgDesc(pkgListType a[], int size);
//int processPacket(String &tnc2);
int digiProcess(AX25Msg &Packet);
void printTime();
bool pkgTxPush(const char *info, size_t len, int dly, uint8_t Ch);
int popTNC2Raw(int &ret);
int pushTNC2Raw(int raw);
int pkgListUpdate(char *call, char *raw, uint16_t type, bool channel, uint16_t audioLvl);
pkgListType getPkgList(int idx);
//String myBeacon(String Path);
int tlmList_Find(char *call);
int tlmListOld();
TelemetryType getTlmList(int idx);
void powerSave();
void powerWakeup();
bool powerStatus();
int packet2Raw(String &tnc2, AX25Msg &Packet);
bool SA868_waitResponse(String &data, String rsp, uint32_t timeout);
//String sendIsAckMsg(String toCallSign, char *msgId);
String trk_gps_postion(String comment);
String trk_fix_position(String comment);
String getPath(int idx);
bool waitPSRAM(bool state);
void convertSecondsToDHMS(char *dmhs,unsigned long totalSeconds);
#endif