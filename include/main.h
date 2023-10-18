/*
 Name:		ESP32APRS T-TWR Plus
 Created:	13-10-2023 14:27:23
 Author:	HS5TQA/Atten
 Support IS: host:aprs.dprns.com port:14580 or aprs.hs5tqa.ampr.org:14580
 Support IS monitor: http://aprs.dprns.com:14501 or http://aprs.hs5tqa.ampr.org:14501
*/

#ifndef MAIN_H
#define MAIN_H

#define VERSION "0.1"
#define VERSION_BUILD 'b'

// #define DEBUG
// #define DEBUG_IS

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
#define I2C_SDA (8)
#define I2C_SCL (9)
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

// const int timeZone = 7; // Bangkok

#define RF_NONE 0
#define RF_SA868_VHF 1 // G-NiceRF SA818,SA868 VHF band 134~174 MHz
#define RF_SA868_UHF 2 // G-NiceRF SA818,SA868 UHF band 400~470 MHz
#define RF_SA868_350 3 // G-NiceRF SA818,SA868 350 band frequency：320-400MHz
#define RF_SR_1WV 4	   // SUNRISE SR110V,FRS-1WV VHF band 136~174 MHz
#define RF_SR_1WU 5	   // SUNRISE SR110U,FRS-1WU UHF band 400~470 MHz
#define RF_SR_1W350 6  // SUNRISE SR350P 350 band frequency：350-390MHz
#define RF_SR_2WVS 7   // SUNRISE SR120V,SR_2WVS VHF band 136~174 MHz
#define RF_SR_2WUS 8   // SUNRISE SR120U,SR_2WUS UHF band 400~470 MHz

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPIFFS.h>
// #include "soc/rtc_wdt.h"
#include <AX25.h>

#include "HardwareSerial.h"
#include "EEPROM.h"

enum M17Flags
{
	DISCONNECTED = 1 << 0,
	CONNECTING = 1 << 1,
	M17_AUTH = 1 << 2,
	M17_CONF = 1 << 3,
	M17_OPTS = 1 << 4,
	CONNECTED_RW = 1 << 5,
	CONNECTED_RO = 1 << 6
};

typedef struct Config_Struct
{
	float timeZone;
	bool synctime;
	bool title;

	// WiFi/BT/RF
	char wifi_mode; // WIFI_AP,WIFI_STA,WIFI_AP_STA,WIFI_OFF
	char wifi_power;
	//--WiFi Client
	// bool wifi_client;
	char wifi_ssid[32];
	char wifi_pass[63];
	//--WiFi AP
	// bool wifi_ap;
	char wifi_ap_ch;
	char wifi_ap_ssid[32];
	char wifi_ap_pass[63];

	//--Blue Tooth
	bool bt_slave;
	bool bt_master;
	char bt_mode;
	char bt_uuid[37];
	char bt_uuid_rx[37];
	char bt_uuid_tx[37];
	char bt_name[20];
	char bt_pin[5];
	char bt_power;

	//--RF Module
	bool rf_en;
	uint8_t rf_type;
	float freq_rx;
	float freq_tx;
	int offset_rx;
	int offset_tx;
	int tone_rx;
	int tone_tx;
	uint8_t band;
	uint8_t sql_level;
	bool rf_power;
	uint8_t volume;
	uint8_t mic;

	// IGATE
	bool igate_en;
	bool rf2inet;
	bool inet2rf;
	bool igate_loc2rf;
	bool igate_loc2inet;
	uint16_t rf2inetFilter;
	uint16_t inet2rfFilter;
	//--APRS-IS
	uint8_t aprs_ssid;
	uint16_t aprs_port;
	char aprs_mycall[10];
	char aprs_host[20];
	char aprs_passcode[6];
	char aprs_moniCall[10];
	char aprs_filter[30];
	//--Position
	bool igate_bcn;
	bool igate_gps;
	float igate_lat;
	float igate_lon;
	float igate_alt;
	uint16_t igate_interval;
	char igate_symbol[3] = "N&";
	char igate_object[10];
	char igate_phg[8];
	char igate_path[72];
	char igate_comment[50];
	//--Filter

	// DIGI REPEATER
	bool digi_en;
	bool digi_loc2rf;
	bool digi_loc2inet;
	uint8_t digi_ssid;
	char digi_mycall[10];
	char digi_path[72];
	uint16_t digi_delay; // ms
	uint16_t digiFilter;
	//--Position
	bool digi_bcn;
	bool digi_compress = false;
	bool digi_altitude = false;
	bool digi_gps;
	float digi_lat;
	float digi_lon;
	float digi_alt;
	uint16_t digi_interval;
	char digi_symbol[3] = "N&";
	char digi_phg[8];
	char digi_comment[50];

	// TRACKER
	bool trk_en;
	bool trk_loc2rf;
	bool trk_loc2inet;
	uint8_t trk_ssid;
	char trk_mycall[10];
	char trk_path[72];
	//--Position
	bool trk_gps;
	float trk_lat;
	float trk_lon;
	float trk_alt;
	uint16_t trk_interval = 60;
	bool trk_smartbeacon = false;
	bool trk_compress = false;
	bool trk_altitude = false;
	bool trk_cst = false;
	bool trk_bat = false;
	bool trk_sat = false;
	bool trk_dx = false;
	uint16_t trk_hspeed = 120;
	uint8_t trk_lspeed = 2;
	uint8_t trk_maxinterval = 15;
	uint8_t trk_mininterval = 5;
	uint8_t trk_minangle = 25;
	uint16_t trk_slowinterval = 600;
	char trk_symbol[3] = "\\>";
	char trk_symmove[3] = "/>";
	char trk_symstop[3] = "\\>";
	// char trk_btext[17] = "";
	char trk_comment[50];
	char trk_item[10] = "";
	// char trk_object[10];
	//--Filter

	// OLED DISPLAY
	bool oled_enable;
	int oled_timeout;
	unsigned char dim;
	unsigned char contrast;
	unsigned char startup;

	// Display
	unsigned int dispDelay;
	unsigned int filterDistant;
	bool h_up = true;
	bool tx_display = true;
	bool rx_display = true;
	uint16_t dispFilter;
	bool dispRF;
	bool dispINET;

	// AFSK,TNC
	bool audio_hpf;
	bool audio_bpf;
	uint8_t preamble;
	uint16_t tx_timeslot;
	char ntp_host[20];

	// VPN wiregurad
	bool vpn;
	bool modem;
	uint16_t wg_port;
	char wg_peer_address[16];
	char wg_local_address[16];
	char wg_netmask_address[16];
	char wg_gw_address[16];
	char wg_public_key[45];
	char wg_private_key[45];

	char path[4][15];

	uint8_t gpio_sql_pin = -1;

} Configuration;

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
	char ssid[5];
	unsigned int pkg;
	uint16_t type;
	uint8_t symbol;
	int16_t audio_level;
	char raw[500];
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
	long timeStamp;
	int Delay;
	char Info[500];
} txQueueType;

const char PARM[] = {"PARM.RF->INET,INET->RF,TxPkts,RxPkts,IGateDropRx"};
const char UNIT[] = {"UNIT.Pkts,Pkts,Pkts,Pkts,Pkts"};
const char EQNS[] = {"EQNS.0,1,0,0,1,0,0,1,0,0,1,0,0,1,0"};

const float ctcss[] = {0, 67, 71.9, 74.4, 77, 79.7, 82.5, 85.4, 88.5, 91.5, 94.8, 97.4, 100, 103.5, 107.2, 110.9, 114.8, 118.8, 123, 127.3, 131.8, 136.5, 141.3, 146.2, 151.4, 156.7, 162.2, 167.9, 173.8, 179.9, 186.2, 192.8, 203.5, 210.7, 218.1, 225.7, 233.6, 241.8, 250.3};
const float wifiPwr[12][2] = {{-4, -1}, {8, 2}, {20, 5}, {28, 7}, {34, 8.5}, {44, 11}, {52, 13}, {60, 15}, {68, 17}, {74, 18.5}, {76, 19}, {78, 19.5}};

const char RF_TYPE[9][11] = {"NONE", "SA868_VHF", "SA868_UHF", "SA868_350", "SR110V_VHF", "SR110U_UHF", "SR350P", "SR120V_VHF", "SR120U_UHF"};

uint8_t checkSum(uint8_t *ptr, size_t count);
void saveEEPROM();
void defaultConfig();
String getValue(String data, char separator, int index);
boolean isValidNumber(String str);
void taskGPS(void *pvParameters);
void taskAPRS(void *pvParameters);
void taskNetwork(void *pvParameters);
void taskTNC(void *pvParameters);
void sort(pkgListType a[], int size);
void sortPkgDesc(pkgListType a[], int size);
int processPacket(String &tnc2);
// String send_fix_location();
int digiProcess(AX25Msg &Packet);
void printTime();
bool pkgTxPush(const char *info, size_t len, int dly);
int popTNC2Raw(int &ret);
int pushTNC2Raw(int raw);
int pkgListUpdate(char *call, char *raw, uint16_t type);
pkgListType getPkgList(int idx);
String myBeacon(String Path);
int tlmList_Find(char *call);
int tlmListOld();
TelemetryType getTlmList(int idx);
void powerSave();
void powerWakeup();
bool powerStatus();
int packet2Raw(String &tnc2, AX25Msg &Packet);
bool waitResponse(String &data, String rsp = "\r\n", uint32_t timeout = 1000);
#endif