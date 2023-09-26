#ifndef GUI_LCD_H
#define GUI_LCD_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "main.h"
#include <parse_aprs.h>
#include <TimeLib.h>
#include "Time.h"
#include <TinyGPSPlus.h>
#include <pbuf.h>
#include "parse_aprs.h"
#include "MenuSystem.h"
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSerifItalic9pt7b.h>
#include <Fonts/Seven_Segment24pt7b.h>

#include <Wire.h>
#include "Adafruit_SSD1306.h"
#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>

#include "XPowersLib.h"

#include "cppQueue.h"

#define keyA ENCODER_B_PIN
#define keyB ENCODER_A_PIN
#define keyPush ENCODER_OK_PIN

#define CON_WIFI 0
#define CON_SERVER 1
#define CON_WEB 2
#define CON_NORMAL 3
#define CON_MENU 4
#define CON_RF 5

//#define PKGLISTSIZE 50

#define PKG_ALL		 0 // Packet is of position type
#define PKG_OBJECT     1 // packet is an object
#define PKG_ITEM       2 // packet is an item
#define PKG_MESSAGE    3 // packet is a message
#define PKG_WX         4 // packet is WX data
#define PKG_TELEMETRY  5 // packet is telemetry
#define PKG_QUERY      6 // packet is a query
#define PKG_STATUS     7 // packet is status 

const char APRS_PATH[9][16] ={"","WIDE1-1","WIDE1-1,WIDE2-1","TRACK3-3","RS0ISS","YBOX-1","W3ADO-1","BJ1SI","PSAT2-1"};

const unsigned char LOGO[] PROGMEM=
{
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80,
0xC0, 0x00, 0x00, 0xC0, 0xC0, 0xE0, 0xE0, 0xF0, 0xF8, 0xF8,
0xFC, 0xFC, 0xFE, 0x7E, 0x7F, 0x7C, 0x70, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xE0, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF8, 0xF8, 0xF8, 0xFC,
0x7C, 0x3C, 0x3C, 0x18, 0x83, 0x9F, 0x9F, 0x9F, 0x8F, 0x8F,
0xCC, 0x41, 0x63, 0x13, 0x01, 0x81, 0xE1, 0x21, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
0x07, 0x07, 0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
0x08, 0x08, 0x08, 0x09, 0x19, 0x19, 0x09, 0x08, 0x08, 0x0C,
0x04, 0x86, 0x83, 0x81, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
0x80, 0x80, 0x80, 0x8C, 0x80, 0x84, 0xC2, 0x80, 0x07, 0x08,
0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xF8, 0x3E, 0x43, 0x7D,
0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E,
0x7E, 0x7E, 0x7E, 0x7E, 0x79, 0x07, 0x7C, 0xF0, 0xC0, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xE7, 0xC3, 0xC3, 0xC3, 0xFF,
0xFF, 0xFF, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0xFF, 0xFF, 0xFF,
0xE7, 0xC3, 0xC3, 0xC3, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x03, 0x1F, 0x3F, 0x3F, 0x3F, 0x1F, 0x03, 0x03, 0x03, 0x03,
0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x1F,
0x3F, 0x3F, 0x3F, 0x1F, 0x03, 0x00, 0x00, 0x00
};

//Create icon from https://rickkas7.github.io/DisplayGenerator
const uint8_t bluetooth_icon[] = {0x10, 0x5c, 0x38, 0x10, 0x38, 0x5c, 0x10};

extern Adafruit_SSD1306 display;
extern cppQueue dispBuffer;
extern int conStat;
extern int conStatNetwork;
extern int raw_count;
//extern pkgListType pkgList[PKGLISTSIZE];
extern pkgListType *pkgList;
//extern TelemetryType Telemetry[TLMLISTSIZE];
extern TelemetryType *Telemetry;
extern Configuration config;
extern int16_t SB_HEADING;
extern unsigned char SB_SPEED;
extern TinyGPSPlus gps;
extern WiFiClient aprsClient;
extern uint16_t tx_interval;		// How often we transmit, in seconds
extern unsigned int tx_counter;		// Incremented every second
extern char send_aprs_table;
extern char send_aprs_symbol;
extern ParseAPRS aprsParse;
extern float vbat;

extern XPowersAXP2101 PMU;

extern uint8_t dispFlagTX;


void mainDisp(void *pvParameters);
void dispWindow(String line, uint8_t mode, bool filter);
void msgBox(String msg);
void topBar(int ws);
void compass_label(signed int startx, signed int starty, unsigned int length, double angle, unsigned int color);
void compass_arrow(signed int startx, signed int starty, unsigned int length, double angle, unsigned int color);
void dispTX(bool port);

// const char *str_status[] = {
// 	"IDLE_STATUS",
// 	"NO_SSID_AVAIL",
// 	"SCAN_COMPLETED",
// 	"CONNECTED",
// 	"CONNECT_FAILED",
// 	"CONNECTION_LOST",
// 	"DISCONNECTED"
// };

#define ALL 0
#define NUMBER 1
#define ALPHA 2
class MyTextBox
{
private:
	int curr_cursor;
public:
	//struct textboxType {
	char text[50];
	int x;
	int y;
	int length;
	bool isSelect;
	char type=0; //0:All,1:Number,2:Alpha Upper
	char char_max=0x7F;
	char char_min=0x20;

	void TextBox();	
	void TextBoxShow();
};

class MyCheckBox
{
public:
	//struct textboxType {
	char text[20];
	int x;
	int y;
	int length;
	bool isSelect;
	bool Checked;

	void Toggle();

	void CheckBoxShow();
};

class MyButtonBox
{
public:
	//struct textboxType {
	char text[30];
	int x;
	int y;
	int length;
	bool isSelect;
	bool Checked;
	bool Border;

	void Toggle();
	void Show();	
};

class MyComboBox
{
private:
	unsigned char current_index = 0;
	long current=0;
public:
	//struct textboxType {
	char text[30];
	char item[10][30];
	int x;
	int y;
	unsigned int length;
	bool isSelect;
	bool isValue = false;
	char type = 0; //0:All,1:Number,2:Alpha Upper,3:Calculator
	unsigned int char_max = 9;
	unsigned int char_min = 0;

	void SelectValue(long val_min, long val_max, long step);
	void AddItem(int index, char* str);
	void AddItem(int index, const char* str);
	void GetItem(int index, char* str);
	void maxItem(unsigned char index);
	unsigned long GetValue();
	unsigned char GetIndex();
	void SetIndex(unsigned int i);
	void SelectItem();
	void Show();
};

class MySymbolBox
{
private:
    unsigned char current_index = 0;

public:
    // struct textboxType {
    String title;
    char tableMode = 0;
    char table;
    char symbol;
    char text[30];
    char item[2][32];
    int x;
    int y;
    unsigned int length;
    bool isSelect;
    bool onSelect = false;
    char type = 0; // 0:All,1:Number,2:Alpha Upper,3:Calculator
    unsigned char char_max = 0x80;
    unsigned char char_min = 0x21;

    unsigned char GetTable();
    unsigned char GetSymbol();
    unsigned char GetIndex();
    void SetIndex(unsigned char i);
    void SelectItem();
    void Show();
};

#endif