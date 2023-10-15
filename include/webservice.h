#include <Update.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <time.h>
#include <TimeLib.h>
#include <TinyGPSPlus.h>

#include "main.h"

// ใช้ตัวแปรโกลบอลในไฟล์ main.cpp
extern statusType status;
extern digiTLMType digiTLM;
extern Configuration config;
extern TaskHandle_t taskNetworkHandle;
extern TaskHandle_t taskAPRSHandle;
extern TaskHandle_t taskTNCHandle;
extern TaskHandle_t taskGpsHandle;
extern time_t systemUptime;
extern String RF_VERSION;
extern pkgListType *pkgList;
extern TinyGPSPlus gps;
extern float vbat;
extern WiFiClient aprsClient;

#ifdef __cplusplus
extern "C"
{
#endif
	uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();

void serviceHandle();
void setHTML(byte page);
void handle_root();
void handle_setting();
void handle_service();
void handle_system();
void handle_firmware();
void handle_default();
#ifdef SDCRAD
void handle_storage();
void handle_download();
void handle_delete();
void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
#endif
void webService();
#ifdef SA818
void handle_radio();
extern void SA818_INIT(bool boot);
#endif
