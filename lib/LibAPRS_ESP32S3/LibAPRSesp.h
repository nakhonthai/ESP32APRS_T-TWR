#ifndef _LIBAPRSESP_H
#define _LIBAPRSESP_H
#include "Arduino.h"
#include <stdint.h>
#include <stdbool.h>

#include "CRC-CCIT.h"
#include "HDLC.h"
#include "AFSK.h"
#include "AX25.h"

void APRS_init();
void APRS_poll(void);

void APRS_setCallsign(char *call, int ssid);
void APRS_setDestination(char *call, int ssid);
void APRS_setMessageDestination(char *call, int ssid);
void APRS_setPath1(char *call, int ssid);
void APRS_setPath2(char *call, int ssid);

void APRS_setPreamble(unsigned long pre);
void APRS_setTail(unsigned long tail);
void APRS_useAlternateSymbolTable(bool use);
void APRS_setSymbol(char sym);

void APRS_setLat(char *lat);
void APRS_setLon(char *lon);
void APRS_setPower(int s);
void APRS_setHeight(int s);
void APRS_setGain(int s);
void APRS_setDirectivity(int s);

void APRS_sendPkt(void *_buffer, size_t length);
void APRS_sendLoc(void *_buffer, size_t length);
void APRS_sendMsg(void *_buffer, size_t length);
void APRS_msgRetry();
void APRS_sendRawPkt(uint8_t *raw, size_t length);

void APRS_printSettings();
void APRS_sendTNC2Pkt(const uint8_t *raw, size_t length);

void base91encode(long ltemp,char *s);
long semicircles(char *degstr, char hemi);
void telemetry_base91(char *cdata, char *output, size_t outputsize);

int freeMemory();
#endif
