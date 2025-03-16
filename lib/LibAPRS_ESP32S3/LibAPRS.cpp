#include "Arduino.h"
#include "AFSK.h"
#include "AX25.h"
// extern "C" {
// //#include "user_interface.h"
// }uint32_t freemem = system_get_free_heap_size();
//Afsk modem;
AX25Ctx AX25;
//extern void aprs_msg_callback(struct AX25Msg *msg);

#define countof(a) sizeof(a) / sizeof(a[0])

unsigned long custom_preamble = 350UL;
unsigned long custom_tail = 50UL;

AX25Call src;
AX25Call dst;
AX25Call path1;
AX25Call path2;

char CALL[7] = "NOCALL";
int CALL_SSID = 0;
char DST[7] = "APE32I";
int DST_SSID = 0;
char PATH1[7] = "WIDE1";
int PATH1_SSID = 1;
char PATH2[7] = "WIDE2";
int PATH2_SSID = 2;

AX25Call path[8];

// Location packet assembly fields
char latitude[9];
char longtitude[10];
char symbolTable = '/';
char symbol = 'n';

uint8_t power = 10;
uint8_t height = 10;
uint8_t gain = 10;
uint8_t directivity = 10;
/////////////////////////

// Message packet assembly fields
char message_recip[7];
int message_recip_ssid = -1;

int message_seq = 0;
char lastMessage[67];
size_t lastMessageLen;
bool message_autoAck = false;
/////////////////////////

// (2^31 / 180) / 380926 semicircles per Base 91 unit
static unsigned long b91val[4] = {23601572L, 259358L, 2851L, 32L};
// Constants for converting lat/lon to semicircles
static long valtable[]  = {1193046471L, 119304647L, 11930465L, 1988411L,198841L, 19884L, 1988L, 199L, 20L, 2L};

// Takes degrees and fractional minutes and returns 2^31/180 degrees
// West and South values are negative
long semicircles(char *degstr, char hemi)
{
	char dgt = 0, *p;
	long ltemp = 0;
	if (degstr[4] == '.') dgt++;		// Skip hundreds place if we know we're doing latitude
	p = degstr;
	for (;dgt<10; p++)
	{
		if (*p == '.') continue;
		if (!isdigit(*p)) break;
		ltemp += (*p & 0x0f) * valtable[dgt];
		dgt++;
	}
	if (hemi) return -ltemp;
	return ltemp;
}

// Converts semicircles to Base 91 units (must be properly biased first)
// Non-reentrant use of ltemp, but we only call this function in one place
void base91encode(long ltemp,char *s)
{
	// (2^31 / 180) / 380926 semicircles per Base 91 unit
	unsigned char c;
	
	for (c=0; c<4; c++)
	{
		s[c] = '!';
		while (ltemp >= b91val[c])
		{
			//reset_watchdog;
			ltemp -= b91val[c];
			s[c]++;
		}
	}
}

void telemetry_base91(char *cdata, char *output, size_t outputsize)
{
    int x, d1, d2;
    int i = 0;
    // Returns first token
    char *token = strtok(cdata, ",");
    output[i++] = '|';
    // Keep printing tokens while one of the
    // delimiters present in str[].
    while (token != NULL)
    {
        // printf(" % s\n", token);
        x = atoi(token);
        if (x > 8280)
            x = 8280;
        if (x < 0)
            x = 0;
        d1 = int(x / 91);
        d2 = x % 91;
        output[i++] = d1 + 33;
        output[i++] = d2 + 33;
        token = strtok(NULL, ",");
    }
    output[i++] = '|';
    outputsize = i;
}

void APRS_init()
{
    //AFSK_init();
    //Ax25Init();
}

void APRS_poll(void)
{
    //ax25_poll(&AX25);
}

void APRS_setCallsign(char *call, int ssid)
{
    memset(CALL, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0)
    {
        CALL[i] = call[i];
        i++;
    }
    CALL_SSID = ssid;
}

void APRS_setDestination(char *call, int ssid)
{
    memset(DST, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0)
    {
        DST[i] = call[i];
        i++;
    }
    DST_SSID = ssid;
}

void APRS_setPath1(char *call, int ssid)
{
    memset(PATH1, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0)
    {
        PATH1[i] = call[i];
        i++;
    }
    PATH1_SSID = ssid;
}

void APRS_setPath2(char *call, int ssid)
{
    memset(PATH2, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0)
    {
        PATH2[i] = call[i];
        i++;
    }
    PATH2_SSID = ssid;
}

void APRS_setMessageDestination(char *call, int ssid)
{
    memset(message_recip, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0)
    {
        message_recip[i] = call[i];
        i++;
    }
    message_recip_ssid = ssid;
}

void APRS_setPreamble(unsigned long pre)
{
    custom_preamble = pre;
}

void APRS_setTail(unsigned long tail)
{
    custom_tail = tail;
}

void APRS_useAlternateSymbolTable(bool use)
{
    if (use)
    {
        symbolTable = '\\';
    }
    else
    {
        symbolTable = '/';
    }
}

void APRS_setSymbol(char sym)
{
    symbol = sym;
}

void APRS_setLat(char *lat)
{
    memset(latitude, 0, 9);
    int i = 0;
    while (i < 8 && lat[i] != 0)
    {
        latitude[i] = lat[i];
        i++;
    }
}

void APRS_setLon(char *lon)
{
    memset(longtitude, 0, 10);
    int i = 0;
    while (i < 9 && lon[i] != 0)
    {
        longtitude[i] = lon[i];
        i++;
    }
}

void APRS_setPower(int s)
{
    if (s >= 0 && s < 10)
    {
        power = s;
    }
}

void APRS_setHeight(int s)
{
    if (s >= 0 && s < 10)
    {
        height = s;
    }
}

void APRS_setGain(int s)
{
    if (s >= 0 && s < 10)
    {
        gain = s;
    }
}

void APRS_setDirectivity(int s)
{
    if (s >= 0 && s < 10)
    {
        directivity = s;
    }
}

void APRS_printSettings()
{
    Serial.println(F("LibAPRS Settings:"));
    Serial.print(F("Callsign:     "));
    Serial.print(CALL);
    Serial.print(F("-"));
    Serial.println(CALL_SSID);
    Serial.print(F("Destination:  "));
    Serial.print(DST);
    Serial.print(F("-"));
    Serial.println(DST_SSID);
    Serial.print(F("Path1:        "));
    Serial.print(PATH1);
    Serial.print(F("-"));
    Serial.println(PATH1_SSID);
    Serial.print(F("Path2:        "));
    Serial.print(PATH2);
    Serial.print(F("-"));
    Serial.println(PATH2_SSID);
    Serial.print(F("Message dst:  "));
    if (message_recip[0] == 0)
    {
        Serial.println(F("N/A"));
    }
    else
    {
        Serial.print(message_recip);
        Serial.print(F("-"));
        Serial.println(message_recip_ssid);
    }
    Serial.print(F("TX Preamble:  "));
    Serial.println(custom_preamble);
    Serial.print(F("TX Tail:      "));
    Serial.println(custom_tail);
    Serial.print(F("Symbol table: "));
    if (symbolTable == '/')
    {
        Serial.println(F("Normal"));
    }
    else
    {
        Serial.println(F("Alternate"));
    }
    Serial.print(F("Symbol:       "));
    Serial.println(symbol);
    Serial.print(F("Power:        "));
    if (power < 10)
    {
        Serial.println(power);
    }
    else
    {
        Serial.println(F("N/A"));
    }
    Serial.print(F("Height:       "));
    if (height < 10)
    {
        Serial.println(height);
    }
    else
    {
        Serial.println(F("N/A"));
    }
    Serial.print(F("Gain:         "));
    if (gain < 10)
    {
        Serial.println(gain);
    }
    else
    {
        Serial.println(F("N/A"));
    }
    Serial.print(F("Directivity:  "));
    if (directivity < 10)
    {
        Serial.println(directivity);
    }
    else
    {
        Serial.println(F("N/A"));
    }
    Serial.print(F("Latitude:     "));
    if (latitude[0] != 0)
    {
        Serial.println(latitude);
    }
    else
    {
        Serial.println(F("N/A"));
    }
    Serial.print(F("Longtitude:   "));
    if (longtitude[0] != 0)
    {
        Serial.println(longtitude);
    }
    else
    {
        Serial.println(F("N/A"));
    }
}

void APRS_sendPkt(void *_buffer, size_t length)
{

    uint8_t *buffer = (uint8_t *)_buffer;

    memcpy(dst.call, DST, 6);
    dst.ssid = DST_SSID;

    memcpy(src.call, CALL, 6);
    src.ssid = CALL_SSID;

    memcpy(path1.call, PATH1, 6);
    path1.ssid = PATH1_SSID;

    memcpy(path2.call, PATH2, 6);
    path2.ssid = PATH2_SSID;

    path[0] = dst;
    path[1] = src;
    path[2] = path1;
    path[3] = path2;

    //ax25_sendVia(&AX25, path, countof(path), buffer, length);
}

void APRS_sendTNC2Pkt(const uint8_t *raw, size_t length)
{
    uint8_t data[300];
    int size=0;
    ax25frame frame;
    ax25_encode(frame, (char *)raw, length);
    size=hdlcFrame(data, 300, &AX25, &frame);
    log_d("TX HDLC Fram size=%d",size);
    void *handle = NULL;
    if(size>0){
        if(NULL == (handle = Ax25WriteTxFrame(data,size))){
            log_e("Failed to write frame to transmit buffer");   
        }
    }
}

void APRS_sendRawPkt(uint8_t *raw, size_t length)
{
    void *handle = NULL;
    if(length>0){
        if(NULL == (handle = Ax25WriteTxFrame(raw,length))){
            log_e("Failed to write frame to transmit buffer");   
        }
    }
}

// Dynamic RAM usage of this function is 30 bytes
void APRS_sendLoc(void *_buffer, size_t length)
{
    size_t payloadLength = 20 + length;
    bool usePHG = false;
    if (power < 10 && height < 10 && gain < 10 && directivity < 9)
    {
        usePHG = true;
        payloadLength += 7;
    }
    uint8_t *packet = (uint8_t *)malloc(payloadLength);
    uint8_t *ptr = packet;
    packet[0] = '=';
    packet[9] = symbolTable;
    packet[19] = symbol;
    ptr++;
    memcpy(ptr, latitude, 8);
    ptr += 9;
    memcpy(ptr, longtitude, 9);
    ptr += 10;
    if (usePHG)
    {
        packet[20] = 'P';
        packet[21] = 'H';
        packet[22] = 'G';
        packet[23] = power + 48;
        packet[24] = height + 48;
        packet[25] = gain + 48;
        packet[26] = directivity + 48;
        ptr += 7;
    }
    if (length > 0)
    {
        uint8_t *buffer = (uint8_t *)_buffer;
        memcpy(ptr, buffer, length);
    }

    APRS_sendPkt(packet, payloadLength);
    free(packet);
}

// Dynamic RAM usage of this function is 18 bytes
void APRS_sendMsg(void *_buffer, size_t length)
{
    if (length > 67)
        length = 67;
    size_t payloadLength = 11 + length + 4;

    uint8_t *packet = (uint8_t *)malloc(payloadLength);
    uint8_t *ptr = packet;
    packet[0] = ':';
    int callSize = 6;
    int count = 0;
    while (callSize--)
    {
        if (message_recip[count] != 0)
        {
            packet[1 + count] = message_recip[count];
            count++;
        }
    }
    if (message_recip_ssid != -1)
    {
        packet[1 + count] = '-';
        count++;
        if (message_recip_ssid < 10)
        {
            packet[1 + count] = message_recip_ssid + 48;
            count++;
        }
        else
        {
            packet[1 + count] = 49;
            count++;
            packet[1 + count] = message_recip_ssid - 10 + 48;
            count++;
        }
    }
    while (count < 9)
    {
        packet[1 + count] = ' ';
        count++;
    }
    packet[1 + count] = ':';
    ptr += 11;
    if (length > 0)
    {
        uint8_t *buffer = (uint8_t *)_buffer;
        memcpy(ptr, buffer, length);
        memcpy(lastMessage, buffer, length);
        lastMessageLen = length;
    }

    message_seq++;
    if (message_seq > 999)
        message_seq = 0;

    packet[11 + length] = '{';
    int n = message_seq % 10;
    int d = ((message_seq % 100) - n) / 10;
    int h = (message_seq - d - n) / 100;

    packet[12 + length] = h + 48;
    packet[13 + length] = d + 48;
    packet[14 + length] = n + 48;

    APRS_sendPkt(packet, payloadLength);
    free(packet);
}

void APRS_msgRetry()
{
    message_seq--;
    APRS_sendMsg(lastMessage, lastMessageLen);
}

int freeMemory()
{
    int free_memory = ESP.getFreeHeap();
    return free_memory;
}
