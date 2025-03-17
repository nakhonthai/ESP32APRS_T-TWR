#ifndef SA_868_H
#define SA_868_H

#include <Arduino.h>

enum class SA868_Mode: uint8_t
{
    OFF = 0,
    RX,
    TX
};

typedef struct SA868_Config_Struct
{
	uint32_t freq_rx;
	uint32_t freq_tx;
	int tone_rx;
	int tone_tx;
	uint8_t band;
	uint8_t sql_level;
	bool rf_power;
	uint8_t volume;
	uint8_t mic;
    SA868_Mode mode;
} SA868_Configuration;

typedef struct SA868_Version_Struct 
{
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint8_t revision;
} SA868_Version;

bool SA868_WaitResponse(HardwareSerial * SerialRF, const char * cmd, String * result);

class SA868
{
    public:
        SA868(HardwareSerial * SerialRF, uint8_t RX_PIN, uint8_t TX_PIN);
        bool init();
        SA868_Version Version();
        int16_t getRSSI();
        void RxOn();
        void TxOn();
        void TxOff();
        void setAudio(bool value);
        void setTxFrequency(uint32_t freq);
        void setRxFrequency(uint32_t freq);
        void setTxTone(uint32_t tone);
        void setRxTone(uint32_t tone);
        void setVolume(uint8_t value);
        void setBandwidth(uint8_t value);
        void setSqlThresh(uint8_t value);
        SA868_Configuration settings();
        bool isHighPower();
        void setHighPower();
        void setLowPower();
    private:
        HardwareSerial * _SerialRF;
        SA868_Configuration _config;
        uint8_t _RX_PIN;
        uint8_t _TX_PIN;
        void setFrequency(uint32_t freq);
        void updateBandwidth();
        void setSqlThresh();
        void setPower();
};
#endif
