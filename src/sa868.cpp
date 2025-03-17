#include <Arduino.h>
#include "main.h"
#include "sa868.h"

bool SA868_WaitResponse(HardwareSerial * SerialRF, const char * cmd, String * result)
{
    uint32_t startMillis = millis();
    uint32_t timeout=200;
    result->clear();

    //ESP_LOGD("SA8x8", "Command '%s'", cmd);
    SerialRF->flush();
    SerialRF->printf(cmd);
    do 
    {
        while (SerialRF->available() > 0)
        {
            int8_t ch = SerialRF->read();
            * result += static_cast<char>(ch);

            if (result->endsWith("\r\n")) {
                //ESP_LOGD("SA8x8", "Response '%s'", result);
                return true;
            }
            delay(20);
        }
    } while (millis() - startMillis < timeout);

    ESP_LOGD("SA8x8", "Command '%s' error: '%s'", cmd, result);
    
    return false;
}

//#ifdef SA868_OPEN_EDITION

bool SA868_WriteAT1846Sreg(HardwareSerial * SerialRF, uint8_t reg, uint16_t value)
{
    char str[200];
    String result;
    sprintf(str, "AT+POKE=%d,%d\r\n", reg, value);
    if (!SA868_WaitResponse(SerialRF, str, &result)) {
        ESP_LOGE("SA8x8", "Error: reg: %02X <- val: %02X", reg, value);
        return false;
    }
    
    return true;
}

uint16_t SA868_ReadAT1846Sreg(HardwareSerial * SerialRF, uint8_t reg)
{
    String data;
    uint16_t value = 0;
    char str[200];
    sprintf(str, "AT+PEEK=%d\r\n", reg);
    if (!SA868_WaitResponse(SerialRF, str, &data))
    {
        ESP_LOGD("SA8x8", "Error: reg: %02X -> \n", reg);
        return 0;
    }
    sscanf(data.c_str(), "%hd\r", &value);

    return value;
}

void SA868_maskSetRegister(HardwareSerial * SerialRF, const uint8_t reg, const uint16_t mask, const uint16_t value)
{
    uint16_t regVal = SA868_ReadAT1846Sreg(SerialRF, reg);
    regVal = (regVal & ~mask) | (value & mask);
    SA868_WriteAT1846Sreg(SerialRF, reg, regVal);
}

uint16_t SA868_maskSetValue(const uint16_t initValue, const uint16_t mask, const uint16_t value)
{
    return (initValue & ~mask) | (value & mask);
}

void SA868_reloadConfig(HardwareSerial * SerialRF)
{
    uint16_t funcMode = SA868_ReadAT1846Sreg(SerialRF, 0x30) & 0x0060;   // Get current op. status
    SA868_maskSetRegister(SerialRF, 0x30, 0x0060, 0x0000);              // RX and TX off
    SA868_maskSetRegister(SerialRF, 0x30, 0x0060, funcMode);            // Restore op. status
}

void SA868::setAudio(bool value)
{
    char str[200];
    String result;
    sprintf(str, "AT+AUDIO=%d\r\n", value);
    if (!SA868_WaitResponse(_SerialRF, str, &result)) 
    {
        ESP_LOGD("SA8x8", "SetAudio Error"); 
    }
}

void SA868_setFuncMode(HardwareSerial * SerialRF, const SA868_Mode mode)
{
    /*
    * Functional mode is controlled by bits 5 (RX on) and 6 (TX on) in
    * register 0x30. With a cast and shift we can set it easily.
    */

    uint16_t value = static_cast< uint16_t >(mode) << 5;
    SA868_maskSetRegister(SerialRF, 0x30, 0x0060, value);
}


SA868::SA868(HardwareSerial * SerialRF, uint8_t RX_PIN, uint8_t TX_PIN) 
: _SerialRF {SerialRF}
, _RX_PIN {RX_PIN}
, _TX_PIN {TX_PIN}
{
  _config.freq_rx = 144390000;
  _config.freq_tx = 144390000;
  _config.tone_rx = 0;
  _config.tone_tx = 0;
  _config.band = 0;
  _config.sql_level = 1;
  _config.rf_power = false;
  _config.volume = 7;
  _config.mic = 8;
  _config.mode = SA868_Mode::OFF;
}

void SA868::setTxFrequency(uint32_t freq)
{
    // todo check for range
    _config.freq_tx = freq;
    if(_config.mode == SA868_Mode::TX)
    {
        setFrequency(freq);
    }
}

void SA868::setRxFrequency(uint32_t freq)
{
    // todo check for range
    _config.freq_rx = freq;
    if (_config.mode == SA868_Mode::RX)
    {
        setFrequency(freq);
    }
}


void SA868::setTxTone(uint32_t tone)
{
    _config.tone_tx = tone;
}

void SA868::setRxTone(uint32_t tone)
{
    _config.tone_rx = tone;
}

bool SA868::init() {
    //_SerialRF->begin(9600, SERIAL_8N1, _RX_PIN, _TX_PIN);

    if(SA868_WriteAT1846Sreg(_SerialRF, 0x30, 0x0001))   // Soft reset
    {
    delay(100);
    SA868_WriteAT1846Sreg(_SerialRF, 0x30, 0x0004);   // Chip enable
    SA868_WriteAT1846Sreg(_SerialRF, 0x04, 0x0FD0);   // 26MHz crystal frequency
    SA868_WriteAT1846Sreg(_SerialRF, 0x1F, 0x1000);   // Gpio6 squelch output
    SA868_WriteAT1846Sreg(_SerialRF, 0x09, 0x03AC);
    SA868_WriteAT1846Sreg(_SerialRF, 0x24, 0x0001);
    SA868_WriteAT1846Sreg(_SerialRF, 0x31, 0x0031);
    SA868_WriteAT1846Sreg(_SerialRF, 0x33, 0x45F5);   // AGC number
    SA868_WriteAT1846Sreg(_SerialRF, 0x34, 0x2B89);   // RX digital gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x3F, 0x3263);   // RSSI 3 threshold
    SA868_WriteAT1846Sreg(_SerialRF, 0x41, 0x470F);   // Tx digital gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x42, 0x1036);
    SA868_WriteAT1846Sreg(_SerialRF, 0x43, 0x00BB);
    
    SA868_WriteAT1846Sreg(_SerialRF, 0x44, 0x06FF);   // Tx digital gain

    SA868_WriteAT1846Sreg(_SerialRF, 0x47, 0x7F2F);   // Soft mute
    SA868_WriteAT1846Sreg(_SerialRF, 0x4E, 0x0082);
    SA868_WriteAT1846Sreg(_SerialRF, 0x4F, 0x2C62);
    SA868_WriteAT1846Sreg(_SerialRF, 0x53, 0x0094);
    SA868_WriteAT1846Sreg(_SerialRF, 0x54, 0x2A3C);
    SA868_WriteAT1846Sreg(_SerialRF, 0x55, 0x0081);
    SA868_WriteAT1846Sreg(_SerialRF, 0x56, 0x0B02);
    SA868_WriteAT1846Sreg(_SerialRF, 0x57, 0x1C00);   // Bypass RSSI low-pass
    SA868_WriteAT1846Sreg(_SerialRF, 0x5A, 0x4935);   // SQ detection time
    SA868_WriteAT1846Sreg(_SerialRF, 0x58, 0xBCCD);
    SA868_WriteAT1846Sreg(_SerialRF, 0x62, 0x3263);   // Modulation detect tresh
    SA868_WriteAT1846Sreg(_SerialRF, 0x4E, 0x2082);
    SA868_WriteAT1846Sreg(_SerialRF, 0x63, 0x16AD);
    SA868_WriteAT1846Sreg(_SerialRF, 0x30, 0x40A4);
    delay(50);

    SA868_WriteAT1846Sreg(_SerialRF, 0x30, 0x40A6);   // Start calibration
    delay(100);
    SA868_WriteAT1846Sreg(_SerialRF, 0x30, 0x4006);   // Stop calibration

    delay(100);

    SA868_WriteAT1846Sreg(_SerialRF, 0x58, 0xBCED);
    SA868_WriteAT1846Sreg(_SerialRF, 0x0A, 0x7BA0);   // PGA gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x41, 0x4731);   // Tx digital gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x44, 0x05FF);   // Tx digital gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x59, 0x09D2);   // Mixer gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x44, 0x05CF);   // Tx digital gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x44, 0x05CC);   // Tx digital gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x48, 0x1A32);   // Noise 1 threshold
    SA868_WriteAT1846Sreg(_SerialRF, 0x60, 0x1A32);   // Noise 2 threshold
    SA868_WriteAT1846Sreg(_SerialRF, 0x3F, 0x29D1);   // RSSI 3 threshold
    SA868_WriteAT1846Sreg(_SerialRF, 0x0A, 0x7BA0);   // PGA gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x33, 0x45F5);   // AGC number
    SA868_WriteAT1846Sreg(_SerialRF, 0x41, 0x470F);   // Tx digital gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x42, 0x1036);
    SA868_WriteAT1846Sreg(_SerialRF, 0x43, 0x00BB);
    
    updateBandwidth();

    // FM mode
    SA868_WriteAT1846Sreg(_SerialRF, 0x33, 0x44A5);
    SA868_WriteAT1846Sreg(_SerialRF, 0x41, 0x4431);
    SA868_WriteAT1846Sreg(_SerialRF, 0x42, 0x10F0);
    SA868_WriteAT1846Sreg(_SerialRF, 0x43, 0x00A9);
    SA868_WriteAT1846Sreg(_SerialRF, 0x58, 0xBC80);   // Bit 0  = 1: CTCSS LPF badwidth to 250Hz
                                    // Bit 3  = 0: enable CTCSS HPF
                                    // Bit 4  = 0: enable CTCSS LPF
                                    // Bit 5  = 0: enable voice LPF
                                    // Bit 6  = 0: enable voice HPF
                                    // Bit 7  = 0: enable pre/de-emphasis
                                    // Bit 11 = 1: bypass VOX HPF
                                    // Bit 12 = 1: bypass VOX LPF
                                    // Bit 13 = 1: bypass RSSI LPF
    //SA868_WriteAT1846Sreg(_SerialRF, 0x44, SA868_maskSetValue(0x06FF, 0x00F0, ((int16_t)_config.volume) << 8));
    //SA868_WriteAT1846Sreg(_SerialRF, 0x44,_config.volume*2 | (((int16_t)_config.volume*2) << 4));
    SA868_WriteAT1846Sreg(_SerialRF, 0x44,0x00FF);
    SA868_WriteAT1846Sreg(_SerialRF, 0x40, 0x0030);

    SA868_maskSetRegister(_SerialRF, 0x57, 0x0001, 0x00);     // Audio feedback off
    SA868_maskSetRegister(_SerialRF, 0x3A, 0x7000, 0x4000);   // Select voice channel

    SA868_maskSetRegister(_SerialRF, 0x30, 0x0008, 0x0000);   // SQ OFF    
    setPower();
    return true;
    }
    return false;
}

void SA868::setBandwidth(uint8_t value)
{
    if (value > 1) {
        value = 1;
    }
    _config.band = value;
    updateBandwidth();
}

void SA868::updateBandwidth()
{
    if (_config.band == 0) { 
        // 12.5kHz bandwidth
        SA868_WriteAT1846Sreg(_SerialRF, 0x15, 0x1100);   // Tuning bit
        SA868_WriteAT1846Sreg(_SerialRF, 0x32, 0x4495);   // AGC target power
        SA868_WriteAT1846Sreg(_SerialRF, 0x3A, 0x4003);   // Modulation detect sel
        //SA868_WriteAT1846Sreg(_SerialRF, 0x3F, 0x28D0);   // RSSI 3 threshold
        SA868_WriteAT1846Sreg(_SerialRF, 0x3F, 0x29D2);   // RSSI 3 threshold
        SA868_WriteAT1846Sreg(_SerialRF, 0x3C, 0x0F1E);   // Peak detect threshold
        SA868_WriteAT1846Sreg(_SerialRF, 0x48, 0x1DB6);   // Noise 1 threshold
        SA868_WriteAT1846Sreg(_SerialRF, 0x62, 0x1425);   // Modulation detect tresh
        SA868_WriteAT1846Sreg(_SerialRF, 0x65, 0x2494);
        SA868_WriteAT1846Sreg(_SerialRF, 0x66, 0xEB2E);   // RSSI comp and AFC range
        SA868_WriteAT1846Sreg(_SerialRF, 0x7F, 0x0001);   // Switch to page 1
        SA868_WriteAT1846Sreg(_SerialRF, 0x06, 0x0014);   // AGC gain table
        SA868_WriteAT1846Sreg(_SerialRF, 0x07, 0x020C);
        SA868_WriteAT1846Sreg(_SerialRF, 0x08, 0x0214);
        SA868_WriteAT1846Sreg(_SerialRF, 0x09, 0x030C);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0A, 0x0314);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0B, 0x0324);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0C, 0x0344);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0D, 0x1344);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0E, 0x1B44);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0F, 0x3F44);
        SA868_WriteAT1846Sreg(_SerialRF, 0x12, 0xE0EB);   // Back to page 0
        SA868_WriteAT1846Sreg(_SerialRF, 0x7F, 0x0000);
        SA868_maskSetRegister(_SerialRF, 0x30, 0x3000, 0x0000);
    } else {
        // 25kHz bandwidth
        SA868_WriteAT1846Sreg(_SerialRF, 0x15, 0x1F00);   // Tuning bit
        SA868_WriteAT1846Sreg(_SerialRF, 0x32, 0x7564);   // AGC target power
        SA868_WriteAT1846Sreg(_SerialRF, 0x3A, 0x4003);   // Modulation detect sel
        SA868_WriteAT1846Sreg(_SerialRF, 0x3F, 0x29D2);   // RSSI 3 threshold
        SA868_WriteAT1846Sreg(_SerialRF, 0x3C, 0x0E1C);   // Peak detect threshold
        SA868_WriteAT1846Sreg(_SerialRF, 0x48, 0x1E38);   // Noise 1 threshold
        SA868_WriteAT1846Sreg(_SerialRF, 0x62, 0x3767);   // Modulation detect tresh
        SA868_WriteAT1846Sreg(_SerialRF, 0x65, 0x248A);
        SA868_WriteAT1846Sreg(_SerialRF, 0x66, 0xFF2E);   // RSSI comp and AFC range
        SA868_WriteAT1846Sreg(_SerialRF, 0x7F, 0x0001);   // Switch to page 1
        SA868_WriteAT1846Sreg(_SerialRF, 0x06, 0x0024);   // AGC gain table
        SA868_WriteAT1846Sreg(_SerialRF, 0x07, 0x0214);
        SA868_WriteAT1846Sreg(_SerialRF, 0x08, 0x0224);
        SA868_WriteAT1846Sreg(_SerialRF, 0x09, 0x0314);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0A, 0x0324);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0B, 0x0344);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0D, 0x1384);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0E, 0x1B84);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0F, 0x3F84);
        SA868_WriteAT1846Sreg(_SerialRF, 0x12, 0xE0EB);
        SA868_WriteAT1846Sreg(_SerialRF, 0x7F, 0x0000);   // Back to page 0
        SA868_maskSetRegister(_SerialRF, 0x30, 0x3000, 0x3000);
    }   
}

void SA868::setVolume(uint8_t value)
{
    _config.volume = value;
    uint16_t volume1 = ((((int16_t)_config.volume)*2)-1) << 4;
    //uint16_t volume2 = (((int16_t)_config.volume)*2)-1;
    uint16_t volume2 = 15;
    SA868_maskSetRegister(_SerialRF, 0x44, 0x00FF, volume1 | volume2);
}

int16_t SA868::getRSSI()
{
    return  -137 + static_cast< int16_t >(SA868_ReadAT1846Sreg(_SerialRF, 0x1B) >> 8);
}

SA868_Version SA868::Version()
{
    SA868_Version version;
    String data;
    if (!SA868_WaitResponse(_SerialRF, "AT+VERSION\r\n", &data))
    {
        ESP_LOGD("SA8x8", "Version Error"); 
        return version;
    }
    sscanf(data.c_str(), "sa8x8-fw/v%hhu.%hhu.%hhu.r%hhu", &version.major, &version.minor, &version.patch, &version.revision);
    ESP_LOGD("SA8x8", "Version %d.%d.%d.%d", version.major, version.minor, version.patch, version.revision); 
    
    return version;
}

void SA868::setFrequency(uint32_t freq)
{
    // AT1846S datasheet specifies a frequency step of 1/16th of kHz per bit.
    // Computation of register value is done using 64 bit to avoid overflows,
    // result is then truncated to 32 bits to fit it into the registers.
    uint64_t val = ((uint64_t) freq * 16) / 1000;
    val &= 0xFFFFFFFF;

    uint16_t fHi = (val >> 16) & 0xFFFF;
    uint16_t fLo = val & 0xFFFF;

    SA868_WriteAT1846Sreg(_SerialRF, 0x29, fHi);
    SA868_WriteAT1846Sreg(_SerialRF, 0x2A, fLo);

    SA868_reloadConfig(_SerialRF); 
}

void SA868::setSqlThresh(uint8_t value)
{
    _config.sql_level = value;
    setSqlThresh();
}

void SA868::setSqlThresh()
{
    if(_config.sql_level > 0)
    {
        uint16_t sql_open = 135-(_config.sql_level*15)+5;
        uint16_t sql_shut = 135-((_config.sql_level*15));
        log_d("sql_open: -%ddBm, sql_shut: -%ddBm", sql_open, sql_shut);
        SA868_maskSetRegister(_SerialRF, 0x45, 0x0400, 0x0000);  // SQ detect mode
        SA868_WriteAT1846Sreg(_SerialRF, 0x48, sql_open<<3); //Sq open threshlod
        SA868_WriteAT1846Sreg(_SerialRF, 0x49, sql_shut<<3); //Sq detect low th,Sq shut threshold
        SA868_maskSetRegister(_SerialRF, 0x30, 0x0008, 0x0008);   // SQ ON
        SA868_maskSetRegister(_SerialRF, 0x54, 0x0080, 0x0080);   // SQ OUT SEL
    }else{
        SA868_maskSetRegister(_SerialRF, 0x30, 0x0008, 0x0000);   // SQ OFF
        SA868_maskSetRegister(_SerialRF, 0x54, 0x0080, 0x0080);
        SA868_maskSetRegister(_SerialRF, 0x45, 0x0400, 0x0000);  // SQ detect mode
    }
}

void SA868::RxOn()
{
    if (_config.mode == SA868_Mode::RX) 
        return;
    SA868::setFrequency(_config.freq_rx);
    SA868_setFuncMode(_SerialRF, SA868_Mode::RX);
    _config.mode = SA868_Mode::RX;
}

void SA868::TxOn()
{
    if (_config.mode == SA868_Mode::TX) 
        return;
    SA868::setFrequency(_config.freq_tx);
    SA868_setFuncMode(_SerialRF, SA868_Mode::TX);
    _config.mode = SA868_Mode::TX;
}

void SA868::TxOff()
{
    if (_config.mode != SA868_Mode::TX) 
        return;
    SA868_setFuncMode(_SerialRF, SA868_Mode::OFF);
    _config.mode = SA868_Mode::OFF;
}

SA868_Configuration SA868::settings()
{
    return _config;
}

bool SA868::isHighPower()
{
    return _config.rf_power;
}

void SA868::setHighPower()
{
    _config.rf_power = true;
    SA868::setPower();
}

void SA868::setLowPower()
{
    _config.rf_power = false;
    SA868::setPower();
}

void SA868::setPower()
{
    char str[20];
    String result;
    sprintf(str, "AT+AMP=%d\r\n", _config.rf_power);
    if (!SA868_WaitResponse(_SerialRF, str, &result))
    {
        ESP_LOGD("SA8x8", "can't enable power amplifier");
    }
}

//#endif