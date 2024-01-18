#include "main.h"
#include "sa868.h"

bool SA868_WaitResponse(HardwareSerial * SerialRF, const char * cmd, String * result)
{
    uint32_t startMillis = millis();
    uint32_t timeout=200;
    result->clear();

    ESP_LOGD("SA8x8", "Command '%s'", cmd);
    SerialRF->printf(cmd);
    do 
    {
        while (SerialRF->available() > 0)
        {
            int8_t ch = SerialRF->read();
            * result += static_cast<char>(ch);

            if (result->endsWith("\r\n")) {
                ESP_LOGD("SA8x8", "Response '%s'", result);
                return true;
            }
        }
    } while (millis() - startMillis < timeout);

    ESP_LOGD("SA8x8", "Command '%s' error: '%s'", cmd, result);
    
    return false;
}