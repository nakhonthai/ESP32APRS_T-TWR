# ESP32-S3 howto ADC and DAC

The ESP32-S3 version has no DAC and no internal I2S.Therefore, another method must be used.

**ADC** Can use the operating mode of adc continue to DMA.
The SA868 Module provides output signals at voltage levels <2Vp-p,it can use adc attention 11db.

##From the example in the picture
![TWR_AUDIO2ESP](image/TWRPlus_AudioOut_signal.png)

**DAC** Can use he operating mode of sigmadelta.Use the pulse width to determine the voltage level through the LPF generate signals, similar to PWM.

## Example sample 9600Hz in the picture
![sigmadelta_9600sample](image/sigmadelta_sample_9k6.jpg)

## Example sample 48,000Hz in the picture
![sigmadelta_48ksample](image/sigmadelta_sample_48k.jpg)

## Measuring point of signal
![TWR_MIC2ESP](image/TWR_audio_mic2esp.jpg)