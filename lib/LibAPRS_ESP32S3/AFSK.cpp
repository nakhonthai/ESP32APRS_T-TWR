#include <Arduino.h>
#include "AFSK.h"
// #include <Wire.h>
#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include <driver/timer.h>
// #ifdef CONFIG_IDF_TARGET_ESP32
#include "hal/adc_hal.h"
#include "hal/adc_hal_common.h"
// #endif
//#include "cppQueue.h"

#include "driver/sdm.h"
#include "driver/gptimer.h"

#include <hal/misc.h>
#include <soc/syscon_struct.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/sigmadelta.h"

#include "driver/gptimer.h"

// #include "esp32/rom/crc.h"

#include "modem.h"

#include "fx25.h"

extern "C"
{
#include "soc/syscon_reg.h"
#include "soc/syscon_struct.h"
}

bool pttON = false;
bool pttOFF = false;

#define DEBUG_TNC

extern unsigned long custom_preamble;
extern unsigned long custom_tail;
int adcVal;

int8_t _sql_pin = 38, _ptt_pin = 41, _pwr_pin, _dac_pin = 18, _adc_pin = 1;
bool _sql_active, _ptt_active = HIGH, _pwr_active;

uint8_t adc_atten;

static const adc_unit_t unit = ADC_UNIT_1;

void sample_dac_isr();
bool hw_afsk_dac_isr = false;

static int Vref = 950;

#define FILTER_TAPS 8 // Number of taps in the FIR filter
uint16_t SAMPLERATE = 38400;
// Resampling configuration
#define INPUT_RATE 38400
#define OUTPUT_RATE 9600
uint16_t RESAMPLE_RATIO = (SAMPLERATE / OUTPUT_RATE); // 38400/9600 = 3
uint16_t BLOCK_SIZE = (SAMPLERATE / 50);              // Must be multiple of resample ratio
float *audio_buffer = NULL;

// AGC configuration
#define AGC_TARGET_RMS 0.2f // Target RMS level (-10dBFS)
#define AGC_ATTACK 0.02f    // Fast attack rate
#define AGC_RELEASE 0.001f  // Slow release rate
#define AGC_MAX_GAIN 10.0f
#define AGC_MIN_GAIN 0.1f

int8_t _led_rx_pin = -1;
int8_t _led_tx_pin = -1;
int8_t _led_strip_pin = 42;
uint8_t r_old = 0, g_old = 0, b_old = 0;
unsigned long rgbTimeout = 0;

#include <Adafruit_NeoPixel.h>
extern Adafruit_NeoPixel *strip;

extern float markFreq;  // mark frequency
extern float spaceFreq; // space freque
extern float baudRate;  // baudrate

/****************** Ring Buffer gen from DeepSeek *********************/
#define BUFFER_SIZE 1500
typedef struct {
    int16_t buffer[BUFFER_SIZE]; // Buffer to store int16_t data
    int head;                    // Index for the next write
    int tail;                    // Index for the next read
    int count;                   // Number of elements in the buffer
} RingBuffer;

// Initialize the ring buffer
void RingBuffer_Init(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

// Check if the buffer is full
bool RingBuffer_IsFull(const RingBuffer *rb) {
    return rb->count == BUFFER_SIZE;
}

// Check if the buffer is empty
bool RingBuffer_IsEmpty(const RingBuffer *rb) {
    return rb->count == 0;
}

// Add an element to the buffer (push)
bool RingBuffer_Push(RingBuffer *rb, int16_t data) {
    if (RingBuffer_IsFull(rb)) {
        return false; // Buffer is full
    }
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % BUFFER_SIZE; // Wrap around using modulo
    rb->count++;
    return true;
}

// Remove an element from the buffer (pop)
bool RingBuffer_Pop(RingBuffer *rb, int16_t *data) {
    if (RingBuffer_IsEmpty(rb)) {
        return false; // Buffer is empty
    }
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % BUFFER_SIZE; // Wrap around using modulo
    rb->count--;
    return true;
}

// Get the number of elements in the buffer
int RingBuffer_Size(const RingBuffer *rb) {
    return rb->count;
}

RingBuffer fifo; // Declare a ring buffer
/******************************************************************** */

void LED_init(int8_t led_tx_pin, int8_t led_rx_pin, int8_t led_strip_pin)
{
  _led_tx_pin = led_tx_pin;
  _led_rx_pin = led_rx_pin;
  _led_strip_pin = led_strip_pin;
  if (led_strip_pin > -1)
  {
    rgbTimeout = millis() + 50;
    strip = new Adafruit_NeoPixel(1, _led_strip_pin, NEO_GRB + NEO_KHZ800);
    strip->begin();
    strip->show();
  }
  if (led_tx_pin > -1)
  {
    pinMode(led_tx_pin, OUTPUT);
    digitalWrite(led_tx_pin, LOW);
  }
  if (led_rx_pin > -1)
  {
    pinMode(led_rx_pin, OUTPUT);
    digitalWrite(led_rx_pin, LOW);
  }
}

portMUX_TYPE ledMux = portMUX_INITIALIZER_UNLOCKED;
void LED_Status(uint8_t red, uint8_t green, uint8_t blue)
{
  //portENTER_CRITICAL_ISR(&ledMux); // ISR start
  if (red == r_old && green == g_old && blue == b_old)
  {
    rgbTimeout = millis() + 50;
  }
  else
  {
    if (millis() > rgbTimeout)
    {
      rgbTimeout = millis() + 10;
      r_old = red;
      g_old = green;
      b_old = blue;
      if (_led_strip_pin > -1 && strip != NULL)
      {
        //if (strip->canShow())
        //{
          strip->setPixelColor(0, strip->Color(red, green, blue));
          strip->show();
        //}
      }
      else
      {
        if (_led_tx_pin > -1)
        {
          if (red > 0)
            digitalWrite(_led_tx_pin, HIGH);
          else
            digitalWrite(_led_tx_pin, LOW);
        }
        if (_led_rx_pin > -1)
        {
          if (green > 0)
            digitalWrite(_led_rx_pin, HIGH);
          else
            digitalWrite(_led_rx_pin, LOW);
        }
      }
    }
  }
  //portEXIT_CRITICAL_ISR(&ledMux);
}

/**
 * @brief Controls PTT output
 * @param state False - PTT off, true - PTT on
 */
void setPtt(bool state)
{
  if (state)
  {
    digitalWrite(17, HIGH);
    delay(10);
    if (_ptt_pin >= GPIO_NUM_MAX)
      _ptt_pin = 41;
    if (_ptt_active)
    {
      pinMode(_ptt_pin, OUTPUT);
      digitalWrite(_ptt_pin, HIGH);
    }
    else
    { // Open Collector to LOW
      pinMode(_ptt_pin, OUTPUT_OPEN_DRAIN);
      digitalWrite(_ptt_pin, LOW);
    }
    // if (config.rf_type == RF_SA8x8_OpenEdit)
    //{
    // sa868.TxOn();
    // setTransmit(true);
    //}
    pttON = true;
    // LED_Status(255, 0, 0);
    // rgbTimeout = millis() + 500;
    int c = 0;
    while (hw_afsk_dac_isr == false)
    {
      if (++c > 100)
        break;
      delay(10);
    }
  }
  else
  {
    if (_ptt_active)
    {
      pinMode(_ptt_pin, OUTPUT);
      digitalWrite(_ptt_pin, LOW);
    }
    else
    { // Open Collector to HIGH
      pinMode(_ptt_pin, OUTPUT_OPEN_DRAIN);
      digitalWrite(_ptt_pin, HIGH);
    }
    digitalWrite(17, LOW);
    // rgbTimeout = 0;
    // LED_Status(0, 0, 0);
    // delay(100);
    pttOFF = true;
  }
}

// Filter coefficients (designed for 38400→9600 resampling)
// cutoff = 4800  # Nyquist for 9600Hz
const float resample_coeffs[FILTER_TAPS] = {
    0.003560, 0.038084, 0.161032, 0.297324, 0.297324, 0.161032, 0.038084, 0.003560};
// Filter instance and buffers
void resample_audio(float *input_buffer)
{
  // Apply anti-aliasing filter and decimate
  for (int i = 0; i < BLOCK_SIZE / RESAMPLE_RATIO; i++)
  {
    float sum = 0;
    for (int j = 0; j < FILTER_TAPS; j++)
    {
      int index = i * RESAMPLE_RATIO + j;
      if (index < BLOCK_SIZE)
      {
        sum += input_buffer[index] * resample_coeffs[j];
      }
    }
    input_buffer[i] = sum;
  }
}

// AGC state
float agc_gain = 1.0f;

tcb_t tcb;

// Audio processing
volatile bool new_samples = false;

float update_agc(float *input_buffer, size_t len)
{
  // Calculate RMS of current block
  float sum_sq = 0;
  for (int i = 0; i < len; i++)
  {
    sum_sq += input_buffer[i] * input_buffer[i];
  }
  float rms = sqrtf(sum_sq / len);

  // Adjust gain based on RMS level
  float error = AGC_TARGET_RMS / (rms + 1e-6f);
  float rate = (error < 1.0f) ? AGC_RELEASE : AGC_ATTACK;
  agc_gain = agc_gain * (1.0f - rate) + (agc_gain * error) * rate;
  agc_gain = fmaxf(fminf(agc_gain, AGC_MAX_GAIN), AGC_MIN_GAIN);
  return agc_gain;
}

#define AX25_FLAG 0x7e
#define AX25_MASK 0xfc      // bit mask of MSb six bits
#define AX25_EOP 0xfc       // end of packet, 7e << 1
#define AX25_STUFF_BIT 0x7c // bit stuffing bit, five of continuous one bits
// #define AX25_MIN_PKT_SIZE (7 * 2 + 1 + 1 + 2) // call sign * 2 + control + PID + FCS
#define AX25_FLAG_BITS 6

#define AX25_ADDR_LEN 7
#define AX25_MIN_PKT_SIZE (AX25_ADDR_LEN * 2 + 1 + 2) // src addr + dst addr + Control + FCS
#define AX25_SSID_MASK 0x0f

extern AX25Ctx AX25;

uint8_t adc_pins[] = {1, 2, 3, 4}; // ADC1 common pins for ESP32S2/S3 + ESP32C3/C6 + ESP32H2

// adc_continuous_handle_t adc_handle[SOC_ADC_PERIPH_NUM];
// extern adc_handle_t adc_handle[SOC_ADC_PERIPH_NUM];
adc_continuous_handle_t AdcHandle = NULL;
adc_cali_handle_t AdcCaliHandle = NULL;

// Calculate how many pins are declared in the array - needed as input for the setup function of ADC Continuous
uint8_t adc_pins_count = sizeof(adc_pins) / sizeof(uint8_t);

// Flag which will be set in ISR when conversion is done
volatile bool adc_coversion_done = false;

// Result structure for ADC Continuous reading
adc_continuous_data_t *result = NULL;

// adc_attenuation_t cfg_adc_atten = ADC_0db;
adc_atten_t cfg_adc_atten = ADC_ATTEN_DB_0;
void afskSetADCAtten(uint8_t val)
{
  adc_atten = val;
  if (adc_atten == 0)
  {
    cfg_adc_atten = ADC_ATTEN_DB_0;
    Vref = 950;
  }
  else if (adc_atten == 1)
  {
    cfg_adc_atten = ADC_ATTEN_DB_2_5;
    Vref = 1250;
  }
  else if (adc_atten == 2)
  {
    cfg_adc_atten = ADC_ATTEN_DB_6;
    Vref = 1750;
  }
  else if (adc_atten == 3)
  {
    cfg_adc_atten = ADC_ATTEN_DB_11;
    Vref = 2450;
  }
  else if (adc_atten == 4)
  {
    cfg_adc_atten = ADC_ATTEN_DB_12;
    Vref = 3300;
  }
}

uint8_t CountOnesFromInteger(uint8_t value)
{
  uint8_t count;
  for (count = 0; value != 0; count++, value &= value - 1)
    ;
  return count;
}

uint16_t CountOnesFromInteger(uint16_t value)
{
  uint16_t count = 0;
  for (int i = 0; i < 16; i++)
  {
    if (value & 0x0001)
      count++;
    value >>= 1;
  }
  return count;
}

#define IMPLEMENTATION FIFO
#define ADC_SAMPLES_COUNT (BLOCK_SIZE / 2)

// #if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
// cppQueue adcq(sizeof(int16_t), 768, IMPLEMENTATION, true); // Instantiate queue
// #else
// cppQueue adcq(sizeof(int16_t), 768, IMPLEMENTATION, true); // Instantiate queue
// #endif

static const char *TAG = "--(TAG ADC DMA)--";
#if defined(CONFIG_IDF_TARGET_ESP32)
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#else
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#endif

// hw_timer_t *timer = NULL;
hw_timer_t *timer_dac = NULL;

int8_t adcEn = 0;
int8_t dacEn = 0;

SemaphoreHandle_t xI2CSemaphore;

#define DEFAULT_SEMAPHORE_TIMEOUT 10

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void AFSK_TimerEnable(bool sts)
{
  // if (xSemaphoreTake(xI2CSemaphore, (TickType_t)DEFAULT_SEMAPHORE_TIMEOUT) == pdTRUE)
  // {
  // vTaskSuspendAll ();
  if (sts == true)
  {
    //   timerAlarmEnable(timer);
    if (AdcHandle != NULL)
    {
      adc_continuous_start(AdcHandle);
    }
    adcEn = 0;
  }
  else
  {
    //   timerAlarmDisable(timer);
    if (AdcHandle != NULL)
    {
      adc_continuous_stop(AdcHandle);
    }
    adcEn = 0;
  }
  // xTaskResumeAll ();
  // }
}

void DAC_TimerEnable(bool sts)
{
  if (timer_dac == NULL)
    return;
  // portENTER_CRITICAL_ISR(&timerMux);
  if (sts == true)
  {
    timerStart(timer_dac);
  }
  else
  {
    timerStop(timer_dac);
  }
  // portEXIT_CRITICAL_ISR(&timerMux);
  dacEn = 0;
}

bool getTransmit()
{
  bool ret = false;
  if (_ptt_pin > -1)
  {
    if ((digitalRead(_ptt_pin) ^ _ptt_active) == 0) // signal active with ptt_active
      ret = true;
    else
      ret = false;
  }
  else
  {
    if (hw_afsk_dac_isr)
      ret = true;
  }
  return ret;
}

void setTransmit(bool val)
{
  hw_afsk_dac_isr = val;
}

bool getReceive()
{
  bool ret = false;
  if ((digitalRead(_ptt_pin) ^ _ptt_active) == 0) // signal active with ptt_active
    return false;                                 // PTT Protection receive
  if (digitalRead(LED_RX_PIN))                    // Check RX LED receiving.
    ret = true;
  return ret;
}

uint8_t modem_config = 0;
void afskSetModem(uint8_t val, bool bpf, uint16_t timeSlot, uint16_t preamble, uint8_t fx25Mode)
{
  if (bpf)
    ModemConfig.flatAudioIn = 1;
  else
    ModemConfig.flatAudioIn = 0;

  modem_config = val;
  if (val == 0)
  {
    ModemConfig.modem = MODEM_300;
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
    SAMPLERATE = 9600;
#elif defined(CONFIG_IDF_TARGET_ESP32)
    SAMPLERATE = 19200;
#else
    SAMPLERATE = 38400;
#endif
    BLOCK_SIZE = (SAMPLERATE / 50); // Must be multiple of resample ratio
    RESAMPLE_RATIO = (SAMPLERATE / 9600);
  }
  else if (val == 1)
  {
    ModemConfig.modem = MODEM_1200;
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
    SAMPLERATE = 9600;
#elif defined(CONFIG_IDF_TARGET_ESP32)
    SAMPLERATE = 19200;
#else
    SAMPLERATE = 19200;
#endif
    BLOCK_SIZE = (SAMPLERATE / 50); // Must be multiple of resample ratio
    RESAMPLE_RATIO = (SAMPLERATE / 9600);
  }
  else if (val == 2)
  {
    ModemConfig.modem = MODEM_1200_V23;
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
    SAMPLERATE = 9600;
#elif defined(CONFIG_IDF_TARGET_ESP32)
    SAMPLERATE = 19200;
#else
    SAMPLERATE = 38400;
#endif
    BLOCK_SIZE = (SAMPLERATE / 50); // Must be multiple of resample ratio
    RESAMPLE_RATIO = (SAMPLERATE / 9600);
  }
  else if (val == 3)
  {
    ModemConfig.modem = MODEM_9600;
    SAMPLERATE = 38400;
    BLOCK_SIZE = (SAMPLERATE / 50); // Must be multiple of resample ratio
    RESAMPLE_RATIO = (SAMPLERATE / 38400);
  }
  if (audio_buffer != NULL)
  {
    free(audio_buffer);
    audio_buffer = NULL;
  }
  audio_buffer = (float *)calloc(BLOCK_SIZE, sizeof(float));
  if (audio_buffer == NULL)
  {
    log_d("Error allocating memory for audio buffer");
    return;
  }
  log_d("Modem: %d, SampleRate: %d, BlockSize: %d", modem_config, SAMPLERATE, BLOCK_SIZE);
  ModemConfig.usePWM = 1;
  ModemInit();
  Ax25Init(fx25Mode);
  if (fx25Mode > 0)
    Fx25Init();
  Ax25TimeSlot(timeSlot);
  Ax25TxDelay(preamble);
}

void afskSetHPF(bool val)
{
  // input_HPF = val;
}
void afskSetBPF(bool val)
{
  // input_BPF = val;
}
void afskSetSQL(int8_t val, bool act)
{
  _sql_pin = val;
  _sql_active = act;
  if (_sql_pin > -1)
  {
    if (_sql_active)
      pinMode(_sql_pin, INPUT);
    else
      pinMode(_sql_pin, INPUT_PULLUP);
  }
}
void afskSetPTT(int8_t val, bool act)
{
  _ptt_pin = val;
  _ptt_active = act;
  if (_ptt_active)
  {
    pinMode(_ptt_pin, OUTPUT);
    digitalWrite(_ptt_pin, LOW);
  }
  else
  { // Open Collector to HIGH
    pinMode(_ptt_pin, OUTPUT_OPEN_DRAIN);
    digitalWrite(_ptt_pin, HIGH);
  }
}
void afskSetPWR(int8_t val, bool act)
{
  _pwr_pin = val;
  _pwr_active = act;
  if (_pwr_pin > -1)
    pinMode(_pwr_pin, OUTPUT);

  if (_pwr_pin > -1)
    digitalWrite(_pwr_pin, !_pwr_active);
}

uint32_t ret_num;
uint8_t *resultADC;

typedef struct
{
  voidFuncPtr fn;
  void *arg;
} interrupt_config_t;

// portMUX_TYPE DRAM_ATTR timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile SemaphoreHandle_t timerSemaphore;

int16_t adcPush;
bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t stAdcHandle, const adc_continuous_evt_data_t *edata, void *user_data)
{

  portENTER_CRITICAL_ISR(&timerMux);
  for (uint32_t k = 0; k < edata->size; k += SOC_ADC_DIGI_RESULT_BYTES)
  {
    adc_digi_output_data_t *p = (adc_digi_output_data_t *)&edata->conv_frame_buffer[k];

#if defined(CONFIG_IDF_TARGET_ESP32)
    if (p->type1.channel > 0)
      continue;
    adcPush = (int)p->type1.data;
#else
    if ((p->type2.channel > 0) || (p->type2.unit > 0))
      continue;
    adcPush = (int16_t)p->type2.data;
#endif
fifo.buffer[fifo.head] = adcPush;
fifo.head = (fifo.head + 1) % BUFFER_SIZE; // Wrap around using modulo
fifo.count++;
// if (!RingBuffer_Push(&fifo, adcPush)) {
//   //printf("Buffer is full!\n");
//   break;
// }
    // if (adcq.isFull())
    // {
    //   adcq.flush();
    //   break;
    // }
    //adcq.push(&adcPush);
  }
  // if (adcq.isFull())
  //   adcq.flush();
  portEXIT_CRITICAL_ISR(&timerMux);
  return true;
}

void adc_continue_init(void)
{
  adc_channel_t channel;
  adc_unit_t adc_unit = ADC_UNIT_1;
  esp_err_t Err = adc_continuous_io_to_channel(_adc_pin, &adc_unit, &channel);
  if (Err != ESP_OK)
  {
    log_e("Pin %u is not ADC pin!", _adc_pin);
  }
  if (adc_unit != 0)
  {
    log_e("Only ADC1 pins are supported in continuous mode!");
  }
  log_d("ADC1 GPIO Pin %u", _adc_pin);

  uint32_t conv_frame_size = (uint32_t)(BLOCK_SIZE * SOC_ADC_DIGI_RESULT_BYTES);
  // uint32_t conv_frame_size = ADC_SAMPLES_COUNT;
  /* On initialise l'ADC : */
  adc_continuous_handle_cfg_t AdcHandleConfig = {
      .max_store_buf_size = (uint32_t)conv_frame_size * 2,
      .conv_frame_size = (uint32_t)conv_frame_size,
  };

  Err = adc_continuous_new_handle(&AdcHandleConfig, &AdcHandle);
  if (Err != ESP_OK)
    log_d("AdMeasure : Adc Continuous Init Failed.");
  else
  {
    /* On configure l'ADC : */
    adc_continuous_config_t AdcConfig = {
#if defined(CONFIG_IDF_TARGET_ESP32)
        .sample_freq_hz = (uint32_t)(SAMPLERATE * 11 / 9),
#else
        .sample_freq_hz = (uint32_t)(SAMPLERATE),
#endif
        .conv_mode = ADC_CONV_SINGLE_UNIT_1, // On utilise uniquement l'ADC1.
        .format = ADC_OUTPUT_TYPE,           // On utilise le type 2. Pris dans l'exemple.
    };
    log_d("ADC Continuous Configuration Done. SAMPLERATE: %d Hz", AdcConfig.sample_freq_hz);

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    AdcConfig.pattern_num = 1;
    // for (int ii = 0; ii < NBR_CHANNELS; ii++)
    //{
    // uint8_t unit = ADC_UNIT_1;
    // uint8_t ch = channel & 0x7;
    adc_pattern[0].atten = cfg_adc_atten;
    adc_pattern[0].channel = channel;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    //}
    AdcConfig.adc_pattern = adc_pattern;
    Err = adc_continuous_config(AdcHandle, &AdcConfig);
    if (Err != ESP_OK)
      ESP_LOGI(TAG, "=====> AdMeasure : Adc Continuous Configuration Failed.");
    else
    {
      // Setup callbacks for complete event
      adc_continuous_evt_cbs_t cbs = {
          .on_conv_done = s_conv_done_cb,
          //.on_pool_ovf can be used in future
      };
      // adc_handle[adc_unit].adc_interrupt_handle.fn = (voidFuncPtr)userFunc;
      Err = adc_continuous_register_event_callbacks(AdcHandle, &cbs, NULL);
      if (Err != ESP_OK)
      {
        log_e("adc_continuous_register_event_callbacks failed!");
      }

      // Attach the pins to the ADC unit
#ifdef ARDUINO_TEST
      for (int i = 0; i < cfg.channels; i++)
      {
        adc_channel = cfg.adc_channels[i];
        adc_continuous_channel_to_io(ADC_UNIT, adc_channel, &io_pin);
        // perimanSetPinBus: uint8_t pin, peripheral_bus_type_t type, void * bus, int8_t bus_num, int8_t bus_channel
        if (!perimanSetPinBus(io_pin, ESP32_BUS_TYPE_ADC_CONT, (void *)(ADC_UNIT + 1), ADC_UNIT, adc_channel))
        {
          LOGE("perimanSetPinBus to Continuous an ADC Unit %u failed!", ADC_UNIT);
          return false;
        }
      }
#endif

/* Mise en place de la calibration permettant d'avoir les résultats en mV directement :
 * On utilise le schéma de calibration de type Courbe (Le seul proposé par la puce). */
#ifdef CONFIG_IDF_TARGET_ESP32
      adc_cali_line_fitting_config_t cali_config = {
#else
      adc_cali_curve_fitting_config_t cali_config = {
#endif
        .unit_id = ADC_UNIT_1,
        .atten = cfg_adc_atten,
        .bitwidth = ADC_BITWIDTH_12,
      };

#ifdef CONFIG_IDF_TARGET_ESP32
      Err = adc_cali_create_scheme_line_fitting(&cali_config, &AdcCaliHandle);
      log_d("Creating ADC_UNIT_%d line cali handle", 1);
//
#else
      Err = adc_cali_create_scheme_curve_fitting(&cali_config, &AdcCaliHandle);
      log_d("Creating ADC_UNIT_%d cerve cali handle", 1);
//
#endif
      if (Err != ESP_OK)
        ESP_LOGI(TAG, "=====> AdMeasure : Fail to create the Curve Fitting Scheme.");
      else
      {
        /* On démarre l'ADC : */
        // SYSCON.saradc_ctrl2.meas_num_limit = 0;
        adc_continuous_start(AdcHandle);
        // adc_continuous_stop(AdcHandle);
        //  SYSCON.saradc_ctrl2.meas_num_limit = 0;
        //  HAL_FORCE_MODIFY_U32_REG_FIELD(SYSCON.saradc_ctrl2, meas_num_limit, 0);
        log_d("ADC Continuous has been start");
      }
    }
  }
}
#define MIC_PIN 18 // Out wave to PIN 18
#define PTT_PIN 41
/*
 * Configure and initialize the sigma delta modulation
 * on channel 0 to output signal on GPIO4
 */
static void sigmadelta_init(void)
{

  sigmadelta_config_t sigmadelta_cfg = {
      .channel = SIGMADELTA_CHANNEL_0,
      .sigmadelta_duty = 127,
      .sigmadelta_prescale = 96,
      .sigmadelta_gpio = static_cast<gpio_num_t>(_dac_pin),
  };
  sigmadelta_config(&sigmadelta_cfg);
}

void AFSK_hw_init(void)
{
  // Set up ADC
  log_d("AFSK hardware Initialize");
  // pinMode(RSSI_PIN, INPUT_PULLUP);
  pinMode(PTT_PIN, OUTPUT);
  pinMode(17, OUTPUT);      // MIC_SEL
  pinMode(MIC_PIN, OUTPUT); // ESP2MIC
  pinMode(1, ANALOG);       // AUDIO2ESP

  pinMode(2, OUTPUT); // TEST

  digitalWrite(17, HIGH);
  digitalWrite(PTT_PIN, HIGH); // PTT not active

  // Set up ADC
  if (_sql_pin > -1)
    pinMode(_sql_pin, INPUT_PULLUP);
  if (_pwr_pin > -1)
    pinMode(_pwr_pin, OUTPUT);
  if (_led_tx_pin > -1)
  {
    pinMode(_led_tx_pin, OUTPUT);
  }
  if (_led_rx_pin > -1)
  {
    pinMode(_led_rx_pin, OUTPUT);
  }

  if (_pwr_pin > -1)
    digitalWrite(_pwr_pin, !_pwr_active);

#ifdef I2S_INTERNAL
  //  Initialize the I2S peripheral
  I2S_Init(I2S_MODE_DAC_BUILT_IN, I2S_BITS_PER_SAMPLE_16BIT);
#else
  RingBuffer_Init(&fifo);
  adc_continue_init();
  sigmadelta_init();

  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
  // Set timer frequency to 20Mhz
  timer_dac = timerBegin(20000000);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer_dac, &sample_dac_isr); // Attaches the handler function to the timer
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarm(timer_dac, (uint64_t)20000000 / CONFIG_AFSK_DAC_SAMPLERATE, true, 0);
  timerStop(timer_dac);

  setTransmit(false);
#endif
}

void AFSK_init(int8_t adc_pin, int8_t dac_pin, int8_t ptt_pin, int8_t sql_pin, int8_t pwr_pin, int8_t led_tx_pin, int8_t led_rx_pin, int8_t led_strip_pin, bool ptt_act, bool sql_act, bool pwr_act)
{
  _adc_pin = adc_pin;
  _dac_pin = dac_pin;
  _ptt_pin = ptt_pin;
  _sql_pin = sql_pin;
  _pwr_pin = pwr_pin;
  _led_tx_pin = led_tx_pin;
  _led_rx_pin = led_rx_pin;
  _led_strip_pin = led_strip_pin;

  _ptt_active = ptt_act;
  _sql_active = sql_act;
  _pwr_active = pwr_act;

  if (xI2CSemaphore == NULL)
  {
    xI2CSemaphore = xSemaphoreCreateMutex();
    if ((xI2CSemaphore) != NULL)
      xSemaphoreGive((xI2CSemaphore));
  }

  tcb_t *tp = &tcb;
  int i = 0;
  tp->port = i;
  tp->kiss_type = i << 4;
  tp->avg = 2048;
  tp->cdt = false;
  tp->cdt_lvl = 0;
  tp->cdt_led_pin = 2;
  tp->cdt_led_on = 2;
  log_d("cdt_led_pin = %d, cdt_led_on = %d, port = %d", tp->cdt_led_pin, tp->cdt_led_on, tp->port);
  // tp->ptt_pin = 12;
#ifdef FX25TNCR2
  tp->sta_led_pin = STA_LED_PIN[i];
#endif

#ifdef FX25_ENABLE
  tp->fx25_parity = FX25_PARITY[i]; // FX.25 parity
#endif
  tp->cdt_sem = xSemaphoreCreateBinary();
  assert(tp->cdt_sem);
  assert(xSemaphoreGive(tp->cdt_sem) == pdTRUE); // initialize

  // KISS default parameter
  // tp->fullDuplex = kiss_fullduplex[i]; // half duplex
  tp->fullDuplex = true;  // full duplex
  tp->SlotTime = 10;      // 100ms
  tp->TXDELAY = 50;       // 500ms
  tp->persistence_P = 63; // P = 0.25
  AFSK_hw_init();
}

int offset = 0;
int dc_offset = 620;

bool sqlActive = false;

uint8_t sinwave = 127;

bool adcq_lock = false;

uint16_t phaseAcc = 0;

void IRAM_ATTR sample_dac_isr()
{
  if (hw_afsk_dac_isr)
  {
    portENTER_CRITICAL_ISR(&timerMux); // ISR start
    sinwave = MODEM_BAUDRATE_TIMER_HANDLER();
    // Sigma-delta duty of one channel, the value ranges from -128 to 127, recommended range is -90 ~ 90.The waveform is more like a random one in this range.
    int8_t sine = (int8_t)(((sinwave - 127) * 12) >> 4); // Redue sine = -85 ~ 85
    sigmadelta_set_duty(SIGMADELTA_CHANNEL_0, sine);
    portEXIT_CRITICAL_ISR(&timerMux); // ISR end
    // Give a semaphore that we can check in the loop
    // xSemaphoreGiveFromISR(timerSemaphore, NULL);
  }
}

extern int mVrms;
extern float dBV;

long mVsum = 0;
int mVsumCount = 0;
uint8_t dcd_cnt = 0;

void AFSK_Poll(bool SA818, bool RFPower)
{

  int mV;
  int x = 0;

  int16_t adc;

  if (_sql_pin > -1)
  {                                                 // Set SQL pin active
    if ((digitalRead(_sql_pin) ^ _sql_active) == 0) // signal active with sql_active
      sqlActive = true;
    else
      sqlActive = false;
  }
  else
  {
    sqlActive = true;
  }

  if (!hw_afsk_dac_isr)
  {
    if (audio_buffer != NULL)
    {
      //while (adcq.getCount() >= BLOCK_SIZE)
      while(RingBuffer_Size(&fifo) >= BLOCK_SIZE)
      {
        // digitalWrite(2, HIGH);
        tcb_t *tp = &tcb;
        mVsum = 0;
        mVsumCount = 0;
        for (x = 0; x < BLOCK_SIZE; x++)
        {
          // while(adcq_lock) delay(1);
          //if (!adcq.pop(&adc)) // Pull queue buffer
          if (!RingBuffer_Pop(&fifo, &adc))
            break;

          tp->avg_sum += adc - tp->avg_buf[tp->avg_idx];
          tp->avg_buf[tp->avg_idx++] = adc;
          if (tp->avg_idx >= TCB_AVG_N)
            tp->avg_idx -= TCB_AVG_N;
          tp->avg = tp->avg_sum / TCB_AVG_N;

          // carrier detect
          adcVal = (int)adc - tp->avg;
          int m = 1;
          if ((RESAMPLE_RATIO > 1) || (ModemConfig.modem == MODEM_9600))
          {
            m = 4;
          }

          if (x % m == 0)
          {
            adc_cali_raw_to_voltage(AdcCaliHandle, adc, &mV);
            mV -= offset;
            // mVsum += powl(mV, 2); // VRMS = √(1/n)(V1^2 +V2^2 + … + Vn^2)
            // mV = (adcVal * Vref) >> 12;
            mVsum += mV * mV; // Accumulate squared voltage values
            mVsumCount++;
          }

          float sample = adcVal / 2048.0f * agc_gain;
          audio_buffer[x] = sample;
        }
        // Update AGC gain
        update_agc(audio_buffer, BLOCK_SIZE);
        adc_cali_raw_to_voltage(AdcCaliHandle, tp->avg, &offset);

        if (mVsumCount > 0)
        {
          tp->cdt_lvl = mVrms = sqrtl(mVsum / mVsumCount); // RMS voltage  VRMS = √(1/mVsumCount)(mVsum)
          mVsum = 0;
          mVsumCount = 0;
          if (mVrms > 10) // >-40dBm
          {
            if (dcd_cnt < 100)
              dcd_cnt++;
          }
          else if (mVrms < 5) // <-46dBm
          {
            if (dcd_cnt > 0)
              dcd_cnt--;
          }
          // Tool conversion dBv <--> Vrms at http://sengpielaudio.com/calculator-db-volt.htm
          // dBV = 20.0F * log10(Vrms);
          // log_d("Audio dc_offset=%d mVrms=%d", offset, mVrms);
        }

        if ((dcd_cnt > 3) || (ModemConfig.modem == MODEM_9600))
        {
          tp->cdt = true;
          //  Process audio block
          if (RESAMPLE_RATIO > 1)
            resample_audio(audio_buffer);

          // Process audio block
          int16_t sample;

          for (int i = 0; i < BLOCK_SIZE / RESAMPLE_RATIO; i++)
          {
            // Convert back to 13-bit audio (0-4095,-2047->2047)
            sample = audio_buffer[i] * 2048;
            MODEM_DECODE(sample, mVrms);
          }
        }
        else
        {
          tp->cdt = false;
        }
        // digitalWrite(2, LOW);
      }
    }
  }
}
