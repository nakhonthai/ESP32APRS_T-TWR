#include <string.h>
#include "AFSK.h"
#include "Arduino.h"
#include <Wire.h>
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include "cppQueue.h"
#include "ButterworthFilter.h"
#include "fir_filter.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/sigmadelta.h"

extern "C"
{
#include "soc/syscon_reg.h"
#include "soc/syscon_struct.h"
}

#define DEBUG_TNC

extern unsigned long custom_preamble;
extern unsigned long custom_tail;
int adcVal;

bool input_HPF = false;
bool input_BPF = false;

static const adc_unit_t unit = ADC_UNIT_1;

void sample_isr();
bool hw_afsk_dac_isr = false;

static filter_t bpf;
static filter_t lpf;
static filter_t hpf;

Afsk *AFSK_modem;

#define ADC_RESULT_BYTE 4
#define ADC_CONV_LIMIT_EN 0

static bool check_valid_data(const adc_digi_output_data_t *data);

#define TIMES 1920
#define GET_UNIT(x) ((x >> 3) & 0x1)
static uint16_t adc1_chan_mask = BIT(0);
static uint16_t adc2_chan_mask = 0;
static adc_channel_t channel[1] = {(adc_channel_t)ADC1_CHANNEL_0};
static const char *TAG = "--(TAG ADC DMA)--";
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2

static void continuous_adc_init(uint16_t adc1_chan_mask, uint16_t adc2_chan_mask, adc_channel_t *channel, uint8_t channel_num)
{
  // channel[0] = (adc_channel_t)digitalPinToAnalogChannel(1);
  log_d("adc_digi_initialize");

  adc_digi_init_config_t adc_dma_config = {
      .max_store_buf_size = 8 * TIMES,
      .conv_num_each_intr = TIMES,
      .adc1_chan_mask = adc1_chan_mask,
      .adc2_chan_mask = adc2_chan_mask,
  };

  ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

  log_d("adc_digi_config");
  adc_digi_configuration_t dig_cfg = {
      .conv_limit_en = ADC_CONV_LIMIT_EN,
      .conv_limit_num = 250,
      .sample_freq_hz = SAMPLERATE,
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,   ////ESP32 only supports ADC1 DMA mode
      .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2 // ADC_DIGI_OUTPUT_FORMAT_TYPE2,
  };

  adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
  dig_cfg.pattern_num = channel_num;

  log_d("adc_digi_controller");
  int i = 0;
  for (int i = 0; i < channel_num; i++)
  {
    uint8_t unit = GET_UNIT(channel[i]);
    uint8_t ch = channel[i] & 0x7;
    adc_pattern[i].atten = ADC_ATTEN_DB_11;
    adc_pattern[i].channel = ch;
    adc_pattern[i].unit = unit;
    adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH; // 11 data bits limit

    ESP_LOGI(TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
    ESP_LOGI(TAG, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
    ESP_LOGI(TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
  }
  dig_cfg.adc_pattern = adc_pattern;
  ESP_ERROR_CHECK(adc_digi_controller_configure(&dig_cfg));
}

#if !CONFIG_IDF_TARGET_ESP32
static bool check_valid_data(const adc_digi_output_data_t *data)
{
  const unsigned int unit = data->type2.unit;
  if (unit > 0)
    return false;
  if (data->type2.channel > 0)
    return false;
  return true;
}
#endif


esp_err_t adc_init()
{
  esp_err_t error_code;

  continuous_adc_init(adc1_chan_mask, adc2_chan_mask, channel, sizeof(channel) / sizeof(adc_channel_t));
  // continuous_adc_init(0, 0, channel, 1);
  //  error_code = adc1_config_width(ADC_WIDTH_BIT_12);
  //  log_d("adc1_config_width error_code=%d", error_code);

  // if (error_code != ESP_OK)
  // {
  //   return error_code;
  // }
  adc_digi_start();

  return ESP_OK;
}

void adcActive(bool sts)
{
  if (sts){
    //adc_init();
    adc_digi_start();
    hw_afsk_dac_isr=0;
  }else{
    adc_digi_stop();
    //adc_digi_deinitialize();
  }
}

int IRAM_ATTR read_adc_dma(uint32_t *ret_num, uint8_t *result)
{
  esp_err_t ret;
  ret = adc_digi_read_bytes(result, TIMES, ret_num, ADC_MAX_DELAY);
  return ret;
}

uint8_t CountOnesFromInteger(uint8_t value)
{
  uint8_t count;
  for (count = 0; value != 0; count++, value &= value - 1)
    ;
  return count;
}

#define IMPLEMENTATION FIFO

cppQueue adcq(sizeof(int16_t), 19200, IMPLEMENTATION); // Instantiate queue

// Forward declerations
int afsk_getchar(void);
void afsk_putchar(char c);

hw_timer_t *timer = NULL;

void AFSK_TimerEnable(bool sts)
{
  if (timer == NULL)
    return;
  if (sts == true)
    timerAlarmEnable(timer);
  else
    timerAlarmDisable(timer);
}

Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, PIXELS_PIN, NEO_GRB + NEO_KHZ800);

void LED_Color(uint8_t r, uint8_t g, uint8_t b)
{
  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();
}

bool getTransmit()
{
  bool ret=false;
  if (digitalRead(PTT_PIN) == 0)
    ret = true;
  if(hw_afsk_dac_isr) ret=true;
  return ret;
}

bool getReceive()
{
  bool ret=false;
  if (digitalRead(PTT_PIN) == 0) return true; //PTT Protection receive
  if(AFSK_modem->hdlc.receiving==true) ret=true;
  return ret;
}

void afskSetHPF(bool val)
{
  input_HPF=val;
}
void afskSetBPF(bool val)
{
  input_BPF=val;
}

uint32_t ret_num;
uint8_t *resultADC;

void AFSK_hw_init(void)
{
  // Set up ADC
  log_d("AFSK hardware Initialize");
  // pinMode(RSSI_PIN, INPUT_PULLUP);
  pinMode(PTT_PIN, OUTPUT);
  pinMode(17, OUTPUT); // MIC_SEL
  pinMode(18, OUTPUT); // ESP2MIC
  pinMode(1, ANALOG);  // AUDIO2ESP

  digitalWrite(17, HIGH);
  digitalWrite(PTT_PIN, HIGH); // PTT not active

  resultADC = (uint8_t *)malloc(TIMES * sizeof(uint8_t));

  adc_init();

  sigmadelta_config_t sdm_config{
      .channel = SIGMADELTA_CHANNEL_0,
      .sigmadelta_duty = 127,    /*!< Sigma-delta duty, duty ranges from -128 to 127. */
      .sigmadelta_prescale = 96, /*!< Sigma-delta prescale, prescale ranges from 0 to 255. */
      .sigmadelta_gpio = MIC_PIN};
  sigmadelta_config(&sdm_config);
  // Sample the play the audio on the DAC.
  timer = timerBegin(0, 4, true);                                                // 80MHz%4 = 20MHz hardware clock
  timerAttachInterrupt(timer, &sample_isr, true);                                // Attaches the handler function to the timer
  timerAlarmWrite(timer, (uint64_t)20000000 / CONFIG_AFSK_DAC_SAMPLERATE, true); // Interrupts when counter == 20MHz/CONFIG_AFSK_DAC_SAMPLERATE
  timerAlarmEnable(timer);
  AFSK_TimerEnable(false);
}

void AFSK_init(Afsk *afsk)
{
  log_d("AFSK software Initialize");
  // Allocate modem struct memory
  memset(afsk, 0, sizeof(*afsk));
  AFSK_modem = afsk;
  // Set phase increment
  afsk->phaseInc = MARK_INC;
  // Initialise FIFO buffers
  fifo_init(&afsk->rxFifo, afsk->rxBuf, sizeof(afsk->rxBuf));
  fifo_init(&afsk->txFifo, afsk->txBuf, sizeof(afsk->txBuf));

  // filter initialize
  filter_param_t flt = {
      .size = FIR_LPF_N,
      .sampling_freq = SAMPLERATE,
      .pass_freq = 0,
      .cutoff_freq = 1200,
  };
  int16_t *lpf_an, *bpf_an, *hpf_an;
  // LPF
  lpf_an = filter_coeff(&flt);
  // BPF
  flt.size = FIR_BPF_N;
  flt.pass_freq = 1000;
  flt.cutoff_freq = 2500;
  bpf_an = filter_coeff(&flt);

  // LPF
  filter_init(&lpf, lpf_an, FIR_LPF_N);
  // BPF
  filter_init(&bpf, bpf_an, FIR_BPF_N);

  // HPF
  flt.size = FIR_BPF_N;
  flt.pass_freq = 1000;
  flt.cutoff_freq = 10000;
  hpf_an = filter_coeff(&flt);
  filter_init(&hpf, hpf_an, FIR_BPF_N);

  AFSK_hw_init();
}

static void AFSK_txStart(Afsk *afsk)
{
  if (!afsk->sending)
  {
    fifo_flush(&AFSK_modem->txFifo);
    // Serial.println("TX Start");
    afsk->sending = true;
    afsk->phaseInc = MARK_INC;
    afsk->phaseAcc = 0;
    afsk->bitstuffCount = 0;
    digitalWrite(PTT_PIN, LOW);
    afsk->preambleLength = DIV_ROUND(custom_preamble * BITRATE, 4800);
    AFSK_DAC_IRQ_START();
    // LED_TX_ON();
    AFSK_TimerEnable(true);
  }
  noInterrupts();
  afsk->tailLength = DIV_ROUND(custom_tail * BITRATE, 4800);
  interrupts();
}

void afsk_putchar(char c)
{
  AFSK_txStart(AFSK_modem);
  while (fifo_isfull_locked(&AFSK_modem->txFifo))
  {
    /* Wait */
    // delay(10);
  }
  fifo_push_locked(&AFSK_modem->txFifo, c);
}

int afsk_getchar(void)
{
  if (fifo_isempty_locked(&AFSK_modem->rxFifo))
  {
    return EOF;
  }
  else
  {
    return fifo_pop_locked(&AFSK_modem->rxFifo);
  }
}

void AFSK_transmit(char *buffer, size_t size)
{
  fifo_flush(&AFSK_modem->txFifo);
  int i = 0;
  while (size--)
  {
    afsk_putchar(buffer[i++]);
  }
}

uint8_t AFSK_dac_isr(Afsk *afsk)
{
  if (afsk->sampleIndex == 0)
  {
    if (afsk->txBit == 0)
    {
      if (fifo_isempty(&afsk->txFifo) && afsk->tailLength == 0)
      {
        AFSK_DAC_IRQ_STOP();
        afsk->sending = false;
        return 0;
      }
      else
      {
        if (!afsk->bitStuff)
          afsk->bitstuffCount = 0;
        afsk->bitStuff = true;
        if (afsk->preambleLength == 0)
        {
          if (fifo_isempty(&afsk->txFifo))
          {
            afsk->tailLength--;
            afsk->currentOutputByte = HDLC_FLAG;
          }
          else
          {
            afsk->currentOutputByte = fifo_pop(&afsk->txFifo);
          }
        }
        else
        {
          afsk->preambleLength--;
          afsk->currentOutputByte = HDLC_FLAG;
        }
        if (afsk->currentOutputByte == AX25_ESC)
        {
          if (fifo_isempty(&afsk->txFifo))
          {
            AFSK_DAC_IRQ_STOP();
            afsk->sending = false;
            return 0;
          }
          else
          {
            afsk->currentOutputByte = fifo_pop(&afsk->txFifo);
          }
        }
        else if (afsk->currentOutputByte == HDLC_FLAG || afsk->currentOutputByte == HDLC_RESET)
        {
          afsk->bitStuff = false;
        }
      }
      afsk->txBit = 0x01;
    }

    if (afsk->bitStuff && afsk->bitstuffCount >= BIT_STUFF_LEN)
    {
      afsk->bitstuffCount = 0;
      afsk->phaseInc = SWITCH_TONE(afsk->phaseInc);
    }
    else
    {
      if (afsk->currentOutputByte & afsk->txBit)
      {
        afsk->bitstuffCount++;
      }
      else
      {
        afsk->bitstuffCount = 0;
        afsk->phaseInc = SWITCH_TONE(afsk->phaseInc);
      }
      afsk->txBit <<= 1;
    }

    afsk->sampleIndex = SAMPLESPERBIT;
  }

  afsk->phaseAcc += afsk->phaseInc;
  afsk->phaseAcc %= SIN_LEN;
  if (afsk->sampleIndex > 0)
    afsk->sampleIndex--;

  return sinSample(afsk->phaseAcc);
}

int hdlc_flag_count = 0;
bool hdlc_flage_end = false;
bool sync_flage = false;
static bool hdlcParse(Hdlc *hdlc, bool bit, FIFOBuffer *fifo)
{
  // Initialise a return value. We start with the
  // assumption that all is going to end well :)
  bool ret = true;

  // Bitshift our byte of demodulated bits to
  // the left by one bit, to make room for the
  // next incoming bit
  hdlc->demodulatedBits <<= 1;
  // And then put the newest bit from the
  // demodulator into the byte.
  hdlc->demodulatedBits |= bit ? 1 : 0;

  // Now we'll look at the last 8 received bits, and
  // check if we have received a HDLC flag (01111110)
  if (hdlc->demodulatedBits == HDLC_FLAG)
  {
    // If we have, check that our output buffer is
    // not full.
    if (!fifo_isfull(fifo))
    {
      // If it isn't, we'll push the HDLC_FLAG into
      // the buffer and indicate that we are now
      // receiving data. For bling we also turn
      // on the RX LED.

      hdlc->receiving = true;
      if (++hdlc_flag_count >= 3)
      {
        fifo_flush(fifo);
        LED_RX_ON();
        sync_flage = true;
      }
      fifo_push(fifo, HDLC_FLAG);
    }
    else
    {
      // If the buffer is full, we have a problem
      // and abort by setting the return value to
      // false and stopping the here.

      ret = false;
      hdlc->receiving = false;
      LED_RX_OFF();
      hdlc_flag_count = 0;
      hdlc_flage_end = false;
    }

    // Everytime we receive a HDLC_FLAG, we reset the
    // storage for our current incoming byte and bit
    // position in that byte. This effectively
    // synchronises our parsing to  the start and end
    // of the received bytes.
    hdlc->currentByte = 0;
    hdlc->bitIndex = 0;
    return ret;
  }
  sync_flage = false;

  // Check if we have received a RESET flag (01111111)
  // In this comparison we also detect when no transmission
  // (or silence) is taking place, and the demodulator
  // returns an endless stream of zeroes. Due to the NRZ
  // coding, the actual bits send to this function will
  // be an endless stream of ones, which this AND operation
  // will also detect.
  if ((hdlc->demodulatedBits & HDLC_RESET) == HDLC_RESET)
  {
    // If we have, something probably went wrong at the
    // transmitting end, and we abort the reception.
    hdlc->receiving = false;
    LED_RX_OFF();
    hdlc_flag_count = 0;
    hdlc_flage_end = false;
    return ret;
  }

  // If we have not yet seen a HDLC_FLAG indicating that
  // a transmission is actually taking place, don't bother
  // with anything.
  if (!hdlc->receiving)
    return ret;

  hdlc_flage_end = true;

  // First check if what we are seeing is a stuffed bit.
  // Since the different HDLC control characters like
  // HDLC_FLAG, HDLC_RESET and such could also occur in
  // a normal data stream, we employ a method known as
  // "bit stuffing". All control characters have more than
  // 5 ones in a row, so if the transmitting party detects
  // this sequence in the _data_ to be transmitted, it inserts
  // a zero to avoid the receiving party interpreting it as
  // a control character. Therefore, if we detect such a
  // "stuffed bit", we simply ignore it and wait for the
  // next bit to come in.
  //
  // We do the detection by applying an AND bit-mask to the
  // stream of demodulated bits. This mask is 00111111 (0x3f)
  // if the result of the operation is 00111110 (0x3e), we
  // have detected a stuffed bit.
  if ((hdlc->demodulatedBits & 0x3f) == 0x3e)
    return ret;

  // If we have an actual 1 bit, push this to the current byte
  // If it's a zero, we don't need to do anything, since the
  // bit is initialized to zero when we bitshifted earlier.
  if (hdlc->demodulatedBits & 0x01)
    hdlc->currentByte |= 0x80;

  // Increment the bitIndex and check if we have a complete byte
  if (++hdlc->bitIndex >= 8)
  {
    // If we have a HDLC control character, put a AX.25 escape
    // in the received data. We know we need to do this,
    // because at this point we must have already seen a HDLC
    // flag, meaning that this control character is the result
    // of a bitstuffed byte that is equal to said control
    // character, but is actually part of the data stream.
    // By inserting the escape character, we tell the protocol
    // layer that this is not an actual control character, but
    // data.
    if ((hdlc->currentByte == HDLC_FLAG ||
         hdlc->currentByte == HDLC_RESET ||
         hdlc->currentByte == AX25_ESC))
    {
      // We also need to check that our received data buffer
      // is not full before putting more data in
      if (!fifo_isfull(fifo))
      {
        fifo_push(fifo, AX25_ESC);
      }
      else
      {
        // If it is, abort and return false
        hdlc->receiving = false;
        LED_RX_OFF();
        hdlc_flag_count = 0;
        ret = false;
#ifdef DEBUG_TNC
        Serial.println("FIFO IS FULL");
#endif
      }
    }

    // Push the actual byte to the received data FIFO,
    // if it isn't full.
    if (!fifo_isfull(fifo))
    {
      fifo_push(fifo, hdlc->currentByte);
    }
    else
    {
      // If it is, well, you know by now!
      hdlc->receiving = false;
      LED_RX_OFF();
      hdlc_flag_count = 0;
      ret = false;
#ifdef DEBUG_TNC
      Serial.println("FIFO IS FULL");
#endif
    }

    // Wipe received byte and reset bit index to 0
    hdlc->currentByte = 0;
    hdlc->bitIndex = 0;
  }
  else
  {
    // We don't have a full byte yet, bitshift the byte
    // to make room for the next bit
    hdlc->currentByte >>= 1;
  }

  return ret;
}

#define DECODE_DELAY 4.458981479161393e-4 // sample delay
#define DELAY_DIVIDEND 325
#define DELAY_DIVISOR 728866
#define DELAYED_N ((DELAY_DIVIDEND * SAMPLERATE + DELAY_DIVISOR / 2) / DELAY_DIVISOR)

static int delayed[DELAYED_N];
static int delay_idx = 0;

void AFSK_adc_isr(Afsk *afsk, int16_t currentSample)
{
  /*
   * Frequency discrimination is achieved by simply multiplying
   * the sample with a delayed sample of (samples per bit) / 2.
   * Then the signal is lowpass filtered with a first order,
   * 600 Hz filter. The filter implementation is selectable
   */

  // deocde bell 202 AFSK from ADC value
  int m = (int)currentSample * delayed[delay_idx];
  // Put the current raw sample in the delay FIFO
  delayed[delay_idx] = (int)currentSample;
  delay_idx = (delay_idx + 1) % DELAYED_N;
  afsk->iirY[1] = filter(&lpf, m >> 7);

  // We put the sampled bit in a delay-line:
  // First we bitshift everything 1 left
  afsk->sampledBits <<= 1;
  // And then add the sampled bit to our delay line
  afsk->sampledBits |= (afsk->iirY[1] > 0) ? 1 : 0;

  // We need to check whether there is a signal transition.
  // If there is, we can recalibrate the phase of our
  // sampler to stay in sync with the transmitter. A bit of
  // explanation is required to understand how this works.
  // Since we have PHASE_MAX/PHASE_BITS = 8 samples per bit,
  // we employ a phase counter (currentPhase), that increments
  // by PHASE_BITS everytime a sample is captured. When this
  // counter reaches PHASE_MAX, it wraps around by modulus
  // PHASE_MAX. We then look at the last three samples we
  // captured and determine if the bit was a one or a zero.
  //
  // This gives us a "window" looking into the stream of
  // samples coming from the ADC. Sort of like this:
  //
  //   Past                                      Future
  //       0000000011111111000000001111111100000000
  //                   |________|
  //                       ||
  //                     Window
  //
  // Every time we detect a signal transition, we adjust
  // where this window is positioned little. How much we
  // adjust it is defined by PHASE_INC. If our current phase
  // phase counter value is less than half of PHASE_MAX (ie,
  // the window size) when a signal transition is detected,
  // add PHASE_INC to our phase counter, effectively moving
  // the window a little bit backward (to the left in the
  // illustration), inversely, if the phase counter is greater
  // than half of PHASE_MAX, we move it forward a little.
  // This way, our "window" is constantly seeking to position
  // it's center at the bit transitions. Thus, we synchronise
  // our timing to the transmitter, even if it's timing is
  // a little off compared to our own.
  if (SIGNAL_TRANSITIONED(afsk->sampledBits))
  {
    if (afsk->currentPhase < PHASE_THRESHOLD)
    {
      afsk->currentPhase += PHASE_INC;
    }
    else
    {
      afsk->currentPhase -= PHASE_INC;
    }
  }

  // We increment our phase counter
  afsk->currentPhase += PHASE_BITS;

  // Check if we have reached the end of
  // our sampling window.
  if (afsk->currentPhase >= PHASE_MAX)
  {
    // If we have, wrap around our phase
    // counter by modulus
    afsk->currentPhase %= PHASE_MAX;

    // Bitshift to make room for the next
    // bit in our stream of demodulated bits
    afsk->actualBits <<= 1;

    //// Alternative using 5 bits ////////////////
    uint8_t bits = afsk->sampledBits & 0x1f;
    uint8_t c = CountOnesFromInteger(bits);
    if (c >= 3)
      afsk->actualBits |= 1;
    /////////////////////////////////////////////////

    // Now we can pass the actual bit to the HDLC parser.
    // We are using NRZ coding, so if 2 consecutive bits
    // have the same value, we have a 1, otherwise a 0.
    // We use the TRANSITION_FOUND function to determine this.
    //
    // This is smart in combination with bit stuffing,
    // since it ensures a transmitter will never send more
    // than five consecutive 1's. When sending consecutive
    // ones, the signal stays at the same level, and if
    // this happens for longer periods of time, we would
    // not be able to synchronize our phase to the transmitter
    // and would start experiencing "bit slip".
    //
    // By combining bit-stuffing with NRZ coding, we ensure
    // that the signal will regularly make transitions
    // that we can use to synchronize our phase.
    //
    // We also check the return of the Link Control parser
    // to check if an error occured.

    if (!hdlcParse(&afsk->hdlc, !TRANSITION_FOUND(afsk->actualBits), &afsk->rxFifo))
    {
      afsk->status |= 1;
      if (fifo_isfull(&afsk->rxFifo))
      {
        fifo_flush(&afsk->rxFifo);
        afsk->status = 0;
#ifdef DEBUG_TNC
        Serial.println("FIFO IS FULL");
#endif
      }
    }
  }
}

#define ADC_SAMPLES_COUNT 192 //
int16_t abufPos = 0;
// extern TaskHandle_t taskSensorHandle;

extern void APRS_poll();
uint8_t poll_timer = 0;
// int adc_count = 0;
int offset_new = 0, offset = 2090, offset_count = 0;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
double sinwave;

void IRAM_ATTR sample_isr()
{
  if (hw_afsk_dac_isr)
  {
    portENTER_CRITICAL_ISR(&timerMux);          // ISR start
    sinwave = (double)AFSK_dac_isr(AFSK_modem); // Return sine wave value 0-255
    // Sigma-delta duty of one channel, the value ranges from -128 to 127, recommended range is -90 ~ 90.The waveform is more like a random one in this range.
    int8_t sine = (int8_t)((sinwave - 127) / 1.5); // Redue sine = -85 ~ 85
    sigmadelta_set_duty(SIGMADELTA_CHANNEL_0, sine);

    // End of packet
    if (AFSK_modem->sending == false)
    {
      digitalWrite(PTT_PIN, HIGH);
      AFSK_TimerEnable(false);
      // LED_TX_OFF();
    }
    portEXIT_CRITICAL_ISR(&timerMux); // ISR end
  }
}

bool tx_en = false;

extern int mVrms;
extern float dBV;
extern bool afskSync;

double Vrms = 0;
long mVsum = 0;
int mVsumCount = 0;
long lastVrms = 0;
bool VrmsFlag = false;
bool sqlActive = false;

void AFSK_Poll(bool SA818, bool RFPower)
{
  int mV;
  int x = 0;
  size_t writeByte;
  int16_t adc;

  if (!hw_afsk_dac_isr)
  {
    memset(resultADC, 0xcc, TIMES * sizeof(uint8_t));
    esp_err_t ret = read_adc_dma(&ret_num, &resultADC[0]);
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE)
    {
      // ESP_LOGI("TASK:", "ret is %x, ret_num is %d", ret, ret_num);
      for (int i = 0; i < ret_num; i += ADC_RESULT_BYTE)
      {
        adc_digi_output_data_t *p = (adc_digi_output_data_t *)&resultADC[i];
        if (check_valid_data(p))
        {
          // if(i<100) log_d("ret%x i=%d  Unit: %d,_Channel: %d, Value: %d\n",ret, i, p->type2.unit, p->type2.channel, p->type2.data);
          adcVal = p->type2.data;
          offset_new += adcVal;
          offset_count++;
          if (offset_count >= ADC_SAMPLES_COUNT) // 192
          {
            offset = offset_new / offset_count;
            offset_count = 0;
            offset_new = 0;
            if (offset > 3000 || offset < 1000) // Over dc offset to default
              offset = 2090;
          }
          adcVal -= offset; // Convert unsinewave to sinewave, offset=1.762V ,raw=2090

          if (sync_flage == true)
          {
            mV = (int)((float)adcVal / 1.1862F); // ADC_RAW ADC_ATTEN_DB_11 to mV
            mVsum += powl(mV, 2);                // VRMS = √(1/n)(V1^2 +V2^2 + … + Vn^2)
            mVsumCount++;
          }

          if (input_HPF)
          {
            adcVal = (int)filter(&hpf, (int16_t)adcVal);
          }
          if (input_BPF)
          {
            adcVal = (int)filter(&bpf, (int16_t)adcVal);
          }

          AFSK_adc_isr(AFSK_modem, (int16_t)adcVal); // Process signal IIR
          if (i % PHASE_BITS == 0)
            APRS_poll(); // Poll check every 1 bit
        }
      }
      // Get mVrms on Sync affter flage 0x7E
      // if (afskSync == false)
      if (sync_flage == false)
      {
        if (mVsumCount > 500)
        {
          mVrms = sqrtl(mVsum / mVsumCount); // VRMS = √(1/mVsumCount)(mVsum)
          mVsum = 0;
          mVsumCount = 0;
          lastVrms = millis() + 500;
          VrmsFlag = true;
          // Tool conversion dBv <--> Vrms at http://sengpielaudio.com/calculator-db-volt.htm
          // dBV = 20.0F * log10(Vrms);
          log_d("Audio dc_offset=%d mVrms=%d", offset, mVrms);
        }
        if (millis() > lastVrms && VrmsFlag)
        {
          afskSync = true;
          VrmsFlag = false;
        }
      }
    }
  }
}
