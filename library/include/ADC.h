/**
 * ADC stuff
 * for our needs we want to constantly take samples
 * take an average of n samples and then notify whoever wants to know about it
 */
#ifndef _ADCMAN_H
#define _ADCMAN_H

#ifndef _CH_HPP_
#include "ch.hpp"
#endif
#ifndef HAL_H
#include "hal.h"
#endif


#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif
#ifndef _ARM_MATH_H
#include "arm_math.h"
#endif
// we need math to have floats :<


#include <algorithm>
#include <array>

using namespace chibios_rt;

#define ADC_CONV_FULLSCALE 4095
#define ADC_CONV_MA 1.2952798432476

// Create buffer to store ADC results. This is
// one-dimensional interleaved array
// we get buf_depth * ch_num samples twice per PWM
#define ADC_BUF_DEPTH 1 // depth of buffer
#define ADC_CH_NUM 6    // number of used ADC channels
#define ADC_BLOCK_SIZE ADC_BUF_DEPTH * ADC_CH_NUM

#define CURRENT_BUFFERS_SIZE 50
#define CURRENT_NUM_BUFFERS 10



// Create buffer of averaged values for mailbox
#define ADC_MB_BUF_DEPTH 200 // how many samples can be stored in the mailbox for threads to read from




class ADCManager {

public:

  //configure the mailbox, remove later? <---------------------------------------
  //static Mailbox<msg_t, ADC_MB_BUF_DEPTH> adcMB;

  // used by the adc driver, writen in C so we use a C style array
  static adcsample_t samples_buf[ADC_BUF_DEPTH * ADC_CH_NUM]; // used by driver to store a series of adc samples

  // this is our internal array, we get to use C++ style arrays
  // used internally to average a couple of samples before passing it on
  static std::array<adcsample_t, (ADC_BLOCK_SIZE * 2)> samples_average_buf; // adc samples get copied to this
  static size_t adcindex; // index of our internal averaging buffer

  // buffers we write the averaged value to, which gets mailed off and processed by someone else
  static ObjectsPool<std::array<float32_t, CURRENT_BUFFERS_SIZE>, CURRENT_NUM_BUFFERS> current_free_buffers;
  static Mailbox<std::array<float32_t, CURRENT_BUFFERS_SIZE>*, CURRENT_NUM_BUFFERS> current_filled_buffers;

  static std::array<float32_t, CURRENT_BUFFERS_SIZE> *current_used_buffer;
  static size_t current_used_buffer_index;

  static enum adcstate_t {
    ADC_UNINIT = 0,                           /**< Not initialized.           */
    ADC_STOP = 1,                             /**< Stopped.                   */
    ADC_READY = 2,                            /**< Ready.                     */
    ADC_ACTIVE = 3,                           /**< Converting.                */
    ADC_COMPLETE = 4,                         /**< Conversion complete.       */
    ADC_ERROR = 5,                            /**< Conversion error.          */
    ADC_NO_BUFF = 6

  } state;


  static event_source_t *current_event_source;


  // no need to convert this float to a more reasonable value you (we can do this after averaging so we onl
  //static void sample2float(float32_t *outBuffer, const uint16_t *inBuffer, uint16_t size);

  static float32_t measurement2MilAmp(float32_t measurement);
  static float32_t measurement2Standard(float32_t measurement);

  static void sortMove(const std::array<adcsample_t, ((ADC_BLOCK_SIZE) * 2)> &input, std::array<adcsample_t, ((ADC_BLOCK_SIZE) * 2)> &output);

  static void startADC(void);
  static void adccb_sin(ADCDriver *adcp, adcsample_t *buffer, size_t n);
  static void adccb_simple(ADCDriver *adcp, adcsample_t *buffer, size_t n);



  ADCManager(event_source *cth);


    /*
     * ADCConversionGroup
     * circular buffer, 1 channel, automagically running
     * channel ADC1_IN0 (pin PA0)
     * based on prescales in mcuconf.h, adc clock is 22.5MHz
     */
  static constexpr ADCConversionGroup adccg_sin = {
      FALSE, //circular buffer
      (uint16_t)(ADC_CH_NUM), // number of channels
      &ADCManager::adccb_sin,
      NULL, // callback  error function, set to NULL for begin
   // Resent fields are stm32 specific. They contain ADC control registers data.
   // Please, refer to ST manual RM0008.pdf to understand what we do. (page 382)
      // CR1 register content, 0 to begin
      0,
      // CR2 register content, start conversion(regular channels), continuous conversion, ADC turned on
      //ADC_CR2_SWSTART | ADC_CR2_CONT | ADC_CR2_ADON,
      ADC_CR2_SWSTART | ADC_CR2_ADON, //not cont, circular buf mode deals with that

      // SMRP1 register content, 0, we're not using the channels in this register
      0,
      // SMRP2 register content, set channel 0 to 56 conversion time =
      // 56 + 12 clocks delay = 68 clocks at 22.5MHz = sample rate of 330.906kHz 6 samples in buffer before we get notified
      // = 110kHz thats way too fast, so many samples for a single pwm period..
      // 112 +12 at 22.5MHz = 181.45kHz / 7 samples = 25.9kHz / 2 for buf size = 12.96kHz per callback (14 samples to average)
      //even better would be interleaving all 3adc's.. but it doesn't like that using the same channel
      //ADC_SMPR2_SMP_AN0(ADC_SAMPLE_112),
      // going with the fastest mode instead, 3 +12 clocks delay at 22.5MHz => getting 6 samples of channel 0 at 250kHz
      // is 4µs to collect all 6 samples
      ADC_SMPR2_SMP_AN0(ADC_SAMPLE_3),
      // SQR1 register content. Set channel sequence length
      ((ADC_CH_NUM - 1) << 20),
      // SQR2 register content, sets the sequence, keep at 0 for only channel 0
      0, //ADC_SQR2_SQ7_N(ADC_CHANNEL_IN0),
      // SQR3 register content. again 0 since we only listen to channel 0
      ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN0) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN0) |
      ADC_SQR3_SQ4_N(ADC_CHANNEL_IN0)| ADC_SQR3_SQ5_N(ADC_CHANNEL_IN0) | ADC_SQR3_SQ6_N(ADC_CHANNEL_IN0),
  };


  /*
   * PC0 = ADC123_IN10
   * PC1 = ADC123_IN11
   * PC2 = ADC123_IN12
   * PC3 = ADC123_IN13
   */
  ADCConversionGroup adccg_simple = {
      FALSE, //circular buffer
      (uint16_t)(ADC_CH_NUM), // number of channels
      &ADCManager::adccb_sin,
      NULL, // callback  error function, set to NULL for begin
   // Resent fields are stm32 specific. They contain ADC control registers data.
   // Please, refer to ST manual RM0008.pdf to understand what we do. (page 382)
      // CR1 register content, 0 to begin
      0,
      // CR2 register content, start conversion(regular channels), continuous conversion, ADC turned on
      //ADC_CR2_SWSTART | ADC_CR2_CONT | ADC_CR2_ADON,
      ADC_CR2_SWSTART | ADC_CR2_ADON, //not cont, circular buf mode deals with that

      // SMRP1 register content, 0, we're not using the channels in this register
      0,
      // SMRP2 register content, set channel 0 to 56 conversion time =
      // 56 + 12 clocks delay = 68 clocks at 22.5MHz = sample rate of 330.906kHz 3 samples in buffer before we get notified
      // = 110kHz thats way too fast, so many samples for a single pwm period..
      // 112 +12 at 22.5MHz = 181.45kHz / 7 samples = 25.9kHz / 2 for buf size = 12.96kHz per callback (14 samples to average)
      //even better would be interleaving all 3adc's.. but it doesn't like that using the same channel
      //ADC_SMPR2_SMP_AN0(ADC_SAMPLE_112),
      ADC_SMPR2_SMP_AN0(ADC_SAMPLE_3),
      // SQR1 register content. Set channel sequence length
      ((ADC_CH_NUM - 1) << 20),
      // SQR2 register content, sets the sequence, keep at 0 for only channel 0
      0, //ADC_SQR2_SQ7_N(ADC_CHANNEL_IN0),
      // SQR3 register content. again 0 since we only listen to channel 0
      ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN0) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN0) |
      ADC_SQR3_SQ4_N(ADC_CHANNEL_IN0)| ADC_SQR3_SQ5_N(ADC_CHANNEL_IN0) | ADC_SQR3_SQ6_N(ADC_CHANNEL_IN0),
  };




};


#endif
