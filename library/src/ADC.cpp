
#ifndef _ADCMAN_H
#include "../include/ADC.h"
#endif

#ifndef _CURRENTCTRLTH_H
#include "../include/currentControl.h"
#endif

//Mailbox<msg_t, ADC_MB_BUF_DEPTH> ADCManager::adcMB;
constexpr ADCConversionGroup ADCManager::adccg_sin;

adcsample_t ADCManager::samples_buf[ADC_BUF_DEPTH * ADC_CH_NUM]; // results array
std::array<adcsample_t, (ADC_BLOCK_SIZE * 2)> ADCManager::samples_average_buf;
//float32_t ADCManager::samples_average[ADC_BLOCK_SIZE * 2]; // adc samples get copied to this
size_t ADCManager::adcindex;


//std::array<std::array<float32_t, CURRENT_BUFFERS_SIZE>, CURRENT_NUM_BUFFERS> ADCManager::current_buffers;
ObjectsPool<std::array<float32_t, CURRENT_BUFFERS_SIZE>, CURRENT_NUM_BUFFERS> ADCManager::current_free_buffers;
//ObjectsPool<std::array<float32_t, CURRENT_BUFFERS_SIZE>, CURRENT_NUM_BUFFERS> ADCManager::current_filled_buffers;
Mailbox<std::array<float32_t, CURRENT_BUFFERS_SIZE>*, CURRENT_NUM_BUFFERS> ADCManager::current_filled_buffers;



std::array<float32_t, CURRENT_BUFFERS_SIZE> *ADCManager::current_used_buffer;
size_t ADCManager::current_used_buffer_index;

ADCManager::adcstate_t ADCManager::state;

event_source_t *ADCManager::current_event_source;




ADCManager::ADCManager(event_source_t *cth) {
  //armFirCoeffFloat[ADC_FIR_NUM_TAPS -1] = 1.0f;
  //chEvtObjectInit(&adc_event_source);
  //ADCManager:currentThread = cth;
  ADCManager::current_event_source = cth;

  // pre filling the available buffer pool, will not work properly if there isn't enough space
  /*for (size_t i = 0; i < CURRENT_NUM_BUFFERS; i++) {
      current_free_buffers.free(&current_buffers[i]);
  }*/
  // this is already done by the objectpool class, magic!

  current_used_buffer = (std::array<float32_t, CURRENT_BUFFERS_SIZE> *) current_free_buffers.alloc();
  if (current_used_buffer != NULL) {
      current_used_buffer_index = 0;
      state = ADC_READY;
      chEvtBroadcastFlags(ADCManager::current_event_source, ADC_EVT_INIT_OK);
      //chEvtBroadcastFlags(&ADCManager::adc_event_source, ADC_EVT_INIT_OK);
  } // we just filled that buffer, so we should be able to fetch from it.. else things are weird
  else {
      state = ADC_ERROR;
      //chEvtBroadcastFlags(&ADCManager::adc_event_source, ADC_EVT_INIT_FAIL);
      chEvtBroadcastFlags(ADCManager::current_event_source, ADC_EVT_INIT_FAIL);
      //adcMB.reset();
  }


  //arm_fir_init_f32(&filter, ADC_FIR_NUM_TAPS, (float32_t *)&armFirCoeffFloat[0], filterState, blocksize);
  //arm_fir_decimate_init_f32(&filter, ADC_FIR_NUM_TAPS, ADC_FIR_DECIMATION_FACTOR, (float32_t *)&armFirCoeffFloat[0], &filterState[0], blocksize);
  //arm_fir_decimate_init_f32(&filter, ADC_FIR_NUM_TAPS, ADC_FIR_DECIMATION_FACTOR, (float32_t *)&armFirCoeffFloat[0], &filterState[0], ADC_FIR_BLOCK_SIZE);
}


void ADCManager::startADC(void) {
  palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG); // set pin to be analog
  //palSetPadMode(GPIOC, 3, PAL_MODE_OUTPUT_PUSHPULL);
  adcStart(&ADCD1, NULL);
  //adcStartConversion(&ADCD1, &ADCManager::adccg, samples_buf, ADC_BUF_DEPTH);
}




/*
 * ADC callback, gets called when the adc buffer is full
 *
 * We trigger twice per center aligned PWM period, at the high center and the low center
 * this callback gets called once per measurement series (so 2 times per PWM)
 *
 * We copy the adc buffer and put it in a different (circular) buffer
 * then every 2 full measurements (once per pwm) we copy and sort it
 * we then average the middle values (thus ignoring (all) the peaks)
 *
 * Example:
 *    1         2        ->     3               ->
 * [1,4,8,3][3,0,99,3]       [0,1,3,3,3,4,8,99]    (3+3+3+4)/4 = 3.25
 * [6,3,9,1][1,4,8,3]        [1,1,3,3,4,6,8,9]     (3+3+4+6)/4 = 4
 * [9,8,2,4][6,3,9,1]        [1,2,3,4,6,8,9,9]     (3+4+6+8)/4 = 5.25
 *
 * our result is now a nice average sample that we get twice per pwm and can do more filtering on later
 * this average signal is only half a pwm period delayed from when it was measured
 *
 * By sorting each series of measurements, we can later sort faster
 * and we can convert to floats at this point
 * [1,3,4,8][0,3,3,99]       [0,1,3,3,3,4,8,99]    (3+3+3+4)/4 = 3.25
 * [1,3,6,9][1,3,4,8]        [1,1,3,3,4,6,8,9]     (3+3+4+6)/4 = 4
 * [2,4,8,9][1,3,6,9]        [1,2,3,4,6,8,9,9]     (3+4+6+8)/4 = 5.25
 *
 * NOTE: callback is in ISR context, be fast and pass on the message
 */
void ADCManager::adccb_sin(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
  (void) adcp;
  (void) n;
  //static adcsample_t tempBuffer1[((ADC_BLOCK_SIZE) * 2)];
  static std::array<adcsample_t, ((ADC_BLOCK_SIZE) * 2)> tempBuffer;

  eventflags_t flagsOut = 0;


  float32_t average;


  if (state == ADC_READY) {
      System::lockFromIsr();

      state = ADC_ACTIVE;
      /* Initialize input and output buffer pointers */
      // buf = block * 2 ->
      // index = 0 -> 0 + block
      // index = block -> (block + block) % block = 0
      adcindex = (adcindex + ADC_BLOCK_SIZE) % (ADC_BLOCK_SIZE * 2);
      //palSetPad(GPIOC, 3);
      std::sort(&buffer[0], &buffer[ADC_BLOCK_SIZE]);
      palClearPad(GPIOC, 3);
      // by sorting here, we have to sort less later
      // but still wish there was a faster sort than this.. if I don't allow folks to set size with #define and stuff
      // then maybe I could use a network sort of some kind?

      // copy to the circular buffer
      std::copy(&buffer[0], &buffer[ADC_BLOCK_SIZE], samples_average_buf.begin() + adcindex);

      //tempBuffer = samples_average_buf; //should copy the content, not just a reference
      //std::sort(tempBuffer.begin(), tempBuffer.end());
      // sort is slow and jittery, maybe a sorting network instead? or something else??
      // this sort that also moves and assumes each half is sorted saves us a lot of time

      sortMove(samples_average_buf, tempBuffer);


      // start halfway 1 block (buffer is 2 blocks large) and go to the other half of a block
      // the spikes (low and high) in the buffer are hopefully ignored this way
      average = 0;
      for (size_t i = ((ADC_BLOCK_SIZE) / 2); i < ((ADC_BLOCK_SIZE) / 2) + (ADC_BLOCK_SIZE); i++) {
          average += static_cast<float32_t>(tempBuffer[i]);
      }

      average = average / (ADC_BLOCK_SIZE);
      average = ADCManager::measurement2MilAmp(average);
      (*current_used_buffer)[current_used_buffer_index] = average;
      current_used_buffer_index++;

      if (current_used_buffer_index >= CURRENT_BUFFERS_SIZE) {
          current_used_buffer_index = 0;
          current_filled_buffers.postI(current_used_buffer);
          // mailbox should never be full since its the same size as our available buffer
          //current_filled_buffers.freeI(current_used_buffer); // free the filled buffer into the other buffer
          flagsOut |= ADC_EVT_FILLEDBLOCK;


          //then fetch a new buffer!
          current_used_buffer = (std::array<float32_t, CURRENT_BUFFERS_SIZE> *) current_free_buffers.allocI();
          if (current_used_buffer != NULL) {
              state = ADC_READY;
              // maybe we set a state here or something?
          } // we just filled that buffer, so we should be able to fetch from it.. else things are weird
          else {
              flagsOut |= ADC_EVT_NOBUFFER;
              state = ADC_NO_BUFF;
              //adcMB.reset();
          }
      } else {
          state = ADC_READY;
      }

      if (flagsOut) {
          // if we set any flags, broadcast them now
         //chEvtBroadcastFlagsI(&ADCManager::adc_event_source, flagsOut);
         chEvtBroadcastFlagsI(ADCManager::current_event_source, flagsOut);

      }

      System::unlockFromIsr();



  }

}

/**
 * takes a buffer where each half is already sorted, and sorts the rest of it into a new buffer
 * we do not want to sort in place since we need the original order for later
 * however, by sorting to a new place we can save a tiny bit of time on having to move before sorting
 * and by using a regular sort on half of the data when it is captured, we should need less time to sort the whole
 *
 * we use this only on things with a fixed size, so we use std::array
 *
 * [ 5 8 12 15 | 6 7 15 17 ]
 */
void ADCManager::sortMove(const std::array<adcsample_t, ((ADC_BLOCK_SIZE) * 2)> &input, std::array<adcsample_t, ((ADC_BLOCK_SIZE) * 2)> &output) {
  size_t j = ADC_BLOCK_SIZE;
  size_t i = 0;
  for (size_t l = 0; l < ((ADC_BLOCK_SIZE) * 2); l++) {
      if (input[i] < input[j]) {
        output[l] = input[i];
        i++;
      }
      else {
        output[l] = input[j];
        j++;
      }
  }
}

/*
 * returns a measurement in milliamp (mA)
 */
float32_t ADCManager::measurement2MilAmp(float32_t measurement) {
  return measurement * ADC_CONV_MA;
}
/*
 * returns a measurement that will be between 0 and 1, with 1 being the full scale of the ADC
 */
float32_t ADCManager::measurement2Standard(float32_t measurement) {
  return measurement / ADC_CONV_FULLSCALE;
}

