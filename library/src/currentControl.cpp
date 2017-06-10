
#ifndef _CURRENTCTRLTH_H
#include "../include/currentControl.h"
#endif

/*
void currentControlTh::setCurrent(float32_t cur) {
  syssts_t sts = chSysGetStatusAndLockX(); // could be called from anywhere, lock while we're writing the data
  // alternatively we could also use some other locks?
  currentSetPoint = cur;
  chSysRestoreStatusX(sts);
}
*/

float32_t currentControlTh::getCurrent() {
  return filterOutput;
}

currentControlTh::currentControlTh() : filterOutput(0.0) {
  arm_fir_decimate_init_f32(&filter, CURRENT_FIR_NUM_TAPS, ADC_FIR_DECIMATION_FACTOR, (float32_t *)&armFirCoeffFloat[0], filterState, (CURRENT_FIR_BLOCK_SIZE));
  chEvtObjectInit(&current_event_source);
}

void currentControlTh::main(void) {
  ADCManager adc(&current_event_source);
  ADCManager::startADC();
  BaseSequentialStream * strm = (BaseSequentialStream *)&SD2;
  event_listener_t adc_listener;
  eventmask_t listenmask = CURRENT_EVENT;
  eventmask_t outputevents = 0;
  eventflags_t  flags;


  uint8_t data[5];
  data[0] = 'c';

  //chprintf(strm, "iinit\r\n");
  // register for an event
  //chEvtRegisterMaskWithFlags(&(ADCManager::current_event_source), &adc_listener, ADC_EVENT, ADC_EVT_INIT_FAIL | ADC_EVT_INIT_OK | ADC_EVT_FILLEDBLOCK);

  chEvtRegisterMaskWithFlags(&current_event_source, &adc_listener, CURRENT_EVENT, ADC_EVT_INIT_FAIL | ADC_EVT_INIT_OK | ADC_EVT_FILLEDBLOCK);
  chprintf(strm, "iregistered!\r\n");

  if (ADCManager::state == 2) {
      chprintf(strm, "iADC Ready!\r\n");
  }

  while(true) {

      //msg_t measurement;
      //msg_t status = ADCManager::adcMB.fetch(&measurement, MS2ST(CURRENT_TIMEOUT));




      eventmask_t evt = chEvtWaitAny(listenmask);
      //chprintf(strm, "ievent!\r\n");
      if (evt & CURRENT_EVENT) {
          //chprintf(strm, "ievent!\r\n");
          flags = chEvtGetAndClearFlags(&adc_listener);
          if (flags & ADC_EVT_INIT_FAIL) {
              chprintf(strm, "iadc init fail!\r\n");
          }
          if (flags & ADC_EVT_INIT_OK) {
              chprintf(strm, "iadc init!\r\n");
          }
          if (flags & ADC_EVT_NOBUFFER) {
              chprintf(strm, "ino buffer!\r\n");
          }
          if (flags & ADC_EVT_FILLEDBLOCK) {
            // adc has something for us
            // for now, theres no event flags, so lets assume he filled up a buffer

            msg_t resp = ADCManager::current_filled_buffers.fetch(&adc_buffer, TIME_IMMEDIATE);

            //adc_buffer = (std::array<float32_t, CURRENT_BUFFERS_SIZE> *) ADCManager::current_filled_buffers.fetchI();
            while (resp == MSG_OK) {
                outputevents = 0;
                //chprintf(strm, "sblock!\r\n");
                //chprintf(strm, "sblock!\r\n");
                // yup, we got a buffer
                System::lock();
                arm_fir_decimate_f32(&filter, &(*adc_buffer)[0], &filterOutput, CURRENT_FIR_BLOCK_SIZE);

                static union {
                      float32_t f;
                      msg_t m;
                } value;
                value.f = filterOutput;
                ADCManager::current_free_buffers.freeI(adc_buffer);
                resp = ADCManager::current_filled_buffers.fetchI(&adc_buffer);

                chEvtBroadcastFlagsI(&current_event_source, CURRENT_EVT_UPDATEAMPLITUDE);
                System::unlock();
                data[0] = 'c';
                data[4] = (value.m >> 24) & 0xFF;
                data[3] = (value.m >> 16) & 0xFF;
                data[2] = (value.m >> 8) & 0xFF;
                data[1] = value.m & 0xFF;
                sdWrite(&SD2, (uint8_t *) data, 5);


            }
            if (flags & ~(ADC_EVT_INIT_FAIL | ADC_EVT_INIT_OK | ADC_EVT_NOBUFFER | ADC_EVT_FILLEDBLOCK)) {
                //chprintf(strm, "i!\r\n");
            }

            //chprintf(strm, "i!\r\n");

          }


      } //end while


  }


}
