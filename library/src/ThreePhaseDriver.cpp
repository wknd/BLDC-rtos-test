
#ifndef _THREEPHASEDRIVER_H
#include "../include/ThreePhaseDriver.h"
#endif


ThreePhaseDriver::ThreePhaseDriver(ControllerTh &cth, uint16_t _mode) : controllerTh(cth), mode(_mode) {


}


void ThreePhaseDriver::enableOutput(void) {
  // could get called in all sorts of contexts, so store the last state before lock
  syssts_t sts = chSysGetStatusAndLockX();
  //enable channels again with with = 0, whatever is controlling the pwm will set the proper value soon after
  //alternatively we could set it to the previous values.. but this seems safer
  if (mode == THREEPHASE_MODE_SIN)
  {
    pwmEnableChannelI(&PWMD8, 0, 0);
    pwmEnableChannelI(&PWMD8, 2, 0);
    pwmEnableChannelI(&PWMD8, 3, 0);
    //msg the thread?

    palSetPad(GPIOA, 6);
    palSetPad(GPIOA, 7);
    palSetPad(GPIOB, 1);
  }
  chSysRestoreStatusX(sts);

}

void ThreePhaseDriver::disableOutput(void) {
  // could get called in all sorts of contexts, so store the last state before lock
  syssts_t sts = chSysGetStatusAndLockX();
  //chSysLock();
  pwmDisableChannelI(&PWMD8, 0);
  pwmDisableChannelI(&PWMD8, 2);
  pwmDisableChannelI(&PWMD8, 3);
  // clearing enable pins for motor driver


  palClearPad(GPIOA, 6);
  palClearPad(GPIOA, 7);
  palClearPad(GPIOB, 1);


  //palClearPad(GPIOA, 5);
  //palClearPad(GPIOA, 6);
  //palClearPad(GPIOA, 7);
  //msg the thread?
  //chSysUnlock();
  chSysRestoreStatusX(sts);

}

void ThreePhaseDriver::sinCallBack(PWMDriver *pwmp) {
  (void)pwmp;
  System::lockFromIsr();
  //chSysLockFromISR();

  //if (((&PWMD8)->tim->CR1 & STM32_TIM_CR1_DIR )) {//we're now counting up, so we're at the middle!
    palTogglePad(GPIOC, 3);
    adcStartConversionI(&ADCD1, &ADCManager::adccg_sin, ADCManager::samples_buf, ADC_BUF_DEPTH);

  //}
    //palClearPad(GPIOC, 3);
  /*else
    palSetPad(GPIOC, 3); */

  //palTogglePad(GPIOC, 3);
  System::unlockFromIsr();
  //chSysUnlockFromISR();
}

/**
 * not center aligned pwm unlike before, so we need another way to avoid the edges for the ADC
 * However if I make it too smart it can't be used as backEMF detector for another project I'm doing
 * so maybe only make the current sensing smart, and try to make backEMF detection free running?
 * Then  we need to use 2 independent ADC's (we have 3 available)
 *
 * However backEMF sensing also needs to be synchronised to the pwm
 * so I'd better create a separate project for this and leave it open loop for now
 *
 */
void ThreePhaseDriver::simpleCallBack(PWMDriver *pwmp) {
  (void)pwmp;
  System::lockFromIsr();
  //chSysLockFromISR();

  //if (((&PWMD8)->tim->CR1 & STM32_TIM_CR1_DIR )) {//we're now counting up, so we're at the middle!
    palSetPad(GPIOC, 3);
    adcStartConversionI(&ADCD1, &ADCManager::adccg_sin, ADCManager::samples_buf, ADC_BUF_DEPTH);

  //}
    //palClearPad(GPIOC, 3);
  /*else
    palSetPad(GPIOC, 3); */

  //palTogglePad(GPIOC, 3);
  System::unlockFromIsr();
  //chSysUnlockFromISR();
}



void ThreePhaseDriver::main(void) {
  chRegSetThreadName("Pwm");
  event_listener_t controller_listener;
  eventmask_t listenmask = CONTROLLER_EVENT;

  eventmask_t evt; // the currently triggered events (mask of all the events that triggered)
  eventflags_t  flags; // used to store the flags that come with the relevant event

  chEvtRegisterMaskWithFlags(&(controllerTh.controller_event_source), &controller_listener, CONTROLLER_EVENT, CONTROLLER_EVT_NEWAMP | CONTROLLER_EVT_NEWPHASE);


	while (true) {
		//ThreadReference tr = this->waitMessage();
	  evt = chEvtWaitAny(listenmask);
	  System::lock();
	  if (evt & CONTROLLER_EVENT) {
      // controller said something

      flags = chEvtGetAndClearFlagsI(&controller_listener);
      if (flags & CONTROLLER_EVT_NEWAMP) {
          currentAmplitude = controllerTh.getAmplitude();
      }
      if (flags & CONTROLLER_EVT_NEWPHASE) {
          currentPhase = controllerTh.getPhase();
      }
      // do I want to do the calculations outside of a lock?
      // if I do, something else could happen while doing the calculations, which is maybe okay?
      // however, then I'm doing the calculations all the time even when the pwm is disabled
      // if I don't I spend more time in a locked state..
      if (mode == THREEPHASE_MODE_SIN) {
          if (pwmIsChannelEnabledI(&PWMD8, 0) && pwmIsChannelEnabledI(&PWMD8, 0) && pwmIsChannelEnabledI(&PWMD8, 0)) {
                  pwmEnableChannelI(&PWMD8, 0, ((sinlookup[currentPhase % THREEPHASE_STEPS_PER_CYCLE] * currentAmplitude)/10000));
                  pwmEnableChannelI(&PWMD8, 2, ((sinlookup[(currentPhase + THREEPHASE_STEPS_PER_CYCLE/3) % THREEPHASE_STEPS_PER_CYCLE] * currentAmplitude)/10000));
                  pwmEnableChannelI(&PWMD8, 3, ((sinlookup[(currentPhase + 2*(THREEPHASE_STEPS_PER_CYCLE/3)) % THREEPHASE_STEPS_PER_CYCLE] * currentAmplitude)/10000));
          }
      } else if (mode == THREEPHASE_MODE_NORMAL) {
          changeNormalState();
      }


	  }
	  System::unlock();
  }

}

// called from locked state!
void ThreePhaseDriver::changeNormalState(void) {
  // we only have 6 phases in normal mode

  // first a little cleanup to make sure we're not going to set impossible states
  if (currentPhase >= 6) {
      currentPhase = currentPhase % 6;
  }
  if (currentAmplitude > 512)
    currentAmplitude = 512;
  if (currentAmplitude < 0)
    currentAmplitude = 0;

  uint16_t realAmp = (currentAmplitude * 7);


  //disableOutput();
  int16_t subState = currentPhase / 2;
  pwmDisableChannelI(&PWMD3, 0);
  pwmDisableChannelI(&PWMD3, 1);
  pwmDisableChannelI(&PWMD3, 3);

  //it takes at least 4µs to measure all the samples for the current sensing
  //so make sure the 2 times we're sampling don't overlap
  // a single PWM clock takes 11ns
  // so we need at least 364 (out of 3600) clocks to do a measurement
  if (realAmp > (3600/2)) // over 50% duty cycle, pick a middle inside the active area
    pwmEnableChannelI(&PWMD3, 2, realAmp/2 );
  else // under 50% duty cycle, pick a middle when the pwm is turned off
    pwmEnableChannelI(&PWMD3, 2, realAmp + (3600-realAmp) / 2);

  switch(subState) { //setting the input pins to the proper values
    case 0:
      palSetPad(GPIOC, 6);
      palClearPad(GPIOC, 8);
      palClearPad(GPIOC, 9);
      break;
    case 1:
      palClearPad(GPIOC, 6);
      palSetPad(GPIOC, 8);
      palClearPad(GPIOC, 9);
      break;
    case 2:
      palClearPad(GPIOC, 6);
      palClearPad(GPIOC, 8);
      palSetPad(GPIOC, 9);
      break;
  }

  switch(currentPhase % 3) { //setting the enable pins to the proper values
    case 0:
      pwmEnableChannelI(&PWMD3, 0, (currentAmplitude * 7)); //enable 1
      pwmEnableChannelI(&PWMD3, 1, (currentAmplitude * 7)); //enable 2
      pwmDisableChannelI(&PWMD3, 3); //disable 3
      break;
    case 1:
      pwmEnableChannelI(&PWMD3, 0, (currentAmplitude * 7)); //enable 1
      pwmDisableChannelI(&PWMD3, 1); //disable 2
      pwmEnableChannelI(&PWMD3, 3, (currentAmplitude * 7)); //enable 3
      break;
    case 2:
      pwmDisableChannelI(&PWMD3, 0); //disable 1
      pwmEnableChannelI(&PWMD3, 1, (currentAmplitude * 7)); //enable 2
      pwmEnableChannelI(&PWMD3, 3, (currentAmplitude * 7)); //enable 3
      break;

  }
/*
 * This could potentially be more efficient,
 * but if we do this our zero V reference is no longer in the middle of our signal,
 * that would make all the math so much harder
  switch (currentPhase) {
    case 0:
      // direction
      palSetPad(GPIOC, 6);
      palClearPad(GPIOC, 8);
      palClearPad(GPIOC, 9);
      // enabled or not
      pwmEnableChannelI(&PWMD3, 0, (currentAmplitude * 7)); //enable 1
      pwmEnableChannelI(&PWMD3, 1, 3600); //enable 2 always
      pwmDisableChannelI(&PWMD3, 3); //disable 3
      break;
    case 1:
      // direction
      palSetPad(GPIOC, 6);
      palSetPad(GPIOC, 8);
      palClearPad(GPIOC, 9);
      // enabled or not
      pwmEnableChannelI(&PWMD3, 0, (currentAmplitude * 7)); //enable 1
      pwmDisableChannelI(&PWMD3, 1); //disable 2
      pwmEnableChannelI(&PWMD3, 3, 3600); //enable 3 always
      break;
    case 2:
      // direction
      palClearPad(GPIOC, 6);
      palSetPad(GPIOC, 8);
      palClearPad(GPIOC, 9);
      // enabled or not
      pwmDisableChannelI(&PWMD3, 0); //disable 1
      pwmEnableChannelI(&PWMD3, 1, (currentAmplitude * 7)); //enable 2
      pwmEnableChannelI(&PWMD3, 3, 3600); //enable 3 always
      break;
    case 3:
      // direction
      palClearPad(GPIOC, 6);
      palSetPad(GPIOC, 8);
      palSetPad(GPIOC, 9);
      // enabled or not
      pwmEnableChannelI(&PWMD3, 0, 3600); //enable 1 always
      pwmEnableChannelI(&PWMD3, 1, (currentAmplitude * 7)); //enable 2
      pwmDisableChannelI(&PWMD3, 3); //disable 3
      break;
    case 4:
      // direction
      palClearPad(GPIOC, 6);
      palClearPad(GPIOC, 8);
      palSetPad(GPIOC, 9);
      // enabled or not
      pwmEnableChannelI(&PWMD3, 0, 3600); //enable 1 always
      pwmDisableChannelI(&PWMD3, 1); //disable 2
      pwmEnableChannelI(&PWMD3, 3, (currentAmplitude * 7)); //enable 3
      break;
    case 5:
      // direction
      palSetPad(GPIOC, 6);
      palClearPad(GPIOC, 8);
      palSetPad(GPIOC, 9);
      // enabled or not
      pwmDisableChannelI(&PWMD3, 0); //disable 1
      pwmEnableChannelI(&PWMD3, 1, 3600); //enable 2 always
      pwmEnableChannelI(&PWMD3, 3, (currentAmplitude * 7)); //enable 3
      break;



  }
*/


}



void ThreePhaseDriver::initDriver(void) {

        // sin wave drive
        /*
         * init pwm pins
         */

        //enable pins


        palClearPad(GPIOA, 6);
        palClearPad(GPIOA, 7);
        palClearPad(GPIOB, 1);

        if (mode == THREEPHASE_MODE_SIN) {
            //input pins
            palSetPadMode(GPIOA, 6, PAL_MODE_OUTPUT_PUSHPULL);
            palSetPadMode(GPIOA, 7, PAL_MODE_OUTPUT_PUSHPULL);
            palSetPadMode(GPIOB, 1, PAL_MODE_OUTPUT_PUSHPULL);

            palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(3));
            palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(3));
            palSetPadMode(GPIOC, 9, PAL_MODE_ALTERNATE(3));
            pwmStart(&PWMD8, &(this->pwmcfg));

            (&PWMD8)->tim->CR1 |= STM32_TIM_CR1_CMS(2); //magic to set it in center aligned mode
            pwmEnablePeriodicNotification(&PWMD8);

        } else {
            // normal mode
            palSetPadMode(GPIOC, 6, PAL_MODE_OUTPUT_PUSHPULL);
            palSetPadMode(GPIOC, 8, PAL_MODE_OUTPUT_PUSHPULL);
            palSetPadMode(GPIOC, 9, PAL_MODE_OUTPUT_PUSHPULL);
            palClearPad(GPIOC, 6);
            palClearPad(GPIOC, 8);
            palClearPad(GPIOC, 9);

            palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(2));
            palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(2));
            palSetPadMode(GPIOB, 1, PAL_MODE_ALTERNATE(2));

            pwmStart(&PWMD3, &(this->pwmcfgSimple));
            pwmEnablePeriodicNotification(&PWMD3);
            pwmEnableChannelNotification(&PWMD3, 2); //callback on channel 3
        }


        //pwmEnableChannelNotification(&PWMD8, 0);
        this->enableOutput();




}


