
#ifndef _CONTROLLERTH_H
#include "../include/Controller.h"
#endif




ControllerTh::ControllerTh(currentControlTh &cur, TesterThread &t, uint16_t _mode) : currentThread(cur), testerThread(t),
                                          currentSetPoint(100.0), setAmplitude(0), setPhase(0), mode(_mode) {
  curPID.Kp = CURRENT_PID_KP;
  curPID.Ki = CURRENT_PID_KI;
  curPID.Kd = CURRENT_PID_KD;
  arm_pid_init_f32(&curPID, 1);

}
void ControllerTh::setCurrent(float32_t cur) {
  syssts_t sts = chSysGetStatusAndLockX(); // could be called from anywhere, lock while we're writing the data
  currentSetPoint = cur;
  chSysRestoreStatusX(sts);
}

float32_t ControllerTh::getAmplitude() {
  return setAmplitude;
}
uint16_t ControllerTh::getPhase() {
  return setPhase;
}


void ControllerTh::main(void) {

  chEvtObjectInit(&controller_event_source);
  float32_t currentError = 0;
  event_listener_t current_listener;
  event_listener_t tester_listener;
  eventmask_t listenmask = CURRENT_EVENT | TESTER_EVENT; // masks to wait for
  eventflags_t outputFlags; // event flags we will be emitting

  eventmask_t evt; // the currently triggered events (mask of all the events that triggered)
  eventflags_t  flags; // used to store the flags that come with the relevant event

  // register to listen for certain events
  chEvtRegisterMaskWithFlags(&currentThread.current_event_source, &current_listener, CURRENT_EVENT, CURRENT_EVT_UPDATEAMPLITUDE | CURRENT_EVT_ERROR);
  chEvtRegisterMask(&testerThread.tester_event_source, &tester_listener, TESTER_EVENT);
  // each source needs a different listener and we need to register it

  if (mode == 1) {
      // startup mode first
      setAmplitude = 400;
      setPhase = 0;
      uint16_t delay = 10000;
      chThdSleepMicroseconds(delay);
      while (delay >= 2500) {
          chThdSleepMicroseconds(delay);
          setPhase++;
          if (setPhase == 6) {
              delay = delay - 25;
              setPhase = 0;

          }
          chEvtBroadcastFlags(&controller_event_source, CONTROLLER_EVT_NEWPHASE | CONTROLLER_EVT_NEWAMP);

      }
      chThdSleepMicroseconds(delay); //extra delay since the next loop will instantly trigger
      mode = 2;
  }
  palSetPad(GPIOA, GPIOA_LED_GREEN);


  while (true) {
      evt = waitAnyEvent(listenmask);//listenmask);//chEvtWaitAny(listenmask);
      outputFlags = 0;
      if (evt & CURRENT_EVENT) {
          // current thread sent us something
          flags = chEvtGetAndClearFlags(&current_listener);
          if (flags & CURRENT_EVT_UPDATEAMPLITUDE) {
              // current thread has a new filtered amplitude, lets do stuff!
              if (mode == 0) {
              System::lock();
                currentError = currentSetPoint - currentThread.getCurrent();
                if ((currentError < 0 && setAmplitude > 0) || (currentError > 0 && setAmplitude < 512))
                {   // only do the calculations if we haven't already reached our limits
                    setAmplitude = arm_pid_f32(&curPID, currentError);
                    if (setAmplitude > 512)
                        setAmplitude = 512;
                    else if (setAmplitude < 0)
                        setAmplitude = 0;
                }

                System::unlock();
              } else {
                 // setAmplitude = 350;
              }
              outputFlags |= CONTROLLER_EVT_NEWAMP;

          }
      }
      if (evt & TESTER_EVENT) {
          flags = chEvtGetAndClearFlags(&tester_listener);
          // tester has set a new phase, lets tell the motor driver about it
          // later this won't be the tester thread but we'll use some other logic based on the position or something
          System::lock();
          if (mode == 0) {
            setPhase = testerThread.getPhase();
            if (setPhase > 720)
              setPhase = 720;
            outputFlags |= CONTROLLER_EVT_NEWPHASE;
          } else {
              // in normal mode, ignore the phase set in test thread
              setPhase = (setPhase + 1) % 6;
              outputFlags |= CONTROLLER_EVT_NEWPHASE;
          }
          System::unlock();
      }

      if (outputFlags) { //if we have anything to say, say it now
          chEvtBroadcastFlags(&controller_event_source, outputFlags);

      }


  }
}
