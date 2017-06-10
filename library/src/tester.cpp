
#ifndef _TESTER_H
#include "../include/tester.h"
#endif

/*
 * passing parameter by const reference as it should be apearantly
 */
TesterThread::TesterThread() : phase(0) {
  chEvtObjectInit(&tester_event_source);
}

uint16_t TesterThread::getPhase() {
  return phase;
}

void TesterThread::main(void) {
  setName("tester");

  phase = 0;
  while (true) {
    if (phase >= 720) {
      phase = phase % 720;
    }
    //if (mysetting.amplitude > THREEPHASE_MAX_AMPLITUDE)
      //mysetting.amplitude = mysetting.amplitude % THREEPHASE_MAX_AMPLITUDE;
    chEvtBroadcast(&tester_event_source);
    //threePhaseDriver.sendMessage((msg_t)&mysetting);
    phase = phase + 1;

    chThdSleepMicroseconds(3000);
  }
}
