#ifndef _CONTROLLERTH_H
#define _CONTROLLERTH_H

#ifndef _CH_HPP_
#include "ch.hpp"
#endif
#ifndef HAL_H
#include "hal.h"
#endif
#ifndef CHPRINTF_H
#include "chprintf.h"
#endif
#ifndef _CURRENTCTRLTH_H
#include "currentControl.h"
#endif
#ifndef _TESTER_H
#include "tester.h"
#endif
#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif
#ifndef _ARM_MATH_H
#include "arm_math.h"
#endif



#define CONTROLLER_EVT_NEWAMP     EVENT_MASK(0)
#define CONTROLLER_EVT_NEWPHASE   EVENT_MASK(1)

// the event mask used to identify an event comming from the current thread
#define CURRENT_EVENT   EVENT_MASK(0)
#define TESTER_EVENT    EVENT_MASK(1)

#define CURRENT_PID_KP 0.2 //0.132987 // 0.02285 //2.285
#define CURRENT_PID_KI 0.1
#define CURRENT_PID_KD 0.0

class ControllerTh : public BaseStaticThread<512> {

protected:
  currentControlTh &currentThread;
  TesterThread &testerThread;

  // current control related stuff
  arm_pid_instance_f32 curPID;
  float32_t currentSetPoint;
  float32_t setAmplitude; // changed to float for more correct calculations later
  uint16_t setPhase;
  uint16_t mode;

  virtual void main(void);

public:
  ControllerTh(currentControlTh &cur, TesterThread &t, uint16_t mode);
  event_source controller_event_source;
  void setCurrent(float32_t cur);
  float32_t getAmplitude();
  uint16_t getPhase();

};



#endif
