/*
 * test thread, it sets random values to the things we made
 */
#ifndef _TESTER_H
#define _TESTER_H

#ifndef _CH_HPP_
#include "ch.hpp"
#endif

using namespace chibios_rt;

class TesterThread : public BaseStaticThread<128> {
protected:
  virtual void main(void);
  uint16_t phase;
public:
  TesterThread();
  event_source_t tester_event_source;
  uint16_t getPhase();

};


#endif
