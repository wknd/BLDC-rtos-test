#ifndef _COMMANDSTH_H
#define _COMMANDSTH_H

#ifndef _CH_HPP_
#include "ch.hpp"
#endif
#ifndef HAL_H
#include "hal.h"
#endif
#ifndef CHPRINTF_H
#include "chprintf.h"
#endif
#ifndef _CONTROLLERTH_H
#include "Controller.h"
#endif


using namespace chibios_rt;




class CommandsTh : public BaseStaticThread<256> {

public:
  CommandsTh(ControllerTh &cTh);

protected:
  virtual void main(void);

  ControllerTh &controllerTh;

};



#endif
