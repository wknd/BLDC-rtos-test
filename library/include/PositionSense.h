#ifndef _POSITIONTH_H
#define _POSITIONTH_H

#ifndef _CH_HPP_
#include "ch.hpp"
#endif
#ifndef HAL_H
#include "hal.h"
#endif
#ifndef CHPRINTF_H
#include "chprintf.h"
#endif


using namespace chibios_rt;



class PositionSense : public BaseStaticThread<256> {

  /*
   *
   * note this sensor has a sample rate of 10.1kHz
   */
public:
  PositionSense();

  /*
   * MAX 10MHz, MSB first, 16bit
   * Bidirectional mode
   * PCLK (which SPI uses) is set to system clock / 4 (45MHz)
   * devide this further to be under the 10MHz max of the magnetic encoder
   * further deviding by 4 is not enough, so the next step is to devide by 8
   * (so set BR to 010 -> set BR1
   *
   *
   */
  static constexpr SPIConfig sense_spicfg = {
    NULL,
    GPIOA,
    10,
    SPI_CR1_BR_1 | SPI_CR1_DFF | SPI_CR1_MSTR, // CR1: divide PCLK by 8, set to 16bit, master mode(driver will set master mode anyway)
    0 //cr2 on 0
  };

protected:
  virtual void main(void);

};



#endif
