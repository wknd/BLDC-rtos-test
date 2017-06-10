
#ifndef _POSITIONTH_H
#include "../include/PositionSense.h"
#endif

constexpr SPIConfig PositionSense::sense_spicfg;


PositionSense::PositionSense() {

}


void PositionSense::main(void) {
  palSetPadMode(GPIOB, 3, PAL_MODE_ALTERNATE(5) |
                        PAL_STM32_OSPEED_HIGHEST);       /* New SCK.     */
  palSetPadMode(GPIOB, 4, PAL_MODE_ALTERNATE(5) |
                        PAL_STM32_OSPEED_HIGHEST);       /* New MISO.    */
  palSetPadMode(GPIOB, 5, PAL_MODE_ALTERNATE(5) |
                        PAL_STM32_OSPEED_HIGHEST);       /* New MOSI.    */
  palSetPadMode(GPIOA, 10, PAL_MODE_OUTPUT_PUSHPULL |
                        PAL_STM32_OSPEED_HIGHEST);       /* New CS.      */
  palSetPad(GPIOB, 10);

  spiAcquireBus(&SPID1);


  while (true) {
      chThdSleepMilliseconds(300);
  }
}
