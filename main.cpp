/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.hpp"
#include "hal.h"
#include "ch_test.h"

#include <string.h>

#include "library/include/currentControl.h"
#include "library/include/Commands.h"
#include "library/include/tester.h"
#include "library/include/ThreePhaseDriver.h"

using namespace chibios_rt;


/* Static threads instances.*/

static TesterThread testerTh;
static currentControlTh currentTh;
static ControllerTh controlTh(currentTh, testerTh, 0);
static CommandsTh commandsTh(controlTh);
static ThreePhaseDriver threePhaseDriver(controlTh, 0);

static SerialConfig uartCfg =
{
    //115200 // bit rate
    921600
};


/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  System::init();

  /*
   * Activates the serial driver 2 using the driver default configuration.
   * PA2(TX) and PA3(RX) are routed to USART2.
   */
  /* sdInit(); should be used as we see in the SerialDriver state machine
  , but it is already implicitely called in halInit();*/
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7)); // used function : USART2_TX
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7)); // used function : USART2_RX


  palSetPadMode(GPIOC, 3, PAL_MODE_OUTPUT_PUSHPULL);
  //palSetPadMode(GPIOA, GPIOA_LED_GREEN, PAL_MODE_OUTPUT_OPENDRAIN);
  sdStart(&SD2, &uartCfg);
  //BaseSequentialStream * strm = (BaseSequentialStream *)&SD2;
  //char data[] = "Hello World ! \n \r";







  currentTh.start(NORMALPRIO+1);
  testerTh.start(NORMALPRIO-2);
  controlTh.start(NORMALPRIO+2);

  commandsTh.start(NORMALPRIO);
  threePhaseDriver.initDriver();
  //chprintf(strm, "sinit\r\n");
  threePhaseDriver.start(NORMALPRIO);


  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
    if (!palReadPad(GPIOC, GPIOC_BUTTON)) {
      //sdWrite(&SD2, (uint8_t *) data, strlen(data)); // Writes "Hello World in the UART output
      //test_execute((BaseSequentialStream *)&SD2);
        //palTogglePad(GPIOC, 3);
      //palSetPad(GPIOC, 3);
      palSetPad(GPIOA, GPIOA_LED_GREEN);
      threePhaseDriver.disableOutput();
      while (true) {
          if (palReadPad(GPIOC, GPIOC_BUTTON)) {
            threePhaseDriver.enableOutput();
            palSetPad(GPIOA, GPIOA_LED_GREEN);
            //palClearPad(GPIOC, 3);
            break;
          } else
          chThdSleepMilliseconds(300);

      }

    }

    chThdSleepMilliseconds(500);
  }
  return 0;
}
