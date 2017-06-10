
#ifndef _COMMANDSTH_H
#include "../include/Commands.h"
#endif


/*
 * Command thread, waits for commands from the serial connection, then does stuff with that information
 */
void CommandsTh::main(void) {
  uint8_t c; //single char buffer, all our commands are 1 char long followed by the actual data
  uint8_t current[4];
  static union {
    float32_t f;
    uint32_t m;
  } value;
  while (true) {
      sdRead(&SD2, &c, 1); //waits FOREVER, or until something comes along
      if (c == 'c') {
          // the desired current follows in the next 4 bytes
          if (sdReadTimeout(&SD2, current, 4, MS2ST(10)) == 4) {
              //if we don't get 4 bytes within the timeframe, something is wrong
              value.m = 0;
              value.m |= current[0] & 0xFF;
              value.m |= ((current[1] & 0xFF) <<8);
              value.m |= ((current[2] & 0xFF) <<16);
              value.m |= ((current[3] & 0xFF) <<24);
              controllerTh.setCurrent(value.f);
              chprintf((BaseSequentialStream *)&SD2, "ievent!\r\n");
          } // else, something is wrong
          else {
              chprintf((BaseSequentialStream *)&SD2, "ifail!\r\n");
          }
      } else if (c == 's') {
          // the desired speed follows in the next 2 bytes
      } else if (c == 'p') {
          // the desired phase location follows in the next 2 bytes
      } else if (c == 'P') {
          // set mode to position control

      } else if (c == 'S') {
          // set mode to speed control
      }

  }

}


CommandsTh::CommandsTh(ControllerTh &cTh) : controllerTh(cTh) {

}
