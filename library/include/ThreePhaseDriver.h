/*
 * PWM thread, it controlls the 3 phases of the motor
 */
#ifndef _THREEPHASEDRIVER_H
#define _THREEPHASEDRIVER_H

#ifndef _CH_HPP_
#include "ch.hpp"
#endif
#ifndef HAL_H
#include "hal.h"
#endif
#ifndef _ADCMAN_H
#include "ADC.h"
#endif
#ifndef _CONTROLLERTH_H
#include "Controller.h"
#endif


using namespace chibios_rt;

#define CONTROLLER_EVENT   EVENT_MASK(0)


/*
struct motorSetting {
  int16_t phase{0};
  int16_t amplitude{0};
  //bool reEnable{false};
 // bool changeEnabled{false};
};
*/


#define THREEPHASE_MAX_AMPLITUDE 512
#define THREEPHASE_STEPS_PER_CYCLE 720

#define THREEPHASE_MODE_SIN 0
#define THREEPHASE_MODE_NORMAL 1





class ThreePhaseDriver : public BaseStaticThread<256> {





public:
	ThreePhaseDriver(ControllerTh &cth, uint16_t _mode);
  void initDriver(void);
  void disableOutput(void);
  void enableOutput(void);

  static void sinCallBack(PWMDriver *pwmp);
  static void simpleCallBack(PWMDriver *pwmp);

protected:
  ControllerTh &controllerTh;
  uint16_t mode;
  virtual void main(void);

  void changeNormalState(void);


  //max value is 7200, computed values are devided by 512 and multiplied with 10000 for easy math
  const uint32_t sinlookup[THREEPHASE_STEPS_PER_CYCLE] = {
      35156, 35463, 35769, 36076, 36383, 36689, 36995, 37301, 37607, 37913, 38219, 38524, 38829, 39134, 39438, 39743, 40046, 40350, 40653, 40955, 41258, 41559, 41861, 42161, 42462, 42761, 43060, 43359, 43657, 43954, 44250, 44546, 44841, 45136, 45429, 45722, 46014, 46305, 46596, 46885, 47174, 47461, 47748, 48034, 48319, 48602, 48885, 49167, 49448, 49727, 50006, 50283, 50559, 50834, 51108, 51381, 51652, 51922, 52191, 52458, 52725, 52989, 53253, 53515, 53776, 54035, 54293, 54550, 54804, 55058, 55310, 55560, 55809, 56056, 56302, 56546, 56789, 57029, 57269, 57506, 57742, 57976, 58208, 58439, 58667, 58894, 59119, 59343, 59564, 59784, 60002, 60218, 60431, 60644, 60854, 61062, 61268, 61472, 61674, 61874, 62073, 62269, 62463, 62655, 62844, 63032, 63218, 63401, 63582, 63762, 63939, 64113, 64286, 64456, 64624, 64790, 64954, 65115, 65274, 65431, 65586, 65738, 65888, 66035, 66180, 66323, 66463, 66601, 66737, 66870, 67001, 67129, 67255, 67379, 67500, 67618, 67734, 67848, 67959, 68068, 68174, 68278, 68379, 68477, 68573, 68667, 68758, 68846, 68932, 69015, 69096, 69174, 69249, 69322, 69392, 69460, 69525, 69588, 69647, 69705, 69759, 69811, 69860, 69907, 69951, 69992, 70031, 70067, 70100, 70131, 70159, 70185, 70207, 70227, 70245, 70260, 70272, 70281, 70288, 70292, 70293, 70292, 70288, 70281, 70272, 70260, 70245, 70227, 70207, 70185, 70159, 70131, 70100, 70067, 70031, 69992, 69951, 69907, 69860, 69811, 69759, 69705, 69647, 69588, 69525, 69460, 69392, 69322, 69249, 69174, 69096, 69015, 68932, 68846, 68758, 68667, 68573, 68477, 68379, 68278, 68174, 68068, 67959, 67848, 67734, 67618, 67500, 67379, 67255, 67129, 67001, 66870, 66737, 66601, 66463, 66323, 66180, 66035, 65888, 65738, 65586, 65431, 65274, 65115, 64954, 64790, 64624, 64456, 64286, 64113, 63939, 63762, 63582, 63401, 63218, 63032, 62844, 62655, 62463, 62269, 62073, 61874, 61674, 61472, 61268, 61062, 60854, 60644, 60431, 60218, 60002, 59784, 59564, 59343, 59119, 58894, 58667, 58439, 58208, 57976, 57742, 57506, 57269, 57029, 56789, 56546, 56302, 56056, 55809, 55560, 55310, 55058, 54804, 54550, 54293, 54035, 53776, 53515, 53253, 52989, 52725, 52458, 52191, 51922, 51652, 51381, 51108, 50834, 50559, 50283, 50006, 49727, 49448, 49167, 48885, 48602, 48319, 48034, 47748, 47461, 47174, 46885, 46596, 46305, 46014, 45722, 45429, 45136, 44841, 44546, 44250, 43954, 43657, 43359, 43060, 42761, 42462, 42161, 41861, 41559, 41258, 40955, 40653, 40350, 40046, 39743, 39438, 39134, 38829, 38524, 38219, 37913, 37607, 37301, 36995, 36689, 36383, 36076, 35769, 35463, 35156, 34850, 34543, 34236, 33930, 33624, 33317, 33011, 32705, 32399, 32094, 31789, 31483, 31179, 30874, 30570, 30266, 29963, 29660, 29357, 29055, 28753, 28452, 28151, 27851, 27551, 27252, 26954, 26656, 26359, 26062, 25766, 25471, 25177, 24883, 24590, 24298, 24007, 23717, 23427, 23139, 22851, 22564, 22279, 21994, 21710, 21427, 21146, 20865, 20585, 20307, 20030, 19753, 19478, 19205, 18932, 18661, 18390, 18122, 17854, 17588, 17323, 17060, 16797, 16537, 16277, 16019, 15763, 15508, 15255, 15003, 14752, 14503, 14256, 14010, 13766, 13524, 13283, 13044, 12807, 12571, 12337, 12104, 11874, 11645, 11418, 11193, 10970, 10748, 10529, 10311, 10095, 9881, 9669, 9459, 9251, 9045, 8840, 8638, 8438, 8240, 8044, 7850, 7658, 7468, 7280, 7095, 6911, 6730, 6551, 6374, 6199, 6027, 5856, 5688, 5522, 5359, 5197, 5038, 4881, 4727, 4575, 4425, 4278, 4132, 3990, 3849, 3711, 3576, 3442, 3312, 3183, 3057, 2934, 2813, 2694, 2578, 2464, 2353, 2245, 2139, 2035, 1934, 1835, 1739, 1646, 1555, 1466, 1381, 1297, 1217, 1139, 1063, 990, 920, 852, 787, 725, 665, 608, 553, 501, 452, 405, 361, 320, 281, 245, 212, 181, 153, 128, 105, 85, 68, 53, 41, 32, 25, 21, 20, 21, 25, 32, 41, 53, 68, 85, 105, 128, 153, 181, 212, 245, 281, 320, 361, 405, 452, 501, 553, 608, 665, 725, 787, 852, 920, 990, 1063, 1139, 1217, 1297, 1381, 1466, 1555, 1646, 1739, 1835, 1934, 2035, 2139, 2245, 2353, 2464, 2578, 2694, 2813, 2934, 3057, 3183, 3312, 3442, 3576, 3711, 3849, 3990, 4132, 4278, 4425, 4575, 4727, 4881, 5038, 5197, 5359, 5522, 5688, 5856, 6027, 6199, 6374, 6551, 6730, 6911, 7095, 7280, 7468, 7658, 7850, 8044, 8240, 8438, 8638, 8840, 9045, 9251, 9459, 9669, 9881, 10095, 10311, 10529, 10748, 10970, 11193, 11418, 11645, 11874, 12104, 12337, 12571, 12807, 13044, 13283, 13524, 13766, 14010, 14256, 14503, 14752, 15003, 15255, 15508, 15763, 16019, 16277, 16537, 16797, 17060, 17323, 17588, 17854, 18122, 18390, 18661, 18932, 19205, 19478, 19753, 20030, 20307, 20585, 20865, 21146, 21427, 21710, 21994, 22279, 22564, 22851, 23139, 23427, 23717, 24007, 24298, 24590, 24883, 25177, 25471, 25766, 26062, 26359, 26656, 26954, 27252, 27551, 27851, 28151, 28452, 28753, 29055, 29357, 29660, 29963, 30266, 30570, 30874, 31179, 31483, 31789, 32094, 32399, 32705, 33011, 33317, 33624, 33930, 34236, 34543, 34850
  };
  //static void pwmpcb(PWMDriver *pwmp);

  uint16_t currentPhase = 0;
  float32_t currentAmplitude = 0;

  /*
   * PWM configuration structure.
   * Cyclic callback disabled, channels 0, 1 and 2 enabled without callbacks,
   * the active state is a logic one.
   * pwm uses timer 3
   * pins     channel   alternate function
   * PA6,PB4,PC6      1         AF2
   * PA7,PB5,PC7      2         AF2
   * PB0,PC8      3         AF2
   * PB1,PC9      4         AF2
   *
   *changed my mind, using timer8
   * picking channel 1 3 and 4 so we can use pins PC6, PC8 and PC9 which are right next to each other
   */
  const PWMConfig pwmcfg = {
    180000000,                   /* (double the 90MHz because we're in center align mode) 180MHz PWM clock frequency.   */
    3600,                       /* PWM period 40µs (in ticks). frequency is 25Khz    */
    &ThreePhaseDriver::sinCallBack,                       /* yes callback */
    {
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* Enable Channel 1 */
        {PWM_OUTPUT_DISABLED, NULL},    /* disable channel 2 */
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* Enable Channel 3 */
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* Enable Channel 4 */

    }, /* HW dependent part.*/
    0, //cr2
    0, //bdtr only in advanced mode
    0 //dier
  };

  const PWMConfig pwmcfgSimple = {
    90000000,                   /* 90MHz   */
    3600,                       /* PWM period 40µs (in ticks). frequency is 25Khz    */
    &ThreePhaseDriver::simpleCallBack,                       /* yes callback */
    {
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* Enable Channel 1 */
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},    /* Enable channel 2 */
        {PWM_OUTPUT_DISABLED, &ThreePhaseDriver::simpleCallBack}, /* disable Channel 3, use as 'middle' callback for ADC */
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* Enable Channel 4 */

    }, /* HW dependent part.*/
    0, //cr2
    0,
    0 //dier
  };


};

#endif
