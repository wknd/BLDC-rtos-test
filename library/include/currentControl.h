#ifndef _CURRENTCTRLTH_H
#define _CURRENTCTRLTH_H

#ifndef _CH_HPP_
#include "ch.hpp"
#endif
#ifndef HAL_H
#include "hal.h"
#endif
#ifndef CHPRINTF_H
#include "chprintf.h"
#endif
#ifndef _ADCMAN_H
#include "ADC.h"
#endif

#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif
#ifndef _ARM_MATH_H
#include "arm_math.h"
#endif

using namespace chibios_rt;

#define CURRENT_EVENT EVENT_MASK(0)

// fir Filter stuff
#define CURRENT_FIR_BLOCK_SIZE CURRENT_BUFFERS_SIZE // is how many samples we will process at a time
//#define ADC_FIR_NUM_TAPS 29 // moar taps is moar better, otherwise we don't get a flat response in our passband
                            // however, more taps means there is a larger delay between a change in signal and us seeing it

#define CURRENT_FIR_NUM_TAPS 50 // larger means more selective filter, but also larger delay..
#define ADC_FIR_DECIMATION_FACTOR 50 //samples comming in at 50kHz, decimating to get 1kHz rate




#define CURRENT_EVT_UPDATEAMPLITUDE   EVENT_MASK(1)
#define CURRENT_EVT_ERROR             EVENT_MASK(2)
#define ADC_EVT_NOBUFFER              EVENT_MASK(3)
#define ADC_EVT_ERROR                 EVENT_MASK(4)
#define ADC_EVT_INIT_FAIL             EVENT_MASK(5)
#define ADC_EVT_INIT_OK               EVENT_MASK(6)
#define ADC_EVT_FILLEDBLOCK           EVENT_MASK(7)


class currentControlTh : public BaseStaticThread<1024> {


protected:
  void main(void);
  const float32_t armFirCoeffFloat[CURRENT_FIR_NUM_TAPS] = {
      0.000172585447087590, 0.000392838750841571, 0.000742832476451476, 0.00116861272809195, 0.00156987874923517, 0.00178275089726002, 0.00159713731703957,
      0.000797729127066019, -0.000773801847487747, -0.00314491352414999, -0.00614253267292099, -0.00934723536262306, -0.0120901590125977,
      -0.0135056153274981, -0.0126417750631200, -0.00861841900699823, -0.000807558511312762, 0.0109969064131891, 0.0264574820601023, 0.0446513619307644,
      0.0641278665747090, 0.0830656512097237, 0.0995098172857835, 0.111651241321993, 0.118099493039371, 0.118099493039371, 0.111651241321993,
      0.0995098172857835, 0.0830656512097237, 0.0641278665747090, 0.0446513619307644, 0.0264574820601023, 0.0109969064131891, -0.000807558511312762,
      -0.00861841900699823, -0.0126417750631200, -0.0135056153274981, -0.0120901590125977, -0.00934723536262306, -0.00614253267292099,
      -0.00314491352414999, -0.000773801847487747, 0.000797729127066019, 0.00159713731703957, 0.00178275089726002, 0.00156987874923517, 0.00116861272809195,
      0.000742832476451476, 0.000392838750841571, 0.000172585447087590


      /*0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02,
      0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02,
      0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02,
      0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02,
      0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02 */
  };

  std::array<float32_t, CURRENT_BUFFERS_SIZE> *adc_buffer;
  arm_fir_decimate_instance_f32 filter;
  float32_t filterState[CURRENT_FIR_BLOCK_SIZE + CURRENT_FIR_NUM_TAPS - 1]; // internal state for the filter
  float32_t filterOutput;




public:
  currentControlTh();
  //void setCurrent(float32_t cur);
  float32_t getCurrent();
  event_source current_event_source;

};







#endif
