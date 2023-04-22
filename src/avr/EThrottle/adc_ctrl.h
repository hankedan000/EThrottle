#pragma once

#include <stdint.h>

namespace adc
{

  enum MeasurementMode_E
  {
    // will not measure this entry at all
    eMM_Disabled,
    // measures the entry every iteration
    eMM_Continuous,
    // measures the entry if the 'needsMeasure' flag is set
    eMM_OneShot
  };

  struct CtrlEntry
  {
    CtrlEntry()
    : adcMUX(0)
    , mode(MeasurementMode_E::eMM_Disabled)
    , needsMeasure(0)
    , value(0)
    {}

    // adc mux value to use for measurement
    unsigned int adcMUX;

    MeasurementMode_E mode;

    volatile uint8_t needsMeasure;

    // latest ADC measurement value
    volatile uint16_t value;
  };

  extern CtrlEntry ppsA;
  extern CtrlEntry ppsB;
  extern CtrlEntry tpsA;
  extern CtrlEntry tpsB;
  extern CtrlEntry driverFB;

  extern uint16_t conversionCount;

  // @return
  // 1 if measurements were started
  // 0 if there are no enabled entries
  uint8_t
  start();

  void
  stop();

}
