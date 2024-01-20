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

  // specify how the ADC should be started once an entry becomes
  // ready for scheduling.
  // Note: values correspond to ADC Auto Trigger Source values (reg ADCSRB)
  enum TriggerMode_E
  {
    eTM_Immediate = 0,
    eTM_Tmr0_MatchA = 3,
    eTM_Tmr0_Ovrf = 4,
    eTM_Tmr1_MatchB = 5,
    eTM_Tmr1_Ovrf = 6,
    eTM_Tmr1_CapEvt = 7,

    eTM_ISR_Tmr0_OCA = 20,
    eTM_ISR_Tmr0_OCB = 21
  };

  enum ADC_State_E
  {
    eADCS_Stopped = 0,
    eADCS_Started = 1,
    eADCS_PendingTrigger = 2,
    eADCS_Complete = 3
  };

  struct CtrlEntry
  {
    CtrlEntry()
    : adcMUX(0)
    , mMode(MeasurementMode_E::eMM_Disabled)
    , tMode(TriggerMode_E::eTM_Immediate)
    , value(0)
    {
      flags.needsMeasure = 0;
      flags.sampled = 0;
    }

    // adc mux value to use for measurement
    unsigned int adcMUX;

    MeasurementMode_E mMode;

    volatile TriggerMode_E tMode;

    struct Flags
    {
      uint8_t needsMeasure : 1;
      uint8_t sampled      : 1;// set to 1 when ADC is sampled. user can clear if desired
    };
    volatile Flags flags;

    // latest ADC measurement value
    volatile uint16_t value;
  };

  extern CtrlEntry ppsA;
  extern CtrlEntry ppsB;
  extern CtrlEntry tpsA;
  extern CtrlEntry tpsB;
  extern CtrlEntry driverFB;

  // running count of individual ADC conversions completed
  volatile extern uint16_t conversionCount;

  // running count of ADC schedule cycles completed
  volatile extern uint16_t conversionCycles;

  // @return
  // 1 if measurements were started
  // 0 if there are no enabled entries
  uint8_t
  start();

  void
  stop();

  uint8_t
  getSchedIdx();

  ADC_State_E
  getState();

}
