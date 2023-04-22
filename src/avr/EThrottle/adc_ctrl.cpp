#include "adc_ctrl.h"

#include <avr/interrupt.h>

namespace adc
{

  // declare external variables
  CtrlEntry ppsA;
  CtrlEntry ppsB;
  CtrlEntry tpsA;
  CtrlEntry tpsB;
  CtrlEntry driverFB;
  uint16_t conversionCount;

  // current 
  volatile uint8_t schedIdx = 0;

  // order in which we perform measurements
  const CtrlEntry *sched[] = {
    &ppsA,
    &ppsB,
    &tpsA,
    &tpsB,
    &driverFB
  };

  #define ARRAY_LEN(a) (sizeof(a) / sizeof(a[0]))

  void
  startADC(
    uint8_t mux)
  {
    // setup ADC mux with Vcc as reference
    ADMUX = (1 << REFS0) | (mux & 0xF);

    // start the ADC conversion
    #define ADC_PS_2   0x1
    #define ADC_PS_4   0x2
    #define ADC_PS_8   0x3
    #define ADC_PS_16  0x4
    #define ADC_PS_32  0x5
    #define ADC_PS_64  0x6
    #define ADC_PS_128 0x7
    ADCSRA = 
        (1 << ADEN) | // enable ADC
        (1 << ADSC) | // start conversion
        (1 << ADIE) | // enable ADC complete interrupt
        ADC_PS_128;   // prescaler
  }

  // @return
  // 1 if a measurement was found and setup
  // 0 otherwise
  uint8_t
  findAndStartNext()
  {
    // find and setup next entry
    uint8_t origIdx = schedIdx;
    uint8_t doSetup = 0;
    uint8_t keepSearching = 1;
    while (keepSearching)
    {
      schedIdx++;
      if (schedIdx >= ARRAY_LEN(sched))
      {
        schedIdx = 0;
      }
      if (schedIdx == origIdx)
      {
        // went all the way around the schedule. stop looking
        keepSearching = 0;
      }

      switch (sched[schedIdx]->mode)
      {
        case MeasurementMode_E::eMM_Disabled:
          continue;
          break;
        case MeasurementMode_E::eMM_Continuous:
          keepSearching = 0;
          doSetup = 1;
          break;
        case MeasurementMode_E::eMM_OneShot:
          if (sched[schedIdx]->needsMeasure)
          {
            keepSearching = 0;
            doSetup = 1;
          }
          break;
      }
    }

    if (doSetup)
    {
      startADC(sched[schedIdx]->adcMUX);
    }
    return doSetup;
  }

  // ADC measurement complete interrupt
  ISR(ADC_vect)
  {
    conversionCount++;
    CtrlEntry *currEntry = sched[schedIdx];
    currEntry->value = ADC;
    currEntry->needsMeasure = 0;

    findAndStartNext();
  }

  uint8_t
  start()
  {
    // disable digital input on pins used for ADC
    DIDR0 = 0x0;
    for (uint8_t i=0; i<ARRAY_LEN(sched); i++)
    {
      if (sched[i]->adcMUX <= 5)
      {
        DIDR0 |= (1 << sched[i]->adcMUX);
      }
    }

    // start conversions
    return findAndStartNext();
  }

  void
  stop()
  {
    ADCSRA = 0x0;// terminate by disabling ADC
  }

}