#include <string.h>
#include <stdlib.h>

#include "adc_ctrl.h"
#include "can.h"
#include "EndianUtils.h"
#include <logging.h>
#include "Throttle.h"
#include "uart.h"

// analog inputs
#define CTRL_BUTN A4

// digital outputs
#define WSPEED_INT 2
                // 3 CAN interrupt
#define CLUTCH_SW  8
#define BRAKE_SW   9
                // 10 CAN CS
                // 11 CAN MOSI
                // 12 CAN MISO
                // 13 CAN SCK

#define PID_SAMPLE_RATE_MS 10// 10ms
Throttle::OutVars *throttleVars = &outPC.throttleOutVars;

void setup() {
  setupLogging(115200);

  throttle.init(PID_SAMPLE_RATE_MS, throttleVars);
  loadThrottlePID_FromFlash(throttle);
  loadSensorCalibrationsFromFlash(throttle);
  loadSensorSetupFromFlash(throttle);

  canSetup();

  if (adc::start())
  {
    WARN("ADC measurements started!");
  }
  else
  {
    WARN("no ADC measurements enabled");
  }
}

void loop() {
  uint32_t loopStartTimeUs = micros();

  doUart();
  canLoop();
  throttle.run();

  // update loop time register
  uint16_t loopTimeUs = micros() - loopStartTimeUs;
  EndianUtils::setBE(outPC.loopTimeUs, loopTimeUs);
}
