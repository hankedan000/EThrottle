#include <avr/sfr_defs.h>
#include <avr/wdt.h>
#include <string.h>
#include <stdlib.h>

#include "adc_ctrl.h"
#include "can.h"
#include "config.h"
#include <EndianUtils.h>
#include <logging_impl_lite.h>
#include "Throttle.h"

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

ThrottleOutVars_T *throttleVars = &outPC.tVars;

// setup watchdog
void wdtInit() {
  cli();// disable interrupts
  wdt_reset();// reset watchdog
  wdt_enable(WDTO_15MS);// start watchdog timer with 15ms timeout
  sei();
}

const char *
showResetCause(
  const uint8_t mcusr)
{
  INFO("reset cause...");
  if (mcusr & (1 << 0))
  {
    INFO("Power-ON");
  }
  if (mcusr & (1 << 1))
  {
    INFO("External");
  }
  if (mcusr & (1 << 2))
  {
    INFO("Brown-Out");
  }
  if (mcusr & (1 << 3))
  {
    INFO("Watchdog");
  }
}

void setup() {
#ifdef MCP_CAN_BOOT_BL
  // retrieves the MCU reset cause (MCUSR register) when using the
  // mcp-can-boot bootloader (https://github.com/crycode-de/mcp-can-boot).
  uint8_t mcusr;
  __asm__ __volatile__ ( "mov %0, r2 \n" : "=r" (mcusr) : );
#endif

#if WATCHDOG_SUPPORT
  wdtInit();// start watchdog
#endif

  setupLogging(115200);
  INFO("WATCHDOG: %s", (WATCHDOG_SUPPORT ? "ON" : "OFF"));
#ifdef MCP_CAN_BOOT_BL
  outPC.mcusr.word = mcusr;
  showResetCause(mcusr);
#else
  outPC.mcusr.word = 0;// arduino bootloader doesn't preserve MCUSR contents
#endif

  throttle.init(PID_SAMPLE_RATE_MS, throttleVars);
  loadThrottlePID_FromFlash(throttle);
  loadSensorCalibrationsFromFlash(throttle);
  loadSensorSetupFromFlash(throttle);

  canSetup();

  if ( ! adc::start())
  {
    WARN("no ADC measurements enabled");
  }
}

void loop() {
  wdt_reset();// throw watchdog a bone
  uint32_t loopStartTimeUs = micros();

  canLoop();
  throttle.run();

  // update ADC status
  outPC.adcStatus.schedIdx = adc::getSchedIdx();
  outPC.adcStatus.state = adc::getState();
  outPC.adcStatus.convCycles = adc::conversionCycles;
  outPC.adcStatus.adcsra = ADCSRA;

  // update loop time register
  uint16_t loopTimeUs = micros() - loopStartTimeUs;
  EndianUtils::setBE(outPC.loopTimeUs, loopTimeUs);
}
