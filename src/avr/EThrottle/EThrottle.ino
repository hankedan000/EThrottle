#include <avr/sfr_defs.h>
#include <avr/wdt.h>
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

// define this if you have the mcp-can-boot bootloader installed
//  * allows for remote reflashing via CAN
//  * watchdog reset support (regular Arduino bootloader doesn't support)
#undef MCP_CAN_BOOT_BL

// toggle for watchdog support
#define WATCHDOG_SUPPORT 0
#ifdef MCP_CAN_BOOT_BL
#define WATCHDOG_SUPPORT 1// auto enable watchdog if using mcp-can-boot bootloader
#endif

// setup watchdog
void wdtInit() {
  cli();// disable interrupts
  wdt_reset();// reset watchdog
  wdt_enable(WDTO_15MS);// start watchdog timer with 15ms timeout
  sei();
}

// retrieves the MCU reset cause (MCUSR register) when using the mcp-can-boot
// bootloader (https://github.com/crycode-de/mcp-can-boot).
// no need to call this explicitly, it will get called by init0.
#ifdef MCP_CAN_BOOT_BL
uint8_t mcusr __attribute__ ((section (".noinit")));
void getMCUSR(void) __attribute__((naked)) __attribute__((section(".init0")));
void getMCUSR(void) {
  __asm__ __volatile__ ( "mov %0, r2 \n" : "=r" (mcusr) : );
}
#endif

void setup() {
#if WATCHDOG_SUPPORT
  wdtInit();// start watchdog
#endif

  setupLogging(115200);
  INFO("WATCHDOG: %s", (WATCHDOG_SUPPORT ? "ON" : "OFF"));
#ifdef MCP_CAN_BOOT_BL
  outPC.mcusr = mcusr;
  INFO("MCUSR: 0x%x", mcusr);
#else
  outPC.mcusr = 0;// arduino bootloader doesn't preserve MCUSR contents
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

  doUart();
  canLoop();
  throttle.run();

  // update loop time register
  uint16_t loopTimeUs = micros() - loopStartTimeUs;
  EndianUtils::setBE(outPC.loopTimeUs, loopTimeUs);
}
