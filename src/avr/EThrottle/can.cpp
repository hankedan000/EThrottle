#include "Arduino.h"
#include "can.h"

#include "uart.h"

SET_MEGA_CAN_SIG("EThrottle");
SET_MEGA_CAN_REV("OpenGPIO-0.1.0     ");

// toggle a pin on different CAN events
#define ENABLE_CAN_STROBES  0 // global enable
#define STROBE_PIN 8
#define STROBE_ON_INT       0
#define STROBE_ON_HANDLE    1

// external interrupt service routine for CAN message on MCP2515
void
canISR()
{
  EIMSK &= ~(1 << INT1);// disable external interrupt
  sei();// enable global interrupts again (prevent timer ISRs from getting too delayed)

#if ENABLE_CAN_STROBES && STROBE_ON_INT
  digitalWrite(STROBE_PIN, 1);
#endif
  canDev.interrupt();
#if ENABLE_CAN_STROBES && STROBE_ON_INT
  digitalWrite(STROBE_PIN, 0);
#endif

  EIMSK |= (1 << INT1);// reenable external interrupt
}

// declared in EThrottle.ino
extern Throttle throttle;

// called any time a table is burned to over CAN
void
onTableBurned(
  uint8_t table)
{
  if (table == 1)
  {
    loadThrottlePID_FromFlash(throttle);
    loadSensorCalibrationsFromFlash(throttle);
    loadSensorSetupFromFlash(throttle);
  }
}

void
onTableWritten(
  uint8_t table,
	uint16_t /*offset*/,
	uint8_t len,
	const uint8_t * /*data*/)
{
  if (table == 3)
  {
    pushedUartCmdBytes(len);
  }
}

void
canSetup()
{
  cli();// disable interrupts

#if ENABLE_CAN_STROBES
  pinMode(STROBE_PIN, OUTPUT);
#endif

  // MCP2515 configuration
  canDev.init();
  canDev.setOnTableBurnedCallback(onTableBurned);
  canDev.setOnTableWrittenCallback(onTableWritten);
  pinMode(CAN_INT, INPUT_PULLUP);// Configuring pin for CAN interrupt input
  attachInterrupt(digitalPinToInterrupt(CAN_INT), canISR, LOW);

  sei();// enable interrupts
}

void
canLoop()
{
#if ENABLE_CAN_STROBES && STROBE_ON_HANDLE
  digitalWrite(STROBE_PIN, 1);
#endif
  canDev.handle();
#if ENABLE_CAN_STROBES && STROBE_ON_HANDLE
  digitalWrite(STROBE_PIN, 0);
#endif

  // update status0
  outPC.status0.bits.needsBurn = canDev.needsBurn();
  outPC.status0.bits.flashDataLost = canDev.flashDataLost();
  outPC.status0.bits.currFlashTable = canDev.currFlashTable();

  // update CAN status registers
  outPC.canStatus = canDev.getCAN_Status();
  outPC.canErrorCount = canDev.getLogicErrorCount();
}