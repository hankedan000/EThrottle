#include "can.h"

#include "uart.h"

SET_MEGA_CAN_SIG("EThrottle");
SET_MEGA_CAN_REV("OpenGPIO-0.1.0     ");

// external interrupt service routine for CAN message on MCP2515
void
canISR()
{
  canDev.interrupt();
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
  canDev.handle();

  // update status0
  outPC.status0.bits.needsBurn = canDev.needsBurn();
  outPC.status0.bits.flashDataLost = canDev.flashDataLost();
  outPC.status0.bits.currFlashTable = canDev.currFlashTable();

  // update CAN status registers
  outPC.canStatus = canDev.getCAN_Status();
  outPC.canErrorCount = canDev.getLogicErrorCount();
}