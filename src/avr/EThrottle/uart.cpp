#include "uart.h"

#include <Arduino.h>
#include <logging.h>

#include "tables.h"
#include "Throttle.h"

// declare extern variable
char uartCmdBuff[UART_CMD_BUFF_SIZE];

// where to insert next received byte into command buffer
uint8_t cmdBuffIdx = 0;

void
doUart()
{
  while (Serial.available() && cmdBuffIdx < UART_CMD_BUFF_SIZE)
  {
    char c = Serial.read();
    if (c == '\n')
    {
      uartCmdBuff[cmdBuffIdx++] = '\0';
      processUartCmd();
    }
    else
    {
      uartCmdBuff[cmdBuffIdx++] = c;
    }
  }
}

uint8_t
uartCmdBuffSize()
{
  return cmdBuffIdx;
}

void
pushedUartCmdBytes(
  uint8_t nBytes)
{
  if ((cmdBuffIdx + nBytes) > UART_CMD_BUFF_SIZE)
  {
    ERROR("nBytes overflows UART buffer");
  }
  else
  {
    cmdBuffIdx += nBytes;
    if (uartCmdBuff[cmdBuffIdx-1] == '\0')
    {
      processUartCmd();
    }
  }
}

void
logPID_Params()
{
  INFO(
    "Kp = %d, Ki = %d, Kd = %d",
    (uint16_t)(throttle.getKp() * 100),
    (uint16_t)(throttle.getKi() * 100),
    (uint16_t)(throttle.getKd() * 100));
}

void
processUartCmd()
{
  if (cmdBuffIdx < 2)
  {
    // need at least a command char + null terminator
    return;
  }
  else if (uartCmdBuff[cmdBuffIdx-1] != '\0')
  {
    return;
  }

  DEBUG("uartCmdBuff = '%s'", uartCmdBuff);

  Throttle::OutVars *throttleVars = &outPC.throttleOutVars;
  char cmd = uartCmdBuff[0];
  double val = atof(cmdBuffIdx+1);
  switch (cmd) {
    case 'a':
      if (throttleVars->status.pidAutoTuneBusy) {
        throttle.stopPID_AutoTune();
        INFO("stopped PID auto-tune");
      } else {
        throttle.startPID_AutoTune();
        INFO("started PID auto-tune");
      }
      break;
    case 's':
      if (throttle.getSetpointSource() == Throttle::SetpointSource_E::eSS_User) {
        throttle.setSetpointOverride(val);
        INFO("setpointOverride = %d",(uint16_t)val);
      }
      break;
    case 't':
      switch (uartCmdBuff[1])
      {
        case '0':
          throttle.disableThrottle();
          break;
        case '1':
          throttle.enableThrottle();
          break;
      }
      break;
    case 'p':
      throttle.updatePID_Coeffs(
        val,
        throttle.getKi(),
        throttle.getKd());
      logPID_Params();
      break;
    case 'i':
      throttle.updatePID_Coeffs(
        throttle.getKp(),
        val,
        throttle.getKd());
      logPID_Params();
      break;
    case 'd':
      throttle.updatePID_Coeffs(
        throttle.getKp(),
        throttle.getKi(),
        val);
      logPID_Params();
      break;
    case 'P':
      logPID_Params();
      break;
    case 'm':
      bool spOverrideState = false;
      switch (throttle.getSetpointSource())
      {
        case Throttle::SetpointSource_E::eSS_PPS:
          throttle.setSetpointSource(Throttle::SetpointSource_E::eSS_User);
          spOverrideState = true;
          break;
        case Throttle::SetpointSource_E::eSS_User:
          throttle.setSetpointSource(Throttle::SetpointSource_E::eSS_PPS);
          break;
      }
      INFO("setpoint override: %s",(spOverrideState ? "ON" : "OFF"));
      if (spOverrideState) {
        INFO("use 's' command to set the override");
      }
      break;
  }

  // mark command processed
  cmdBuffIdx = 0;
}