#pragma once

#include <stdint.h>

#define UART_CMD_BUFF_SIZE 20

extern char uartCmdBuff[UART_CMD_BUFF_SIZE];

void
doUart();

uint8_t
uartCmdBuffSize();

// a      -> auto tune PID controller
// p###   -> set 'Kp' to ### (where ### is a float from 0.0 to 1.0)
// i###   -> set 'Ki' to ### (where ### is a float from 0.0 to 1.0)
// d###   -> set 'Kd' to ### (where ### is a float from 0.0 to 1.0)
// s##### -> set 'pidSetpoint' to ##### (where ##### is 0 to 10000)
// t#     -> throttle enable/disable (0: disable, 1: enable)
// P      -> print current PID parameters
// m      -> toggle mode (from pedal vs. from terminal)
void
processUartCmd();