#include <string.h>
#include <stdlib.h>

#include "can.h"
#include "EndianUtils.h"
#include <logging.h>
#include "Throttle.h"

// analog inputs
#define TPS_A_PIN A4 // non-inverted TPS pin
#define TPS_B_PIN A5 // inverted TPS pin
#define PPS_A_PIN A0
#define PPS_B_PIN A1

// digital outputs
#define MOTOR_P 5// pin needs PWM support
#define MOTOR_N 6// pin needs PWM support

Throttle::RAM_Vars throttleVars;
Throttle throttle(
  TPS_A_PIN,TPS_B_PIN,
  PPS_A_PIN,PPS_B_PIN,
  MOTOR_P,MOTOR_N,
  &throttleVars);

uint8_t pidSampleRate_ms = 10;

void setup() {
  setupLogging(115200);

  // help reduce buzzing sounds from throttle body
  // FIXME move into Throttle class's init()
  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz

  throttle.init(pidSampleRate_ms);
  canSetup();
}

size_t serBuffIdx = 0;
char serBuff[20] = "400";
char serCmd = '\0';

void logPID_Params() {
  INFO(
    "Kp = %f, Ki = %f, Kd = %f",
    throttleVars.Kp,
    throttleVars.Ki,
    throttleVars.Kd);
}

// a    -> auto tune PID controller
// s### -> set 'pidSetpoint' to ###
// p### -> set 'Kp' to ###
// i### -> set 'Ki' to ###
// d### -> set 'Kd' to ###
// P    -> print current PID parameters
void doSerial() {
  while (Serial.available())
  {
    if (serCmd == '\0') {
      serCmd = Serial.read();
      serBuffIdx = 0;
    } else {
      serBuff[serBuffIdx] = Serial.read();
      if (serBuff[serBuffIdx] == '\n') {
        serBuff[serBuffIdx] = '\0';// null terminate for atoi() to work
        double val = 0;
        if (serBuffIdx > 0) {
          val = atof(serBuff);
        }
        switch (serCmd) {
          case 'a':
            if (throttleVars.status.pidAutoTuneBusy) {
              throttle.stopPID_AutoTune();
              INFO("stopped PID auto-tune");
            } else {
              throttle.startPID_AutoTune();
              INFO("started PID auto-tune");
            }
            break;
          case 's':
            if (throttleVars.ctrl.setpointOverride.enabled) {
              throttleVars.ctrl.setpointOverride.value = val;
              INFO("setpointOverride = %f",val);
            }
            break;
          case 'p':
            throttleVars.Kp = val;
            break;
          case 'i':
            throttleVars.Ki = val;
            break;
          case 'd':
            throttleVars.Kd = val;
            break;
          case 'P':
            logPID_Params();
            break;
          case 'm':
            throttleVars.ctrl.setpointOverride.enabled ^= 1;
            const bool spOverrideState = throttleVars.ctrl.setpointOverride.enabled;
            INFO("setpoint override: %s",(spOverrideState ? "ON" : "OFF"));
            if (spOverrideState) {
              INFO("use 's' command to set the override");
            }
            break;
        }

        if (serCmd == 'p' || serCmd == 'i' || serCmd == 'd') {
          throttle.updatePID_Coeffs();
          logPID_Params();
        }

        serCmd = '\0';// restart command parsing
      }
      serBuffIdx++;
    }
  }
}

void loop() {
  uint32_t loopStartTimeUs = micros();

  doSerial();
  canLoop();
  throttle.run();

  // update loop time register
  uint16_t loopTimeUs = micros() - loopStartTimeUs;
  EndianUtils::setBE(outPC.loopTimeUs, loopTimeUs);
}
