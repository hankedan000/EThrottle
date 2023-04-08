#include <string.h>
#include <stdlib.h>

#include "can.h"
#include "EndianUtils.h"
#include "FlashUtils.h"
#include <logging.h>
#include "Throttle.h"

// analog inputs
#define TPS_A_PIN A0
#define TPS_B_PIN A1
#define PPS_A_PIN A2
#define PPS_B_PIN A3
#define CTRL_BUTN A4
#define DRIVER_FB A5

// digital outputs
#define WSPEED_INT 2
                // 3 CAN interrupt
#define DRIVER_DIS 4
#define DRIVER_P   5// pin needs PWM support
#define DRIVER_N   6// pin needs PWM support
#define DRIVER_FS  7
#define CLUTCH_SW  8
#define BRAKE_SW   9
                // 10 CAN CS
                // 11 CAN MOSI
                // 12 CAN MISO
                // 13 CAN SCK

Throttle::OutVars *throttleVars = &outPC.throttleOutVars;
Throttle throttle(
  TPS_A_PIN,TPS_B_PIN,
  PPS_A_PIN,PPS_B_PIN,
  DRIVER_P,DRIVER_N,
  DRIVER_DIS,
  DRIVER_FS,
  DRIVER_FB,
  throttleVars);

uint8_t pidSampleRate_ms = 10;

void
loadThrottlePID_FromFlash()
{
  throttle.updatePID_Coeffs(
    FlashUtils::flashRead_BE<uint16_t>(PAGE1_FIELD_OFFSET(throttleKp)) / 100.0,
    FlashUtils::flashRead_BE<uint16_t>(PAGE1_FIELD_OFFSET(throttleKi)) / 100.0,
    FlashUtils::flashRead_BE<uint16_t>(PAGE1_FIELD_OFFSET(throttleKd)) / 100.0);
}

void
storeThrottlePID_ToFlash()
{
  FlashUtils::flashWrite_BE(PAGE1_FIELD_OFFSET(throttleKp), (uint16_t)(throttle.getKp() * 100.0));
  FlashUtils::flashWrite_BE(PAGE1_FIELD_OFFSET(throttleKi), (uint16_t)(throttle.getKi() * 100.0));
  FlashUtils::flashWrite_BE(PAGE1_FIELD_OFFSET(throttleKd), (uint16_t)(throttle.getKd() * 100.0));
}

void setup() {
  setupLogging(115200);

  // help reduce buzzing sounds from throttle body
  // FIXME move into Throttle class's init()
  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz

  throttle.init(pidSampleRate_ms);
  loadThrottlePID_FromFlash();
  canSetup();
}

size_t serBuffIdx = 0;
char serBuff[20] = "400";
char serCmd = '\0';

void logPID_Params() {
  INFO(
    "Kp = %f, Ki = %f, Kd = %f",
    throttle.getKp(),
    throttle.getKi(),
    throttle.getKd());
}

// a    -> auto tune PID controller
// s### -> set 'pidSetpoint' to ###
// p### -> set 'Kp' to ###
// i### -> set 'Ki' to ###
// d### -> set 'Kd' to ###
// P    -> print current PID parameters
// m    -> toggle mode (from pedal vs. from terminal)
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
            if (throttleVars->status.pidAutoTuneBusy) {
              throttle.stopPID_AutoTune();
              INFO("stopped PID auto-tune");
            } else {
              throttle.startPID_AutoTune();
              INFO("started PID auto-tune");
            }
            break;
          case 's':
            if (throttleVars->ctrl.setpointOverride.enabled) {
              throttleVars->ctrl.setpointOverride.value = val;
              INFO("setpointOverride = %f",val);
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
            throttleVars->ctrl.setpointOverride.enabled ^= 1;
            const bool spOverrideState = throttleVars->ctrl.setpointOverride.enabled;
            INFO("setpoint override: %s",(spOverrideState ? "ON" : "OFF"));
            if (spOverrideState) {
              INFO("use 's' command to set the override");
            }
            break;
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
