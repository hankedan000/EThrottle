#include <string.h>
#include <stdlib.h>

#include "can.h"
#include "EndianUtils.h"
#include <logging.h>
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

#define PID_SAMPLE_RATE_MS 10// 10ms
Throttle::OutVars *throttleVars = &outPC.throttleOutVars;

void setup() {
  setupLogging(115200);

  throttle.init(PID_SAMPLE_RATE_MS, throttleVars);
  loadThrottlePID_FromFlash(throttle);
  loadSensorCalibrationsFromFlash(throttle);

  canSetup();
}

size_t serBuffIdx = 0;
char serBuff[20] = "400";
char serCmd = '\0';

void logPID_Params() {
  INFO(
    "Kp = %d, Ki = %d, Kd = %d",
    (uint16_t)(throttle.getKp() * 100),
    (uint16_t)(throttle.getKi() * 100),
    (uint16_t)(throttle.getKd() * 100));
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
            if (throttle.getSetpointSource() == Throttle::SetpointSource_E::eSS_User) {
              throttle.setSetpointOverride(val);
              INFO("setpointOverride = %d",(uint16_t)val);
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
