#include <string.h>
#include <stdlib.h>

#include "Throttle.h"

// analog inputs
#define TPS_A_PIN A4 // non-inverted TPS pin
#define TPS_B_PIN A5 // inverted TPS pin
#define PPS_A_PIN A0
#define PPS_B_PIN A1

// digital outputs
#define MOTOR_P 3// pin needs PWM support
#define MOTOR_N 5// pin needs PWM support

Throttle::RAM_Vars throttleVars;
Throttle throttle(
  TPS_A_PIN,TPS_B_PIN,
  PPS_A_PIN,PPS_B_PIN,
  MOTOR_P,MOTOR_N,
  &throttleVars);

uint8_t pidSampleRate_ms = 10;

void setup() {
  // help reduce buzzing sounds from throttle body
  // FIXME move into Throttle class's init()
  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz

  throttle.init(pidSampleRate_ms);
  
  Serial.begin(115200);
}

size_t serBuffIdx = 0;
char serBuff[20] = "400";
char serCmd = '\0';

void logPID_Params() {
  Serial.print("Kp = " );
  Serial.print(throttleVars.Kp);
  Serial.print(", Ki = " );
  Serial.print(throttleVars.Ki);
  Serial.print(", Kd = " );
  Serial.println(throttleVars.Kd);
}

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
          case 's':
            if (throttleVars.ctrl0.setpointOverride.enabled) {
              throttleVars.ctrl0.setpointOverride.value = val;
              Serial.print("setpointOverride = ");
              Serial.println(val);
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
            throttleVars.ctrl0.setpointOverride.enabled ^= 1;
            if (throttleVars.ctrl0.setpointOverride.enabled) {
              Serial.println("setpoint from overrides");
              Serial.println("use 's' command or 'a' to override");
            } else {
              Serial.println("setpoint from PPS");
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
  doSerial();

  // FIXME call this at a regular interval based on 'pidSampleRate_ms'
  throttle.run();
}
