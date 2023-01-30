#include <PID_v1.h>
#include <string.h>
#include <stdlib.h>

// analog inputs
#define TPS_N_PIN A4 // non-inverted TPS pin
#define TPS_I_PIN A5 // inverted TPS pin
#define PPS_A_PIN A0
#define PPS_B_PIN A1

// digital outputs
#define MOTOR_P 3// pin needs PWM support
#define MOTOR_N 5// pin needs PWM support

// define this if the motor driver is an h-bridge variant. this
// enables us to reverse the throttle motor polarity to close
// the blade faster than the return spring would allow.
// if unset, then the code will rely on the return spring to
// bring the throttle blade closed (slow but still functional)
#define SUPPORT_H_BRIDGE

// the highest setpoint the throttle PID controller can target
// this is the highest TPS sensor reading we get if you manually
// open the throttle to "wide open".
#define MAX_SETPOINT 856 // TPS reading at 100% throttle
#define MIN_SETPOINT 190 // TPS reading at 0% throttle

// TODO put these in a calibrations table
#define PPSA_0_ADC   183 // PPSA ADC reading at 0% pedal
#define PPSB_0_ADC   29  // PPSB ADC reading at 0% pedal
#define PPSA_100_ADC 1023// PPSA ADC reading at 100% pedal
#define PPSB_100_ADC 875 // PPSB ADC reading at 100% pedal

// raw ADC values for throttle position sensors
uint16_t tpsN = 0;
uint16_t tpsI = 0;

double pidSetpoint = 0;
double pidIn = 0;
double pidOut = 0;

// raw ADC values for pedal position sensors
uint16_t ppsa = 0;
uint16_t ppsb = 0;

/**
 * pedal position sensor percentage.
 *  * computed in doPedal()
 *  * saturated to fit within valid range
 *  * range: 0-10000 (0-100%)
 */
int16_t pps = 0;

//Specify the links and initial tuning parameters
double Kp=6.0, Ki=20.0, Kd=0.0;
PID pid(&pidIn,&pidOut,&pidSetpoint,Kp,Ki,Kd, DIRECT);

void setup() {
  // help reduce buzzing sounds from throttle body
  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz

  pinMode(TPS_N_PIN, INPUT);
  pinMode(TPS_I_PIN, INPUT);
  pinMode(PPS_A_PIN, INPUT);
  pinMode(PPS_B_PIN, INPUT);

  pinMode(MOTOR_P, OUTPUT);
  pinMode(MOTOR_N, OUTPUT);

  analogWrite(MOTOR_P, 0);
  analogWrite(MOTOR_N, 0);

  pid.SetMode(AUTOMATIC);
#ifdef SUPPORT_H_BRIDGE
  // negative range is used to reverse motor to close throttle
  pid.SetOutputLimits(-255, 255);
#else
  pid.SetOutputLimits(0, 255);
#endif
  pid.SetSampleTime(10);
  pidSetpoint = 400;
  
  Serial.begin(115200);
}

size_t serBuffIdx = 0;
char serBuff[20] = "400";
char serCmd = '\0';
bool logPID_In = false;
bool autoSetpoint = false;
bool setpointFromPPS = true;
uint16_t prevAutoSetpoint_ms = 0;
double prevSetpoint = 0.0;

void logPID_Params() {
  Serial.print("Kp = " );
  Serial.print(pid.GetKp());
  Serial.print(", Ki = " );
  Serial.print(pid.GetKi());
  Serial.print(", Kd = " );
  Serial.println(pid.GetKd());
}

// a    -> toggle auto setpoint (setpoint with flip from 0 to setpoint every 2s)
// s### -> set 'pidSetpoint' to ###
// p### -> set 'Kp' to ###
// i### -> set 'Ki' to ###
// d### -> set 'Kd' to ###
// l    -> toggle logging of 'pidIn'
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
            prevSetpoint = pidSetpoint;
            autoSetpoint = ! autoSetpoint;
            break;
          case 's':
            if ( ! setpointFromPPS) {
              pidSetpoint = val;
              Serial.print("pidSetpoint = ");
              Serial.println(val);
            }
            break;
          case 'p':
            Kp = val;
            break;
          case 'i':
            Ki = val;
            break;
          case 'd':
            Kd = val;
            break;
          case 'l':
            logPID_In = ! logPID_In;
            break;
          case 'P':
            logPID_Params();
            break;
          case 'm':
            setpointFromPPS = ! setpointFromPPS;
            if (setpointFromPPS) {
              Serial.println("setpoint from PPS");
            } else {
              Serial.println("setpoint from overrides");
              Serial.println("use 's' command or 'a' to override");
            }
            break;
        }

        if (serCmd == 'p' || serCmd == 'i' || serCmd == 'd') {
          pid.SetTunings(Kp,Ki,Kd);
          logPID_Params();
        }

        serCmd = '\0';// restart command parsing
      }
      serBuffIdx++;
    }
  }
}

void doPedal() {
  ppsa = analogRead(PPS_A_PIN);
  ppsb = analogRead(PPS_B_PIN);

  // TODO add logic to safety check the raw ADC values

//  Serial.print("ppsa: ");
//  Serial.print(ppsa);
//  Serial.print(", ppsb: ");
//  Serial.println(ppsb);

  // normalize PPS reading based on calibrated min/max values
  int16_t ppsa_norm = map(ppsa, PPSA_0_ADC, PPSA_100_ADC, 0, 1024);
  int16_t ppsb_norm = map(ppsb, PPSB_0_ADC, PPSB_100_ADC, 0, 1024);

//  Serial.print("ppsa_norm: ");
//  Serial.print(ppsa_norm);
//  Serial.print(", ppsb_norm: ");
//  Serial.println(ppsb_norm);

  // once we've safety checked the ADCs we can just use one sensor value
  // to compute the final PPS percentage. i'm favoring ppsb here because
  // it does a better job at covering the full range of motion. ppsa
  // saturates right before 100% throttle.
  // TODO: add configurable 'favored pps' option
  pps = map(ppsb_norm, 0, 1024, 0, 10000);
  if (pps < 0) {
    pps = 0;
  } else if (pps > 10000) {
    pps = 10000;
  }

//  Serial.print("pps: ");
//  Serial.println(pps);
}

void doThrottle() {
//  tpsN = analogRead(TPS_N_PIN);
  tpsI = analogRead(TPS_I_PIN);

  if (setpointFromPPS) {
    pidSetpoint = map(pps,0,10000,MIN_SETPOINT,MAX_SETPOINT);
  } else if (autoSetpoint) {
    uint16_t currMillis = millis();
    if ((currMillis - prevAutoSetpoint_ms) > 2000) {
      if (pidSetpoint > 0) {
        pidSetpoint = 0;
      } else {
        pidSetpoint = prevSetpoint;
      }
      
      prevAutoSetpoint_ms = currMillis;
    }
  }

  // sanitize PID inputs
  if (pidSetpoint > MAX_SETPOINT) {
    pidSetpoint = MAX_SETPOINT;
  } else if (pidSetpoint < MIN_SETPOINT) {
    pidSetpoint = MIN_SETPOINT;
  }

  pidIn = tpsI;
  pid.Compute();

  if (logPID_In) {
    Serial.print("pidIn:");
    Serial.println(pidIn);
  }

  // negative PWM means we need to close throttle; results in inverse
  // motor polarity in H-Bridge driver use cases, or just undriven
  // motor otherwise (relies on throttle body return spring to close)
  int16_t pwmOut = pidOut;

#ifdef SUPPORT_H_BRIDGE
  // handle h-bridge polarity inversion logic
  if (pwmOut > 0) {
    analogWrite(MOTOR_P, pwmOut);
    analogWrite(MOTOR_N, 0);
  } else if (pwmOut < 0) {
    analogWrite(MOTOR_P, 0);
    analogWrite(MOTOR_N, pwmOut * -1);// * -1 to write PWM magnitude only
  } else {
    analogWrite(MOTOR_P, 0);
    analogWrite(MOTOR_N, 0);
  }
#else
  // no h-bridge means we can only drive the motor 1 way
  analogWrite(MOTOR_P, pwmOut);
  analogWrite(MOTOR_N, 0);
#endif
}

void loop() {
  doSerial();
  doPedal();
  doThrottle();
}