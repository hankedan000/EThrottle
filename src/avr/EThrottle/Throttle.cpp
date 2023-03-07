#include "Throttle.h"

#include <Arduino.h>

Throttle::Throttle(
    uint8_t tpsPinA,
    uint8_t tpsPinB,
    uint8_t ppsPinA,
    uint8_t ppsPinB,
    uint8_t motorPinP,
    uint8_t motorPinN,
    RAM_Vars *vars)
 : tpsPinA_(tpsPinA)
 , tpsPinB_(tpsPinB)
 , ppsPinA_(ppsPinA)
 , ppsPinB_(ppsPinB)
 , motorPinP_(motorPinP)
 , motorPinN_(motorPinN)
 , vars_(vars)
 , pidSampleRate_ms_(100)
 , pid_(&pidIn_,&pidOut_,&pidSetpoint_,0,0,0, DIRECT)
{
  // FIXME restore clibration from EEPROM
  ppsCalA_.min = 183;
  ppsCalA_.max = 1023;
  ppsCalB_.min = 29;
  ppsCalB_.max = 875;
  tpsCalA_.min = 0;
  tpsCalA_.max = 1023;
  tpsCalB_.min = 190;
  tpsCalB_.max = 856;

  // FIXME restore coefficients from EEPROM
  vars_->Kp = 6.0;
  vars_->Ki = 20.0;
  vars_->Kd = 0.0;
  updatePID_Coeffs();
}

void
Throttle::init(
    uint8_t pidSampleRate_ms)
{
  pidSampleRate_ms_ = pidSampleRate_ms;

  pinMode(tpsPinA_, INPUT);
  pinMode(tpsPinB_, INPUT);
  pinMode(ppsPinA_, INPUT);
  pinMode(ppsPinB_, INPUT);

  pinMode(motorPinP_, OUTPUT);
  pinMode(motorPinN_, OUTPUT);

  // disable motor
  analogWrite(motorPinP_, 0);
  analogWrite(motorPinN_, 0);
  
  // setup the PID controller class
  pid_.SetMode(AUTOMATIC);
#ifdef SUPPORT_H_BRIDGE
  // negative range is used to reverse motor to close throttle
  pid_.SetOutputLimits(-255, 255);
#else
  pid_.SetOutputLimits(0, 255);
#endif
  pid_.SetSampleTime(pidSampleRate_ms_);
}

void
Throttle::run()
{
  doPedal();
  doThrottle();
}

void
Throttle::updatePID_Coeffs()
{
  pid_.SetTunings(vars_->Kp,vars_->Ki,vars_->Kd);
}

void
Throttle::doPedal()
{
  vars_->ppsA = analogRead(ppsPinA_);
  vars_->ppsB = analogRead(ppsPinB_);

  // TODO add logic to safety check the raw ADC values

  // Serial.print("ppsA: ");
  // Serial.print(vars_->ppsA);
  // Serial.print(", ppsB: ");
  // Serial.println(vars_->ppsB);

  // normalize PPS readings based on calibrated min/max values
  int16_t ppsA_Norm = map(vars_->ppsA, ppsCalA_.min, ppsCalA_.max, 0, 1024);
  int16_t ppsB_Norm = map(vars_->ppsB, ppsCalB_.min, ppsCalB_.max, 0, 1024);

  // Serial.print("ppsA_Norm: ");
  // Serial.print(ppsA_Norm);
  // Serial.print(", ppsB_Norm: ");
  // Serial.println(ppsB_Norm);

  // once we've safety checked the ADCs we can just use one sensor value
  // to compute the final PPS percentage. i'm favoring ppsb here because
  // it does a better job at covering the full range of motion. ppsa
  // saturates right before 100% throttle.
  // TODO: add configurable 'favored pps' option
  vars_->pps = map(ppsA_Norm, 0, 1024, 0, 10000);
  if (vars_->pps < 0) {
    vars_->pps = 0;
  } else if (vars_->pps > 10000) {
    vars_->pps = 10000;
  }

  // Serial.print("pps: ");
  // Serial.println(vars_->pps);
}

void
Throttle::doThrottle()
{
  // vars_->tpsA = analogRead(tpsPinA_);
  vars_->tpsB = analogRead(tpsPinB_);

  // normalize TPS readings based on calibrated min/max values
  int16_t tpsA_Norm = map(vars_->tpsA, tpsCalA_.min, tpsCalA_.max, 0, 10000);
  int16_t tpsB_Norm = map(vars_->tpsB, tpsCalB_.min, tpsCalB_.max, 0, 10000);

  uint16_t setpoint = 0;
  if (vars_->ctrl0.setpointOverride.enabled) {
    setpoint = vars_->ctrl0.setpointOverride.value;
  } else {
    setpoint = vars_->pps;
  }

  // sanitize PID inputs
  if (setpoint > 10000) {
    setpoint = 10000;
  }

  // update PID control variables
  pidSetpoint_ = setpoint;  
  pidIn_ = tpsB_Norm;
  pid_.Compute();

  // Serial.print("pidIn:");
  // Serial.println(pidIn);

  // negative PWM means we need to close throttle; results in inverse
  // motor polarity in H-Bridge driver use cases, or just undriven
  // motor otherwise (relies on throttle body return spring to close)
  vars_->motorOut = pidOut_;

#ifdef SUPPORT_H_BRIDGE
  // handle h-bridge polarity inversion logic
  if (vars_->motorOut > 0) {
    analogWrite(motorPinP_, vars_->motorOut);
    analogWrite(motorPinN_, 0);
  } else if (vars_->motorOut < 0) {
    analogWrite(motorPinP_, 0);
    analogWrite(motorPinN_, vars_->motorOut * -1);// * -1 to write PWM magnitude only
  } else {
    analogWrite(motorPinP_, 0);
    analogWrite(motorPinN_, 0);
  }
#else
  // no h-bridge means we can only drive the motor 1 way
  analogWrite(motorPinP_, vars_->motorOut);
  analogWrite(motorPinN_, 0);
#endif
}