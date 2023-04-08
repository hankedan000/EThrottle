#include "EndianUtils.h"
#include "Throttle.h"

#include <Arduino.h>
#include <logging.h>

Throttle::Throttle(
    uint8_t tpsPinA,
    uint8_t tpsPinB,
    uint8_t ppsPinA,
    uint8_t ppsPinB,
    uint8_t driverPinP,
    uint8_t driverPinN,
    uint8_t driverPinDis,
    uint8_t driverPinFS,
    uint8_t driverPinFB,
    OutVars *outVars)
 : tpsPinA_(tpsPinA)
 , tpsPinB_(tpsPinB)
 , ppsPinA_(ppsPinA)
 , ppsPinB_(ppsPinB)
 , driverPinP_(driverPinP)
 , driverPinN_(driverPinN)
 , driverPinDis_(driverPinDis)
 , driverPinFS_(driverPinFS)
 , driverPinFB_(driverPinFB)
 , outVars_(outVars)
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
  updatePID_Coeffs(0.33,0.17,0.0);
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

  pinMode(driverPinP_, OUTPUT);
  pinMode(driverPinN_, OUTPUT);
  pinMode(driverPinDis_, OUTPUT);
  pinMode(driverPinFS_, INPUT);
  pinMode(driverPinFB_, INPUT);

  enableMotor();
  analogWrite(driverPinP_, 0);
  analogWrite(driverPinN_, 0);
  
  // setup the PID controller class
  pid_.SetMode(AUTOMATIC);
#ifdef SUPPORT_H_BRIDGE
  // negative range is used to reverse motor to close throttle
  pid_.SetOutputLimits(-255, 255);
  tuner_.setOutputRange(-255, 255);
#else
  pid_.SetOutputLimits(0, 255);
  tuner_.setOutputRange(0, 255);
#endif

  pid_.SetSampleTime(pidSampleRate_ms_);
  tuner_.setZNMode(PIDAutotuner::ZNModeNoOvershoot);
  tuner_.setLoopInterval(pidSampleRate_ms_ * 1000);
}

void
Throttle::disableMotor() const
{
  digitalWrite(driverPinDis_, 1);
}

void
Throttle::enableMotor() const
{
  digitalWrite(driverPinDis_, 0);
}

void
Throttle::run()
{
  doPedal();
  doThrottle();

  if (outVars_)
  {
    EndianUtils::setBE(outVars_->tpsA, tpsA_);
    EndianUtils::setBE(outVars_->tpsB, tpsB_);
    EndianUtils::setBE(outVars_->tps, tps_);
    EndianUtils::setBE(outVars_->ppsA, ppsA_);
    EndianUtils::setBE(outVars_->ppsB, ppsB_);
    EndianUtils::setBE(outVars_->pps, pps_);
    EndianUtils::setBE(outVars_->tpsTarget, tpsTarget_);
    EndianUtils::setBE(outVars_->motorOut, motorOut_);
  }
}

void
Throttle::updatePID_Coeffs(
  double Kp,
  double Ki,
  double Kd)
{
  pid_.SetTunings(Kp,Ki,Kd);
}

double
Throttle::getKp()
{
  return pid_.GetKp();
}

double
Throttle::getKi()
{
  return pid_.GetKi();
}

double
Throttle::getKd()
{
  return pid_.GetKd();
}

void
Throttle::startPID_AutoTune()
{
  if (outVars_->status.pidAutoTuneBusy) {
    return;
  }

  const uint16_t AUTOTUNE_TARGET = 5000;// 50%

  outVars_->status.pidAutoTuneBusy = 1;
  outVars_->ctrl.setpointOverride.enabled = 1;
  outVars_->ctrl.setpointOverride.value = AUTOTUNE_TARGET;// 50%
  tuner_.setTargetInputValue(AUTOTUNE_TARGET);
  tuner_.startTuningLoop(micros());
}

void
Throttle::stopPID_AutoTune()
{
  outVars_->status.pidAutoTuneBusy = 0;
  outVars_->ctrl.setpointOverride.enabled = 0;
  outVars_->ctrl.setpointOverride.value = 0;
}

void
Throttle::doPedal()
{
  ppsA_ = analogRead(ppsPinA_);
  ppsB_ = analogRead(ppsPinB_);

  // TODO add logic to safety check the raw ADC values

  // normalize PPS readings based on calibrated min/max values
  int16_t ppsA_Norm = map(ppsA_, ppsCalA_.min, ppsCalA_.max, 0, 10000);
  int16_t ppsB_Norm = map(ppsB_, ppsCalB_.min, ppsCalB_.max, 0, 10000);

  DEBUG("ppsA: %d, ppsB: %d", ppsA_, ppsB_);
  DEBUG("ppsA_Norm: %d, ppsB_Norm: %d", ppsA_Norm, ppsB_Norm);

  // once we've safety checked the ADCs we can just use one sensor value
  // to compute the final PPS percentage. i'm favoring ppsb here because
  // it does a better job at covering the full range of motion. ppsa
  // saturates right before 100% throttle.
  // TODO: add configurable 'favored pps' option
  pps_ = ppsA_Norm;
  if (pps_ < 0) {
    pps_ = 0;
  } else if (pps_ > 10000) {
    pps_ = 10000;
  }

  DEBUG("pps: %d", pps);
}

void
Throttle::doThrottle()
{
  tpsA_ = analogRead(tpsPinA_);
  tpsB_ = analogRead(tpsPinB_);

  // normalize TPS readings based on calibrated min/max values
  int16_t tpsA_Norm = map(tpsA_, tpsCalA_.min, tpsCalA_.max, 0, 10000);
  int16_t tpsB_Norm = map(tpsB_, tpsCalB_.min, tpsCalB_.max, 0, 10000);

  DEBUG("tpsA: %d, tpsB: %d", tpsA_Norm, tpsB_Norm);

  // TODO merge tpsA and tpsB & safety check
  tps_ = tpsB_Norm;

  if (outVars_->ctrl.setpointOverride.enabled) {
    tpsTarget_ = outVars_->ctrl.setpointOverride.value;
  } else {
    tpsTarget_ = pps_;
  }

  // update PID control variables
  pidSetpoint_ = tpsTarget_;  
  pidIn_ = tps_;
  pid_.Compute();

  DEBUG("pidIn_: %f", pidIn_);

  // handle PID auto-tune logic
  if (outVars_->status.pidAutoTuneBusy) {
    pidOut_ = tuner_.tunePID(pidIn_, micros());

    if (tuner_.isFinished()) {
      stopPID_AutoTune();

      INFO("auto tune done!");
      INFO("Kp: %f; Ki: %f; Kd: %f",
        tuner_.getKp(),
        tuner_.getKi(),
        tuner_.getKd());
    }
  }

  // negative PWM means we need to close throttle; results in inverse
  // motor polarity in H-Bridge driver use cases, or just undriven
  // motor otherwise (relies on throttle body return spring to close)
  motorOut_ = pidOut_;

#ifdef SUPPORT_H_BRIDGE
  // handle h-bridge polarity inversion logic
  if (motorOut_ > 0) {
    analogWrite(driverPinP_, motorOut_);
    analogWrite(driverPinN_, 0);
  } else if (motorOut_ < 0) {
    analogWrite(driverPinP_, 0);
    analogWrite(driverPinN_, motorOut_ * -1);// * -1 to write PWM magnitude only
  } else {
    analogWrite(driverPinP_, 0);
    analogWrite(driverPinN_, 0);
  }
#else
  // no h-bridge means we can only drive the motor 1 way
  analogWrite(driverPinP_, motorOut_);
  analogWrite(driverPinN_, 0);
#endif
}