#include "EndianUtils.h"
#include "Throttle.h"

#include <Arduino.h>
#include <logging.h>

Throttle throttle(
  TPS_A_PIN,TPS_B_PIN,
  PPS_A_PIN,PPS_B_PIN,
  DRIVER_P,DRIVER_N,
  DRIVER_DIS,
  DRIVER_FS,
  DRIVER_FB);

Throttle::Throttle(
    uint8_t tpsPinA,
    uint8_t tpsPinB,
    uint8_t ppsPinA,
    uint8_t ppsPinB,
    uint8_t driverPinP,
    uint8_t driverPinN,
    uint8_t driverPinDis,
    uint8_t driverPinFS,
    uint8_t driverPinFB)
 : tpsPinA_(tpsPinA)
 , tpsPinB_(tpsPinB)
 , ppsPinA_(ppsPinA)
 , ppsPinB_(ppsPinB)
 , driverPinP_(driverPinP)
 , driverPinN_(driverPinN)
 , driverPinDis_(driverPinDis)
 , driverPinFS_(driverPinFS)
 , driverPinFB_(driverPinFB)
 , outVars_(nullptr)
 , pidSampleRate_ms_(100)
 , pid_(&pidIn_,&pidOut_,&pidSetpoint_,0,0,0, DIRECT)
 , setpointSource_(SetpointSource_E::eSS_PPS)
 , userSetpoint_(0)
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

  // zero out coefficients to be safe
  updatePID_Coeffs(0.0,0.0,0.0);
}

void
Throttle::init(
    uint8_t pidSampleRate_ms,
    OutVars *outVars)
{
  pidSampleRate_ms_ = pidSampleRate_ms;
  outVars_ = outVars;

  pinMode(driverPinP_, OUTPUT);
  pinMode(driverPinN_, OUTPUT);
  pinMode(driverPinDis_, OUTPUT);
  pinMode(driverPinFS_, INPUT);

  enableMotor();
  analogWrite(driverPinP_,0);
  analogWrite(driverPinN_,0);

  // help reduce buzzing sounds from throttle body
  // set timer0 prescaler of 8 (gives PWM freq of 7812.5Hz)
  // FIXME timer0 is used for digitaly pins 5 and 6 which
  // doesn't scale with programmable I/O pins like class claims
  #define TMR0_CLKPR_OFF  0x0
  #define TMR0_CLKPR_1    0x1
  #define TMR0_CLKPR_8    0x2
  #define TMR0_CLKPR_64   0x3
  #define TMR0_CLKPR_256  0x4
  #define TMR0_CLKPR_1024 0x5
  TCCR0B = TMR0_CLKPR_8;

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
Throttle::setSetpointSource(
  SetpointSource_E source)
{
  setpointSource_ = source;
}

Throttle::SetpointSource_E
Throttle::getSetpointSource() const
{
  return setpointSource_;
}

void
Throttle::setSetpointOverride(
  double value)
{
  if (setpointSource_ == SetpointSource_E::eSS_User)
  {
    if (value >= 0.0 && value <= 100.0)
    {
      userSetpoint_ = value * 100.0;
    }
  }
}

void
Throttle::run()
{
  doPedal();
  doThrottle();

  DEBUG("driverFB: %4d; motorCurrent: %4d mA", driverFB_, motorCurrent_mA_);

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
    EndianUtils::setBE(outVars_->motorCurrent_mA, motorCurrent_mA_);
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
  tuner_.setTargetInputValue(AUTOTUNE_TARGET);
  tuner_.startTuningLoop(micros());
}

void
Throttle::stopPID_AutoTune()
{
  outVars_->status.pidAutoTuneBusy = 0;
}

void
Throttle::doCurrentMonitor()
{
  // FIXME this gets called within timer0 output compare interrupt
  // context. if i perform this analogRead() it seems like we never
  // unblock from the call.
  driverFB_ = analogRead(driverPinFB_);

  // feedback pin drives 1/375th the current to ground through 100ohm resistor.
  // V_fb = ADC * 5.0v / 1023
  // V_fb = I * R = I_fb * 100ohm
  // I_fb = I_m * 1/375
  //  V_fb -> voltage see over 100ohm resistor
  //  I_fb -> current out of driver's feedback pin (proportional to I_m)
  //  I_m  -> current through motor
  // solve for 'I_m' using the above equations:
  //  I_m = ADC * (5/1023) * (375/100)
  //  I_m = ADC * 0.018328 <- in amps
  motorCurrent_mA_ = driverFB_ * 18.328;
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

  switch (setpointSource_)
  {
    case SetpointSource_E::eSS_PPS:
      tpsTarget_ = pps_;
      break;
    case SetpointSource_E::eSS_User:
      tpsTarget_ = userSetpoint_;
      break;
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
      INFO(
        "Kp: %d, Ki: %d, Kd: %d",
        (uint16_t)(tuner_.getKp() * 100),
        (uint16_t)(tuner_.getKi() * 100),
        (uint16_t)(tuner_.getKd() * 100));
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
