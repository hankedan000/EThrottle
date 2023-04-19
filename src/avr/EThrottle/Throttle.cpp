#include "FlashUtils.h"
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
 , sensorSetup_()
 , ppsCompareThresh_(50)
 , tpsCompareThresh_(50)
{
  status_.pidAutoTuneBusy = 0;
  status_.motorEnabled = 0;
  status_.ppsComparisonFault = 0;
  status_.tpsComparisonFault = 0;

  // zero out coefficients to be safe
  updatePID_Coeffs(0.0,0.0,0.0);

  // defaults. these can get changed via setSensorSetup()
  sensorSetup_.comparePPS = 0;
  sensorSetup_.preferPPS_A = 1;
  sensorSetup_.compareTPS = 0;
  sensorSetup_.preferTPS_A = 0;
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
Throttle::disableMotor()
{
  digitalWrite(driverPinDis_, 1);
  status_.motorEnabled = 0;
}

void
Throttle::enableMotor()
{
  digitalWrite(driverPinDis_, 0);
  status_.motorEnabled = 1;
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
Throttle::setRangeCalPPS_A(
  Throttle::RangeCalibration rc)
{
  ppsCalA_ = rc;
}

void
Throttle::setRangeCalPPS_B(
  Throttle::RangeCalibration rc)
{
  ppsCalB_ = rc;
}

void
Throttle::setRangeCalTPS_A(
  Throttle::RangeCalibration rc)
{
  tpsCalA_ = rc;
}

void
Throttle::setRangeCalTPS_B(
  Throttle::RangeCalibration rc)
{
  tpsCalB_ = rc;
}

void
Throttle::setSensorSetup(
  Throttle::SensorSetup setup,
  const FlashTableDescriptor &ppsCompDesc,
  const FlashTableDescriptor &tpsCompDesc,
  uint16_t ppsCompareThresh,
  uint16_t tpsCompareThresh)
{
  sensorSetup_ = setup;
  ppsCompDesc_ = ppsCompDesc;
  tpsCompDesc_ = tpsCompDesc;
  ppsCompareThresh_ = ppsCompareThresh;
  tpsCompareThresh_ = tpsCompareThresh;
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
    outVars_->status = status_;
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
  if (status_.pidAutoTuneBusy) {
    return;
  }

  const uint16_t AUTOTUNE_TARGET = 5000;// 50%

  status_.pidAutoTuneBusy = 1;
  tuner_.setTargetInputValue(AUTOTUNE_TARGET);
  tuner_.startTuningLoop(micros());
}

void
Throttle::stopPID_AutoTune()
{
  status_.pidAutoTuneBusy = 0;
}

void
Throttle::doCurrentMonitor()
{
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

  // normalize PPS readings based on calibrated min/max values
  int16_t ppsA_Norm = map(ppsA_, ppsCalA_.min, ppsCalA_.max, 0, 10000);
  int16_t ppsB_Norm = map(ppsB_, ppsCalB_.min, ppsCalB_.max, 0, 10000);

  DEBUG("ppsA: %d, ppsB: %d", ppsA_, ppsB_);
  DEBUG("ppsA_Norm: %d, ppsB_Norm: %d", ppsA_Norm, ppsB_Norm);

  // Compute PPS percentage based on prefered sensor's normalized
  // percentage. The tuner should set the prefered sensor (A or B)
  // based on which one gives readings over the full range of the
  // accelerator pedal.
  // Note: pps_ can get set to 0% if PPS safety checks fail.
  pps_ = (sensorSetup_.preferPPS_A ? ppsA_Norm : ppsB_Norm);
  if (pps_ < 0) {
    pps_ = 0;
  } else if (pps_ > 10000) {
    pps_ = 10000;
  }

  // safety check the raw ADC values
  if (sensorSetup_.comparePPS)
  {
    const int16_t preferADC = (sensorSetup_.preferPPS_A ? ppsA_ : ppsB_);
    const int16_t otherADC = (sensorSetup_.preferPPS_A ? ppsB_ : ppsA_);

    // lookup what we expect the other sensor's ADC value to be based
    // on the prefered sensor's reading.
    const int16_t otherExpected = FlashUtils::lerpS16(
      ppsCompDesc_.xBinsFlashOffset,
      ppsCompDesc_.yBinsFlashOffset,
      ppsCompDesc_.nBins,
      preferADC);
    const int16_t ppsDelta = otherADC - otherExpected;
    DEBUG(
      "preferADC = %d, otherADC = %d, otherExpected = %d, ppsDelta = %d",
      preferADC,
      otherADC,
      otherExpected,
      ppsDelta);

    if (outVars_)
    {
      EndianUtils::setBE(outVars_->ppsSafetyDelta, ppsDelta);
    }

    if (abs(ppsDelta) >= ppsCompareThresh_)
    {
      pps_ = 0;// default to 0% pedal position to be safe
      // TODO add error counter?
      status_.ppsComparisonFault = 1;
    }
  }

  DEBUG("pps_: %d", pps_);
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

  // Compute TPS percentage based on prefered sensor's normalized
  // percentage. The tuner should set the prefered sensor (A or B)
  // based on which one gives readings over the full range of the
  // throttle blade.
  // Note: tps_ can get set to 0% if TPS safety checks fail.
  tps_ = (sensorSetup_.preferTPS_A ? tpsA_Norm : tpsB_Norm);

  // safety check the raw ADC values
  if (sensorSetup_.compareTPS)
  {
    const int16_t preferADC = (sensorSetup_.preferTPS_A ? tpsA_ : tpsB_);
    const int16_t otherADC = (sensorSetup_.preferTPS_A ? tpsB_ : tpsA_);

    // lookup what we expect the other sensor's ADC value to be based
    // on the prefered sensor's reading.
    const int16_t otherExpected = FlashUtils::lerpS16(
      tpsCompDesc_.xBinsFlashOffset,
      tpsCompDesc_.yBinsFlashOffset,
      tpsCompDesc_.nBins,
      preferADC);
    const int16_t tpsDelta = otherADC - otherExpected;
    DEBUG(
      "preferADC = %d, otherADC = %d, otherExpected = %d, tpsDelta = %d",
      preferADC,
      otherADC,
      otherExpected,
      tpsDelta);

    if (outVars_)
    {
      EndianUtils::setBE(outVars_->tpsSafetyDelta, tpsDelta);
    }

    if (abs(tpsDelta) >= tpsCompareThresh_)
    {
      // Best to disable motor since we have unreliable position info for the
      // throttle blade. This assumes the throttle has a return spring that will
      // close the throttle blade when the motor is unpowered.
      disableMotor();
      tps_ = 0;
      // TODO add error counter?
      status_.tpsComparisonFault = 1;
    }
  }

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
  if (status_.pidAutoTuneBusy) {
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
