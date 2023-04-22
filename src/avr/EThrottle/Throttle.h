#ifndef THROTTLE_H_
#define THROTTLE_H_

#include <PID_v1.h>
#include <stdint.h>
#include "pidautotuner.h"

// define this if the motor driver is an h-bridge variant. this
// enables us to reverse the throttle motor polarity to close
// the blade faster than the return spring would allow.
// if unset, then the code will rely on the return spring to
// bring the throttle blade closed (slow but still functional)
#define SUPPORT_H_BRIDGE

// digital outputs
#define DRIVER_DIS 4
#define DRIVER_P   5// pin needs PWM support (PD5 OC0B)
#define DRIVER_N   6// pin needs PWM support (PD6 OC0A)
#define DRIVER_FS  7

class Throttle
{
public:

  enum SetpointSource_E
  {
    eSS_PPS = 0,
    eSS_User = 1
  };

  struct RangeCalibration
  {
    uint16_t min;
    uint16_t max;
  };

  struct SensorSetup
  {
    uint8_t comparePPS    : 1;
    uint8_t preferPPS_A   : 1;
    uint8_t compareTPS    : 1;
    uint8_t preferTPS_A   : 1;
    uint8_t rsvd          : 4;
  };

  struct FlashTableDescriptor
  {
    uint16_t xBinsFlashOffset;
    uint16_t yBinsFlashOffset;
    uint8_t nBins;
  };

  struct Status
  {
    uint8_t pidAutoTuneBusy    : 1;
    uint8_t ppsComparisonFault : 1;
    uint8_t tpsComparisonFault : 1;
    uint8_t throttleEnabled    : 1;
    uint8_t motorEnabled       : 1;
    uint8_t motorDriverFault   : 1;// over current or over temp.
    uint8_t reserved           : 2;
  };

  enum FaultClearCmd_E
  {
    eFCC_All = 'A',
    eFCC_Driver = 'd',
    eFCC_PPS = 'p',
    eFCC_TPS = 't',
  };

  /**
   * various RAM variables that we allow public access to
   * for instrumentation and data logging purposes.
   * All values are stored in big-endian (MegaSquirt is BE).
   */
  struct OutVars
  {
    // ADC readings from throttle position sensors A & B
    // range: [0 to 1023]
    uint16_t tpsA;
    uint16_t tpsB;
    // finalized throttle position based on both A & B sensor readings
    // range: [-10000 to 10000] (ie. -100 to 100%)
    int16_t tps;

    // ADC readings from pedal position sensors A & B
    // range: [0 to 1023]
    uint16_t ppsA;
    uint16_t ppsB;
    // finalized pedal position based on both A & B sensor readings
    // range: [-10000 to 10000] (ie. -100 to 100%)
    int16_t pps;

    // Throttle position the PID controller is targeting
    // tpsTarget = idleAdder + ppsAdder
    // range: [-10000 to 10000] (ie. -100 to 100%)
    int16_t tpsTarget;

    // PWM motor driver output
    // range: [-255 to 255] if in h-bridge mode (negative means reverse)
    //        [0 to 255] if in normal mode
    int16_t motorOut;

    // motor current in milliamps
    // range: [0 to 65535] in milliamps
    uint16_t motorCurrent_mA;

    Status status;

    // when using redundant PPS sensors, this value represents the delta
    // of the secondary sensor's expected value compared to its measure
    // value.
    int16_t ppsSafetyDelta;

    // when using redundant TPS sensors, this value represents the delta
    // of the secondary sensor's expected value compared to its measure
    // value.
    int16_t tpsSafetyDelta;

    // portion of the tpsTarget that's coming from engine idle control.
    // currently just a fixed value from flash, but will eventually have
    // its own PID too.
    // range: [-10000 to 10000] (ie. -100 to 100%)
    int16_t idleAdder;

    // portion of the tpsTarget that's coming from the accelerator pedal
    // ppsAdder = ((10000 - idleAdder) * pps) / 10000
    // range: [-10000 to 10000] (ie. -100 to 100%)
    int16_t ppsAdder;

    // ADC readings from motor driver feedback pin
    // range: [0 to 1023]
    uint16_t driverFB;
  };

public:
  Throttle(
    uint8_t driverPinP,
    uint8_t driverPinN,
    uint8_t driverPinDis,
    uint8_t driverPinFS);

  /**
   * Configures IO pins and internal classes.
   * Normally called in setup()
   */
  void
  init(
    uint8_t pidSampleRate_ms,
    OutVars *outVars);

  /**
   * disables the motor driver and the PID control loop.
   * sensor reading still occurs.
   */
  void
  disableThrottle();

  /**
   * enables the motor driver and the PID control loop
   */
  void
  enableThrottle();

  void
  setSetpointSource(
    SetpointSource_E source);

  SetpointSource_E
  getSetpointSource() const;

  void
  setRangeCalPPS_A(
    RangeCalibration rc);

  void
  setRangeCalPPS_B(
    RangeCalibration rc);

  void
  setRangeCalTPS_A(
    RangeCalibration rc);

  void
  setRangeCalTPS_B(
    RangeCalibration rc);

  void
  setSensorSetup(
    SensorSetup setup,
    const FlashTableDescriptor &ppsCompDesc,
    const FlashTableDescriptor &tpsCompDesc,
    uint16_t ppsCompareThresh,
    uint16_t tpsCompareThresh,
    uint16_t tpsStall);

  /**
   * Setter for the override setpoint value. Only does something if
   * the setpoint source is set to 'eSS_User' via setSetpointSource().
   * @param[in] value
   * the override value in percent (0.0 to 100.0)
   */
  void
  setSetpointOverride(
    double value);

  const Status &
  status() const;

  void
  clearFault(
    FaultClearCmd_E cmd);

  /**
   * Call this method every sample interval.
   * Ideally this method is called within a timer interrupt
   * routine.
   */
  void
  run();

  // pushes the PID coefficients into the controller
  void
  updatePID_Coeffs(
    double Kp,
    double Ki,
    double Kd);

  double
  getKp();

  double
  getKi();

  double
  getKd();

  void
  startPID_AutoTune();

  void
  stopPID_AutoTune();

private:
  void
  disableMotor();

  void
  enableMotor();

  void
  doPedal();

  void
  doThrottle();

  void
  doMotorCurrent();

private:
    uint8_t driverPinP_;
    uint8_t driverPinN_;
    uint8_t driverPinDis_;
    uint8_t driverPinFS_;

    // calibration for the pedal position sensor readings.
    // 'min' value is the ADC reading at 0% throttle
    // 'max' value is the ADC reading at 100% throttle
    RangeCalibration ppsCalA_;
    RangeCalibration ppsCalB_;

    // calibration for the throttle position sensor readings.
    // 'min' value is the ADC reading at 0% throttle
    // 'max' value is the ADC reading at 100% throttle
    RangeCalibration tpsCalA_;
    RangeCalibration tpsCalB_;
    
    // ADC readings from throttle position sensors A & B
    // range: [0 to 1023]
    uint16_t tpsA_;
    uint16_t tpsB_;
    // finalized throttle position based on both A & B sensor readings
    // range: [-10000 to 10000] (ie. -100 to 100%)
    int16_t tps_;

    // ADC readings from pedal position sensors A & B
    // range: [0 to 1023]
    uint16_t ppsA_;
    uint16_t ppsB_;
    // finalized pedal position based on both A & B sensor readings
    // range: [-10000 to 10000] (ie. -100 to 100%)
    int16_t pps_;

    // Throttle position the PID controller is targeting
    // tpsTarget_ = max(tpsStall_, tpsStall_ + idleAdder_ + ppsAdder_)
    // range: [-10000 to 10000] (ie. -100 to 100%)
    int16_t tpsTarget_;

    // the minimum tps value that the engine can continue to run.
    // going below this tps value will cause the engine to stall.
    // range: [-10000 to 10000] (ie. -100 to 100%)
    int16_t tpsStall_;

    // portion of the tpsTarget that's coming from engine idle control.
    // currently just a fixed value from flash, but will eventually have
    // its own PID too.
    // range: [-10000 to 10000] (ie. -100 to 100%)
    int16_t idleAdder_;

    // portion of the tpsTarget that's coming from the accelerator pedal
    // ppsAdder = ((10000 - tpsStall_ - idleAdder) * pps) / 10000
    // range: [-10000 to 10000] (ie. -100 to 100%)
    int16_t ppsAdder_;

    // PWM motor driver output
    // range: [-255 to 255] if in h-bridge mode (negative means reverse)
    //        [0 to 255] if in normal mode
    int16_t motorOut_;
    
    // ADC readings from motor driver current feedback pin
    // range: [0 to 1023]
    uint16_t driverFB_;

    // Motor current calculated based on driver feedback pin voltage
    // range: [0 to 65535] in milliamps
    uint16_t motorCurrent_mA_;

    // RAM variables
    OutVars *outVars_;

    // status maintained in RAM
    Status status_;

    // rate at which the PID algorith runs in milliseconds
    uint8_t pidSampleRate_ms_;

    // P,I, and D coefficient fed into the PID controller
    // to update this settings, call updatePID_Coeffs()
    double Kp, Ki, Kd;

    // current real-world TPS reading fed into the PID controller
    double pidIn_;
    // PID's requested PWM for motor controler
    // Range: [-255 to +255] for h-bridge driver (allows reverse)
    //        [0 to +255] for normal driver
    double pidOut_;
    // target TPS position fed into PID controller
    double pidSetpoint_;
    // the PID controller itself
    PID pid_;

    PIDAutotuner tuner_;

    SetpointSource_E setpointSource_;

    // user specified setpoint via setSetpointOverride()
    // range: [0 to 10000] (ie. 0% to 100%)
    uint16_t userSetpoint_;

    SensorSetup sensorSetup_;
    FlashTableDescriptor ppsCompDesc_;
    FlashTableDescriptor tpsCompDesc_;
    // threshold used to compare the absolute value of the ADC
    // delta in the sensor comparison logic. usually set to 50 or so.
    uint16_t ppsCompareThresh_;
    uint16_t tpsCompareThresh_;

};

extern Throttle throttle;

#endif