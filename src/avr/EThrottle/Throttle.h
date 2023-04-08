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

class Throttle
{
public:

  enum SetpointSource_E
  {
    eSS_PPS,
    eSS_User
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
    // range: [-10000 to 10000] (ie. -100 to 100%)
    int16_t tpsTarget;

    // PWM motor driver output
    // range: [-255 to 255] if in h-bridge mode (negative means reverse)
    //        [0 to 255] if in normal mode
    int16_t motorOut;

    struct Control
    {
      // allows user to override the PID setup value for testing
      // normally the set point comes from the normalize pedal
      // position, but this value can be used instead if enabled.
      struct SetpointOverride
      {
        // range: [0 to 10000] (ie. 0 to 100%)
        uint16_t value    : 14;
        uint16_t reserved : 1;
        // set to 1 to enable override
        uint16_t enabled  : 1;
      } setpointOverride;
    } ctrl;

    struct Status
    {
      uint8_t pidAutoTuneBusy : 1;
      uint8_t reserved        : 7;
    } status;
  };

public:
  Throttle(
    uint8_t tpsPinA,
    uint8_t tpsPinB,
    uint8_t ppsPinA,
    uint8_t ppsPinB,
    uint8_t driverPinP,
    uint8_t driverPinN,
    uint8_t driverPinDis,
    uint8_t driverPinFS,
    uint8_t driverPinFB,
    OutVars *outVars);

  /**
   * Configures IO pins and internal classes.
   * Normally called in setup()
   */
  void
  init(
    uint8_t pidSampleRate_ms);

  void
  disableMotor() const;

  void
  enableMotor() const;

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
  doPedal();

  void
  doThrottle();

private:
    uint8_t tpsPinA_;
    uint8_t tpsPinB_;
    uint8_t ppsPinA_;
    uint8_t ppsPinB_;
    uint8_t driverPinP_;
    uint8_t driverPinN_;
    uint8_t driverPinDis_;
    uint8_t driverPinFS_;
    uint8_t driverPinFB_;

    struct RangeCalibration
    {
      uint16_t min;
      uint16_t max;
    };

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
    // range: [-10000 to 10000] (ie. -100 to 100%)
    int16_t tpsTarget_;

    // PWM motor driver output
    // range: [-255 to 255] if in h-bridge mode (negative means reverse)
    //        [0 to 255] if in normal mode
    int16_t motorOut_;

    // RAM variables
    OutVars *outVars_;

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

};

#endif