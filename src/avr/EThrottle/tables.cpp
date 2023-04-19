#include "tables.h"

#include "FlashUtils.h"

OutPC_T outPC;

void
loadThrottlePID_FromFlash(
  Throttle &throttle)
{
  throttle.updatePID_Coeffs(
    FlashUtils::flashRead_BE<uint16_t>(PAGE1_FIELD_OFFSET(throttleKp)) / 100.0,
    FlashUtils::flashRead_BE<uint16_t>(PAGE1_FIELD_OFFSET(throttleKi)) / 100.0,
    FlashUtils::flashRead_BE<uint16_t>(PAGE1_FIELD_OFFSET(throttleKd)) / 100.0);
}

void
storeThrottlePID_ToFlash(
  Throttle &throttle)
{
  FlashUtils::flashWrite_BE(PAGE1_FIELD_OFFSET(throttleKp), (uint16_t)(throttle.getKp() * 100.0));
  FlashUtils::flashWrite_BE(PAGE1_FIELD_OFFSET(throttleKi), (uint16_t)(throttle.getKi() * 100.0));
  FlashUtils::flashWrite_BE(PAGE1_FIELD_OFFSET(throttleKd), (uint16_t)(throttle.getKd() * 100.0));
}

void
loadSensorCalibrationsFromFlash(
  Throttle &throttle)
{
  Throttle::RangeCalibration rc;

  rc.min = FlashUtils::flashRead_BE<uint16_t>(PAGE1_FIELD_OFFSET(ppsCalA.min));
  rc.max = FlashUtils::flashRead_BE<uint16_t>(PAGE1_FIELD_OFFSET(ppsCalA.max));
  throttle.setRangeCalPPS_A(rc);
  rc.min = FlashUtils::flashRead_BE<uint16_t>(PAGE1_FIELD_OFFSET(ppsCalB.min));
  rc.max = FlashUtils::flashRead_BE<uint16_t>(PAGE1_FIELD_OFFSET(ppsCalB.max));
  throttle.setRangeCalPPS_B(rc);
  rc.min = FlashUtils::flashRead_BE<uint16_t>(PAGE1_FIELD_OFFSET(tpsCalA.min));
  rc.max = FlashUtils::flashRead_BE<uint16_t>(PAGE1_FIELD_OFFSET(tpsCalA.max));
  throttle.setRangeCalTPS_A(rc);
  rc.min = FlashUtils::flashRead_BE<uint16_t>(PAGE1_FIELD_OFFSET(tpsCalB.min));
  rc.max = FlashUtils::flashRead_BE<uint16_t>(PAGE1_FIELD_OFFSET(tpsCalB.max));
  throttle.setRangeCalTPS_B(rc);
}

void
loadSensorSetupFromFlash(
  Throttle &throttle)
{
  Page1_T::SensorSetupUnion setupU;
  setupU.word = EEPROM.read(PAGE1_FIELD_OFFSET(sensorSetup.word));

  Throttle::FlashTableDescriptor ppsFTD;
  ppsFTD.xBinsFlashOffset = PAGE1_FIELD_OFFSET(ppsCompCurve_xBins);
  ppsFTD.yBinsFlashOffset = PAGE1_FIELD_OFFSET(ppsCompCurve_yBins);
  ppsFTD.nBins = SENSOR_COMPARE_CURVE_N_BINS;

  Throttle::FlashTableDescriptor tpsFTD;
  tpsFTD.xBinsFlashOffset = PAGE1_FIELD_OFFSET(tpsCompCurve_xBins);
  tpsFTD.yBinsFlashOffset = PAGE1_FIELD_OFFSET(tpsCompCurve_yBins);
  tpsFTD.nBins = SENSOR_COMPARE_CURVE_N_BINS;

  throttle.setSensorSetup(setupU.bits, ppsFTD, tpsFTD);
}