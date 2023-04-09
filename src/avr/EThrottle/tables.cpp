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