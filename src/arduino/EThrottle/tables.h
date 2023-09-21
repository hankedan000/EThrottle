#ifndef TABLES_H_
#define TABLES_H_

#include <EEPROM.h>
#include <stdint.h>
#include <MegaCAN_ExtDevice.h>
#include <MegaCAN_RT_BroadcastHelper.h>
#include "Throttle.h"

#define PAGE0_SIZE 128
#define PAGE1_SIZE 128
#define PAGE2_SIZE 128
#define PAGE1_FLASH_OFFSET 0
#define PAGE2_FLASH_OFFSET (PAGE1_FLASH_OFFSET + PAGE1_SIZE)

#define ADC_MAPPING_CURVE_N_BINS 6

// forward declare
struct Page1_T;
struct Page2_T;
struct OutPC_T;

#define OUTPC_FIELD_OFFSET(FIELD_NAME) (offsetof(OutPC_T, FIELD_NAME))
#define PAGE1_FIELD_OFFSET(FIELD_NAME) (PAGE1_FLASH_OFFSET + offsetof(Page1_T, FIELD_NAME))
#define PAGE2_FIELD_OFFSET(FIELD_NAME) (PAGE2_FLASH_OFFSET + offsetof(Page2_T, FIELD_NAME))

struct OutPC_T
{
  uint8_t canStatus;                // offset 0
  uint8_t canErrorCount;            // offset 1
  uint16_t loopTimeUs;              // offset 2
  union Status0_T
  {
    struct Status0Bits_T
    {
      uint8_t needsBurn      : 1;
      uint8_t flashDataLost  : 1;
      uint8_t reserved       : 2;
      uint8_t currFlashTable : 4;
    } bits;
    uint8_t value;
  } status0;                        // offset 4
  // contents of MCUSR (only valid if using mcp-can-boot bootloader)
  uint8_t mcusr;                    // offset 5
  uint8_t reserved0[4];             // offset 6
  Throttle::OutVars throttleOutVars;// offset 10 (31bytes)
  struct ADC_Status_T
  {
    uint8_t schedIdx   : 4;
    uint8_t state      : 2;
    uint8_t rsvd0      : 2;
    uint8_t convCycles : 8;
    uint8_t adcsra     : 8;
  } adcStatus;                      // offset 41 (3bytes)
  uint8_t reserved1[84];            // offset 44
};
static_assert(sizeof(OutPC_T) == PAGE0_SIZE);

struct ADC_MappingControl_T
{
  union
  {
    struct ADC_MappingControlBits_T
    {
      // set to 1 to enable mapping of this ADC channel
      uint8_t enabled  : 1;
      uint8_t reserve0 : 3;
      // the mapping curve to use (0->A, 1->B, 2->C, 3->D)
      uint8_t curve    : 2;
      uint8_t reserve1 : 2;
    } bits;
    uint8_t value;
  };
};

#define SENSOR_COMPARE_CURVE_N_BINS 4

struct Page1_T
{
  // Throttle PID controller coefficients
  // range: [0 to 10000] (ie. 0.00 to 1.00)
  uint16_t throttleKp;
  uint16_t throttleKi;
  uint16_t throttleKd;
  uint16_t reserved0;

  // min/max ADC range calibration for Pedal Position Sensors (PPS)
  //  min is ADC at 0% throttle
  //  max is ADC at 100% throttle
  Throttle::RangeCalibration ppsCalA;
  Throttle::RangeCalibration ppsCalB;
  // min/max ADC range calibration for Throttle Position Sensors (TPS)
  //  min is ADC at 0% throttle
  //  max is ADC at 100% throttle
  Throttle::RangeCalibration tpsCalA;
  Throttle::RangeCalibration tpsCalB;

  union SensorSetupUnion
  {
    Throttle::SensorSetup bits;
    uint8_t word;
  } sensorSetup;
  uint8_t  reserved1;
  uint16_t reserved2;
  uint16_t ppsCompCurve_xBins[SENSOR_COMPARE_CURVE_N_BINS];
  uint16_t ppsCompCurve_yBins[SENSOR_COMPARE_CURVE_N_BINS];
  uint16_t tpsCompCurve_xBins[SENSOR_COMPARE_CURVE_N_BINS];
  uint16_t tpsCompCurve_yBins[SENSOR_COMPARE_CURVE_N_BINS];
  uint16_t ppsCompareThresh;
  uint16_t tpsCompareThresh;
  uint16_t tpsStall;
  uint16_t msqRtBcastBaseId;// default is 1520
  uint8_t reserved[60];
};
static_assert(sizeof(Page1_T) == PAGE1_SIZE);
static_assert(sizeof(Page1_T) <= MEGA_CAN_EXT_MAX_FLASH_TABLE_SIZE);

struct Page2_T
{
  MegaCAN::RT_Broadcast_T rtBcast;
  uint8_t reserved[121];
};
static_assert(sizeof(Page2_T) == PAGE2_SIZE);
static_assert(sizeof(Page2_T) <= MEGA_CAN_EXT_MAX_FLASH_TABLE_SIZE);

extern OutPC_T outPC;

enum TableId_E {
  eTI_OUT_PC = 0,
  eTI_CFG_PAGE1 = 1,
  eTI_CFG_PAGE2 = 2,
  eTI_UART_CMD = 3
};

#define NUM_TABLES 4
static const MegaCAN::TableDescriptor_t TABLES[NUM_TABLES] = {
  {&outPC,            sizeof(OutPC_T),     MegaCAN::TableType_E::eRam  , -1                }, // table 0
  {MegaCAN::tempPage, sizeof(Page1_T),     MegaCAN::TableType_E::eFlash, PAGE1_FLASH_OFFSET}, // table 1
  {MegaCAN::tempPage, sizeof(Page2_T),     MegaCAN::TableType_E::eFlash, PAGE2_FLASH_OFFSET}, // table 2
  // we handle the cmds directly from CAN msg, so no need to provide a valid buffer
  {nullptr,           8,                   MegaCAN::TableType_E::eRam  , -1                }  // table 3 (cmd table)
};

// accessor utilities
void
loadThrottlePID_FromFlash(
  Throttle &throttle);

void
storeThrottlePID_ToFlash(
  Throttle &throttle);

void
loadSensorCalibrationsFromFlash(
  Throttle &throttle);

void
loadSensorSetupFromFlash(
  Throttle &throttle);

#endif