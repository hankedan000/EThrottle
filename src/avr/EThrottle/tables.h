#ifndef TABLES_H_
#define TABLES_H_

#include <EEPROM.h>
#include <stdint.h>
#include "MegaCAN_ExtDevice.h"
#include "MegaCAN_RT_BroadcastHelper.h"
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
  uint8_t reserved0[5];             // offset 5
  Throttle::OutVars throttleOutVars;// offset 10 (19bytes)
  uint8_t reserved1[99];
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

struct Page1_T
{
  // Throttle PID controller coefficients
  // range: [0 to 10000] (ie. 0.00 to 1.00)
  uint16_t throttleKp;
  uint16_t throttleKi;
  uint16_t throttleKd;
  uint8_t reserved[122];
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

#define NUM_TABLES 3
static const MegaCAN::TableDescriptor_t TABLES[NUM_TABLES] = {
  {&outPC,            sizeof(OutPC_T), MegaCAN::TableType_E::eRam  , -1                }, // table 0
  {MegaCAN::tempPage, sizeof(Page1_T), MegaCAN::TableType_E::eFlash, PAGE1_FLASH_OFFSET}, // table 1
  {MegaCAN::tempPage, sizeof(Page2_T), MegaCAN::TableType_E::eFlash, PAGE2_FLASH_OFFSET}  // table 2
};

#endif