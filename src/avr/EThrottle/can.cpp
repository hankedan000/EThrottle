#include "Arduino.h"
#include "can.h"

#include "ecu_vars.h"
#include "MSG_defn.h"
#include "uart.h"

SET_MEGA_CAN_SIG("EThrottle");
SET_MEGA_CAN_REV("OpenGPIO-1.0.0     ");

// toggle a pin on different CAN events
#define ENABLE_CAN_STROBES  0 // global enable
#define STROBE_PIN 8
#define STROBE_ON_INT       0
#define STROBE_ON_HANDLE    1

// external interrupt service routine for CAN message on MCP2515
void
canISR()
{
  EIMSK &= ~(1 << INT1);// disable external interrupt
  sei();// enable global interrupts again (prevent timer ISRs from getting too delayed)

#if ENABLE_CAN_STROBES && STROBE_ON_INT
  digitalWrite(STROBE_PIN, 1);
#endif
  canDev.interrupt();
#if ENABLE_CAN_STROBES && STROBE_ON_INT
  digitalWrite(STROBE_PIN, 0);
#endif

  EIMSK |= (1 << INT1);// reenable external interrupt
}

// declared in EThrottle.ino
extern Throttle throttle;

// called any time a table is burned to over CAN
void
onTableBurned(
  uint8_t table)
{
  if (table == 1)
  {
    loadThrottlePID_FromFlash(throttle);
    loadSensorCalibrationsFromFlash(throttle);
    loadSensorSetupFromFlash(throttle);
  }
}

void
onTableWritten(
  uint8_t table,
	uint16_t /*offset*/,
	uint8_t len,
	const uint8_t * /*data*/)
{
  if (table == 3)
  {
    pushedUartCmdBytes(len);
  }
}

void
canSetup()
{
  cli();// disable interrupts

#if ENABLE_CAN_STROBES
  pinMode(STROBE_PIN, OUTPUT);
#endif

  // MCP2515 configuration
  canDev.init();
  canDev.setOnTableBurnedCallback(onTableBurned);
  canDev.setOnTableWrittenCallback(onTableWritten);
  pinMode(CAN_INT, INPUT_PULLUP);// Configuring pin for CAN interrupt input
  attachInterrupt(digitalPinToInterrupt(CAN_INT), canISR, LOW);

  sei();// enable interrupts
}

void
canLoop()
{
#if ENABLE_CAN_STROBES && STROBE_ON_HANDLE
  digitalWrite(STROBE_PIN, 1);
#endif
  canDev.handle();
#if ENABLE_CAN_STROBES && STROBE_ON_HANDLE
  digitalWrite(STROBE_PIN, 0);
#endif

  // update status0
  outPC.status0.bits.needsBurn = canDev.needsBurn();
  outPC.status0.bits.flashDataLost = canDev.flashDataLost();
  outPC.status0.bits.currFlashTable = canDev.currFlashTable();

  // update CAN status registers
  outPC.canStatus = canDev.getCAN_Status();
  outPC.canErrorCount = canDev.getLogicErrorCount();
}

EThrottleCAN::EThrottleCAN(
    uint8_t cs,
    uint8_t myId,
    uint8_t intPin,
    MegaCAN::CAN_Msg *buff,
    uint8_t buffSize,
    const MegaCAN::TableDescriptor_t *tables,
    uint8_t numTables)
 : MegaCAN::ExtDevice(cs,myId,intPin,buff,buffSize,tables,numTables)
{
}

void
EThrottleCAN::getOptions(
  struct MegaCAN::Options *opts)
{
  opts->handleStandardMsgsImmediately = true;
}

void
EThrottleCAN::applyCanFilters(
  MCP_CAN *can)
{
  #define ID_TYPE_EXT 1
  #define ID_TYPE_STD 0

  // filter MegaSquirt extended frames destined for this endpoint into RXB0
  uint32_t mask = 0x0;
  uint32_t filt = 0x0;
  MS_HDR_t *maskHdr = reinterpret_cast<MS_HDR_t*>(&mask);
  MS_HDR_t *filtHdr = reinterpret_cast<MS_HDR_t*>(&filt);
  maskHdr->toId = 0xf;// only check the 4bit toId in the megasquirt header
  filtHdr->toId = canId() & 0xf;// make sure the message is for me!
  can->init_Mask(0,ID_TYPE_EXT,mask);
  can->init_Filt(0,ID_TYPE_EXT,filt);
  can->init_Filt(1,ID_TYPE_EXT,filt);

  // filter Megasquirt broadcast frames into RXB1
  can->init_Mask(1,ID_TYPE_STD,0x00000000);
  can->init_Filt(2,ID_TYPE_STD,0x00000000);
  can->init_Filt(3,ID_TYPE_STD,0x00000000);
  can->init_Filt(4,ID_TYPE_STD,0x00000000);
  can->init_Filt(5,ID_TYPE_STD,0x00000000);
}

void
EThrottleCAN::handleStandard(
    const uint32_t &id,
    const uint8_t &length,
    uint8_t *data)
{
  const uint8_t msgId = id - ecuRtBcastBaseId;
  DEBUG("handleStandard");

  switch (msgId)
  {
    case 0:
    {
      ecu::rpm = MSG00_t::get_rpm(data);

      DEBUG(
          "time = %d; pw1 = %d; pw2 = %d; rpm = %d",
          MSG00_t::get_seconds(data),
          MSG00_t::get_pw1(data),
          MSG00_t::get_pw2(data),
          ecu::rpm);
      break;
    }
    default:
    {
      // silently ignore
      break;
    }
  }// switch
}