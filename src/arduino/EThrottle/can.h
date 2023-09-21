#pragma once

#include <MegaCAN_ExtDevice.h>
#include "tables.h"

#define CAN_CS  10
#define CAN_INT 3
#define CAN_ID  5
#define CAN_MSG_BUFFER_SIZE 8

// call this in main arduino setup()
void
canSetup();

// call this in main arduino loop()
void
canLoop();

// extend the base class so we can customize CAN filters and handle standard messages too
class EThrottleCAN : public MegaCAN::ExtDevice
{
public:
  EThrottleCAN(
			uint8_t cs,
			uint8_t myId,
			uint8_t intPin,
			MegaCAN::CAN_Msg *buff,
			uint8_t buffSize,
			const MegaCAN::TableDescriptor_t *tables,
			uint8_t numTables);

protected:
  /**
   * override this base method so that we can override default options
   */
  virtual void
  getOptions(
    struct MegaCAN::Options *opts) override;

  /**
   * override this base method so that we can set filters for broadcast
   * frame reception (11bit protocol) as well as extended message destined
   * for the our ID.
   */
  virtual void
  applyCanFilters(
    MCP_CAN *can) override;

  /**
   * Called when a standard 11bit megasquirt broadcast frame is received.
   * 
   * @param[in] id
   * The 11bit CAN identifier
   * 
   * @param[in] length
   * The number of data bytes in the CAN frame
   * 
   * @param[in] data
   * A pointer to the data segment of the CAN frame
   */
  virtual void
  handleStandard(
      const uint32_t id,
      const uint8_t length,
      uint8_t *data) override;

	virtual bool
	writeToTable(
		const uint8_t table,
		const uint16_t offset,
		const uint8_t len,
		const uint8_t *data) override;

private:

};

// the realtime data base message ID used by megasquirt
extern uint16_t ecuRtBcastBaseId;

inline MegaCAN::CAN_Msg canBuff[CAN_MSG_BUFFER_SIZE];
inline EThrottleCAN canDev(CAN_CS,CAN_ID,CAN_INT,canBuff,CAN_MSG_BUFFER_SIZE,TABLES,NUM_TABLES);