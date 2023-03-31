#pragma once

#include "MegaCAN_ExtDevice.h"
#include "tables.h"

#define CAN_CS  10
#define CAN_INT 3
#define CAN_ID  5
#define CAN_MSG_BUFFER_SIZE 8

inline MegaCAN::CAN_Msg canBuff[CAN_MSG_BUFFER_SIZE];
inline MegaCAN::ExtDevice canDev(CAN_CS,CAN_ID,CAN_INT,canBuff,CAN_MSG_BUFFER_SIZE,TABLES,NUM_TABLES);

// call this in main arduino setup()
void
canSetup();

// call this in main arduino loop()
void
canLoop();