#ifndef _DEBUG_H
#define _DEBUG_H

#include "stm32l0xx.h"	
#include <stdbool.h>


#pragma pack(1)

typedef enum _DebugCmdID
{
	EraseFlash = 0x4523,	//#E	
	SystemReset= 0x5223,	//#R
	LogReport=   0x4C23,    //#L
	TxModeSet=   0x5423,    //#T
	ReadFlash=   0x4623,    //#F
	Upgrade=     0x5523,    //#U
	MeterModeSet=0x4D23,    //#M
	TestMode=    0x4323,    //#C
	CMD_MAX=     0xFF23	
}DebugCmdID;

typedef struct _DebugCmd
{
	u16  cmd_id;
	u8   cmd_data[4];
	u16  cmd_tail;
	
}DebugCmd;

#endif