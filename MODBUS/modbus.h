#ifndef _MODBUS_H
#define _MODBUS_H

#include "mb.h"
#include "mbport.h"
#include "stdlib.h"
#include "string.h"
#include "port.h"
#include "mb.h"
#include "mbproto.h"
#include "includes.h"
#define REG_INPUT_START 0x0001
#define REG_INPUT_NREGS 48
extern USHORT usRegInputBuf[REG_INPUT_NREGS];
//static USHORT   usRegInputStart = REG_INPUT_START;
#define REG_HOLDING_START 0x0001
#define REG_HOLDING_NREGS 48
extern uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];
#define REG_COILS_START 0x0001
#define REG_COILS_SIZE 16
extern UCHAR ucRegCoilsBuf[REG_COILS_SIZE];
#define REG_DISCRETE_START 0x0001
#define REG_DISCRETE_SIZE 16
extern UCHAR ucRegDiscreteBuf[REG_DISCRETE_SIZE];



#endif


