/*
  * FreeModbus Libary: LPC214X Port
  * Copyright (C) 2007 Tiago Prado Lone <tiago@maxwellbohr.com.br>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2007/04/24 23:15:18 wolti Exp $
 */

#include "port.h"
#include "usart.h"	
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
/* ----------------------- Start implementation -----------------------------*/
void vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
	if(TRUE==xRxEnable)
	{
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	}
	else
	{
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);	
	}

	if(TRUE==xTxEnable)
	{
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	}
	else
	{
	   USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	}
}

void
vMBPortClose( void )
{
	USART_ITConfig(USART1, USART_IT_TXE|USART_IT_RXNE, DISABLE);
	USART_Cmd(USART1, DISABLE);
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	uart_init(ulBaudRate);
// 	USART_InitTypeDef USART_InitStructure;
// 	USART_ClockInitTypeDef  USART_ClockInitStructure;

// 	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
// 	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
// 	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
// 	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
// 	USART_ClockInit(USART1, &USART_ClockInitStructure);

// 	USART_InitStructure.USART_BaudRate = ulBaudRate;
// 	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
// 	USART_InitStructure.USART_StopBits = USART_StopBits_1;
// 	USART_InitStructure.USART_Parity = USART_Parity_No ;
// 	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	
// 	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
// 	USART_Init(USART1, &USART_InitStructure);

// 	/* Enable USART1 */
// 	USART_Cmd(USART1, ENABLE);

	return TRUE;
}



BOOL xMBPortSerialPutByte( CHAR ucByte )
{
	USART_SendData(USART1, ucByte);
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)/*等待发送完成*/
  {
  
  }		
	return TRUE;
}

BOOL xMBPortSerialGetByte( CHAR * pucByte )
{
	*pucByte = USART_ReceiveData(USART1);
	return TRUE;
}

/* 
 * Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
void prvvUARTTxReadyISR(void)
{
    pxMBFrameCBTransmitterEmpty();
}

/* 
 * Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
void prvvUARTRxISR(void)
{
    pxMBFrameCBByteReceived();
}
