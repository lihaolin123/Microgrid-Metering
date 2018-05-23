//========================================================================
// 文件名: modbus.c
// 作  者: 
// 日  期: 
// 描  述: 主要实现了4个modbus回调函数
//
// 版  本:
//      
//========================================================================
#include "modbus.h"
extern void xMBUtilSetBits( UCHAR * ucByteBuf, USHORT usBitOffset, UCHAR ucNBits, UCHAR ucValue );
extern UCHAR xMBUtilGetBits( UCHAR * ucByteBuf, USHORT usBitOffset, UCHAR ucNBits );
USHORT usRegInputBuf[REG_INPUT_NREGS]={0};
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS]= {0};
UCHAR ucRegCoilsBuf[REG_COILS_SIZE]={0};
UCHAR ucRegDiscreteBuf[REG_DISCRETE_SIZE]= {0};
//////////////////////////////////////////////////////////////////////////////////////////
///
//void NVIC_Configuration(void)
//{
//  NVIC_InitTypeDef NVIC_InitStructure;
//  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);	
//	
//	//Usart1 NVIC 配置
//    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级0
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级2
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器	

//// 	//TIM2 
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
//	NVIC_Init(&NVIC_InitStructure);	
//}
/****************************************************************************
* 名	  称：eMBRegInputCB
* 功    能：读取输入寄存器，对应功能码是 04 eMBFuncReadInputRegister
* 入口参数：pucRegBuffer: 数据缓存区，用于响应主机   
*						usAddress: 寄存器地址
*						usNRegs: 要读取的寄存器个数
* 出口参数：
* 注	  意：上位机发来的 帧格式是: SlaveAddr(1 Byte)+FuncCode(1 Byte)
*								+StartAddrHiByte(1 Byte)+StartAddrLoByte(1 Byte)
*								+LenAddrHiByte(1 Byte)+LenAddrLoByte(1 Byte)+
*								+CRCAddrHiByte(1 Byte)+CRCAddrLoByte(1 Byte)
*							3 区
****************************************************************************/
eMBErrorCode eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START ) && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - REG_INPUT_START );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ = ( UCHAR )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ = ( UCHAR )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

/****************************************************************************
* 名	  称：eMBRegHoldingCB
* 功    能：对应功能码有  06 写保持寄存器 eMBFuncWriteHoldingRegister 
*						 16 写多个保持寄存器 eMBFuncWriteMultipleHoldingRegister
*						 03 读保持寄存器 eMBFuncReadHoldingRegister
*						 23 读写多个保持寄存器 eMBFuncReadWriteMultipleHoldingRegister
* 入口参数：pucRegBuffer: 数据缓存区，用于响应主机   
*						usAddress: 寄存器地址
*						usNRegs: 要读写的寄存器个数
*						eMode: 功能码
* 出口参数：
* 注	  意：4 区
****************************************************************************/
eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
	eMBErrorCode    eStatus = MB_ENOERR;
	int             iRegIndex;
	if((usAddress >= REG_HOLDING_START)&&\
	((usAddress+usNRegs) <= (REG_HOLDING_START + REG_HOLDING_NREGS)))
	{
		iRegIndex = (int)(usAddress - REG_HOLDING_START);
		switch(eMode)
		{                                       
			case MB_REG_READ://读 MB_REG_READ = 0
        while(usNRegs > 0)
				{
					*pucRegBuffer++ = (u8)(usRegHoldingBuf[iRegIndex] >> 8);            
					*pucRegBuffer++ = (u8)(usRegHoldingBuf[iRegIndex] & 0xFF); 
					iRegIndex++;
					usNRegs--;					
				}                            
        break;
			case MB_REG_WRITE://写 MB_REG_WRITE = 0
				while(usNRegs > 0)
				{         
					usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
					usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
					iRegIndex++;
					usNRegs--;
        }				
			}
	}
	else//错误
	{
		eStatus = MB_ENOREG;
	}	
	
	return eStatus;
}

/****************************************************************************
* 名	  称：eMBRegCoilsCB
* 功    能：对应功能码有 01 读线圈 eMBFuncReadCoils
*						 05 写线圈 eMBFuncWriteCoil
*						 15 写多个线圈 eMBFuncWriteMultipleCoils
* 入口参数：pucRegBuffer: 数据缓存区，用于响应主机   
*						usAddress: 线圈地址
*						usNCoils: 要读写的线圈个数
*						eMode: 功能码
* 出口参数：
* 注	  意： 
*						0 区
****************************************************************************/

/** 
* @brief 线圈寄存器处理函数
* @param pucRegBuffer 读操作---返回数据指针，写操作--返回数据指针 
* usAddress 寄存器起始地址 
* usNRegs 寄存器长度 
* eMode 操作方式
* @retval eStatus 寄存器状态 
*/  
eMBErrorCode  eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,  eMBRegisterMode eMode )  
{  
	//错误状态  
	eMBErrorCode eStatus = MB_ENOERR;  
	//寄存器个数  
	int16_t iNCoils = ( int16_t )usNCoils;  
	//寄存器偏移量  
	int16_t usBitOffset;  
  
	//检查寄存器是否在指定范围内  
	if( ( (int16_t)usAddress >= REG_COILS_START ) &&  ( usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )  
	{  
		//计算寄存器偏移量  
		usBitOffset = ( int16_t )( usAddress - REG_COILS_START );  
		switch ( eMode )  
		{  
			case MB_REG_READ:  
			while( iNCoils > 0 )  
			{  
				*pucRegBuffer++ = xMBUtilGetBits(ucRegCoilsBuf, usBitOffset,(uint8_t)(iNCoils > 8 ? 8 : iNCoils));  
				iNCoils -= 8;  
				usBitOffset += 8;  
			}  
			break;  
			case MB_REG_WRITE:  
			while( iNCoils > 0 )  
			{  
				xMBUtilSetBits( ucRegCoilsBuf, usBitOffset,  ( uint8_t )( iNCoils > 8 ? 8 : iNCoils ),  *pucRegBuffer++ );  
				iNCoils -= 8;  
			}  
			break;  
		}  
	}  
	else  
	{  
		eStatus = MB_ENOREG;  
	}  
	return eStatus;  
}  
 /****************************************************************************
* 名	  称：eMBRegDiscreteCB
* 功    能：读取离散寄存器，对应功能码有：02 读离散寄存器 eMBFuncReadDiscreteInputs
* 入口参数：pucRegBuffer: 数据缓存区，用于响应主机   
*						usAddress: 寄存器地址
*						usNDiscrete: 要读取的寄存器个数
* 出口参数：
* 注	  意：1 区
****************************************************************************/
 
/** 
* @brief 开关输入寄存器处理函数，开关输入寄存器，可读 
* @param pucRegBuffer 读操作---返回数据指针，写操作--返回数据指针 
* usAddress 寄存器起始地址 
* usNRegs 寄存器长度 
* eMode 操作方式，读或者写 
* @retval eStatus 寄存器状态 
*/  
eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )  
{  
	//错误状态  
	eMBErrorCode eStatus = MB_ENOERR;  
	//操作寄存器个数  
	int16_t iNDiscrete = ( int16_t )usNDiscrete;  
	//偏移量  
	uint16_t usBitOffset;  
	  
	//判断寄存器时候再制定范围内  
	if( ( (int16_t)usAddress >= REG_DISCRETE_START ) &&  ( usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_SIZE ) )  
	{  
		usBitOffset = ( uint16_t )( usAddress - REG_DISCRETE_START );  
		  
		while( iNDiscrete > 0 )  
		{  
			*pucRegBuffer++ = xMBUtilGetBits( ucRegDiscreteBuf, usBitOffset,  ( uint8_t)( iNDiscrete > 8 ? 8 : iNDiscrete ) );  
			iNDiscrete -= 8;  
			usBitOffset += 8;  
		}  
	}  
	else  
	{  
		eStatus = MB_ENOREG;  
	}  
	return eStatus;  
}  

