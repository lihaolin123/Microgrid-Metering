#ifndef _CS5464_H
#define _CS5464_H
#include "sys.h"
#include "delay.h"
#define WRITE	0x40	
#define START0	0xE0	
#define	START1	0xE8	
#define SYNC1	0xFF	
#define SYNC0	0xFE	
#define SOFTWARE_RESET	0X80
#define SLEEP			0X90
#define WAKEUP_HALT		0XA0
#define STAND_BY		0XB0	
#define DC_Offset		0x80	
#define DC_Gain			0x90	
#define AC_Offset		0xA0	
#define AC_Gain			0xB0	
#define Current_Channel1	0x01
#define Voltage_Channel1	0x02
#define Current_Channel2	0x04
#define Voltage_Channel2	0x08
#define	Page_0	0x00
#define	Page_1	0x01
#define	Page_2	0x02
#define	Config	0x00	
#define	I1		0x02	
#define	V1  	0x04	
#define	P1_5464  0x06	
#define P1AVG  	0x08	
#define I1RMS  	0x0A	
#define V1RMS  	0x0C	
#define I2		0x0E	
#define V2		0x10	
#define P2_5464 0x12	
#define P2AVG  	0x14	
#define I2RMS  	0x16	
#define V2RMS  	0x18	
#define Q1AVG  	0x1A	
#define Q1  	0x1C	
#define Status  0x1E	
#define Q2AVG  	0x20	
#define Q2  	0x22	
#define I1PEAK  0x24	
#define V1PEAK  0x26	
#define S1  	0x28	
#define PF1 	0x2A	
#define I2PEAK  0x2C	
#define V2PEAK  0x2E	
#define S2  	0x30	
#define PF2  	0x32	
#define Mask  	0x34	
#define T		0x36	
#define Ctrl  	0x38	
#define EPULSE  0x3A	
#define SPULSE  0x3C	
#define QPULSE  0x3E	
#define Page	0x3E 	
#define I1OFF	0x00	
#define I1GAIN	0x02	
#define V1OFF	0x04 	
#define V1GAIN  0x06 	
#define P1OFF	0x08 	
#define I1ACOFF	0x0A 	
#define V1ACOFF	0x0C	
#define I2OFF	0x0E	
#define I2GAIN  0x10 	
#define V2OFF	0x12	
#define V2GAIN  0x14	
#define P2OFF	0x16	
#define I2ACOFF	0x18
#define V2ACOFF	0x1A	
#define PulseWidth	0x1C
#define PulseRate	0x1E
#define Modes	0x20
#define Epsilon	0x22
#define IchanLEVEL	0x24
#define CycleCount  0x26
#define Q1WB	0x28
#define Q2WB	0x2A
#define TGAIN	0x2C
#define TOFF	0x2E
#define EMIN(IrmsMIN) 0x30
#define TSETTLE	0x32 
#define LoadMIN	0x34 
#define VFRMS	0x36
#define G		0x38
#define Time	0x3A
#define V1SagDUR	0x00
#define V1SagLEVEL	0x02
#define I1FaultDUR  0x08	
#define I1FaultLEVEL 0x0A
#define V2SagDUR	0x10
#define V2SagLEVEL	0x12
#define I2FaultDUR  0x18
#define I2FaultLEVEL 0x1A
#define EWA		0x80 
#define INTCONFIG_0	0x00
#define INTCONFIG_1	0x08
#define INTCONFIG_2	0x10
#define INTCONFIG_3	0x18
#define iCPU 	0x10
#define K_1		0x01
#define K_2		0x02
#define K_3		0x03
#define K_4		0x04
#define K_5		0x05
#define K_6		0x06
#define K_7		0x07
#define K_8		0x08
#define K_9		0x09
#define K_10	0x0A
#define K_11	0x0B
#define K_12	0x0C
#define K_13	0x0D
#define K_14	0x0E
#define K_15	0x0F
#define K_16	0x00
#define DRDY	0x80
#define CRDY	0x10
#define I1FAULT 0x08
#define V1SAG	0x04
#define TUP		0x80
#define I1OD	0x08
#define LSD		0x04
#define FUP		0x02
#define IC		0x01
#define I2gain_250mv	0x00
#define I2gain_50mv		0x10
#define STOP_E2PROM		0x01
#define I1gain_250mv	0x00
#define I1gain_50mv		0x20
#define INTOD_Normal	0x00
#define INTOD_Open		0x01
#define NOCPU_Enabled   0x00
#define NOCPU_Disabled  0x04
#define NOOSC_Enabled	0x00
#define NOOSC_Disabled	0x02
#define E3MODE_0		0x00
#define E3MODE_1		0x04
#define E3MODE_2		0x08
#define E3MODE_3		0x0C
#define POS				0x02
#define AFC				0X01
//��������
extern void CS5464IOInit(void);
extern void RESET_CS5464(void);
extern void getRegister(unsigned char registerName,unsigned char *array);
extern void getCs5464Status(unsigned char *cs5464Status);
extern void clearCs5464Status(unsigned char *cs5464Status);
extern void CS5464SPIInit(void);

#endif
