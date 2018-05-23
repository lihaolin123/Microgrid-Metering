//========================================================================
// 文件名: cs5464.c
// 作  者: HaoLin Li(lihaolinwork@foxmail.com)
// 日  期: 
// 描  述: 
//
// 版  本:
//========================================================================
#include "dc_cs5464.h"	 
#define RESET_54640()	GPIO_ResetBits(GPIOD,GPIO_Pin_11);
#define RESET_54641()	GPIO_SetBits(GPIOD,GPIO_Pin_11);
#define SDI_54640()		GPIO_ResetBits(GPIOD,GPIO_Pin_12);
#define SDI_54641()		GPIO_SetBits(GPIOD,GPIO_Pin_12);
#define SCLK_54640()	GPIO_ResetBits(GPIOD,GPIO_Pin_13);
#define SCLK_54641()	GPIO_SetBits(GPIOD,GPIO_Pin_13);
#define SDO_5464()	GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_15)
#define CS_54640()	GPIO_ResetBits(GPIOD,GPIO_Pin_14);
#define CS_54641()	GPIO_SetBits(GPIOD,GPIO_Pin_14);
static  unsigned char cs5464rw[3]={0,0,0}; 
static void send_5464(unsigned char send_data)
{
      unsigned char  i;
	  SCLK_54640();
	  for(i = 0;i < 8;i++)
	  {
		if(send_data & 0x80)
		{
			SDI_54641();
		}
		else
		{
			SDI_54640();
		}
		SCLK_54641();
		send_data <<= 1;
		//delay_us(1);
		MyDelay_us(1);
		SCLK_54640();
	 }
}

static void write_5464(unsigned char *pbuf,unsigned char command)
{
	unsigned char j,k;
	send_5464(command);
	for(j = 0;j < 3;j++)
	{
	SCLK_54640();
	for(k = 0;k < 8;k++)
	{
		if(*pbuf&0x80)
		{
			SDI_54641();
		}else
		{
			SDI_54640();
		}
		SCLK_54641();
		*pbuf <<= 1;
		//delay_us(1);
		MyDelay_us(1);
		SCLK_54640();
	}
	pbuf += 1;
	}
}

static void read_5464(unsigned char *pbuf,unsigned char command)
{
	unsigned char j,k,const_fe;
	send_5464(command);
	for(j = 0;j < 3;j++)
	{
		*pbuf = 0x00;
		const_fe = 0xfe;
		for(k = 0;k < 8;k++)
		{
		SCLK_54640();
		*pbuf <<= 1;
		if(SDO_5464())
		{
			*pbuf |= 0x01;
		}
		if(const_fe&0x80)
		{
			SDI_54641();
		}else
		{
			SDI_54640();
		}
		const_fe <<= 1;
		//delay_us(1);
		MyDelay_us(1);
		SCLK_54641();
		}
		pbuf += 1;
     }
}

static void writePage(unsigned char page)
{
	cs5464rw[2] = page;
	write_5464(cs5464rw,Page+WRITE);
}
static void Conversion(unsigned char continuous)
{
	send_5464(continuous?START1:START0);
}
/**
*描述:引脚初始化
*参数:
*返回:
**/
void DC_CS5464_GPIO_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_11| GPIO_Pin_12| GPIO_Pin_13| GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
}
/**
*描述:读寄存器
*参数:
*返回:
**/
void Get_DC_CS6464_Register(unsigned char registerName,unsigned char *array)
{
	read_5464(array,registerName); 
}
/**
*描述:获取状态
*参数:
*返回:
**/
void Get_DC_CS5464_Status(unsigned char *cs5464Status)
{
	read_5464(cs5464Status,Status); 
}
/**
*描述:清除
*参数:
*返回:
**/
void Clear_DC_CS5464_Status(unsigned char *cs5464Status)
{
	read_5464(cs5464Status,Status|WRITE); 
}
//CS5464有两种掉电模式来节电, 等待模式和睡眠模式。在等待模式，除了参考电压和时钟以外所有电路都被关闭。
//在睡眠模式，除了指令解码器外，所有电路都关闭。待机和引发一个硬件复位的睡眠模式之后用唤醒/暂停命令恢复。
//软件复位命令模拟引脚复位,不是电源控制功能。
void DC_CS5464_Power_Control(u8 commond)
{
	send_5464(commond); 
}
//CS5464必须处于有效状态，清除DRDY位	
//当DRDY被置位，此时读出来，存储至EEPROM 下次直接写入直流偏移寄存器中，无需校准
void DC_CS5464_Offset()
{
	
	DC_CS5464_Power_Control(WAKEUP_HALT);
	send_5464(DC_Offset|Current_Channel1|Voltage_Channel1);
	MyDelay_ms(4000);
}
void DC_CS5464_V_Gain()
{
	//当前还未确定直流量程范围！！！光伏？？？
}

void DC_CS5464_I_Gain()
{
	//________________________
}
/**
*描述:初始话CS5464
*参数:
*返回:
**/
static void DC_CS5464_SPI_Init()
{
	CS_54641();	
	CS_54640();
	SDI_54640();
	SCLK_54640();
    MyDelay_us(1);
	RESET_54640();
	MyDelay_ms(1);
	RESET_54641();
	MyDelay_ms(1);
	send_5464(SYNC1);
	send_5464(SYNC1);
	send_5464(SYNC1);
    send_5464(SYNC0);
}
/**
*描述:配置CS5464
*参数:
*返回:
**/
void DC_CS5464_Config(void)
{	
    unsigned char *temp;
    DC_CS5464_SPI_Init();
	cs5464rw[0] = 0x00;
	cs5464rw[1] = INTCONFIG_2;
	cs5464rw[2] = K_1;
	temp = &cs5464rw[0];
	write_5464(temp,Config+WRITE);
//	write_5464(cs5464rw,Config+WRITE);
	cs5464rw[0] = 0x00; 
	cs5464rw[1] = 0x00;
	cs5464rw[2] = I1gain_250mv|NOCPU_Disabled;
	temp = &cs5464rw[0];
	write_5464(temp,Ctrl|WRITE);
	writePage(0x01);	
	cs5464rw[0] = 0x00; 
	cs5464rw[1] = 0x00;
	cs5464rw[2] = 0x01;
	temp = &cs5464rw[0];
	write_5464(temp,Modes|WRITE);
	cs5464rw[0] = 0x00;             
	cs5464rw[1] = 0x0F;
	cs5464rw[2] = 0xA0;
	temp = &cs5464rw[0];
	write_5464(temp,CycleCount|WRITE);
	writePage(0x00);	
	cs5464rw[0] = 0xff;              
	cs5464rw[1] = 0xff;
	cs5464rw[2] = 0xff;
	temp = &cs5464rw[0];
	write_5464(temp,Status|WRITE);
	cs5464rw[0] = 0x00;  
	cs5464rw[1] = 0x00;
	cs5464rw[2] = 0x00;
	temp = &cs5464rw[0];
	write_5464(temp,Mask|WRITE);
	DC_CS5464_Offset();
	Conversion(1);
}

