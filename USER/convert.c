//========================================================================
// 文件名: convert.c
// 作  者: HaoLin Li(lihaolinwork@foxmail.com)
// 日  期: 2018/04
// 描  述: 
//
// 版  本:
//      
//========================================================================
#include "convert.h"
#include "math.h"
float ComplementConver(unsigned char *array)
{
	float temp=0;
	if ((array[0] >> 7 & 1) == 1)
	{
		temp = -1;
	}
	int mask = -7;
	for (int k = 0; k < 7; k++)
	{
		temp += (array[0] >> k & 1) * pow(2, mask);
		mask++;
	}
	mask = -15;
	for (int k = 0; k < 8; k++)
	{
		temp += (array[1] >> k & 1) * pow(2, mask);
		mask++;
	}
	mask = -23;
	for (int k = 0; k < 8; k++)
	{
		temp += (array[2] >> k & 1) * pow(2, mask);
		mask++;
	}
	return temp;
}
float UnsignedConver(unsigned char *array)
{
	float temp = 0.0;
	int mask = -8;
	for (int k = 0; k < 8; k++)
	{
		temp += (array[0] >> k & 1) * pow(2, mask);
		mask++;
	}
	mask = -16;
	for (int k = 0; k < 8; k++)
	{
		temp += (array[1] >> k & 1) * pow(2, mask);
		mask++;
	}
	mask = -24;
	for (int k = 0; k < 8; k++)
	{
		temp += (array[2] >> k & 1) * pow(2, mask);
		mask++;
	}
	return temp;
}
float TConversion(unsigned char *array)
{
	float temp = 0.0;
	int mask = 0;
	if ((array[0] >> 7 & 1) == 1)
	{
		temp = -128;
	}
	for (int k = 0; k < 7; k++)
	{
		temp += (array[0] >> k & 1) * pow(2, mask);
		mask++;
	}
	mask = -8;
	for (int k = 0; k < 8; k++)
	{
		temp += (array[1] >> k & 1) * pow(2, mask);
		mask++;
	}
	return temp;

}
unsigned char tempArray[4];
union
{
    char a[4];
    float b;
}tempf;
void FloatToBinary(float valuef)
{
     tempf.b   = valuef;      
     tempArray[0] = tempf.a[0];
     tempArray[1] = tempf.a[1];
     tempArray[2] = tempf.a[2];
     tempArray[3] = tempf.a[3];
}
void FloatToBinaryByU16Array(float f,unsigned short *p)
{
	unsigned short t=0;
    FloatToBinary(f);
	t= tempArray[0];
	t=(t<<8)|tempArray[1];
	*p=t;
	t= tempArray[2];
	t=(t<<8)|tempArray[3];
	*p++;
	*p=t;
}




