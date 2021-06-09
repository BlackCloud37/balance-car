#include "gpio.h"
#include "infrare.h"
#include "control.h"
#include "math.h"
/*
	四通道红外模块
	result从高位到低位分别标识通道Rb Ra Lb La 
	检测到黑线的通道相应的位置1
*/
/*
char InfraredDetect(void)
{
	char resut = 0;
	
	if(Lb)
		resut |= infrared_channel_Lb;
	else if(La)
		resut |= infrared_channel_La;
	else if(Ra)
		resut |= infrared_channel_Ra;
	else if(Rb)
		resut |= infrared_channel_Rb;
	return resut;
}
*/

static char InfrareError = 0;//0:Ok; 1:error
void InfrareSelfCheck(void)
{
		char cnt = 0;
		if(La==1)cnt++;
		if(Lb==1)cnt++;
		if(Ra==1)cnt++;
		if(Rb==1)cnt++;
		if(cnt == 4)// 如果每个通道都是高电平，则判断为红外模块没有接上
			InfrareError = 1;
}

int IsInfrareOK(void)
{
	return g_fCarAngle >= -10 && g_fCarAngle <= 1;
}