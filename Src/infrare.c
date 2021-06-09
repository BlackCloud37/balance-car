#include "gpio.h"
#include "infrare.h"
#include "control.h"
#include "math.h"
/*
	��ͨ������ģ��
	result�Ӹ�λ����λ�ֱ��ʶͨ��Rb Ra Lb La 
	��⵽���ߵ�ͨ����Ӧ��λ��1
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
		if(cnt == 4)// ���ÿ��ͨ�����Ǹߵ�ƽ�����ж�Ϊ����ģ��û�н���
			InfrareError = 1;
}

int IsInfrareOK(void)
{
	return g_fCarAngle >= -10 && g_fCarAngle <= 1;
}