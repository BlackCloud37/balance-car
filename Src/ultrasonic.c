#include "ultrasonic.h"
#include "tim.h"
#include "main.h"

unsigned int TIM1CH4_CAPTURE_STA;
//bit7:捕获完成标志 
//bit6：捕获到高点平标志
//bit5~0：捕获到高电平后定时器溢出的次数
unsigned int TIM1CH4_CAPTURE_VAL;

// 超声波检测距离，单位cm
int Distance = 100;
// 超声波自检标识，0--模块没插上，1--模块正常
int UltraError = 0;

/*
    触发一次超声波测距，并读取上次测量结果
*/
void Read_Distane(void)
{   
     HAL_GPIO_WritePin(TRIG_GPIO_Port,TRIG_Pin,GPIO_PIN_SET);
     HAL_Delay(1);  
     HAL_GPIO_WritePin(TRIG_GPIO_Port,TRIG_Pin,GPIO_PIN_RESET);

    if(TIM1CH4_CAPTURE_STA&0X80)//成功捕获到了一次高电平
    {
			int oldDistance = Distance;
			Distance = TIM1CH4_CAPTURE_STA&0X3F;
			Distance *= 65536;                    //溢出时间总和
			Distance += TIM1CH4_CAPTURE_VAL;      //得到总的高电平时间
			Distance = Distance*170/10000;        //超声波测距是测量声波发出去和反射回来是时间间隔，用微妙做单位。
			if (Distance > 1000) {
			  // 溢出
				Distance = 60;
			}
			Distance = oldDistance * 0.1 + Distance * 0.9; // 假模假样滤波
			TIM1CH4_CAPTURE_STA=0;                //开启下一次捕获
    }                
}

/*
    模块自检，用于上电时检测超声波模块是否插上
*/
void UltraSelfCheck(void)
{
    HAL_Delay(1000);//新版超声波模块上电内部初始化要等1秒，先延时1秒

    if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)){//读ECHO引脚，看看有没有高电平信号
        HAL_Delay(50);
        if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11))//再读ECHO引脚
            UltraError = 1;//1表示正常插上
    }
}

int IsUltraOK(void)
{
    return UltraError;
}
