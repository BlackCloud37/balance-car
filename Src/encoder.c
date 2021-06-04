#include "tim.h"//包含tim头文件
#include "encoder.h"

int GetTim4Encoder(void)//获取TIM4定时器读出来的编码器脉冲
{
    int iTim4Encoder = (short)(__HAL_TIM_GET_COUNTER(&htim4));//先读取脉冲数
    __HAL_TIM_SET_COUNTER(&htim4,0);//再计数器清零
    return iTim4Encoder;//返回脉冲数
}

int GetTim2Encoder(void)//获取TIM4定时器读出来的编码器脉冲
{
    int iTim2Encoder = (short)(__HAL_TIM_GET_COUNTER(&htim2));//先读取脉冲数
    __HAL_TIM_SET_COUNTER(&htim2,0);//再计数器清零
    return iTim2Encoder;//返回脉冲数
}
