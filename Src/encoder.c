#include "tim.h"//����timͷ�ļ�
#include "encoder.h"

int GetTim4Encoder(void)//��ȡTIM4��ʱ���������ı���������
{
    int iTim4Encoder = (short)(__HAL_TIM_GET_COUNTER(&htim4));//�ȶ�ȡ������
    __HAL_TIM_SET_COUNTER(&htim4,0);//�ټ���������
    return iTim4Encoder;//����������
}

int GetTim2Encoder(void)//��ȡTIM4��ʱ���������ı���������
{
    int iTim2Encoder = (short)(__HAL_TIM_GET_COUNTER(&htim2));//�ȶ�ȡ������
    __HAL_TIM_SET_COUNTER(&htim2,0);//�ټ���������
    return iTim2Encoder;//����������
}
