#include "ultrasonic.h"
#include "tim.h"
#include "main.h"

unsigned int TIM1CH4_CAPTURE_STA;
//bit7:������ɱ�־ 
//bit6�����񵽸ߵ�ƽ��־
//bit5~0�����񵽸ߵ�ƽ��ʱ������Ĵ���
unsigned int TIM1CH4_CAPTURE_VAL;

// �����������룬��λcm
int Distance = 0;
// �������Լ��ʶ��0--ģ��û���ϣ�1--ģ������
int UltraError = 0;

/*
    ����һ�γ�������࣬����ȡ�ϴβ������
*/
void Read_Distane(void)
{   
     HAL_GPIO_WritePin(TRIG_GPIO_Port,TRIG_Pin,GPIO_PIN_SET);
     HAL_Delay(1);  
     HAL_GPIO_WritePin(TRIG_GPIO_Port,TRIG_Pin,GPIO_PIN_RESET);

    if(TIM1CH4_CAPTURE_STA&0X80)//�ɹ�������һ�θߵ�ƽ
    {
        Distance = TIM1CH4_CAPTURE_STA&0X3F;
        Distance *= 65536;                     //���ʱ���ܺ�
        Distance += TIM1CH4_CAPTURE_VAL;        //�õ��ܵĸߵ�ƽʱ��
        Distance = Distance*170/10000;//����������ǲ�����������ȥ�ͷ��������ʱ��������΢������λ��
//�����е�������ÿ��340�ס�һ΢��ʱ����������ľ�����340��0.000001=0.00034��=0.034���ף�����2�����������:0.017=1.7/100�������1.7��������
        TIM1CH4_CAPTURE_STA=0;                //������һ�β���
    }                
}

/*
    ģ���Լ죬�����ϵ�ʱ��ⳬ����ģ���Ƿ����
*/
void UltraSelfCheck(void)
{
    HAL_Delay(1000);//�°泬����ģ���ϵ��ڲ���ʼ��Ҫ��1�룬����ʱ1��

    if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)){//��ECHO���ţ�������û�иߵ�ƽ�ź�
        HAL_Delay(50);
        if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11))//�ٶ�ECHO����
            UltraError = 1;//1��ʾ��������
    }
}

int IsUltraOK(void)
{
    return UltraError;
}