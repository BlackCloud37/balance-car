#include "button.h"
#include "main.h"
#include "stm32f1xx_it.h"

int iButtonCount;//i����int�ͱ�����ButtonCount��ʾ������������
int iButtonFlag;//i����int�ͱ�����ButtonFlag��ʾ�ذ�����־��1�������°�����0Ϊû�����°���
int g_iButtonState;//g��globle����ȫ�ֱ��������������ط����ã�i����int�ͱ�����ButtonState��ʾ������־��1�����£�0�����ɿ�
void ButtonScan(void){
	/*
  if(   HAL_GPIO_ReadPin(Button_GPIO_Port,Button_Pin) == GPIO_PIN_RESET )//������ż�⵽�͵�ƽ
  {
      iButtonCount++;                         //�������£�����iButtonCount��1
      if(iButtonCount>=30)                    //1ms�жϷ�����������һ�Σ�iButtonCount���ڵ���30�����������ȶ�����30ms
         {
            if(iButtonFlag==0)                  //�ж���û���ذ�����1Ϊ�У�0Ϊû��
                {
                    g_iButtonState=1;                 //���ð�����־
                    iButtonCount=0;
                    iButtonFlag=1;                  //�����ذ�����־
                    }
                else                              //����ذ����������¼���
                    iButtonCount=0;
                }
    else                                  //���û���ȶ�����30ms�������û�а��°���
         g_iButtonState=0;

         }
else                                      //���һֱ�޼�⵽�͵�ƽ����һֱ�ް�������
    {
         iButtonCount=0;                  //����iButtonCount
         g_iButtonState=0;                  //���������־
         iButtonFlag=0;                   //����ذ�����־
    }
		*/
}
