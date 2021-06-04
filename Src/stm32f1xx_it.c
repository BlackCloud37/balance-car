/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "button.h"
#include "control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	SoftTimerCountDown();//��ʱ����ʱ����
	
  g_nMainEventCount++;//ÿ��һ���жϣ����¼������Զ���1

  g_nSpeedControlPeriod++;//�ٶȻ��������ڼ������Զ���1
  SpeedControlOutput(); //�ٶȻ�����ƽ����������ٶȵ�pwm�ı��������25msʱ�̼�������������������ɲ�ƽ�������ȣ���δ�����ǰ����25ms���ڼ���һ�εõ���pwm���䵽5��5msʱ��ȥ�����ƽ�����𲽱ƽ�������ļ���ֵ�� 
  if(g_nMainEventCount>=5)//SysTick��1msһ�Σ������ж�������5����5ms����һ��
    {
        g_nMainEventCount=0;//���¼�ѭ��ÿ5msѭ��һ�Σ��������㣬���¼�ʱ��    
        GetMotorPulse();
    }else if(g_nMainEventCount==1){//��1msʱ��Ƭ�λ�ȡ���ݺͽǶȼ���
        GetMpuData();//��ȡMPU-6050����
        AngleCalculate();    //���нǶȼ���        
    }else if(g_nMainEventCount==2){
        AngleControl();        //��1msʱ��Ƭ�ν��нǶȿ���
    }else if(g_nMainEventCount==3){
        g_nSpeedControlCount++;
        if(g_nSpeedControlCount >= 5)
        {
            SpeedControl();     //�ٶȿ��ƣ�25ms����һ��
            g_nSpeedControlCount=0; //����
            g_nSpeedControlPeriod=0;//����
        }

    }else if(g_nMainEventCount==4){    
        MotorOutput();         //������������ÿ5msִ��һ��
    }
    //ButtonScan();
	/*
	g_nMainEventCount++;       //ÿ��һ���жϣ����¼������Զ���1
  if(g_nMainEventCount>=5)   //SysTick��1msһ�Σ������ж�������5����5ms����һ��
  {
    g_nMainEventCount=0;     //���¼�ѭ��ÿ5msѭ��һ�Σ��������㣬���¼�ʱ��
    GetMotorPulse();         //ÿ5ms����һ������
  }else if(g_nMainEventCount==1){//��1msʱ��Ƭ�λ�ȡ���ݺͽǶȼ���
    GetMpuData();            //��ȡMPU-6050����
    AngleCalculate();        //���нǶȼ���        
  }else if(g_nMainEventCount==2){
    AngleControl();          //��1msʱ��Ƭ�ν��нǶȿ���
  }else if(g_nMainEventCount==3){
                             //��1msʱ��Ƭ����ʱɶ������
  }else if(g_nMainEventCount==4){    
    MotorOutput();           //������������ÿ5msִ��һ��
  }
	*/
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  HAL_SYSTICK_IRQHandler(); //����������䣬���Ӱ�����ʧЧ
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim1)
    {
        if((TIM1CH4_CAPTURE_STA&0X80)==0)//��δ�ɹ����������ĸߵ�ƽ���塣����ֹ�ɹ�������һ�θߵ�ƽ����󣬻�δ�ȵ�main�������ʱ�䣬�ֲ�����һ�α仯�ء���
        {
            if(TIM1CH4_CAPTURE_STA&0X40)        //�����˱仯�أ���֮ǰ�Ѿ����������أ���ôһ�����½��أ���ô��Ϊ����������һ�θߵ�ƽ����ʱ��         
            {                  
                TIM1CH4_CAPTURE_STA|=0X80;        //��ǳɹ�����һ�θߵ�ƽ����
                TIM1CH4_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);//��ȡ��ǰ�Ĳ���ֵ.
                TIM_RESET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_4);   //һ��Ҫ�����ԭ�������ã���
                TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_4,TIM_ICPOLARITY_RISING);//����TIM1ͨ��4�����ز���
            }else                                  //��δ��ʼ,��һ�β���������
            {
                TIM1CH4_CAPTURE_STA=0;            //���
                TIM1CH4_CAPTURE_VAL=0;
                TIM1CH4_CAPTURE_STA|=0X40;        //��Ǵ�������������
                __HAL_TIM_DISABLE(&htim1);      //�رն�ʱ��1�����������ö�ʱ��Ϊ�����½��ء����´β������½��ص�ʱ����������ĸߵ�ƽ����
                __HAL_TIM_SET_COUNTER(&htim1,0);
                TIM_RESET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_4);   //һ��Ҫ�����ԭ�������ã���
                TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_4,TIM_ICPOLARITY_FALLING);//��ʱ��1ͨ��4����Ϊ�½��ز���
                __HAL_TIM_ENABLE(&htim1);        //ʹ�ܶ�ʱ��5
            }                
        }
    }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim1)
    {
        if((TIM1CH4_CAPTURE_STA&0X80)==0)//��δ�ɹ�����
        {
            if(TIM1CH4_CAPTURE_STA&0X40)//�Ѿ����ߵ�ƽ��
            {
                if((TIM1CH4_CAPTURE_STA&0X3F)==0X3F)            //�Ѿ����������Ƶ����ߵ�ƽ����ʱ���ˣ���Ϊ�Ѿ������������ĸߵ�ƽ����ʱ��
                {
                    TIM1CH4_CAPTURE_STA|=0X80;        //��ǳɹ������ˣ���ʵ���ʱ��û�м�⵽�½���
                    TIM1CH4_CAPTURE_VAL=0XFFFF;
                }
                else                         //һ������ᵽ���λ�ã���ѭ����+1
                    TIM1CH4_CAPTURE_STA++;
            }     
        }
    }
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
