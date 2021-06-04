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
	SoftTimerCountDown();//软定时器计时函数
	
  g_nMainEventCount++;//每进一次中断，主事件函数自动加1

  g_nSpeedControlPeriod++;//速度环控制周期计算量自动加1
  SpeedControlOutput(); //速度环控制平滑输出处理，速度的pwm改变量如果在25ms时刻计算出后立刻输出，会造成不平滑抖动等，这段代码就是把这个25ms周期计算一次得到的pwm分配到5个5ms时间去输出，平滑地逐步逼近输出最后的计算值！ 
  if(g_nMainEventCount>=5)//SysTick是1ms一次，这里判断语句大于5就是5ms运行一次
    {
        g_nMainEventCount=0;//主事件循环每5ms循环一次，这里清零，重新计时。    
        GetMotorPulse();
    }else if(g_nMainEventCount==1){//这1ms时间片段获取数据和角度计算
        GetMpuData();//获取MPU-6050数据
        AngleCalculate();    //进行角度计算        
    }else if(g_nMainEventCount==2){
        AngleControl();        //这1ms时间片段进行角度控制
    }else if(g_nMainEventCount==3){
        g_nSpeedControlCount++;
        if(g_nSpeedControlCount >= 5)
        {
            SpeedControl();     //速度控制，25ms进行一次
            g_nSpeedControlCount=0; //清零
            g_nSpeedControlPeriod=0;//清零
        }

    }else if(g_nMainEventCount==4){    
        MotorOutput();         //电机输出函数，每5ms执行一次
    }
    //ButtonScan();
	/*
	g_nMainEventCount++;       //每进一次中断，主事件函数自动加1
  if(g_nMainEventCount>=5)   //SysTick是1ms一次，这里判断语句大于5就是5ms运行一次
  {
    g_nMainEventCount=0;     //主事件循环每5ms循环一次，这里清零，重新计时。
    GetMotorPulse();         //每5ms捕获一次脉冲
  }else if(g_nMainEventCount==1){//这1ms时间片段获取数据和角度计算
    GetMpuData();            //获取MPU-6050数据
    AngleCalculate();        //进行角度计算        
  }else if(g_nMainEventCount==2){
    AngleControl();          //这1ms时间片段进行角度控制
  }else if(g_nMainEventCount==3){
                             //这1ms时间片段暂时啥都不干
  }else if(g_nMainEventCount==4){    
    MotorOutput();           //电机输出函数，每5ms执行一次
  }
	*/
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  HAL_SYSTICK_IRQHandler(); //必须加这个语句，不加按键会失效
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
        if((TIM1CH4_CAPTURE_STA&0X80)==0)//还未成功捕获完整的高电平脉冲。（防止成功捕获了一次高电平脉冲后，还未等到main处理计算时间，又捕获到了一次变化沿。）
        {
            if(TIM1CH4_CAPTURE_STA&0X40)        //触发了变化沿，而之前已经有了上升沿，那么一定是下降沿，那么认为完整捕获了一次高电平持续时间         
            {                  
                TIM1CH4_CAPTURE_STA|=0X80;        //标记成功捕获到一次高电平脉宽
                TIM1CH4_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);//获取当前的捕获值.
                TIM_RESET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_4);   //一定要先清除原来的设置！！
                TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_4,TIM_ICPOLARITY_RISING);//配置TIM1通道4上升沿捕获
            }else                                  //还未开始,第一次捕获上升沿
            {
                TIM1CH4_CAPTURE_STA=0;            //清空
                TIM1CH4_CAPTURE_VAL=0;
                TIM1CH4_CAPTURE_STA|=0X40;        //标记触发到了上升沿
                __HAL_TIM_DISABLE(&htim1);      //关闭定时器1，来重新配置定时器为捕获下降沿。等下次捕获到了下降沿的时候就是完整的高电平脉冲
                __HAL_TIM_SET_COUNTER(&htim1,0);
                TIM_RESET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_4);   //一定要先清除原来的设置！！
                TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_4,TIM_ICPOLARITY_FALLING);//定时器1通道4设置为下降沿捕获
                __HAL_TIM_ENABLE(&htim1);        //使能定时器5
            }                
        }
    }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim1)
    {
        if((TIM1CH4_CAPTURE_STA&0X80)==0)//还未成功捕获
        {
            if(TIM1CH4_CAPTURE_STA&0X40)//已经到高电平了
            {
                if((TIM1CH4_CAPTURE_STA&0X3F)==0X3F)            //已经到了软件设计的最大高电平持续时间了，认为已经捕获了完整的高电平持续时间
                {
                    TIM1CH4_CAPTURE_STA|=0X80;        //标记成功捕获了，其实这个时候并没有检测到下降沿
                    TIM1CH4_CAPTURE_VAL=0XFFFF;
                }
                else                         //一般情况会到这个位置，让循环数+1
                    TIM1CH4_CAPTURE_STA++;
            }     
        }
    }
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
