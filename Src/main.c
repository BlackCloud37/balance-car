/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "u8g2.h"
#include "u8x8.h"
#include "infrare.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned short SoftTimer[5] = {0, 0, 0, 0, 0};

void SoftTimerCountDown(void)
{
    char i;
    for(i = 0;  i < 5; i++){
        if(SoftTimer[i] > 0)SoftTimer[i]--;
    }
}
//秒级任务
void SecTask()
{
    if(SoftTimer[0])return;
    else{
        SoftTimer[0] = 1000;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  u8g2_t u8g2;
	char cStr[3];
  char cStr2[6];
	char cStr3[5];
	int modeCnt = 0;
	/* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);//开启TIM4的编码器接口模式
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);//开启TIM2的编码器接口模式
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);//开启TIM1的捕获通道4，并且开启捕获中断
	__HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);//使能更新中断
	
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);//开启TIM3_CH2的PWM输出
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);//初始化BIN1引脚为低电平
  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);//初始化BIN2引脚为高电平
	
  if(!MPU_Init())//如果MPU6050初始化成功，返回0，!0则为1
  {
		printf("MPU-6050 Init Successfully");//成功了则打印 MPU-6050 Init Successfully
  }
	u8g2_Setup_ssd1306_128x64_noname_f(&u8g2,U8G2_R0,u8x8_byte_4wire_sw_spi,u8x8_stm32_gpio_and_delay);//初始化u8g2
  u8g2_InitDisplay(&u8g2);//初始zai化显示器
  u8g2_SetPowerSave(&u8g2,0);//唤醒显示器
	u8g2_SetFont(&u8g2,u8g2_font_6x12_mr);//设置英文字体
	
	SetMode(TAILING_MODE);
	
	/* USER CODE END 2 */
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		KeepDirect();
		SecTask();              // 秒级任务
		if(SoftTimer[1] == 0) { 
			SoftTimer[1] = 40;
			RunMode();
		}
		
		
		
		if(SoftTimer[2] == 0)
		{
			SoftTimer[2] = 20;//20毫秒刷新一次
			u8g2_ClearBuffer(&u8g2);//清空缓冲区的内容

			u8g2_DrawStr(&u8g2,0,10,"Pulse:");//输出固定不变的字符串Angle：        
			sprintf(cStr,"%d %d",g_iLeftTurnRoundCnt, g_iRightTurnRoundCnt);//将角度数据格式化输出到字符串cStr            
			u8g2_DrawStr(&u8g2,50,10,cStr);//输出实时变化的角度数据

			u8g2_DrawStr(&u8g2,0,20,"Distance:");//输出固定不变的字符串Distane：            
			sprintf(cStr2,"%5.1f",(float)Distance);//将超声波距离数据格式化输出到字符串cStr2
			u8g2_DrawStr(&u8g2,50,20,cStr2);//输出实时变化的超声波距离
			
			
			u8g2_DrawStr(&u8g2, 0, 30, "IR:");
			sprintf(cStr3, "%d %d %d %d", 
				Lb, La, Ra, Rb
			);
			//sprintf(cStr3, "%d %c", (int)g_SonicDoing, g_SonicAction);
			u8g2_DrawStr(&u8g2,50,30,cStr3);
			
			u8g2_DrawStr(&u8g2, 0, 40, "Direct:");
			char cStr4[20];

			sprintf(cStr4, "%3.1f %d", GetDirect(), g_iCurrentDeg);
			u8g2_DrawStr(&u8g2, 50, 40, cStr4);
			/*
			u8g2_DrawStr(&u8g2, 0, 50, "MODE:");
			sprintf(cStr3, "%d %d %d", SoftTimer[3], modeCnt, g_currentMode);
			u8g2_DrawStr(&u8g2,50,50,cStr3);
			*/
			u8g2_SendBuffer(&u8g2);//绘制缓冲区的内容
			
			Read_Distane();//每20ms读一次超声波数据
		}
		
		if(SoftTimer[3] == 0) {
			SoftTimer[3] = 5000;
			modeCnt++;
			/*
			switch(modeCnt) {
				case 0:
					currentDeg = 0;
			    break;
				case 1:
					currentDeg = 90;
					break;
				case 2:
					currentDeg = -90;
					break;
				case 3:
					currentDeg = 0;
				default:
					currentDeg = 0;
			}
			*/
			/*
			if (modeCnt == 0)
				SetMode(FORWARD_MODE);
			else if (modeCnt == 2)
				SetMode(BACKWARD_MODE);
			else if (modeCnt == 4)
				SetMode(LEFTMOVE_MODE);
			else if (modeCnt == 6)
				SetMode(RIGHTMOVE_MODE);
			else
				SetMode(STOP_MODE);
			*/
		}
		
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
