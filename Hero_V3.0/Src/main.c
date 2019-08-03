/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Can_Filter.h"
#include "Transmit.h"
#include "Receive.h"
#include "Remote_Control.h"
#include "Shoot_Control.h"
#include "Power_Control.h"
#include "IMU.h"
#include "Gimbal_Control.h"
#include "MiniPC.h"
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
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
 {  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_UART8_Init();
  MX_USART6_UART_Init();
  MX_CAN2_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
	MX_TIM4_Init();
	MX_TIM7_Init();
	MX_TIM12_Init();
  MX_UART7_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	GMotor_Yaw.encoderCtrl.lastangle = 4096;
	GMotor_Pitch.encoderCtrl.lastangle = 4096;
  Can1_Filter_Init();
  Can2_Filter_Init();
	IMU_Data_Receive_Start();
  HAL_Delay(3000);
  
  Referee_Data_Receive_Start();
	Mini_PC_Receive_Start();
  //Yaw_Init();
  Control_Init();
  //HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);

	IMU_round = 0;
  
	//GMotor_Yaw.posCtrl.round = 0;
	//GMotor_Yaw.encoderCtrl.lastangle = 4096;
  HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim12);
	IMU_Calibration();
	
	RemotreCtl_Data_Receive_Start();
	Bullet_Num = 0;
  while(Bullet_Num_Init != Init_OK);
	Bullet_Num	= Bullet_Num_Initnum;
	
	HAL_TIM_Base_Start_IT(&htim2);
	
  HAL_DAC_Start(&hdac,DAC_CHANNEL_1);		
  HAL_ADC_Start_DMA(&hadc1,value,3);		
	HAL_Delay(100);
  //OpenDcDc();								
  
	GMotor_Yaw.velCtrl.acc = 5000;
	GMotor_Yaw.velCtrl.dec = 5000;
	GMotor_Pitch.velCtrl.acc = 1000;
  GMotor_Pitch.velCtrl.dec = 1200;
	int LED_State[9]={0};
	int blink=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		GetCapVoltage();
//		HAL_Delay(1);
    /* USER CODE END WHILE */
	//GetCapVoltage();//芃陔萇萇揤
	//static int flag_timer = 0,flag_lightup[3] = {0};
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);		//宎梓祩ㄩ謠
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);			//宎梓祩ㄩ鏢

		/* LED袨怓硌尨离弇 */  
		LED_State[3]=Primary_DC_State;								//硌尨ㄩ壽寀謠
		LED_State[4]=Secondary_DC_State;							//硌尨ㄩ壽寀謠
		switch(CAP_STA){
		   case CAP_USEDOUT:										//LED5羶萇鏢
				LED_State[5]=GPIO_PIN_SET;
				break;
		   case CAP_OK:												//LDE5ㄩ匢佶	
				blink++;
				if(blink>50){
					LED_State[5]=!LED_State[5];
					blink=0;
				}
				break;
		   case CAP_FULL:											//LDE5ㄩ都謠
				LED_State[5]=GPIO_PIN_RESET;
				break;
		}
		LED_State[6]=exstate;										//萇喀寀謠
		LED_State[7]=(PowerHeatData.chassisPower >=80 ? GPIO_PIN_RESET:GPIO_PIN_SET);
																	//閉80W寀謠
	   
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, LED_State[3]);			//場撰DC-DC硌尨
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, LED_State[4]);			//棒撰DC-DC硌尨
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, LED_State[5]);			//萇袨怓
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, LED_State[6]);			//菁攫萇埭恁寁
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, LED_State[7]);			//怀堤髡薹袨怓
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, LED_State[8]);			//帤妏蚚
		 
		HAL_Delay(10);
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  while(1)
  {
  }
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
