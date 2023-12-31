/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <icons.h>
#include <ssd1306.h>
#include <ssd1306_conf.h>
#include <ssd1306_fonts.h>

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t rxData;
float Present_Voltage = 0.00;
short Percent_Battery = 0;
uint16_t ADC_Data = 0;
float D_Drop_Voltage = 0.4 ;
float Max_Bat_Voltage = 4.2;
float Weak_Bat_Voltage = 10.8;
float Number_of_Resister = 4;
uint8_t bluetooth_status = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Control Motor using timer */
void My_Motor_Control(int32_t left_vel ,int32_t right_vel)
{
	// Left motor control
	if(left_vel < 0)
	{
		TIM4->CCR1 = left_vel;
		TIM4->CCR2 = 0;
	}
	else
	{
		TIM4->CCR1 = 0;
		TIM4->CCR2 = left_vel;
	}

	//Right motor control
	if(right_vel < 0)
	{
		TIM4->CCR4 = right_vel;
		TIM4->CCR3 = 0;
	}
	else
	{
		TIM4->CCR4 = 0;
		TIM4->CCR3 = right_vel;
	}
}



/* Oled part */
void my_Low_Power_Alert()
{
	  Percent_Battery =  (float)HAL_ADC_GetValue(&hadc1)*3.3*4 /( 4096*(Max_Bat_Voltage - D_Drop_Voltage) ) *100;
	 if(Percent_Battery < 25.0 )
	 {
		 HAL_Delay(10);
	  	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);

	 }else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
}

void Display_PID(float Kp_val , float Ki_val , float Kd_val , uint16_t M_Speed_val )
{
	  char Kp_Val_Str[6] ;
	  char Ki_Val_Str[6] ;
	  char Kd_Val_Str[6] ;
	  char M_Speed_Val_Str[6];

	  /* Display PID value */
	  sprintf(Kp_Val_Str , "%.3f" , Kp_val);
	  sprintf(Ki_Val_Str , "%.3f" , Ki_val);
	  sprintf(Kd_Val_Str , "%.3f" , Kd_val);
	  sprintf(M_Speed_Val_Str , "%d" , M_Speed_val);

/* Display Kp */
	  ssd1306_SetCursor(75 , 20);
	  ssd1306_WriteString("P: ",Font_6x8,White);
	  ssd1306_WriteString(Kp_Val_Str,Font_6x8,White);

/* Display Ki */
	  ssd1306_SetCursor(75 , 30);
	  ssd1306_WriteString("I: ",Font_6x8,White);
	  ssd1306_WriteString(Ki_Val_Str,Font_6x8,White);

/* Display Kd */
	  ssd1306_SetCursor(75 , 40);
	  ssd1306_WriteString("D: ",Font_6x8,White);
	  ssd1306_WriteString(Kd_Val_Str,Font_6x8,White);

	  /* Display M_Speed Value */
	  ssd1306_SetCursor(75, 50);
	  ssd1306_WriteString("Sp:",Font_6x8,White);
	  ssd1306_WriteString(M_Speed_Val_Str,Font_6x8,White);
}

void Display_Battery()
{
	Percent_Battery =  (float)HAL_ADC_GetValue(&hadc1)*3.3*4 /( 4096*(Max_Bat_Voltage - D_Drop_Voltage) ) *100;
	char BatStr[3] ;
	/* Display battery status */
		  sprintf(BatStr, "%d", Percent_Battery);
		  if (Percent_Battery <= 5)
		  {
			  ssd1306_DrawBitmap(2, 17, battery_5 , 25 ,25 ,White); //Bat [0 - 5%]
			  ssd1306_DrawBitmap(115, 1, plug , 16, 16, White);
			  my_Low_Power_Alert();
		  }
		  else if (Percent_Battery <= 25 && Percent_Battery >= 5)
		  {
			  ssd1306_DrawBitmap(2, 17, battery_4 , 25, 25, White); //Bat [5 - 25%]
			  ssd1306_DrawBitmap(115, 1, plug , 16, 16, White);
			  my_Low_Power_Alert();
		  }
		  else if (Percent_Battery <=50 && Percent_Battery >=25)
		  {
			  ssd1306_DrawBitmap(2, 17, battery_3 , 25, 25, White); //Bat [25 - 50%]

		  }
		  else if (Percent_Battery <=75 && Percent_Battery >=50)
		  {
			  ssd1306_DrawBitmap(2, 17, battery_2 , 25, 25, White); //Bat [50 - 75%]
		  }
		  else
		  {
			  ssd1306_DrawBitmap(2, 17, battery_1 , 25, 25, White); //Bat [75 - 100%]
		  }

		  ssd1306_SetCursor(30 , 21);
		  ssd1306_WriteString(BatStr,Font_11x18 , White);
		  ssd1306_WriteString("%",Font_7x10 , White);

}

void Display_Frame()
{
	/* Draw Frames */
		  ssd1306_Line(1, 17, 128, 17, White);
		  ssd1306_Line(70, 18, 70, 64, White);
		  ssd1306_Line(1, 41, 69, 41,  White);

}

void Display_Logo()
{
	/* Display Logo */
		  ssd1306_DrawBitmap(1, 1, LogoPIFAVENGER , 90, 16, White); //Logo

}

void Display_Bluetooth_status(uint8_t bluetooth_on)
{
	  /* Display Bluetooth Status*/
	  if(bluetooth_on == 1)
	  {
	  	  ssd1306_DrawBitmap(107, 4, bluetooth , 10, 10, White);

	  }
}


void my_Oled_Display(float Kp_val , float Ki_val , float Kd_val , uint16_t M_Speed_val )
{

	  bluetooth_status = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
	  ssd1306_Fill(Black);
	  Display_Logo();
	  Display_Frame();
	  Display_Bluetooth_status(bluetooth_status);
	  Display_Battery();
	  Display_PID(Kp_val,Ki_val,Kd_val,M_Speed_val);

/* Initialize Screen */
	  ssd1306_UpdateScreen();


}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_IT(&hadc1);
  HAL_UART_Receive_IT(&huart1,&rxData,1);

  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
  ssd1306_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  my_Oled_Display(0.394 , 0.819 , 2.981 , 500 );
	  My_Motor_Control(800 ,800);
	  HAL_Delay(100);
	  My_Motor_Control(-800 ,800);
	  HAL_Delay(100);
	  My_Motor_Control(800 ,-800);
	  HAL_Delay(100);
	  My_Motor_Control(-800 ,-800);
	  HAL_Delay(100);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 159;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//ADC Interrupt
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	  if(hadc->Instance == hadc1.Instance)
	  {
		  ADC_Data =  HAL_ADC_GetValue(&hadc1);

	  }
}
//UART Interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  if(huart->Instance==USART1)
  {

    if(rxData==78) // Ascii value of 'N' is 78 (N for NO)
    {
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
    }
    else if (rxData==89) // Ascii value of 'Y' is 89 (Y for YES)
    {
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
    }
    HAL_UART_Receive_IT(&huart1,&rxData,1); // Enabling interrupt receive again
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
