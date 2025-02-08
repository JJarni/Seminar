/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //int status = 0;
  int status = 0;
  uint8_t msgt = 0x0, msgr = 0x0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(status) {
	  	  case 0:	//slucaj greske, titrajuce zuto
	  		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1500);
	  	  	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);	//gasenje svega osim zutog
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5, 0);	//^
			  HAL_Delay(3000);

	  		  break;

	  	  case 1:	//Zeleno za aute
	  		  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);	//zeleno za aute
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);	//crveno za pješake
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_4, 0);
	  		  break;

	  	  case 2:	//Zeleno za pjesake
	  		  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	  		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 3000);
	  		  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);		//upaljeno zuto 3s prije promjene na crveno
	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);		//gasi zeleno za aute
			  HAL_Delay(3000);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);		//gasi crveno za pjesake
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_4, 1);	//zeleno pjesaci i crveno auti
	  		  HAL_Delay(20000);		//20s za prijelaz pjesaka
	 		  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	 		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);	//gasimo zeleno pjesake
  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);	//crveno za pjesake
  			  HAL_Delay(3000);		//3s zutog prije promjene
  			  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	  		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1500);	//Vracanje PWM Duty cyclea za titranje
	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);	//zeleno za aute
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_4, 0);
			  status = 1;
	  		  break;
	  	  }
	  	  msgt = 0x0;
	  	  msgr = 0x0;
	  	  HAL_UART_Receive(&huart4, &msgr, sizeof(msgr), 100);	//primanje podatka
	  	  if(msgr != 0x0){
	  		 msgt = ~msgr;
	  		 HAL_UART_Transmit(&huart4, &msgt, sizeof(msgt), 10);	//slanje odgovora
	  		 status = 1;
	  	  }else{
	  		 status = 0;

	  	  }
	  	  if(msgr == 0x6){
	  		  status = 2;
	  	  }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
