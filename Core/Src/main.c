/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Component includes */
#include "spl06-007.h"
#include "i2c-lcd.h"
#include "ssd1306.h"

/* OLED Driver Includes */
#include "fonts.h"
#include "bitmap.h"
#include "horse_anim.h"
#include "utsmabitmap.h"

/* Big LCD Driver Includes */
#include "ST7920_SERIAL.h"
#include "delay.h"
#include "bitmap.h"

/* Standard C Includes */
#include "stdio.h"
#include "string.h"

/* Utility Includes*/
#include "i2cScanner.h"

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	/* I2C LCD Variables */
	int row=0;
	int col=0;

	/* I2C SPL06 Variables */
	SPL06_007 pressureSensor;

	/* UART Variables */
	uint8_t msg[40];

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  	  /* Start I2C Scan */
  	  i2cScanner();

  	  /* OLED Initialise */
	  SSD1306_Init();

  	  /* LCD Init */
  	  lcd_init();

	  /* Initialise pressure sensor */
	  uint8_t ret = SPL06_007_Initialise( &pressureSensor, &hi2c1 );
	  if (ret == 0){
		  // Success
		  sprintf(msg, "Successfully Connected to SPL006. \r\n");
		  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  } else {
		  // Errors
		  sprintf(msg, "Number of errors: %d\r\n", ret);
		  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  }

	  /* LCD Welcome */
	  lcd_send_string ("Design Challenge");
	  HAL_Delay(500);
	  lcd_put_cur(1, 0);
	  lcd_send_string("Vehicle Online");
	  HAL_Delay(2000);
	  lcd_clear ();

	  /* OLED UTSMA Logo */
	  SSD1306_DrawBitmap(0,0,utsma, 128, 64, 1);
	  SSD1306_UpdateScreen();

	  HAL_Delay(2000);

	  /* OLED Scrolling */
	  SSD1306_ScrollRight(0x00, 0x0f);     // scroll entire screen right
	  HAL_Delay (2000);
	  SSD1306_ScrollLeft(0x00, 0x0f);  	   // scroll entire screen left
	  HAL_Delay (2000);
	  SSD1306_Scrolldiagright(0x00, 0x0f); // scroll entire screen diagonal right
	  HAL_Delay (2000);
	  SSD1306_Scrolldiagleft(0x00, 0x0f);  // scroll entire screen diagonal left
	  HAL_Delay (2000);
	  SSD1306_Stopscroll();   			   // stop scrolling
	  SSD1306_InvertDisplay(1);   		   // invert the display
	  HAL_Delay(2000);
	  SSD1306_InvertDisplay(0);  		   // normalize the display

	  /* Big LCD Code */
	  delay_init();

	  ST7920_Init();
	  ST7920_SendString(0,0, "HELLO WORLD");
	  ST7920_SendString(1,0, "FROM");
	  ST7920_SendString(2,0, "CONTROLLERSTECH");
	  ST7920_SendString(3,0, "1234567890!@#$%^");
	  HAL_Delay(2000);
	  ST7920_Clear();
	  ST7920_GraphicMode(1);
	  ST7920_DrawBitmap(utsma);
	  HAL_Delay(2000);
	  ST7920_Clear();
	  ST7920_DrawBitmap(logo);
	  ST7920_Clear();
	  HAL_Delay(100);
	  DrawCircle(110, 31, 12);
	  DrawCircle(110, 31, 16);
	  DrawLine(3, 60, 127, 33);
	  ST7920_Update();
	  DrawRectangle (100, 12, 20, 14);
	  ST7920_Update();
	  DrawFilledRectangle(30, 20, 30, 10);
	  ST7920_Update();
	  DrawFilledCircle(15, 30, 6);
	  ST7920_Update();
	  DrawFilledTriangle(1,5,10,5,6,15);
	  ST7920_Update();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Reading Raw Temperature */
	  uint32_t rawTemp = SPL06_007_getRawTemp(&pressureSensor);
	  sprintf(msg, "RAW TEMP: %d\r\n", rawTemp);
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	  /* Reading Compensated Temperature */
	  uint32_t compTemp = SPL06_007_calcCompTemp(&pressureSensor, rawTemp);
	  sprintf(msg, "COMP TEMP: %d\r\n", compTemp);
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	  /* DEBUG - Print coefficient 0 */
	  int16_t  c0 = SPL06_007_getSplitHighCoefficient(&pressureSensor, SPL06_REG_C0, SPL06_REG_C01C1);
	  sprintf(msg, "C0: 0x%02X\r\n", c0);
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	  /* DEBUG - Print coefficient 1 */
	  int16_t  c1 = SPL06_007_getSplitLowCoefficient(&pressureSensor, SPL06_REG_C01C1, SPL06_REG_C1);
	  sprintf(msg, "C1: 0x%02X\r\n", c1);
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	  /* LCD Messaging */
	  lcd_put_cur(0, 0);
	  lcd_send_string ("State A");
	  lcd_put_cur(1, 0);
	  sprintf(msg, "Temp: %d", rawTemp);
	  lcd_send_string (msg);
	  HAL_Delay(1000);
	  lcd_clear ();

	  //// HORSE ANIMATION START //////
	  SSD1306_Clear();
	  SSD1306_DrawBitmap(0,0,horse1,128,64,1);
	  SSD1306_UpdateScreen();

	  SSD1306_Clear();
	  SSD1306_DrawBitmap(0,0,horse2,128,64,1);
	  SSD1306_UpdateScreen();

	  SSD1306_Clear();
	  SSD1306_DrawBitmap(0,0,horse3,128,64,1);
	  SSD1306_UpdateScreen();

	  SSD1306_Clear();
	  SSD1306_DrawBitmap(0,0,horse4,128,64,1);
	  SSD1306_UpdateScreen();

	  SSD1306_Clear();
	  SSD1306_DrawBitmap(0,0,horse5,128,64,1);
	  SSD1306_UpdateScreen();

	  SSD1306_Clear();
	  SSD1306_DrawBitmap(0,0,horse6,128,64,1);
	  SSD1306_UpdateScreen();


	  SSD1306_Clear();
	  SSD1306_DrawBitmap(0,0,horse7,128,64,1);
	  SSD1306_UpdateScreen();

	  SSD1306_Clear();
	  SSD1306_DrawBitmap(0,0,horse8,128,64,1);
	  SSD1306_UpdateScreen();


	  SSD1306_Clear();
	  SSD1306_DrawBitmap(0,0,horse9,128,64,1);
	  SSD1306_UpdateScreen();


	  SSD1306_Clear();
	  SSD1306_DrawBitmap(0,0,horse10,128,64,1);
	  SSD1306_UpdateScreen();
	  //// HORSE ANIMATION ENDS //////

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0010061A;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
