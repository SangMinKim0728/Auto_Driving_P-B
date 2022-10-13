/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// UART ?��?��?�� ?��?�� ?��?��
#define RxBufferSize	0xFF
#define countof(a)	(sizeof(a) / sizeof(*(a)))
uint8_t RxBuffer[RxBufferSize];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
char i=0, cnt=1;
char final_flag = 0;
char msg[10] = { };
char temp_msg[10] = { };
int port_num = 0;
char total_cnt = 0;

uint8_t TxBuffer02[] ="\r----------------------------------------------------------\n";
uint8_t TxBuffer03[] ="\r              ** Power Board Port Control **              \n";
uint8_t TxBuffer04[] ="\r----------------------------------------------------------\n";
uint8_t TxBuffer05[] ="\r[PA6]  : LIDAR_IN_CTRL			\n";
uint8_t TxBuffer06[] ="\r[PB14] : WAKE2_1				\n";
uint8_t TxBuffer07[] ="\r[PB12] : WAKE2_2				\n";
uint8_t TxBuffer08[] ="\r[PB15] : WAKE1_1				\n";
uint8_t TxBuffer09[] ="\r[PB13] : WAKE1_2				\n";
uint8_t TxBuffer10[] ="\r[PB9]  : PWRON_LED_CTL			\n";
uint8_t TxBuffer11[] ="\r[PB1]  : GPU_IN_CTRL			\n";
uint8_t TxBuffer12[] ="\r[PC12] : FAN_12V_ON			\n";
uint8_t TxBuffer13[] ="\r[PB11] : PWR_SHUTDOWN			\n";
uint8_t TxBuffer14[] ="\r[PB6]  : VCC_5V_EN_1			\n";
uint8_t TxBuffer15[] ="\r[PB0]  : AP_GATEWAY_IN_CTRL	\n";
uint8_t TxBuffer16[] ="\r[PC5]  : VCC_5V_EN_2			\n";
uint8_t TxBuffer17[] ="\r----------------------------------------------------------\n";
uint8_t TxBuffer18[] ="\rEX) Type 'PA6 H' or 'PB12 L' 	\n";
uint8_t TxBuffer19[] ="\r----------------------------------------------------------\n\r[Input Command] : ";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
struct GPIO_PORT {
	GPIO_TypeDef	*GPIOx;
	uint16_t		GPIO_Pin;
	GPIO_PinState	PinState;
}gpio;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
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
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  Menu_Config();
  HAL_UART_Receive_IT(&huart4, (uint8_t*)RxBuffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
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

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_6
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB11 PB12
                           PB13 PB14 PB15 PB6
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_6
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void Menu_Config(void)
{
	/*
	printf("\n----------------------------------------------------------\r");
	printf("              ** Power Board Port Control **              \r");
	printf("----------------------------------------------------------\r");
	printf("[PA6]  : LIDAR_IN_CTRL			\r");
	printf("[PB14] : WAKE2_1				\r");
	printf("[PB12] : WAKE2_2				\r");
	printf("[PB15] : WAKE1_1				\r");
	printf("[PB13] : WAKE1_2				\r");
	printf("[PB9]  : PWRON_LED_CTL			\r");
	printf("[PB1]  : GPU_IN_CTRL			\r");
	printf("[PC12] : FAN_12V_ON				\r");
	printf("[PB11] : PWR_SHUTDOWN			\r");
	printf("[PB6]  : VCC_5V_EN_1			\r");
	printf("[PB0]  : AP_GATEWAY_IN_CTRL		\r");
	printf("[PC5]  : VCC_5V_EN_2			\r");
	printf("----------------------------------------------------------\r");
	printf("EX) Type 'PA6 H' or 'PB12 L' 	\r");
	printf("----------------------------------------------------------\r");
	*/

	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer02, (countof(TxBuffer02) - 1), 0xFFFF);
	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer03, (countof(TxBuffer03) - 1), 0xFFFF);
	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer04, (countof(TxBuffer04) - 1), 0xFFFF);
	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer05, (countof(TxBuffer05) - 1), 0xFFFF);
	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer06, (countof(TxBuffer06) - 1), 0xFFFF);
	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer07, (countof(TxBuffer07) - 1), 0xFFFF);
	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer08, (countof(TxBuffer08) - 1), 0xFFFF);
	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer09, (countof(TxBuffer09) - 1), 0xFFFF);
	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer10, (countof(TxBuffer10) - 1), 0xFFFF);
	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer11, (countof(TxBuffer11) - 1), 0xFFFF);
	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer12, (countof(TxBuffer12) - 1), 0xFFFF);
	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer13, (countof(TxBuffer13) - 1), 0xFFFF);
	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer14, (countof(TxBuffer14) - 1), 0xFFFF);
	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer15, (countof(TxBuffer15) - 1), 0xFFFF);
	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer16, (countof(TxBuffer16) - 1), 0xFFFF);
	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer17, (countof(TxBuffer17) - 1), 0xFFFF);
	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer18, (countof(TxBuffer18) - 1), 0xFFFF);
	HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer19, (countof(TxBuffer19) - 1), 0xFFFF);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandler)
{
	if(RxBuffer[0] == '\r')
	{
		final_flag = 1;
	}
	else
	{
		msg[i] = RxBuffer[0];
		i++;
	}

	HAL_UART_Receive_IT(&huart4, (uint8_t*)RxBuffer, 1);
}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
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
