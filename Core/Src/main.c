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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "tm1637.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DELAY_BEFORE_UPDATE 4
#define TIM3_PSC 548
#define TIM3_DEFAULT_FREQ 500
#define TIM3_MAX_FREQ 999
#define TIM3_MIN_FREQ 15
#define ENCODER_GAIN 5
#define CIRCLE_MASK 0x63
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
const uint8_t circle[1] = {CIRCLE_MASK};
const uint8_t emptySegment[1] = {0x0};

volatile uint8_t updateDelayCounter = 0;
uint16_t timesDetected = 0;
uint16_t measuredFreq = 0;
int16_t encoderPos = 0;
uint16_t magFreq = TIM3_DEFAULT_FREQ;

bool hallState = 0;
bool hallPrevState = 0;
bool hallProcessing = 0;

tm1637_t Mag_Display;
tm1637_t Hall_Display;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Определяет направление вращения энкодера (1 - по часовой, 0 - против)
bool FindRotDirection(void){
	bool clkState = HAL_GPIO_ReadPin(Encoder_A_GPIO_Port, Encoder_A_Pin);
	bool dtState = HAL_GPIO_ReadPin(Encoder_B_GPIO_Port, Encoder_B_Pin);

	// Если состояния CLK и DT совпадают, вращение по часовой стрелке.
	// Если различаются, вращение против часовой стрелки.
	return (clkState == dtState);
}

//Обновляет заданную частоту магнита
void UpdMagFreq(void){
	//Проверка на три вращения в одну сторону подряд
	bool encState = FindRotDirection();
	for(int i = 0; i != 3; ++i){
		if (encState != FindRotDirection())
			return;
	}

	//Счисление заданной частоты
	if(encState)
		magFreq += ENCODER_GAIN;
	else if(magFreq == 999)
		magFreq = 995; //Уменьшение с 999 до 995, чтобы сохранять кратные 5-ти значениям
	else
		magFreq -= ENCODER_GAIN;
	magFreq = (magFreq < TIM3_MIN_FREQ) ? TIM3_MIN_FREQ : magFreq;
	magFreq = (magFreq > TIM3_MAX_FREQ) ? TIM3_MAX_FREQ : magFreq;
	tm1637_write_int(&Mag_Display, magFreq, 0);
}

//Проверяет наличие восходящего фронта магнитного поля
void FieldDetect(void){
	hallProcessing = true; //Блокирующий флаг для прерываний по датчику Холла
	for(int i = 0; i != 5; ++i){
		if(HAL_GPIO_ReadPin(Hall_Sensor_GPIO_Port, Hall_Sensor_Pin)){ //Датчик Холла использует инвертированную логику
			hallProcessing = false;
			return;
		}
	}
	hallProcessing = false;
	++timesDetected; //Если восходящий фронт зафиксирован - увеличиваем количество фиксаций
}

//Обновляет частоту таймера 3 в соответствии с заданной
void UpdTim3Freq(uint16_t desiredFreq){
	uint32_t sysClockFreq = HAL_RCC_GetSysClockFreq();
	uint32_t arr = sysClockFreq/(desiredFreq * (TIM3_PSC + 1)) - 1;
	if (arr > 0xFFFF) arr = 0xFFFF;

	HAL_TIM_PWM_Stop(&htim3, 1);
	TIM3->ARR = arr;
	TIM3->CCR1 = arr/2;
	HAL_TIM_PWM_Start(&htim3, 1);
}

void EncoderInterruptHandler(void){
	UpdMagFreq();
	updateDelayCounter = 1; //Запуск счётчика до обновления
	tm1637_write_segment(&Mag_Display, circle, 1, 3); //Условное обозначение, что частота была изменена и подлежит обновлению
}

//Обрабатывает прерывание по двум возможным каналам
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if((GPIO_Pin == Hall_Sensor_Pin) && (!hallProcessing)){ //Прерывание по датчику Холла
		FieldDetect();
	}else if(GPIO_Pin == Encoder_A_Pin){ //Прерывание по энкодеру
		EncoderInterruptHandler();
	}
}

//Обрабатывает прерывание по таймеру
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		measuredFreq = timesDetected; //По истечению секунды измеренная частота будет равна количеству фиксаций
		timesDetected = 0; //Сброс счётчика фиксаций

		if(updateDelayCounter){ //Если счётчик запущён, ждём ещё три прерывания
			if(updateDelayCounter == DELAY_BEFORE_UPDATE){
				UpdTim3Freq(magFreq); //Частота обновляется
				updateDelayCounter = 0; //Счётчик сбрасывается
				tm1637_write_segment(&Mag_Display, emptySegment, 1, 3); //Условное обозначение сбрасыватся
			}else updateDelayCounter++;
		}
		tm1637_write_int(&Hall_Display, measuredFreq, 0);
	}
}

//Локальная функция инициализации дисплеев, проведение теста (моргание всеми сегментами)
void DisplayInit(void){
	  tm1637_init(&Mag_Display, Mag_Display_CLK_GPIO_Port, Mag_Display_CLK_Pin, Mag_Display_DIO_GPIO_Port, Mag_Display_DIO_Pin);
	  tm1637_init(&Hall_Display, Hall_Display_CLK_GPIO_Port, Hall_Display_CLK_Pin, Hall_Display_DIO_GPIO_Port, Hall_Display_DIO_Pin);

	  tm1637_brightness(&Mag_Display, 4);
	  tm1637_brightness(&Hall_Display, 4);

	  tm1637_fill(&Mag_Display, 1);
	  tm1637_fill(&Hall_Display, 1);
	  HAL_Delay(1000);
	  tm1637_fill(&Mag_Display, 0);
	  tm1637_fill(&Hall_Display, 0);

	  tm1637_write_int(&Mag_Display, magFreq, 0);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  DisplayInit();

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1151;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 62499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 548;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 261;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 130;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Mag_Display_DIO_Pin|Mag_Display_CLK_Pin|Hall_Display_DIO_Pin|Hall_Display_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Encoder_B_Pin */
  GPIO_InitStruct.Pin = Encoder_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Encoder_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Encoder_A_Pin */
  GPIO_InitStruct.Pin = Encoder_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Encoder_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Hall_Sensor_Pin */
  GPIO_InitStruct.Pin = Hall_Sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Hall_Sensor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Mag_Display_DIO_Pin Mag_Display_CLK_Pin Hall_Display_DIO_Pin Hall_Display_CLK_Pin */
  GPIO_InitStruct.Pin = Mag_Display_DIO_Pin|Mag_Display_CLK_Pin|Hall_Display_DIO_Pin|Hall_Display_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
