/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VOLTAGE_COEFFICIENT 0.2725 //Коэффициент напряжения
#define MEASUREMENT_FREQUENCY 4000 //Частота измерений
#define ZERO_AXIS 2031 //Ось нуля
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */
uint16_t adc_val[6] = { 0 }; //Показания АЦП

/* Расчет действующего значения напряжения */
float u_rms[3] = { 0.0 }; //Действующее значение напряжения

uint32_t u_mom_2[3] = { 0 }; //Квадрат мгновенного значения напряжения
uint32_t u_mom_2_sum[3] = { 0 }; //Сумма квадратов мгновенных значений напряжения
uint32_t u_mom_2_sum_curr[3] = { 0 }; //Текущая сумма квадратов мгновенных значений напряжения
uint32_t u_mom_2_sum_prev[3] = { 0 }; //Предыдущая сумма квадратов мгновенных значений напряжения

/* Расчет действующего значения тока */
float i_rms[3] = { 0.0 }; //Действующее значение тока

uint32_t i_mom_2[3] = { 0 }; //Квадрат мгновенного значения тока
uint32_t i_mom_2_sum[3] = { 0 }; //Сумма квадратов мгновенных значений тока
uint32_t i_mom_2_sum_curr[3] = { 0 }; //Текущая сумма квадратов мгновенных значений тока
uint32_t i_mom_2_sum_prev[3] = { 0 }; //Предыдущая сумма квадратов мгновенных значений тока

/* Расчет действующего линейного значения напряжения фаз */
float u_l[3] = { 0.0, 0.0, 0.0 }; //Действующее линейное значение напряжение фазы

int32_t u_mom_diff_2[3] = { 0 }; //Квадрат разности мгновенных значений напряжения фаз
int32_t u_mom_diff_2_sum[3] = { 0 }; //Сумма квадратов разности мгновенных значений напряжения фаз
int32_t u_mom_diff_2_sum_curr[3] = { 0 }; //Текущая сумма квадратов разности мгновенных значений напряжения фаз
int32_t u_mom_diff_2_sum_prev[3] = { 0 }; //Предыдущая сумма квадратов разности мгновенных значений напряжения фаз

uint16_t num_of_meas[6] = { 0 }; //Количество измерений
uint16_t num_of_meas_curr[6] = { 0 }; //Текущее количество измерений
uint16_t num_of_meas_prev[6] = { 0 }; //Предыдущее количество измерений

uint16_t dead_zone_2 = 400; //Квадрат мертвой зоны около опорного напряжения
uint8_t dead_zone_flag[6] = { 0 }; //Флаг перехода измерения в мертвую зону

/* Расчет частоты */
float freq[6] = { 0 }; //Частота
float freq_filt[6] = { 0 }; //Частота отфильтрованная по верхней и нижней границе чувствительности

uint8_t freq_lim_upp = 60; //Верхняя граница чувствительности по частоте Гц
uint8_t freq_lim_low = 40; //Нижняя граница чувствительности по частоте Гц

uint16_t freq_num_of_meas[6] = { 0 }; //Количество измерений для измерения частоты
uint16_t freq_num_of_meas_curr[6] = { 0 }; //Текущее количество измерений для измерения частоты

float prev_freq[3] = { 0 }; //Предыдущая частота
float freq_view[3] = { 0 }; //Вывод частоты

uint16_t freq_hits_num = 0; //Количество совпадений частоты

uint8_t flag_calc[6] = { 0 }; //Флаг разрешения вычисления напряжений

int32_t period = 0;
int32_t pulse_width = 0;
float fr = 0; //Вывод частоты








double flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Обработка показаний ацп */
void voltage_processing (uint8_t phase)
{
  /* Если измерение больше запрещенной зоны */
  if (u_mom_2[phase] > dead_zone_2)
    {

      /* Если был переход через запрещенную зону */
      if (dead_zone_flag[phase] == 1)
	{
	  dead_zone_flag[phase] = 0;

	  /* Сохраняем вычисленные значения и разрешаем расчет */
	  if (flag_calc[phase] == 0)
	    {
	      flag_calc[phase] = 1;

	      u_mom_2_sum_curr[phase] = u_mom_2_sum[phase];
	      u_mom_diff_2_sum_curr[phase] = u_mom_diff_2_sum[phase];
	      freq_num_of_meas_curr[phase] = freq_num_of_meas[phase];
	      num_of_meas_curr[phase] = num_of_meas[phase];
	    }

	  /* Обнуляем значения */
	  u_mom_2_sum[phase] = 0;
	  u_mom_diff_2_sum[phase] = 0;
	  num_of_meas[phase] = 0;
	  freq_num_of_meas[phase] = 0;
	}

      /* Если не было перехода через запрещенную зону */
      if (dead_zone_flag[phase] == 0)
	{
	  u_mom_diff_2_sum[phase] += u_mom_diff_2[phase];
	  u_mom_2_sum[phase] += u_mom_2[phase];
	  ++num_of_meas[phase];
	}
    }

  /* Если измерение меньше запрещенной зоны */
  else
    {
      dead_zone_flag[phase] = 1;
      ++freq_num_of_meas[phase];
    }
}


void current_processing (uint8_t phase)
{
  /* Если измерение больше запрещенной зоны */
  if (i_mom_2[phase] > 400)
    {
      /* Если был переход через запрещенную зону */
      if (dead_zone_flag[phase + 3] == 1)
	{
	  dead_zone_flag[phase + 3] = 0;
	  /* Сохраняем вычисленные значения и разрешаем расчет */
	  if (flag_calc[phase + 3] == 0)
	    {
	      flag_calc[phase + 3] = 1;

	      i_mom_2_sum_curr[phase] = i_mom_2_sum[phase];
	      freq_num_of_meas_curr[phase + 3] = freq_num_of_meas[phase + 3];
	      num_of_meas_curr[phase + 3] = num_of_meas[phase + 3];
	    }
	  /* Обнуляем значения */
	  i_mom_2_sum[phase] = 0;
	  num_of_meas[phase + 3] = 0;
	  freq_num_of_meas[phase + 3] = 0;
	}
      /* Если не было перехода через запрещенную зону */
      if (dead_zone_flag[phase + 3] == 0)
	{
	  i_mom_2_sum[phase] += i_mom_2[phase];
	  ++num_of_meas[phase + 3];
	}
    }
  /* Если измерение меньше запрещенной зоны */
  else
    {
      dead_zone_flag[phase + 3] = 1;
      ++freq_num_of_meas[phase + 3];
    }
}


/* Расчет фазных и линейных напряжений */
void voltage_calculation (uint8_t phase)
{

  freq[phase] = MEASUREMENT_FREQUENCY / (float) (freq_num_of_meas_curr[phase] + num_of_meas_curr[phase]);

  if (freq[phase] > freq_lim_low && freq[phase] < freq_lim_upp)
    {
      u_rms[phase] = (sqrt ((u_mom_2_sum_curr[phase] + u_mom_2_sum_prev[phase]) / (num_of_meas_curr[phase] + num_of_meas_prev[phase]))) * VOLTAGE_COEFFICIENT;
      u_l[phase] = (sqrt ((u_mom_diff_2_sum_curr[phase] + u_mom_diff_2_sum_prev[phase]) / (num_of_meas_curr[phase] + num_of_meas_prev[phase]))) * VOLTAGE_COEFFICIENT;
      freq_filt[phase] = freq[phase];
    }

  u_mom_2_sum_prev[phase] = u_mom_2_sum_curr[phase];
  u_mom_diff_2_sum_prev[phase] = u_mom_diff_2_sum_curr[phase];
  num_of_meas_prev[phase] = num_of_meas_curr[phase];

  flag_calc[phase] = 0;
}


/* Расчет действующего значения тока */
void current_calculation (uint8_t phase)
{

  freq[phase + 3] = MEASUREMENT_FREQUENCY / (float) (freq_num_of_meas_curr[phase + 3] + num_of_meas_curr[phase + 3]);

  if (freq[phase + 3] > freq_lim_low && freq[phase + 3] < freq_lim_upp)
    {
      i_rms[phase] = (sqrt ((float)(i_mom_2_sum_curr[phase] + i_mom_2_sum_prev[phase]) / (float)(num_of_meas_curr[phase + 3] + num_of_meas_prev[phase + 3]))) * 0.006623;
      freq_filt[phase + 3] = freq[phase + 3];
    }

  i_mom_2_sum_prev[phase] = i_mom_2_sum_curr[phase];
  num_of_meas_prev[phase + 3] = num_of_meas_curr[phase + 3];

  flag_calc[phase + 3] = 0;
}


/* Фильтрация частоты */
void
freq_filtering (uint8_t phase)
{
  if (freq_hits_num == 0)
    {
      prev_freq[phase] = freq_filt[phase];
      ++freq_hits_num;
    }
  if (freq_filt[phase] == prev_freq[phase] && freq_hits_num != 0)
    {
      prev_freq[phase] = freq_filt[phase];
      ++freq_hits_num;
    }

  if (freq_filt[phase] != prev_freq[phase] && freq_hits_num != 0)
    {
      prev_freq[phase] = freq_filt[phase];
      freq_hits_num = 0;
    }

  if (freq_hits_num == 1050)
    {
      freq_view[phase] = prev_freq[phase];
      freq_hits_num = 0;
    }
}

void
freq_filtering_1 (uint8_t phase)
{
  if (freq_hits_num == 0)
    {
      prev_freq[phase] = fr;
      ++freq_hits_num;
    }
  if (fr == prev_freq[phase] && freq_hits_num != 0)
    {
      prev_freq[phase] = fr;
      ++freq_hits_num;
    }

  if (fr != prev_freq[phase] && freq_hits_num != 0)
    {
      prev_freq[phase] = fr;
      freq_hits_num = 0;
    }

  if (freq_hits_num == 10)
    {
      freq_view[phase] = prev_freq[phase];
      freq_hits_num = 0;
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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM15_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc);
  HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc_val, 6);

  HAL_TIM_Base_Start(&htim15);
  HAL_TIM_Base_Start_IT(&htim15);

  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
      if (flag_calc[0] == 1 && flag_calc[1] == 1  && flag_calc[2] == 1)
      	{
	  voltage_calculation(0);
	  voltage_calculation(1);
	  voltage_calculation(2);
      	}

      if (flag_calc[3] == 1)
	{
	  current_calculation (0);
	}
      if (flag_calc[4] == 1)
      	{
      	  current_calculation (1);
      	}
      if (flag_calc[5] == 1)
      	{
      	  current_calculation (2);
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

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T15_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 1999;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc)
{
  /* Вычисление квадратов мгновенного напряжения */
  u_mom_2[0] = (adc_val[0] - ZERO_AXIS) * (adc_val[0] - ZERO_AXIS);
  u_mom_2[1] = (adc_val[1] - ZERO_AXIS) * (adc_val[1] - ZERO_AXIS);
  u_mom_2[2] = (adc_val[2] - ZERO_AXIS) * (adc_val[2] - ZERO_AXIS);

  /* Вычисление квадратов разности мгновенных напряжений фаз */
  u_mom_diff_2[0] = (adc_val[0] - adc_val[1]) * (adc_val[0] - adc_val[1]);
  u_mom_diff_2[1] = (adc_val[1] - adc_val[2]) * (adc_val[1] - adc_val[2]);
  u_mom_diff_2[2] = (adc_val[2] - adc_val[0]) * (adc_val[2] - adc_val[0]);

  /* Вычисление квадратов мгновенного тока */
  i_mom_2[0] = (adc_val[3] - 2025) * (adc_val[3] - 2025);
  i_mom_2[1] = (adc_val[4] - 2025) * (adc_val[4] - 2025);
  i_mom_2[2] = (adc_val[5] - 2025) * (adc_val[5] - 2025);

  /* Обработка вычисленных значений напряжения */
  voltage_processing (0);
  voltage_processing (1);
  voltage_processing (2);

  /* Обработка вычисленных значений тока */
  current_processing (0);
  current_processing (1);
  current_processing (2);

  /* Запуск нового измерения */
  HAL_ADC_Start_DMA (hadc, (uint32_t*) adc_val, 6);
}






























void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
    {
      if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
	  TIM3->CNT = 0;

	  period = HAL_TIM_ReadCapturedValue (&htim3, TIM_CHANNEL_1);
	  pulse_width = HAL_TIM_ReadCapturedValue (&htim3, TIM_CHANNEL_2);
	  fr = 1000000.0/(float)(period/2);
	  freq_filtering_1 (0);
	}
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
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line num,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
