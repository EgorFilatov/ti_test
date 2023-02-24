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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VOLTAGE_COEFFICIENT 0.2725 //Коэффициент напряжения
#define CURRENT_COEFFICIENT 0.00625 //Коэффициент тока
#define MEASUREMENT_FREQUENCY 1000 //Частота измерений
#define ZERO_AXIS 2030 //Средняя точка
#define DEAD_ZONE_SQRT 400 //Квадрат мертвой зоны около опорного напряжения
#define FREQ_LIM_UPP 60 //Верхняя граница чувствительности по частоте Гц
#define FREQ_LIM_LOW 40 //Нижняя граница чувствительности по частоте Гц
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
uint16_t adc[6] = { 0 }; //Показания АЦП

uint32_t adc_sqrt[6] = { 0 }; //Квадрат мгновенного значения ацп
uint32_t summ_adc_sqrt[6] = { 0 }; //Сумма квадратов мгновенных значений ацп
uint32_t saved_summ_adc_sqrt[6] = { 0 }; //Сохраненная сумма квадратов мгновенных значений ацп
uint32_t prev_saved_summ_adc_sqrt[6] = { 0 }; //Предыдущая сумма квадратов мгновенных значений ацп

uint8_t adc_diff_sqrt[3] = { 0 }; //Квадрат разности мгновенных значения ацп
int32_t summ_adc_diff_sqrt[3] = { 0 }; //Сумма квадратов разности мгновенных значений ацп
int32_t saved_summ_adc_diff_sqrt[3] = { 0 }; //Сохраненная сумма квадратов разности мгновенных значений ацп
int32_t prev_saved_summ_adc_diff_sqrt[3] = { 0 }; //Предыдущая сумма квадратов разности мгновенных значений ацп

float u_rms[3] = { 0.0 }; //Действующее значение напряжения
float i_rms[3] = { 0.0 }; //Действующее значение тока

float u_l[3] = { 0.0 }; //Действующее линейное значение напряжение фазы

uint16_t num_of_meas[6] = { 0 }; //Количество измерений
uint16_t saved_num_of_meas[6] = { 0 }; //Текущее количество измерений
uint16_t prev_saved_num_of_meas[6] = { 0 }; //Предыдущее количество измерений

uint8_t flag_dead_zone = 0; //Флаг перехода измерения в мертвую зону

uint8_t flag_calc = 0; //Флаг разрешения вычисления

uint8_t count_overflow = 0;
uint8_t flag_fr = 0;
uint32_t per = 0;
uint32_t saved_per[3] = { 0 };
float freq = 0;

uint8_t flag_filter = 0;
uint16_t m_1[6] = { 0 };
uint16_t m_2[6] = { 0 };
uint16_t m_3[6] = { 0 };
uint16_t adc_mean[6] = { 0 };
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
/* Функция извлечения квадратного корня */
uint16_t sqrt_my (uint32_t L)
{
  if (L < 2)
    return (uint16_t) L;

  uint32_t div;
  uint32_t rslt;
  uint32_t temp;

  if (L & 0xFFFF0000L)
    if (L & 0xFF000000L)
      if (L & 0xF0000000L)
	if (L & 0xE0000000L)
	  div = 43771;
	else
	  div = 22250;
      else if (L & 0x0C000000L)
	div = 11310;
      else
	div = 5749;
    else if (L & 0x00F00000L)
      if (L & 0x00C00000L)
	div = 2923;
      else
	div = 1486;
    else if (L & 0x000C0000L)
      div = 755;
    else
      div = 384;
  else if (L & 0xFF00L)
    if (L & 0xF000L)
      if (L & 0xC000L)
	div = 195;
      else
	div = 99;
    else if (L & 0x0C00L)
      div = 50;
    else
      div = 25;
  else if (L & 0xF0L)
    if (L & 0x80L)
      div = 13;
    else
      div = 7;
  else
    div = 3;

  rslt = L;

  while (1)
    {
      temp = L / div;
      temp += div;

      div = temp >> 1;
      div += temp & 1;

      if (rslt > div)
	rslt = div;
      else
	{
	  if (L / rslt == rslt - 1 && L % rslt == 0)
	    rslt--;

	  return (uint16_t) rslt;
	}
    }
}


/* Обработка показаний ацп */
void adc_processing (uint8_t phase)
{
  /* Если измерение больше запрещенной зоны */
  if (adc_sqrt[phase] > DEAD_ZONE_SQRT)
    {
      /* Был ли выход из запрещенной зоны */
      if (flag_dead_zone & (1 << phase))
	{
	  flag_dead_zone &= ~(1 << phase); //Сбрасываем флаг запрещенной зоны

	  /* Сохраняем вычисленные значения если не ведутся расчеты */
	  if (!(flag_calc & (1 << phase)))
	    {
	      flag_calc |= (1 << phase); //Разрешаем расчет

	      saved_summ_adc_sqrt[phase] = summ_adc_sqrt[phase];
	      saved_num_of_meas[phase] = num_of_meas[phase];
	      (phase < 3) ? (saved_summ_adc_diff_sqrt[phase] = summ_adc_diff_sqrt[phase]) : 0;
	    }
	  /* Обнуляем значения вычисленные значения */
	  summ_adc_sqrt[phase] = 0;
	  num_of_meas[phase] = 0;
	  (phase < 3) ? (summ_adc_diff_sqrt[phase] = 0) : 0;
	}

      /* Выхода из запрещенной зоны небыло */
      if (!(flag_dead_zone & (1 << (phase + 3))))
	{
	  ++num_of_meas[phase];
	  summ_adc_sqrt[phase] += adc_sqrt[phase];
	  (phase < 3) ? (summ_adc_diff_sqrt[phase] += adc_diff_sqrt[phase]) : 0;
	}
    }
  /* Если измерение меньше запрещенной зоны */
  else
    {
      flag_dead_zone |= (1 << phase);
    }
}

/* Расчет фазных и линейных напряжений */
void calculation (uint8_t phase)
{

  if (phase < 3)
    {
      u_rms[phase] = (sqrt_my((saved_summ_adc_sqrt[phase] + prev_saved_summ_adc_sqrt[phase]) / (saved_num_of_meas[phase] + prev_saved_num_of_meas[phase]))) * VOLTAGE_COEFFICIENT;
      u_l[phase] = (sqrt_my((saved_summ_adc_diff_sqrt[phase] + prev_saved_summ_adc_diff_sqrt[phase]) / (saved_num_of_meas[phase] + prev_saved_num_of_meas[phase]))) * VOLTAGE_COEFFICIENT;
    }
  else
    {
      i_rms[phase - 3] = 0.006173 * (sqrt_my((saved_summ_adc_sqrt[phase] + prev_saved_summ_adc_sqrt[phase]) / (saved_num_of_meas[phase] + prev_saved_num_of_meas[phase]))) + 0.0123457;
    }
  prev_saved_summ_adc_sqrt[phase] = saved_summ_adc_sqrt[phase];
  prev_saved_num_of_meas[phase] = saved_num_of_meas[phase];
  (phase < 3) ? (prev_saved_summ_adc_diff_sqrt[phase] = saved_summ_adc_diff_sqrt[phase]) : 0;

  flag_calc &= ~(1 << phase);
}


void freq_calculation ()
{
  switch (flag_fr)
    {
    case 0:
      flag_fr = 1;
      saved_per[0] = per;

      break;

    case 1:
      flag_fr = 2;
      saved_per[1] = per;
      break;

    case 2:
      flag_fr = 0;
      saved_per[2] = per;

      freq = 48000000.0 / ((saved_per[0] < saved_per[1]) ? ((saved_per[1] < saved_per[2]) ? saved_per[1] : ((saved_per[2] < saved_per[0]) ? saved_per[0] : saved_per[2])) : ((saved_per[0] < saved_per[2]) ? saved_per[0] : ((saved_per[2] < saved_per[1]) ? saved_per[1] : saved_per[2])));
      break;
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
  HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc, 6);

  HAL_TIM_Base_Start_IT(&htim15);

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
      if (flag_calc & (1 << 0) && flag_calc & (1 << 1)  && flag_calc & (1 << 2))
      	{
	  calculation(0);
	  calculation(1);
	  calculation(2);
      	}

      if (flag_calc & (1 << 3))
	{
	  calculation (3);
	}
      if (flag_calc & (1 << 4))
	{
	  calculation (4);
	}
      if (flag_calc & (1 << 5))
	{
	  calculation (5);
	}

      freq_calculation();
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65000;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc)
{
  switch (flag_filter)
    {
    case 0:
      flag_filter = 1;

      m_1[0] = adc[0];
      m_1[1] = adc[1];
      m_1[2] = adc[2];
      m_1[3] = adc[3];
      m_1[4] = adc[4];
      m_1[5] = adc[5];
      break;

    case 1:
      flag_filter = 2;

      m_2[0] = adc[0];
      m_2[1] = adc[1];
      m_2[2] = adc[2];
      m_2[3] = adc[3];
      m_2[4] = adc[4];
      m_2[5] = adc[5];
      break;

    case 2:
      flag_filter = 0;

      m_3[0] = adc[0];
      m_3[1] = adc[1];
      m_3[2] = adc[2];
      m_3[3] = adc[3];
      m_3[4] = adc[4];
      m_3[5] = adc[5];

      adc_mean[0] = (m_1[0] < m_2[0]) ? ((m_2[0] < m_3[0]) ? m_2[0] : ((m_3[0] < m_1[0]) ? m_1[0] : m_3[0])) : ((m_1[0] < m_3[0]) ? m_1[0] : ((m_3[0] < m_2[0]) ? m_2[0] : m_3[0]));
      adc_mean[1] = (m_1[1] < m_2[1]) ? ((m_2[1] < m_3[1]) ? m_2[1] : ((m_3[1] < m_1[1]) ? m_1[1] : m_3[1])) : ((m_1[1] < m_3[1]) ? m_1[1] : ((m_3[1] < m_2[1]) ? m_2[1] : m_3[1]));
      adc_mean[2] = (m_1[2] < m_2[2]) ? ((m_2[2] < m_3[2]) ? m_2[2] : ((m_3[2] < m_1[2]) ? m_1[2] : m_3[2])) : ((m_1[2] < m_3[2]) ? m_1[2] : ((m_3[2] < m_2[2]) ? m_2[2] : m_3[2]));
      adc_mean[3] = (m_1[3] < m_2[3]) ? ((m_2[3] < m_3[3]) ? m_2[3] : ((m_3[3] < m_1[3]) ? m_1[3] : m_3[3])) : ((m_1[3] < m_3[3]) ? m_1[3] : ((m_3[3] < m_2[3]) ? m_2[3] : m_3[3]));
      adc_mean[4] = (m_1[4] < m_2[4]) ? ((m_2[4] < m_3[4]) ? m_2[4] : ((m_3[4] < m_1[4]) ? m_1[4] : m_3[4])) : ((m_1[4] < m_3[4]) ? m_1[4] : ((m_3[4] < m_2[4]) ? m_2[4] : m_3[4]));
      adc_mean[5] = (m_1[5] < m_2[5]) ? ((m_2[5] < m_3[5]) ? m_2[5] : ((m_3[5] < m_1[5]) ? m_1[5] : m_3[5])) : ((m_1[5] < m_3[5]) ? m_1[5] : ((m_3[5] < m_2[5]) ? m_2[5] : m_3[5]));

      /* Вычисление квадрата мгновенного значения ацп */
      adc_sqrt[0] = (adc_mean[0] - ZERO_AXIS) * (adc_mean[0] - ZERO_AXIS);
      adc_sqrt[1] = (adc_mean[1] - ZERO_AXIS) * (adc_mean[1] - ZERO_AXIS);
      adc_sqrt[2] = (adc_mean[2] - ZERO_AXIS) * (adc_mean[2] - ZERO_AXIS);

      adc_sqrt[3] = (adc_mean[3] - (ZERO_AXIS + 4)) * (adc_mean[3] - (ZERO_AXIS + 4));
      adc_sqrt[4] = (adc_mean[4] - (ZERO_AXIS + 7)) * (adc_mean[4] - (ZERO_AXIS + 7));
      adc_sqrt[5] = (adc_mean[5] - (ZERO_AXIS - 5)) * (adc_mean[5] - (ZERO_AXIS - 5));

      /* Вычисление квадрата разности мгновенных значения ацп */
      adc_diff_sqrt[0] = (adc_mean[0] - adc_mean[1]) * (adc[0] - adc_mean[1]);
      adc_diff_sqrt[1] = (adc_mean[1] - adc_mean[2]) * (adc[1] - adc_mean[2]);
      adc_diff_sqrt[2] = (adc_mean[2] - adc_mean[0]) * (adc[2] - adc_mean[0]);

      /* Обработка вычисленных значений */
      adc_processing (0);
      adc_processing (1);
      adc_processing (2);
      adc_processing (3);
      adc_processing (4);
      adc_processing (5);
      break;
    }

  /* Запуск нового измерения */
  HAL_ADC_Start_DMA (hadc, (uint32_t*) adc, 6);
}


void
HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
    {
      if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
	  per = HAL_TIM_ReadCapturedValue (&htim3, TIM_CHANNEL_1) + (65001 * count_overflow);
	  count_overflow = 0;
	  TIM3->CNT = 0;
	}
    }
}


/*
void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
    {
      if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
	  switch (flag_fr)
	    {
	    case 0:
	      flag_fr = 1;
	      saved_per[0] = HAL_TIM_ReadCapturedValue (&htim3, TIM_CHANNEL_1) + (65001 * count_overflow);
	      count_overflow = 0;
	      break;

	    case 1:
	      flag_fr = 2;
	      saved_per[1] = HAL_TIM_ReadCapturedValue (&htim3, TIM_CHANNEL_1) + (65001 * count_overflow);
	      count_overflow = 0;
	      break;

	    case 2:
	      flag_fr = 0;
	      saved_per[2] = HAL_TIM_ReadCapturedValue (&htim3, TIM_CHANNEL_1) + (65001 * count_overflow);
	      count_overflow = 0;

	      freq = 48000000.0 / ((saved_per[0] < saved_per[1]) ? ((saved_per[1] < saved_per[2]) ? saved_per[1] : ((saved_per[2] < saved_per[0]) ? saved_per[0] : saved_per[2])) : ((saved_per[0] < saved_per[2]) ? saved_per[0] : ((saved_per[2] < saved_per[1]) ? saved_per[1] : saved_per[2])));
	      break;
	    }
	  TIM3->CNT = 0;
	}
    }
}
*/


void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
    {
      ++count_overflow;
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
