/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- Servo pulse width (microseconds) ----------------------------------
 * TIM2 clock  = 64 MHz (APB1 timer clock, APB1 divider=/2 nen nhan x2)
 * Prescaler   = 63     -> counter clock = 1 MHz, 1 tick = 1 us
 * Period      = 19999  -> PWM freq = 50 Hz, phu hop SG90
 *
 * SG90:  0 do = 0.5 ms =  500 us
 *       90 do = 1.5 ms = 1500 us
 *      180 do = 2.5 ms = 2500 us
 *       30 do = 0.833 ms = 833 us  <- vi tri nghi / diem bat dau
 * ----------------------------------------------------------------------- */
#define SERVO_MIN_US      833U     /* ~30 do - vi tri nghi / diem bat dau  */
#define SERVO_MAX_US      2500U    /* ~180 do - diem ket thuc              */
#define SERVO_UPDATE_MS   20U      /* cap nhat servo dung chu ky PWM 20 ms */

/* ---- Nguong phan loai mua (ADC 12-bit: 0-4095) --------------------------
 * MH-RD AO thuong: kho = gia tri cao, uot = gia tri thap.
 * Nen do thuc te va hieu chinh cac nguong nay khi lap mach that.
 * ----------------------------------------------------------------------- */
#define RAIN_NONE_THRESH      3500U   /* khong mua: ADC >= 3500             */
#define RAIN_LIGHT_THRESH     2500U   /* mua nhe  : 2500 <= ADC < 3500      */
#define RAIN_MEDIUM_THRESH    1500U   /* mua vua  : 1500 <= ADC < 2500      */
                                         /* mua to   : ADC < 1500             */
#define ADC_SAMPLE_COUNT      8U      /* so mau ADC de lay trung binh       */

/* ---- Toc do gat (buoc dich chuyen moi 20 ms) ----------------------------
 * Cap nhat servo theo chu ky 20 ms de giam giat va tranh soc dong.
 * ----------------------------------------------------------------------- */
#define STEP_NONE      10U
#define STEP_LIGHT     40U
#define STEP_MEDIUM    80U
#define STEP_HEAVY     120U

/* ---- Pin aliases -------------------------------------------------------- */
#define RAIN_DO_PIN    GPIO_PIN_2   /* PA2 - MH-RD DO  */
#define RAIN_DO_PORT   GPIOA
#define BTN_PIN        GPIO_PIN_0   /* PB0 - Nut bam   */
#define BTN_PORT       GPIOB

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile bool     wiper_enabled = false;   /* trang thai bat/tat           */
volatile bool     btn_event = false;       /* co yeu cau toggle tu nut bam */
volatile uint32_t last_btn_tick = 0;       /* timestamp lan nhan cuoi (ms) */

static int32_t servo_pos = (int32_t)SERVO_MIN_US;
static bool    servo_dir = false;  /* false = tien ve MAX, true = lui ve MIN */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static uint16_t ADC_Read(void);
static uint8_t  Rain_GetLevel(void);
static uint16_t Wiper_GetStep(uint8_t rain_level);
static void     Wiper_ProcessButton(void);
static void     Servo_SetPos(int32_t pulse_us);
static void     Wiper_Park(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Dat vi tri servo (co gioi han an toan)
 */
static void Servo_SetPos(int32_t pulse_us)
{
  if (pulse_us < (int32_t)SERVO_MIN_US) pulse_us = (int32_t)SERVO_MIN_US;
  if (pulse_us > (int32_t)SERVO_MAX_US) pulse_us = (int32_t)SERVO_MAX_US;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)pulse_us);
}

/**
 * @brief  Dua servo ve vi tri nghi (30 do) va reset trang thai
 */
static void Wiper_Park(void)
{
  Servo_SetPos((int32_t)SERVO_MIN_US);
  servo_pos = (int32_t)SERVO_MIN_US;
  servo_dir = false;
}

/**
 * @brief  Doc gia tri ADC tu PA1 (MH-RD AO)
 * @return Gia tri ADC 12-bit (0 - 4095)
 */
static uint16_t ADC_Read(void)
{
  uint32_t sum = 0;

  for (uint8_t i = 0; i < ADC_SAMPLE_COUNT; i++)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    sum += HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
  }

  return (uint16_t)(sum / ADC_SAMPLE_COUNT);
}

/**
 * @brief  Xac dinh cap do mua tu gia tri AO
 * @return 3 = mua to, 2 = mua vua, 1 = mua nhe, 0 = khong mua
 */
static uint8_t Rain_GetLevel(void)
{
  uint16_t adc = ADC_Read();
  if (adc >= RAIN_NONE_THRESH)   return 0;
  if (adc >= RAIN_LIGHT_THRESH)  return 1;
  if (adc >= RAIN_MEDIUM_THRESH) return 2;
  return 3;
}

/**
 * @brief  Doi cap do mua thanh buoc quet servo moi 20 ms
 */
static uint16_t Wiper_GetStep(uint8_t rain_level)
{
  switch (rain_level)
  {
    case 3:  return STEP_HEAVY;
    case 2:  return STEP_MEDIUM;
    case 1:  return STEP_LIGHT;
    default: return STEP_NONE;
  }
}

/**
 * @brief  Xu ly su kien nut bam o vong lap chinh de tranh bounce trong ISR
 */
static void Wiper_ProcessButton(void)
{
  if (btn_event)
  {
    btn_event = false;
    wiper_enabled = !wiper_enabled;

    if (!wiper_enabled)
    {
      Wiper_Park();
    }
  }
}
/**
 * @brief  EXTI Callback - xu ly nut bam bat/tat voi debounce 200 ms
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BTN_PIN)
  {
    uint32_t now = HAL_GetTick();
    if ((now - last_btn_tick) > 200U)
    {
      btn_event = true;
      last_btn_tick = now;
    }
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
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  Wiper_Park();      /* servo ve vi tri nghi khi khoi dong */
  HAL_Delay(300);    /* cho servo on dinh                  */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Wiper_ProcessButton();
    /* He thong tat -> ve vi tri nghi */
    if (!wiper_enabled)
    {
      Wiper_Park();
      HAL_Delay(20);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      continue;
    }

    /* He thong da bat: AO quyet dinh 4 muc toc do, DO chi doc de tham khao */
    (void)HAL_GPIO_ReadPin(RAIN_DO_PORT, RAIN_DO_PIN);

    uint16_t servo_step = Wiper_GetStep(Rain_GetLevel());

    /* Di chuyen servo theo tung buoc on dinh moi 20 ms */
    if (!servo_dir)
    {
      servo_pos += (int32_t)servo_step;
      if (servo_pos >= (int32_t)SERVO_MAX_US)
      {
        servo_pos = (int32_t)SERVO_MAX_US;
        servo_dir = true;
      }
    }
    else
    {
      servo_pos -= (int32_t)servo_step;
      if (servo_pos <= (int32_t)SERVO_MIN_US)
      {
        servo_pos = (int32_t)SERVO_MIN_US;
        servo_dir = false;
      }
    }

    Servo_SetPos(servo_pos);
    HAL_Delay(SERVO_UPDATE_MS);
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);          /* hieu chinh ADC */
  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PA2 - MH-RD DO: input pull-up (LOW = dang mua) */
  GPIO_InitStruct.Pin  = RAIN_DO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RAIN_DO_PORT, &GPIO_InitStruct);

  /* PB0 - Nut bam: EXTI canh xuong, pull-up (active LOW) */
  GPIO_InitStruct.Pin  = BTN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_PORT, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
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
