/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "user_funcs.h"

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim1_ch2;
DMA_HandleTypeDef hdma_tim1_ch3_up;
DMA_HandleTypeDef hdma_tim1_ch4_trig_com;

UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART2_UART_Init(void);

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART2_UART_Init();

  /* Define battery pins */
  pwm_timers b1_tims;
  b1_tims.conv_timer = htim1;
  b1_tims.dchg_timer = htim3;

  batpins battery1;
  battery1.v_adc_chan = ADC_CHANNEL_0;
  battery1.i_adc_chan = ADC_CHANNEL_1;
  battery1.chg_port = chg_onoff_1_GPIO_Port;
  battery1.chg_pin = chg_onoff_1_Pin;
  battery1.dchg_pin = TIM_CHANNEL_3;
  battery1.conv_chg_pin = TIM_CHANNEL_1;
  battery1.conv_dchg_pin = TIM_CHANNEL_2;
  battery1.pwm_tims = b1_tims;

  pwm_timers b2_tims;
  b2_tims.conv_timer = htim1;
  b2_tims.dchg_timer = htim3;

  batpins battery2;
  battery2.v_adc_chan = ADC_CHANNEL_13;
  battery2.i_adc_chan = ADC_CHANNEL_6;
  battery2.chg_port = chg_onoff_2_GPIO_Port;
  battery2.chg_pin = chg_onoff_2_Pin;
  battery2.dchg_pin = TIM_CHANNEL_4;
  battery2.conv_chg_pin = TIM_CHANNEL_3;
  battery2.conv_dchg_pin = TIM_CHANNEL_4;
  battery2.pwm_tims = b2_tims;

  /* Initialize battery properties: set points and initial ADC readings */
  /* Battery 1 */
  batprops props_bat1;
  props_bat1.i_adc_val = 0;
  props_bat1.v_adc_val = 0;
  props_bat1.adc_val_old = adc_read(battery1.i_adc_chan);
  props_bat1.id_adc_stpt = 600 + props_bat1.adc_val_old;
  props_bat1.ic_adc_stpt = props_bat1.adc_val_old - 750;
  //props_bat1.v_adc_stpt = 1000; // Not sure what this should be.
  props_bat1.conv_bst_stpt = 300; // Need to calibrate this to boost to desired voltage
  props_bat1.pwm_chg_stpt = 0; 	  // Initialized to 0. Program will change as needed.
  props_bat1.pwm_dchg_stpt = 720; // Initialize near where discharge FET turns on

  /* Initialize pins to 0 */
  HAL_TIM_PWM_Stop_DMA(&battery1.pwm_tims.conv_timer, battery1.conv_dchg_pin);
  HAL_TIM_PWM_Stop_DMA(&battery1.pwm_tims.conv_timer, battery1.conv_chg_pin);
  HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_4);
  //CONV_DCHG_OFF; // PWM control
  DISCHARGE_OFF; // Discharge control
  CHARGING_OFF; // Charging on/off

  /* Initialize Converter output */
  uint32_t dc_pwm = 1200;
  uint32_t sine = 5;
  //pwm_Set(htim1, TIM_CHANNEL_2, 99);
  //CHARGING_OFF;
  //CHARGING_ON;

  //pwm_sine_Start(battery2.pwm_tims.conv_timer, battery2.conv_dchg_pin, dc_pwm, sine); // Boost (discharge)
  //pwm_Set(battery2.pwm_tims.dchg_timer, battery2.dchg_pin, 735);
  //HAL_GPIO_WritePin(battery2.chg_port, battery2.chg_pin, GPIO_PIN_SET); // Charging On
  pwm_sine_Start(battery2.pwm_tims.conv_timer, battery2.conv_chg_pin, dc_pwm, sine); // Buck (charge)
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  /* Initialize global variables */
  TimeCounter = 0;
  pi_j = 0;
  uint32_t i = 0;
  uint32_t voltage = 0;
  uint32_t current = 720;
  status bat_stat = OK;

  while (1)
  {

	  if(TimeCounter>=2*1600) // 4ms, ie 2 periods of 500Hz sine wave
	  {
		  switch(bat_stat) {
		  case DISCHARGE:
			  //CHARGING_OFF;
			  //CONV_CHG_OFF;
			  pwm_sine_Start(battery1.pwm_tims.conv_timer, battery1.conv_dchg_pin,\
					  props_bat1.conv_bst_stpt, SINE);
			  bat_stat = dchg_ctrl(battery1, &props_bat1, i);
			  break;
		  case CC:
			  //DISCHARGE_OFF;
			  //CONV_DCHG_OFF;
			  bat_stat = chg_ctrl(battery1, &props_bat1, i);
			  break;
		  case CV:
			  //DISCHARGE_OFF;
			  //CONV_DCHG_OFF;
			  bat_stat = chg_ctrl(battery1, &props_bat1, i);
			  break;
		  case FULL:
			  HAL_Delay(REST);
			  props_bat1.i_adc_val = 0;
			  props_bat1.v_adc_val = 0;
			  props_bat1.adc_val_old = adc_read(battery1.i_adc_chan);
			  bat_stat = DISCHARGE;
			  // implement a timer here so battery rests for ~1hr
			  break;
		  case LVDC:
			  HAL_Delay(REST);
			  props_bat1.i_adc_val = 0;
			  props_bat1.v_adc_val = 0;
			  props_bat1.adc_val_old = adc_read(battery1.i_adc_chan);
			  bat_stat = CC;
		  	  // implement a timer here so battery rests for ~1hr
			  break;
		  case OK:
			  props_bat1.i_adc_val = 0; // normally reset in d/chg func, but not used so reset here
			  props_bat1.v_adc_val = 0; // normally reset in d/chg func, but not used so reset here
			  bat_stat = CC;
			  //props_bat1.pwm_chg_stpt = 1300;
			  // what to do here?
			  break;
		  default:
			  bat_stat = DISCHARGE;
			  break;
		  }
		  TimeCounter = 0;
		  i = 0;
	  }
/*	  if(bat_stat == LVDC)
	  {
		  while(1)
		  {
			  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		      HAL_Delay(200);
		  }
	  }*/
	  current = adc_read(battery1.i_adc_chan);
	  voltage = adc_read(battery1.v_adc_chan);
	  if(current>3950 || current<100)
	  {
		  DISCHARGE_OFF;
		  CHARGING_OFF;
		  while(1){}
	  }
	  props_bat1.i_adc_val = props_bat1.i_adc_val + current;
	  props_bat1.v_adc_val = props_bat1.v_adc_val + voltage;
	  i++;
	  HAL_SYSTICK_IRQHandler();
  }


}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
void MX_ADC_Init(void)
{

  //ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc.Init.Resolution = ADC_RESOLUTION12b;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV; //EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = OVR_DATA_PRESERVED;
  hadc.Init.SamplingTimeCommon = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_Init(&hadc);

  hstat = HAL_ADCEx_Calibration_Start(&hadc);
  HAL_Delay(10);
}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = TIM_PERIOD;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(&htim1);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = TIM_PERIOD;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

}

/* TIM16 init function */
void MX_TIM16_Init(void)
{

  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 0;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim16);

  HAL_TIM_PWM_Init(&htim16);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1);

}

/* TIM17 init function */
void MX_TIM17_Init(void)
{

  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 0;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim17);

  HAL_TIM_PWM_Init(&htim17);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : chg_onoff_1_Pin chg_onoff_2_Pin chg_onoff_3_Pin */
  GPIO_InitStruct.Pin = chg_onoff_1_Pin|chg_onoff_2_Pin|chg_onoff_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
