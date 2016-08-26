/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "user_funcs.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch2;
DMA_HandleTypeDef hdma_tim2_ch3;
DMA_HandleTypeDef hdma_tim2_ch4;

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM21_Init(void);
void conv_init(batpins batx);

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
  MX_TIM2_Init();
  MX_TIM21_Init();

  batpins battery3;
  batpins battery4;
  pwm_timers b3_tims;
  pwm_timers b4_tims;
  batprops props_bat3;
  batprops props_bat4;

  /* Battery 3 */
  b3_tims.conv_timer = htim2;
  b3_tims.dchg_timer = htim21;

  battery3.v_adc_chan = ADC_CHANNEL_4;
  battery3.i_adc_chan = ADC_CHANNEL_8;
  battery3.chg_port = chg_onoff_3_GPIO_Port;
  battery3.chg_pin = chg_onoff_3_Pin;
  battery3.dchg_pin = TIM_CHANNEL_1;
  battery3.conv_chg_pin = TIM_CHANNEL_1;
  battery3.conv_dchg_pin = TIM_CHANNEL_2;
  battery3.pwm_tims = b3_tims;

  props_bat3.i_adc_val = 0;
  props_bat3.v_adc_val = 0;
  props_bat3.adc_val_old = adc_read(battery3.i_adc_chan);
  props_bat3.id_adc_stpt = 400 + props_bat3.adc_val_old;
  props_bat3.ic_adc_stpt = props_bat3.adc_val_old - 200;
  props_bat3.conv_bst_stpt = 200; // Need to calibrate this to boost to desired voltage
  props_bat3.pwm_chg_stpt = 0; 	  // Initialized to 0. Program will change as needed.
  props_bat3.pwm_dchg_stpt = 720; // Initialize near where discharge FET turns on
  props_bat3.pi = 0;

  /* Battery 4 */
  b4_tims.conv_timer = htim2;
  b4_tims.dchg_timer = htim21;

  battery4.v_adc_chan = ADC_CHANNEL_11;
  battery4.i_adc_chan = ADC_CHANNEL_10;
  battery4.chg_port = chg_onoff_4_GPIO_Port;
  battery4.chg_pin = chg_onoff_4_Pin;
  battery4.dchg_pin = TIM_CHANNEL_2;
  battery4.conv_chg_pin = B4_CHG_CHAN; // Change in h file (used in multiple locations, dma_offset func)
  battery4.conv_dchg_pin = TIM_CHANNEL_4;
  battery4.pwm_tims = b4_tims;

  props_bat4.i_adc_val = 0;
  props_bat4.v_adc_val = 0;
  props_bat4.adc_val_old = adc_read(battery4.i_adc_chan);
  props_bat4.id_adc_stpt = 500 + props_bat4.adc_val_old;
  props_bat4.ic_adc_stpt = props_bat4.adc_val_old - 200;
  props_bat4.conv_bst_stpt = 200; // Need to calibrate this to boost to desired voltage
  props_bat4.pwm_chg_stpt = 0; 	  // Initialized to 0. Program will change as needed.
  props_bat4.pwm_dchg_stpt = 720; // Initialize near where discharge FET turns on
  props_bat4.pi = 0;

  /* Initialize global variables */
#ifdef BAT1
  TimeCounter3 = 0;
  TimeCounter4 = 0;
  uint32_t restStartms3 = 0;
  uint32_t i3 = 0;
  uint32_t voltage3 = 0;
  uint32_t current3 = 720;
  status bat_stat3 = OK;
#endif

#ifdef BAT2
  uint32_t restStartms4 = 0;
  uint32_t i4 = 0;
  uint32_t voltage4 = 0;
  uint32_t current4 = 720;
  status bat_stat4 = OK;
#endif

  //uint32_t dc_pwm[10] = {100, 500, 200, 300, 400, 500, 600, 700, 250, 750};
  //uint32_t test2[2] = {100, 900};
  //uint32_t sine = 0;

  /* Initialize converter and charge / discharge pins   */
  conv_init(battery3);
  conv_init(battery4);

  //HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, &dc_pwm, (uint16_t)2);
  //HAL_Delay(10);
  //HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_4, &dc_pwm[8], (uint16_t)2);
  //HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, 200, (uint16_t)SINE_RES_500HZ);
  //HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_4, 800, (uint16_t)SINE_RES_500HZ);
  //pwm_sine_Start(battery3.pwm_tims.conv_timer, battery3.conv_dchg_pin, dc_pwm, sine); // Boost (discharge)
  //pwm_sine_Start(battery3.pwm_tims.conv_timer, battery3.conv_chg_pin,  dc_pwm, sine); // Buck (charge)
  //pwm_sine_Start(battery4.pwm_tims.conv_timer, battery4.conv_dchg_pin, test2, sine); // Boost (discharge)
  //pwm_sine_Start(battery4.pwm_tims.conv_timer, battery4.conv_chg_pin, 400, sine); // Buck (charge)
  //pwm_Set(battery3.pwm_tims.dchg_timer, battery3.dchg_pin, 750);
  //pwm_Set(battery4.pwm_tims.dchg_timer, battery4.dchg_pin, 760);
  //pwm_sine_Start(battery3.pwm_tims.conv_timer, battery3.conv_chg_pin, dc_pwm, sine); // Buck (charge)
  //HAL_GPIO_WritePin(battery3.chg_port, battery3.chg_pin, GPIO_PIN_SET); // Charging On
  //HAL_GPIO_WritePin(battery4.chg_port, battery4.chg_pin, GPIO_PIN_SET); // Charging On
  //pwm_sine_Start(battery4.pwm_tims.conv_timer, battery4.conv_chg_pin, dc_pwm, sine); // Buck (charge)
  //HAL_TIM_PWM_Start(battery3.pwm_tims.conv_timer, battery3.conv_dchg_pin);
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  uint8_t u8_oc3 = 0;
  uint8_t u8_oc4 = 0;

  // Wait for batteries to be connected
  while(adc_read(battery3.v_adc_chan) < 500 || adc_read(battery4.v_adc_chan) < 500) {}

  /* Infinite loop */
  while (1)
  {

#ifdef BAT1
	  /* First battery */
	  if(TimeCounter3>=5) // 4ms, ie 2 periods of 500Hz sine wave
	  	  {
	  		  switch(bat_stat3) {
	  		  case DISCHARGE:
	  			  bat_stat3 = discharge_main(battery3, &props_bat3, &restStartms3, i3, bat_stat3);
	  			  break;
	  		  case CC:
	  			  bat_stat3 = chg_ctrl(battery3, &props_bat3, i3);
	  			  //HAL_Delay(1);
	  			  break;
	  		  case CV:
	  			  bat_stat3 = cv_main(battery3, &props_bat3, &restStartms3, i3, bat_stat3);
	  			  break;
	  		  case FULL:
	  			  if(HAL_GetTick() - restStartms3 >= REST)
	  			  {
						props_bat3.i_adc_val = 0;
						props_bat3.v_adc_val = 0;
						props_bat3.adc_val_old = adc_read(battery3.i_adc_chan);
						bat_stat3 = DISCHARGE;
	  			  }
	  			  break;
	  		  case LVDC:
	  			if(HAL_GetTick() - restStartms3 >= REST)
	  			  {
					  props_bat3.i_adc_val = 0;
					  props_bat3.v_adc_val = 0;
					  props_bat3.adc_val_old = adc_read(battery3.i_adc_chan);
					  bat_stat3 = CC;
	  			  }
	  			  break;
	  		  case OK:
	  			  props_bat3.i_adc_val = 0; // normally reset in d/chg func, but not used so reset here
	  			  props_bat3.v_adc_val = 0; // normally reset in d/chg func, but not used so reset here
	  			  bat_stat3 = CC;
	  			  break;
	  		  case OVERCURRENT:
	  			  bat_stat3 = OVERCURRENT;
	  			  break;
	  		  default:
	  			  bat_stat3 = OK;
	  			  break;
	  		  }
	  		  TimeCounter3 = 0;
	  		  i3 = 0;
	  	  }

  	  /* Update ADC readings */
  	  current3 = adc_read(battery3.i_adc_chan);
  	  voltage3 = adc_read(battery3.v_adc_chan);
  	  props_bat3.i_adc_val = props_bat3.i_adc_val + current3;
  	  props_bat3.v_adc_val = props_bat3.v_adc_val + voltage3;

  	  /* Over-current protection */
  	  if(current3>3950 || current3<100)
  	  {
  		  u8_oc3++;
  		  if(u8_oc3 > 15)
  		  {
  			conv_init(battery3);
  			bat_stat3 = OVERCURRENT;
  		  }
  	  }
  	  else
  	  {
  		  u8_oc3 = 0;
  	  }
  	  i3++;
#endif

#ifdef BAT2
	  /* Second battery */
	  if(TimeCounter4>=5) // 4ms, ie 2 periods of 500Hz sine wave
		  {
			  switch(bat_stat4) {
			  case DISCHARGE:
				  bat_stat4 = discharge_main(battery4, &props_bat4, &restStartms4, i4, bat_stat4);
				  break;
			  case CC:
				  bat_stat4 = chg_ctrl(battery4, &props_bat4, i4);
				  break;
			  case CV:
				  bat_stat4 = cv_main(battery4, &props_bat4, &restStartms4, i4, bat_stat4);
				  break;
			  case FULL:
				  if(HAL_GetTick() - restStartms4 >= REST)
				  {
						props_bat4.i_adc_val = 0;
						props_bat4.v_adc_val = 0;
						props_bat4.adc_val_old = adc_read(battery4.i_adc_chan);
						bat_stat4 = DISCHARGE;
				  }
				  break;
			  case LVDC:
				if(HAL_GetTick() - restStartms4 >= REST)
				  {
					  props_bat4.i_adc_val = 0;
					  props_bat4.v_adc_val = 0;
					  props_bat4.adc_val_old = adc_read(battery4.i_adc_chan);
					  bat_stat4 = CC;
				  }
				  break;
			  case OK:
				  props_bat4.i_adc_val = 0; // normally reset in d/chg func, but not used so reset here
				  props_bat4.v_adc_val = 0; // normally reset in d/chg func, but not used so reset here
				  bat_stat4 = DISCHARGE;
				  break;
			  case OVERCURRENT:
				  bat_stat4 = OVERCURRENT;
				  break;
			  default:
				  bat_stat4 = OK;
				  break;
			  }
			  TimeCounter4 = 0;
			  i4 = 0;
		  }

	  /* Update ADC readings */
  	  current4 = adc_read(battery4.i_adc_chan);
  	  voltage4 = adc_read(battery4.v_adc_chan);
  	  props_bat4.i_adc_val = props_bat4.i_adc_val + current4;
  	  props_bat4.v_adc_val = props_bat4.v_adc_val + voltage4;

  	  /* Over-current protection */
  	  if(current4>3950 || current4<100)
  	  {
  		  u8_oc4++;
  		  if(u8_oc4 > 15)
  		  {
  			conv_init(battery4);
  			bat_stat4 = OVERCURRENT;
  		  }
  	  }
  	  else
  	  {
  		  u8_oc4 = 0;
  	  }
  	  i4++;

#endif

	  HAL_SYSTICK_IRQHandler();

  }


}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

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
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION12b;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIG_EDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc.Init.Overrun = OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  HAL_ADC_Init(&hadc);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  //sConfig.Channel = ADC_CHANNEL_11;
  //sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  //HAL_ADC_ConfigChannel(&hadc, &sConfig);
  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM_PERIOD;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);

}

/* TIM21 init function */
void MX_TIM21_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 0;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = TIM_PERIOD;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim21);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  HAL_TIM_PWM_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_2);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : chg_onoff_3_Pin chg_onoff_4_Pin */
  GPIO_InitStruct.Pin = chg_onoff_3_Pin|chg_onoff_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* conv_boot
 * Initialize all converter pins to off
 */
void conv_init(batpins batx)
{
	/* First battery */
	HAL_GPIO_WritePin(batx.chg_port, batx.chg_pin, GPIO_PIN_RESET); // Charging off
	pwm_Set(batx.pwm_tims.dchg_timer, batx.dchg_pin, 0);
	HAL_TIM_PWM_Stop(&batx.pwm_tims.dchg_timer, batx.dchg_pin); // Discharge off
	HAL_TIM_PWM_Stop_DMA(&batx.pwm_tims.conv_timer, batx.conv_chg_pin); // Conv chg off
	HAL_TIM_PWM_Stop_DMA(&batx.pwm_tims.conv_timer, batx.conv_dchg_pin); // Conv dchg off
}

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
