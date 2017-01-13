#include "user_funcs.h"

/* Global variables */


void HAL_SYSTICK_IRQHandler(void)
{
  TimeCounter3++;
  TimeCounter4++;
  HAL_SYSTICK_Callback();
}

uint32_t adc_read(uint32_t u32_adc_chan)
{
	ADC_ChannelConfTypeDef sConfigADC;
	uint32_t u32_adc_result = 0;
	uint8_t j = 0;
	uint8_t u8_num_conv = 10;

	/* Configure channel */
	sConfigADC.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfigADC.Channel = u32_adc_chan;
	hstat = HAL_ADC_ConfigChannel(&hadc, &sConfigADC);

	/* Perform conversion */
	hstat = HAL_ADC_Start(&hadc);
	while(j<u8_num_conv)
	{
		hstat = HAL_ADC_PollForConversion(&hadc, 10);
		u32_adc_result = u32_adc_result + HAL_ADC_GetValue(&hadc);
		j++;
	}
	u32_adc_result = u32_adc_result / u8_num_conv;
	hstat = HAL_ADC_Stop(&hadc);

	/* Disable channel */
	sConfigADC.Rank = ADC_RANK_NONE;
	hstat = HAL_ADC_ConfigChannel(&hadc, &sConfigADC);

	return u32_adc_result;
}

/* pwm_Start
 * Parameters:
 * 		htimx = timer handle
 * 		tim_channel = timer channel
 * 		u32_dc_duty_cycle = [0-1000] (ie, 0.1% precision) PWM duty cycle
 */
void pwm_Set(TIM_HandleTypeDef htimx, uint32_t tim_channel, uint32_t u32_duty_cycle)
{
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.Pulse = (u32_duty_cycle * TIM_PERIOD) / (uint32_t)1000;
	hstat = HAL_TIM_PWM_ConfigChannel(&htimx, &sConfigOC, tim_channel);
	hstat = HAL_TIM_PWM_Start(&htimx, tim_channel);
}

/* pwm_sine_Start
 * Description: Inject a sinusoidal signal using PWM
 * Inputs: htimx - timer handle of PWM pin (htim1, etc.)
 * 		   tim_channel - channel of PWM pin (TIM_CHANNEL_1, etc.)
 * 		   u32_duty_cycle - the DC duty cycle on which the sine wave will be added. [0 - htimx.Init.Period]
 * 		   NOTE: the duty cycle scale for this function is different from pwm_Start.
 * 		   u16_ampl - the amplitude of the sine wave [0 - htimx.Init.Period]
 * 		   u8_conv_id - offset for the DMA memory for sine wave
 */
void pwm_sine_Start(TIM_HandleTypeDef htimx, uint32_t tim_channel, uint32_t u32_dc_duty_cycle, uint16_t u16_ampl, uint8_t u8_conv_id)
{
	int32_t i32_pwm_ampl[SINE_RES_500HZ] = {0}; // amplitude of the sine wave expressed as a % of the pwm period
	int32_t dc_check[SINE_RES_500HZ] = {0};
	int32_t i32_max_period = (int32_t)htimx.Init.Period;
	uint32_t *p_dma; // pointer to the location of DMA memory

	for(uint8_t i=0; i<(uint8_t)SINE_RES_500HZ; i++)
	{
		i32_pwm_ampl[i] = ((int32_t)i16_sine500hz_lookup[i] * (int32_t)u16_ampl * i32_max_period) / (1000 * 1000);
		dc_check[i] = (int32_t)u32_dc_duty_cycle + i32_pwm_ampl[i];
		u32_sine_duty_cycle[i+u8_conv_id*SINE_RES_500HZ] = (uint32_t)max(min(i32_max_period, dc_check[i]), 0);
	}
	p_dma = &u32_sine_duty_cycle[u8_conv_id*SINE_RES_500HZ];

	HAL_TIM_PWM_Start_DMA(&htimx, tim_channel, p_dma, (uint16_t)SINE_RES_500HZ);
	HAL_Delay(10);

}

/* pi_ctrl
 * Description: Updates the PWM setting based on ADC reading
 * Inputs:  u32_stpt - the ADC set point to control to [0 - 4095]
 * 			int32_t pwm_val - previously set PWM value [0 - 1000]
 * 		    u32_adc_chan - the ADC channel to read from (ADC_CHANNEL_1, etc)
 * 		    pi_j - the pi loop for this battery
 * 		    status - battery status. Can be CC, CV or DISCHARGE
 * Outputs: pwm_val - next PWM value
 */
uint32_t pi_ctrl(uint32_t u32_stpt, uint32_t pwm_val, uint32_t u32_adc_val, \
		int32_t *pij, uint32_t u32_adc_val_old, status mode)
{
   int32_t p = 0;
   int32_t diff = 0;
   int32_t diff_old = 0;
   int32_t d_slope = 0;
   uint32_t sign = 0;
   uint32_t sign_old = 0;
   uint32_t err = 8;
   uint32_t mask = 1<<31;
   int32_t pwm_val_new = (int32_t)pwm_val;
   int32_t p_gain = 0;
   int32_t i_gain = 0;
   int32_t pi_j = *pij;



   diff = (int32_t)u32_adc_val - (int32_t)u32_stpt;
   diff_old = (int32_t)u32_adc_val_old - (int32_t)u32_stpt;
   d_slope = diff - diff_old;
   sign = (diff & mask)>>31; // 1 is negative (2's complement for signed int)
   sign_old = (diff_old & mask)>>31;

   /* Set pi_j (integral gain) to 0 if set point has been crossed between readings */
   if(sign != sign_old)
   {
	   pi_j = 0;
   }

   /* determine if ADC reading is outside allowable error from set point */
   if((uint32_t)abs(diff) > err)
   {
	   /* Select PI gains based on charging or discharging modes */
	  if (mode == DISCHARGE && sign) // ADC value is below stpt: diff<0
	  {
		  /* GAINS 25 Nov 2016 */
		  int32_t g = 10;
		  if(d_slope > 3) {
		  	  g = 20;
		  	  pi_j = 0;
		  }
		  else if(d_slope > 15) { /* d_slope = diff - diff_old */
		  	  g = -1;
		  	  pi_j = 0;
		  }
		  else if(d_slope < -5 && diff < 200) {
		  	  g = 1;
		  	  pi_j = 0;
		  }
		  pi_j++;
		  if (pi_j > abs(g)) {
			  pi_j = 0;
		  }
		  pwm_val_new += (pi_j/g);
	  }
	  else if (mode == DISCHARGE && !sign)
	  {
		  pi_j++;
		  if (pi_j > 2) pi_j = 0;
		  pwm_val_new -= pi_j/2;
	  }
	  else if ((mode == CC || CV) && sign) // ADC value is below stpt: diff<0
	  {
		  int g = 1;
		  if(abs(diff) < 600) {
			  g = 20;
		  }
		  //if(d_slope < -3) {
		//	  g = 20;
		  //}
		  pi_j++;
		  if (pi_j > g) pi_j = 0;
		  pwm_val_new += pi_j/g;//min((p+pi_j*(1+i_gain*(p/2))), 2); // slow increase. 4,1000
		  /*pi_j++;
		  if (pi_j > 2) pi_j = 0;
		  pwm_val_new += pi_j/2; // + abs(diff)/200);*/
	  }
	  else if ((mode == CC || CV) && !sign)
	  {
		  pi_j++;
		  if (pi_j > 30) pi_j = 0;
		  pwm_val_new -= pi_j/30;//max((p-pi_j*(1-i_gain*(p/2))), -4); // fast decrease for safety.30,1000
		  /*pi_j++;
		  if (pi_j > 2) pi_j = 0;
		  pwm_val_new -= pi_j/2;*/
	  }
   }
   else
   {
	   pi_j = 0;
   }
   if(pwm_val_new < 0)
   {
	   pwm_val_new = 0;
   }
   else if(pwm_val_new > (uint32_t)TIM_PERIOD)
   {
	   pwm_val_new = (uint32_t)TIM_PERIOD;
   }
   *pij = pi_j;
   return (uint32_t)pwm_val_new;
}

status dchg_ctrl(batpins batteryx, batprops *batpropsx, uint32_t counter)
{
	status bat_stat = DISCHARGE;
	int32_t pij = batpropsx->pi;

	batpropsx->i_adc_val = batpropsx->i_adc_val / counter; // Average current reading
	batpropsx->v_adc_val = (batpropsx->v_adc_val / counter) + CBCOMP; // Average voltage reading

	/* Determine appropriate pwm value for the discharge FET */
	/* 1Nov2016 - changed max 100->0. LPF on dchg fet is very slow acting. 100 may be too high. */
	batpropsx->pwm_dchg_stpt = max(0,pi_ctrl(batpropsx->id_adc_stpt, batpropsx->pwm_dchg_stpt,\
			batpropsx->i_adc_val, &pij, batpropsx->adc_val_old, bat_stat));

	/* Check for low voltage disconnect; if so increment switch counter
	 * This is done to "de-bounce" the switch. Don't want spurious changes.*/
	if(batpropsx->v_adc_val < (uint32_t)LVDC_ADC_VAL)
	{
		batpropsx->sw_ctr++;
	}
	else
	{
		batpropsx->sw_ctr = 0;
	}
	/* Threshold has been reached, implement slow switch */
	if(batpropsx->sw_ctr >= SWTHR)
	{
		if(batpropsx->pwm_dchg_stpt > 1)
		{
			batpropsx->pwm_dchg_stpt -= 1;
		}
		else
		{
			batpropsx->sw_ctr = 0;
			batpropsx->pwm_dchg_stpt = 0;
			HAL_TIM_PWM_Stop_DMA(&batteryx.pwm_tims.conv_timer, batteryx.conv_dchg_pin); // once DMA is used, only DMA can be used (at least this is all that works)
			bat_stat = LVDC;
		}
	}

	/* Set the discharge FET */
	HAL_GPIO_WritePin(batteryx.chg_port, batteryx.chg_pin, GPIO_PIN_RESET); // Ensure chg off
	pwm_Set(batteryx.pwm_tims.dchg_timer, batteryx.dchg_pin, batpropsx->pwm_dchg_stpt);

	/* Re-set ADC readings and counter */
	batpropsx->adc_val_old = batpropsx->i_adc_val;
	batpropsx->pi = pij;
	batpropsx->i_adc_val = 0;
	batpropsx->v_adc_val = 0;

	return bat_stat;
}

/* chg_ctrl
 * Description: CC/CV charging of the battery. End charging when battery reaches voltage setpoint
 * Inputs:
 * 		batteryx = struct containing information on the battery GPIO pins
 * 		batprops = properties of the battery (including adc readings and set points)
 * Returns:
 * 		Battery status
 */
status chg_ctrl(batpins batteryx, batprops *batpropsx, uint32_t counter, uint32_t i_adc_mdpt)
{
	uint32_t u32_adc_val = 0; // this will be set to i or v depending on mode (CC/CV)
	uint32_t u32_adc_stpt = 0; // this will be set to i or v depending on mode (CC/CV)
	int32_t pij = batpropsx->pi;
	status bat_stat = CV;

	batpropsx->i_adc_val = batpropsx->i_adc_val / counter; // Average current reading
	batpropsx->v_adc_val = (batpropsx->v_adc_val / counter) - CBCOMP; // Average voltage reading

	/* Determine charging mode */
	if(batpropsx->v_adc_val >= (uint32_t)CV_ADC_VAL)
	{
		bat_stat = CV;
		u32_adc_val = batpropsx->v_adc_val;
		u32_adc_stpt = (uint32_t)CV_ADC_VAL+10; // added buffer to prevent bouncing b/w CC & CV
	}
	else
	{
		bat_stat = CC;
		/* In cc chg mode, lower ADC val = greater current. Therefore adc_val & adc_stpt
		 * inputs to pi_ctrl() are reversed. */
		/* Change 20 Apr - Instead of reversing stpts and readings (which is confusing to
		 * debug), transform adc val and setpoint to have same sign and direction as in discharge mode.
		 * Therefore no changes are needed in pi_ctrl()
		 */
		u32_adc_val = i_adc_mdpt + (i_adc_mdpt - batpropsx->i_adc_val);
		u32_adc_stpt = i_adc_mdpt + (i_adc_mdpt - batpropsx->ic_adc_stpt);
	}

	/* Determine appropriate pwm value for the charge FET on DC-DC converter */

	batpropsx->pwm_chg_stpt = max(30, pi_ctrl(u32_adc_stpt, batpropsx->pwm_chg_stpt,\
				u32_adc_val, &pij, batpropsx->adc_val_old, bat_stat));


	/* Check for full battery, else set converter PWM */
	if(bat_stat == CV && batpropsx->i_adc_val >= (uint32_t)FULL_ADC_VAL)
	{
		// If slow stop to be implemented, need to use PWM on charging pin (rather than GPIO).
		batpropsx->sw_ctr++;
		if(batpropsx->sw_ctr >= SWTHR) { // Only declare bat full if condition has been met SWTHR times
			batpropsx->sw_ctr = 0;
			batpropsx->pwm_chg_stpt = 0;
			HAL_GPIO_WritePin(batteryx.chg_port, batteryx.chg_pin, GPIO_PIN_RESET); // Charging off
			HAL_TIM_PWM_Stop_DMA(&batteryx.pwm_tims.conv_timer, batteryx.conv_chg_pin); // Turn off converter
			bat_stat = FULL;
		}

	}
	else
	{
		pwm_sine_Start(batteryx.pwm_tims.conv_timer, batteryx.conv_chg_pin, \
				batpropsx->pwm_chg_stpt, (uint16_t)SINE, dma_offset(batteryx));
		HAL_GPIO_WritePin(batteryx.chg_port, batteryx.chg_pin, GPIO_PIN_SET); // Charging on
	}

	/* Re-set ADC readings and counter */
	batpropsx->adc_val_old = u32_adc_val;
	batpropsx->pi = pij;
	batpropsx->i_adc_val = 0;
	batpropsx->v_adc_val = 0;

	return bat_stat;
}

/* The functions below implement features that previously were explicitly
 * written into the while() loop of main.c
 */

/* discharge_main()
 * Inputs: batprops *batx = properties of the battery to discharge
 * 		   batpins pinsx = pins controlling the battery to discharge
 * 		   *restStartms = if LVDC, start of rest timer
 * 		   loops = number of loops to average ADC readings
 * 		   bat_stat = battery status
 * Output: bat_stat
 */
status discharge_main(batpins pinsx, batprops *batx, uint32_t* restStartms, \
		uint32_t loops, status bat_stat)
{
	pwm_sine_Start(pinsx.pwm_tims.conv_timer, pinsx.conv_dchg_pin, \
			batx->conv_bst_stpt, SINE, dma_offset(pinsx)+1);
	bat_stat = dchg_ctrl(pinsx, batx, loops);
	if(bat_stat == LVDC)
	{
		*restStartms = HAL_GetTick();
	}

	return bat_stat;
}

/* cv_main()
 * Inputs: batpins pinsx
 * 		   batprops batx
 * 		   uint32_t *restStartms
 * 		   status bat_stat
 * Outputs: bat_stat
 */
status cv_main(batpins pinsx, batprops *batx, uint32_t* restStartms, \
		uint32_t loops, uint32_t i_adc_mdpt, status bat_stat)
{
	bat_stat = chg_ctrl(pinsx, batx, loops, i_adc_mdpt);
	if(bat_stat == FULL)
	{
		*restStartms = HAL_GetTick();
	}

	return bat_stat;
}

/* dma_offset
 * Description: To avoid overwriting the same DMA location for the sine wave functions, there is one DMA location that is long
 * 				enough to hold the sine waves for all 4 converters. Therefore when writing a new wave for one converter, an
 * 				offset is required to only change the memory for one converter.
 */
uint8_t dma_offset(batpins pinsx)
{
	uint8_t offset = 0;
	if (pinsx.conv_chg_pin == B4_CHG_CHAN)
	{
		offset = 2; // 0 = B3_CHG_CHAN
	}
	return offset;
}
