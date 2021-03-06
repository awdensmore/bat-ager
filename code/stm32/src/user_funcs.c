#include "user_funcs.h"

/* Global variables */


void HAL_SYSTICK_IRQHandler(void)
{
  TimeCounter++;
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
		hstat = HAL_ADC_PollForConversion(&hadc, 100);
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
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
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
 */
void pwm_sine_Start(TIM_HandleTypeDef htimx, uint32_t tim_channel, uint32_t u32_dc_duty_cycle, uint16_t u16_ampl)
{
	//u32_sine_duty_cycle = {0};
	int32_t i32_pwm_ampl[SINE_RES_500HZ] = {0}; // amplitude of the sine wave expressed as a % of the pwm period
	//uint32_t u32_pwm_duty_cycle[SINE_RESOLUTION] = {0}; // duty cycle expressed as [0 - htimx.Init.Period]
	int32_t dc_check[SINE_RES_500HZ] = {0};
	int32_t i32_max_period = (int32_t)htimx.Init.Period;

	for(uint8_t i=0; i<(uint8_t)SINE_RES_500HZ; i++)
	{
		i32_pwm_ampl[i] = ((int32_t)i16_sine500hz_lookup[i] * (int32_t)u16_ampl * i32_max_period) / (1000 * 1000);
		dc_check[i] = (int32_t)u32_dc_duty_cycle + i32_pwm_ampl[i];
		u32_sine_duty_cycle[i] = (uint32_t)max(min(i32_max_period, dc_check[i]), 0);
	}

	HAL_TIM_PWM_Start_DMA(&htimx, tim_channel, u32_sine_duty_cycle, (uint16_t)SINE_RES_500HZ);
	HAL_Delay(10);

}

/* pi_ctrl
 * Description: Updates the PWM setting based on ADC reading
 * Inputs:  u32_stpt - the ADC set point to control to [0 - 4095]
 * 			int32_t pwm_val - previously set PWM value [0 - 1000]
 * 		    u32_adc_chan - the ADC channel to read from (ADC_CHANNEL_1, etc)
 * 		    status - battery status. Can be CC, CV or DISCHARGE
 * Outputs: pwm_val - next PWM value
 */
uint32_t pi_ctrl(uint32_t u32_stpt, uint32_t pwm_val, uint32_t u32_adc_val, \
		uint32_t u32_adc_val_old, status mode)
{
   int32_t p = 0;
   //int32_t i = 0; unused - probably can remove
   int32_t diff = 0;
   int32_t diff_old = 0;
   uint32_t sign = 0;
   uint32_t sign_old = 0;
   uint32_t err = 10;
   uint32_t mask = 1<<31;
   int32_t pwm_val_new = (int32_t)pwm_val;
   int32_t p_gain = 0;
   int32_t i_gain = 0;

   /* Select PI gains based on charging or discharging modes */
   if (mode == DISCHARGE)
   {
	   p_gain = 30;
	   i_gain = 2;
   }
   else
   {
	   p_gain = 30;
	   i_gain = 6;
   }

   diff = (int32_t)u32_adc_val - (int32_t)u32_stpt;
   diff_old = (int32_t)u32_adc_val_old - (int32_t)u32_stpt;
   sign = (diff & mask)>>31;
   sign_old = (diff_old & mask)>>31;

   /* Set pi_j (integral gain) to 0 if set point has been crossed between readings */
   if(sign != sign_old)
   {
	   pi_j = 0;
   }

   /* determine if ADC reading is outside allowable error from set point */
   if((uint32_t)abs(diff) > err)
   {
	   if(sign) // ADC reading is below set point
	   {
		   p = -(diff / p_gain);
		   pi_j++;
		   pwm_val_new += min((p+pi_j*(1+i_gain*(p/2))), 30); // slow increase. 20
	   }
	   else // ADC reading is above set point
	   {
		   p = -(diff / p_gain);
		   pi_j++;
		   pwm_val_new += max((p-pi_j*(1-i_gain*(p/2))), -50); // fast decrease for safety. 20
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
   return (uint32_t)pwm_val_new;
}

status dchg_ctrl(batpins batteryx, batprops *batpropsx, uint32_t counter)
{
	status bat_stat = DISCHARGE;

	batpropsx->i_adc_val = batpropsx->i_adc_val / counter; // Average current reading
	batpropsx->v_adc_val = batpropsx->v_adc_val / counter; // Average voltage reading

	/* Determine appropriate pwm value for the discharge FET */
	batpropsx->pwm_dchg_stpt = max(720,pi_ctrl(batpropsx->id_adc_stpt, batpropsx->pwm_dchg_stpt,\
			batpropsx->i_adc_val, batpropsx->adc_val_old, bat_stat));

	/* Check for low voltage disconnect */
	if(batpropsx->v_adc_val < (uint32_t)LVDC_ADC_VAL)
	{
		batpropsx->pwm_dchg_stpt = 0;
		HAL_TIM_PWM_Stop_DMA(&batteryx.pwm_tims.conv_timer, batteryx.conv_dchg_pin); // once DMA is used, only DMA can be used (at least this is all that works)
		bat_stat = LVDC;
	}

	/* Set the discharge FET */
	pwm_Set(batteryx.pwm_tims.dchg_timer, batteryx.dchg_pin, batpropsx->pwm_dchg_stpt);

	/* Re-set ADC readings and counter */
	batpropsx->adc_val_old = batpropsx->i_adc_val;
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
status chg_ctrl(batpins batteryx, batprops *batpropsx, uint32_t counter)
{
	uint32_t u32_adc_val = 0; // this will be set to i or v depending on mode (CC/CV)
	uint32_t u32_adc_stpt = 0; // this will be set to i or v depending on mode (CC/CV)
	status bat_stat = OK;

	batpropsx->i_adc_val = batpropsx->i_adc_val / counter; // Average current reading
	batpropsx->v_adc_val = batpropsx->v_adc_val / counter; // Average voltage reading

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
		u32_adc_val = batpropsx->ic_adc_stpt;
		u32_adc_stpt = batpropsx->i_adc_val;
	}

	/* Determine appropriate pwm value for the charge FET on DC-DC converter */

	batpropsx->pwm_chg_stpt = max(1050, pi_ctrl(u32_adc_stpt, batpropsx->pwm_chg_stpt,\
				u32_adc_val, batpropsx->adc_val_old, bat_stat));

	/* Check for full battery, else set converter PWM */
	if(bat_stat == CV && batpropsx->i_adc_val >= (uint32_t)FULL_ADC_VAL)
	{
		batpropsx->pwm_chg_stpt = 0;
		HAL_GPIO_WritePin(batteryx.chg_port, batteryx.chg_pin, GPIO_PIN_RESET); // Charging off
		HAL_TIM_PWM_Stop_DMA(&batteryx.pwm_tims.conv_timer, batteryx.conv_chg_pin); // Turn off converter
		bat_stat = FULL;
	}
	else
	{
		pwm_sine_Start(batteryx.pwm_tims.conv_timer, batteryx.conv_chg_pin, \
				batpropsx->pwm_chg_stpt, (uint16_t)SINE);
		HAL_GPIO_WritePin(batteryx.chg_port, batteryx.chg_pin, GPIO_PIN_SET); // Charging on
	}

	/* Re-set ADC readings and counter */
	batpropsx->adc_val_old = u32_adc_val;
	batpropsx->i_adc_val = 0;
	batpropsx->v_adc_val = 0;

	return bat_stat;
}
