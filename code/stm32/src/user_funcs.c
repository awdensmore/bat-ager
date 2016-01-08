#include "user_funcs.h"

void pwm_sine_Start(TIM_HandleTypeDef htimx, uint32_t tim_channel, uint32_t u32_dc_duty_cycle, uint8_t u8_ampl)
{
	int32_t i32_pwm_ampl[SINE_RESOLUTION] = {0}; // amplitude of the sine wave expressed as a % of the pwm period
	uint32_t u32_pwm_duty_cycle[SINE_RESOLUTION] = {0}; // duty cycle expressed as [0 - htimx.Init.Period]
	int32_t dc_check[SINE_RESOLUTION] = {0};
	int32_t i32_max_period = (int32_t)htimx.Init.Period;

	for(uint8_t i=0; i<(uint8_t)SINE_RESOLUTION; i++)
	{
		i32_pwm_ampl[i] = ((int32_t)i16_sine_lookup[i] * (int32_t)u8_ampl * i32_max_period) / (1000 * 100);
		dc_check[i] = (int32_t)u32_dc_duty_cycle + i32_pwm_ampl[i];
		u32_pwm_duty_cycle[i] = (uint32_t)max(min(i32_max_period, dc_check[i]), 0);
	}

	HAL_TIM_PWM_Start_DMA(&htimx, tim_channel, u32_pwm_duty_cycle, (uint16_t)SINE_RESOLUTION);
	//HAL_Delay(10);

}

uint32_t adc_read(ADC_HandleTypeDef hadc, uint32_t u32_adc_chan)
{
	ADC_ChannelConfTypeDef sConfig;
	uint32_t u32_adc_result = 0;

	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.Channel = u32_adc_chan;
	hstat = HAL_ADC_ConfigChannel(&hadc, &sConfig);

	hstat = HAL_ADC_Start(&hadc);
	hstat = HAL_ADC_PollForConversion(&hadc, 100);
	u32_adc_result = HAL_ADC_GetValue(&hadc);
	hstat = HAL_ADC_Stop(&hadc);

	sConfig.Rank = ADC_RANK_NONE;
	hstat = HAL_ADC_ConfigChannel(&hadc, &sConfig);

	return u32_adc_result;
}
