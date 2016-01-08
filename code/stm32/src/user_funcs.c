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

//void adc_read
