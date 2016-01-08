#include "stm32f0xx_hal.h"

/* Typedefs */
/*
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim1_ch2;
DMA_HandleTypeDef hdma_tim3_ch1_trig;
*/

/* Macros */
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

/* Sine wave of amplitude = 1000
 * Resolution of 30 for 30kHz switching speed */
#define SINE_RESOLUTION 30
//static const uint32_t ADC_CHANNELS[10] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_8, \
		 	 	 	 	 	 	 	 	  ADC_CHANNEL_9, ADC_CHANNEL_10, ADC_CHANNEL_11, ADC_CHANNEL_12, ADC_CHANNEL_13};

static const int16_t i16_sine_lookup[SINE_RESOLUTION] =
{
  0, 103, 203, 293, 371, 433, 475, 497, 497, 475, 733, 371, 293, 203, 103, \
  0, -104, -204, -294, -372, -434, -476, -798, -798, -476, -434, -372, -294, -204, -104};

void pwm_sine_Start(TIM_HandleTypeDef htimx, uint32_t tim_channel, uint32_t u32_dc_duty_cycle, uint8_t u8_ampl);
