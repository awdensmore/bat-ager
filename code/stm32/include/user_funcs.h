/* PIN DESCRIPTIONS (function, user pin designation, STM designation)
 * (10) ADC, (10) PWM (of which, 4 DMA), (3) GPIO
 *
 * ADC
 * ADC_CHANNEL_0  -> ADC_1v_Pin -> PA0
 * ADC_CHANNEL_1  -> ADC_1i_Pin -> PA1
 * ADC_CHANNEL_6  -> ADC_2i_Pin -> PA6
 * ADC_CHANNEL_7  -> ADC_3v_Pin -> PA7
 * ADC_CHANNEL_8  -> ADC_3i_Pin -> PB0
 * ADC_CHANNEL_9  -> ADC_4v_Pin -> PB1
 * ADC_CHANNEL_10 -> ADC_4i_Pin -> PC0
 * ADC_CHANNEL_11 -> ADC_5v_Pin -> PC1
 * ADC_CHANNEL_12 -> ADC_5i_Pin -> PC2
 * ADC_CHANNEL_13 -> ADC_2v_Pin -> PC3
 *
 * PWM
 * TIM1_CH1 -> fet_ctrl_1a_Pin -> PA8  (DMA1_Channel2)
 * TIM1_CH2 -> fet_ctrl_1b_Pin -> PA9  (DMA1_Channel3)
 * TIM1_CH3 -> fet_ctrl_2a_Pin -> PA10 (DMA1_Channel5) ch3_up - might not work?
 * TIM1_CH4 -> fet_ctrl_2b_Pin -> PA11 (DMA1_Channel4) ch4_trig_com - might not work?
 *
 * TIM3_CH1 -> fet_ctrl_3a_Pin -> PC6
 * TIM3_CH2 -> fet_ctrl_3b_Pin -> PC7
 * TIM3_CH3 -> dchg_ctrl_1_Pin -> PC8
 * TIM3_CH4 -> dchg_ctrl_2_Pin -> PC9
 *
 * TIM16_CH1 -> dchg_ctrl_3_Pin -> PB8
 *
 * TIM17_CH1 -> fet_Ctrl_5b_Pin -> PB9
 *
 * GPIO
 * GPIO_PIN10 -> chg_onoff_1_Pin -> C10
 * GPIO_PIN11 -> chg_onoff_2_Pin -> C11
 * GPIO_PIN12 -> chg_onoff_3_Pin -> C12
 */

#include "stm32f0xx_hal.h"

/* Typedefs */
HAL_StatusTypeDef hstat;
ADC_HandleTypeDef hadc;
/*
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim1_ch2;
DMA_HandleTypeDef hdma_tim3_ch1_trig;
*/

/* Global variables */
volatile int32_t pi_j; // integral timer value for PI control loop
volatile int32_t pi_pwm_val;

/* Constants */
static const uint32_t TIM_PERIOD = 1000;

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

static const int16_t i16_sine_lookup[SINE_RESOLUTION] =
{
  0, 103, 203, 293, 371, 433, 475, 497, 497, 475, 733, 371, 293, 203, 103, \
  0, -104, -204, -294, -372, -434, -476, -798, -798, -476, -434, -372, -294, -204, -104};

void pwm_Start(TIM_HandleTypeDef htimx, uint32_t tim_channel, uint32_t u32_duty_cycle);
void pwm_sine_Start(TIM_HandleTypeDef htimx, uint32_t tim_channel, uint32_t u32_dc_duty_cycle, uint8_t u8_ampl);
uint32_t adc_read(uint32_t u32_adc_chan);
void pi_ctrl(uint32_t u32_stpt, uint32_t u32_adc_chan, TIM_HandleTypeDef htimx, uint32_t u32_tim_chan);
