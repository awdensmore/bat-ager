/* PIN DESCRIPTIONS (function, user pin designation, STM designation)
 * (4) ADC, (6) PWM (of which, 4 DMA), (2) GPIO
 *
 * ADC
 * ADC_CHANNEL_4  -> ADC_3v_Pin -> PA4
 * ADC_CHANNEL_8  -> ADC_3i_Pin -> PB0
 * ADC_CHANNEL_11 -> ADC_4v_Pin -> PC1
 * ADC_CHANNEL_10 -> ADC_4i_Pin -> PC0
 *
 * PWM
 * TIM2_CH1 -> fet_ctrl_3a_Pin -> PA0  (DMA1_Channel5)
 * TIM2_CH2 -> fet_ctrl_3b_Pin -> PA1  (DMA1_Channel3)
 * TIM2_CH3 -> fet_ctrl_4a_Pin -> PB10 (DMA1_Channel1)
 * TIM2_CH4 -> fet_ctrl_4b_Pin -> PB11 (DMA1_Channel4)
 *
 * TIM21_CH1 -> dchg_ctrl_3_Pin -> PB13
 * TIM21_CH2 -> dchg_ctrl_4_Pin -> PB14
 *
 * GPIO
 * GPIO_PIN_8 -> chg_onoff_3_Pin -> PC8
 * GPIO_PIN_9 -> chg_onoff_4_Pin -> PC9
 */

#include "stm32l0xx_hal.h"

/* HAL Typedefs */
HAL_StatusTypeDef hstat;
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;

/* Defines */
#define BAT1
//#define BAT2
#define SINE_RES_1KHZ  32
#define SINE_RES_500HZ 64
#define LVDC_ADC_VAL 2885 // ADC reading below which disconnect load from battery. Rt=4.7k, Rb=1.2k11.4v
#define CV_ADC_VAL 3028 // THIS IS A GUESS!!! Switch to CV charging when voltage is >= to this.12v
#define FULL_ADC_DIFF 30 // Below this current ADC value, battery is fully charged.
#define I_ADC_MIDPOINT 2005// ADC reading at which current = 0A. 2035 Rig1 (conv2 / sensor3)
#define FULL_ADC_VAL (I_ADC_MIDPOINT - FULL_ADC_DIFF)
#define SINE 6 // % Amplitude of sine wave, scale of [0 - 1000]
#define REST (uint32_t)10*1*1000 // 30 minutes rest between charge/discharge cycles

/* Global variables */
//volatile int32_t pi_j3, pi_j4; // integral timer value for PI control loop
volatile uint32_t u32_sine_duty_cycle[SINE_RES_500HZ];
uint32_t TimeCounter3, TimeCounter4;

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

/* Enumeration */
enum bat_status { OK, DISCHARGE, LVDC, CC, CV, FULL, OVERCURRENT};

/* Structures */
typedef enum bat_status status;// {
typedef struct pwm_timers {
	TIM_HandleTypeDef conv_timer; // timer for the DC-DC converter PWM pins
	TIM_HandleTypeDef dchg_timer; // timer for the discharge PWM pin
}pwm_timers;
typedef struct batpins {
	uint32_t v_adc_chan;    // the ADC channel measuring the battery voltage (ie ADC_CHANNEL_1)
	uint32_t i_adc_chan;    // the ADC channel measuring the battery current
	GPIO_TypeDef* chg_port;		// The GPIO port for the on/off PFET
	uint32_t chg_pin;       // the GPIO for the on/off PFET on the power distribution PCB
	uint32_t dchg_pin;      // the PWM pin for the discharge control pin on the power distribution PCB
	uint32_t conv_chg_pin;  // the PWM pin for the buck FET on the DC-DC converter
	uint32_t conv_dchg_pin; // the PWM pin for the boost FET on the DC-DC converter
	pwm_timers pwm_tims;    // timers for the PWM pins (dchg, conv_chg & conv_dchg)
}batpins;
typedef struct batprops {
	uint32_t ic_adc_stpt; // current set point during charging
	uint32_t id_adc_stpt; // current set point during discharge
	uint32_t conv_bst_stpt; // pwm set point for boost (discharging) FET on DC-DC converter
	uint32_t pwm_chg_stpt; // pwm set point for charging FET on DC-DC converter
	uint32_t pwm_dchg_stpt; // pwm set point for the load FET on distribution board
	uint32_t i_adc_val;
	uint32_t adc_val_old;
	uint32_t v_adc_val;
	int32_t pi; // counter for the integral gain in PI loop
}batprops;



/* Sine wave of amplitude = 1000
 * Resolution of 30 for 30kHz switching speed */


static const int16_t i16_sine1khz_lookup[SINE_RES_1KHZ] =
{
  0, 98, 191, 278, 354, 416, 490, 462, 500, 490, 462, 416, 354, 278, 191, 98, \
  0, -98, -191, -278, -354, -416, -490, -462, -500, -490, -462, -416, -354, -278, -191, -98
};

static const int16_t i16_sine500hz_lookup[SINE_RES_500HZ] =
{
  0, 49, 98, 145, 191, 236, 278, 317, 354, 387, 416, 441, 462, 478, 490, 498, \
  500, 498, 490, 478, 462, 441, 416, 387, 354, 317, 278, 236, 191, 145, 98, 49, \
  0, -49, -98, -145, -191, -236, -278, -317, -354, -387, -416, -441, -462, -478, -490, -498, \
  -500, -498, -490, -478, -462, -441, -416, -387, -354, -317, -278, -236, -191, -145, -98, -49
};

// MODIFIED HAL_GetTick in stm32l0xx.hal.c (b/c HAL_SYSTICK_Config changed from 1000->10000
void HAL_SYSTICK_IRQHandler(void);
void pwm_Set(TIM_HandleTypeDef htimx, uint32_t tim_channel, uint32_t u32_duty_cycle);
void pwm_sine_Start(TIM_HandleTypeDef htimx, uint32_t tim_channel, uint32_t u32_dc_duty_cycle, uint16_t u8_ampl);
uint32_t adc_read(uint32_t u32_adc_chan);
uint32_t pi_ctrl(uint32_t u32_stpt, uint32_t pwm_val, uint32_t u32_adc_val, \
int32_t *pij, uint32_t u32_adc_val_old, status mode);
uint8_t oc_check(int32_t i32_pwm_val, uint8_t u8_oc_trip);
status dchg_ctrl(batpins batteryx, batprops *batpropsx, uint32_t counter);
status chg_ctrl(batpins batteryx, batprops *batpropsx, uint32_t counter);
status discharge_main(batpins pinsx, batprops *batx, uint32_t* restStartms, \
		uint32_t loops, status bat_stat);
status cv_main(batpins pinsx, batprops *batx, uint32_t* restStartms, \
uint32_t loops, status bat_stat);
