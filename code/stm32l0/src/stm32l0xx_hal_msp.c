/**
  ******************************************************************************
  * File Name          : stm32l0xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

extern DMA_HandleTypeDef hdma_tim2_ch1;
extern DMA_HandleTypeDef hdma_tim2_ch2;
extern DMA_HandleTypeDef hdma_tim2_ch3;
extern DMA_HandleTypeDef hdma_tim2_ch4;

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    __ADC1_CLK_ENABLE();
  
    /**ADC GPIO Configuration    
    PC0     ------> ADC_IN10
    PC1     ------> ADC_IN11
    PA4     ------> ADC_IN4
    PB0     ------> ADC_IN8 
    */
    GPIO_InitStruct.Pin = ADC_4i_Pin|ADC_4v_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ADC_3v_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADC_3v_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ADC_3i_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADC_3i_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }

}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __ADC1_CLK_DISABLE();
  
    /**ADC GPIO Configuration    
    PC0     ------> ADC_IN10
    PC1     ------> ADC_IN11
    PA4     ------> ADC_IN4
    PB0     ------> ADC_IN8 
    */
    HAL_GPIO_DeInit(GPIOC, ADC_4i_Pin|ADC_4v_Pin);

    HAL_GPIO_DeInit(ADC_3v_GPIO_Port, ADC_3v_Pin);

    HAL_GPIO_DeInit(ADC_3i_GPIO_Port, ADC_3i_Pin);

  }
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */

}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_pwm->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __TIM2_CLK_ENABLE();
  
    /**TIM2 GPIO Configuration    
    PA0     ------> TIM2_CH1
    PA1     ------> TIM2_CH2
    PB10     ------> TIM2_CH3
    PB11     ------> TIM2_CH4 
    */
    GPIO_InitStruct.Pin = fet_ctrl_3a_Pin|fet_ctrl_3b_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = fet_ctrl_4a_Pin|fet_ctrl_4b_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_tim2_ch1.Instance = DMA1_Channel5;
    hdma_tim2_ch1.Init.Request = DMA_REQUEST_8;
    hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;
    hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_tim2_ch1);

    __HAL_LINKDMA(htim_pwm,hdma[TIM_DMA_ID_CC1],hdma_tim2_ch1);

    hdma_tim2_ch2.Instance = DMA1_Channel3;
    hdma_tim2_ch2.Init.Request = DMA_REQUEST_8;
    hdma_tim2_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim2_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch2.Init.Mode = DMA_CIRCULAR;
    hdma_tim2_ch2.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_tim2_ch2);

    __HAL_LINKDMA(htim_pwm,hdma[TIM_DMA_ID_CC2],hdma_tim2_ch2);

    hdma_tim2_ch3.Instance = DMA1_Channel1;
    hdma_tim2_ch3.Init.Request = DMA_REQUEST_8;
    hdma_tim2_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim2_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch3.Init.Mode = DMA_CIRCULAR;
    hdma_tim2_ch3.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_tim2_ch3);

    __HAL_LINKDMA(htim_pwm,hdma[TIM_DMA_ID_CC3],hdma_tim2_ch3);

    hdma_tim2_ch4.Instance = DMA1_Channel4;
    hdma_tim2_ch4.Init.Request = DMA_REQUEST_8;
    hdma_tim2_ch4.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim2_ch4.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch4.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch4.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch4.Init.Mode = DMA_CIRCULAR;
    hdma_tim2_ch4.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_tim2_ch4);

    __HAL_LINKDMA(htim_pwm,hdma[TIM_DMA_ID_CC4],hdma_tim2_ch4);

  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(htim_pwm->Instance==TIM21)
  {
  /* USER CODE BEGIN TIM21_MspInit 0 */

  /* USER CODE END TIM21_MspInit 0 */
    /* Peripheral clock enable */
    __TIM21_CLK_ENABLE();
  
    /**TIM21 GPIO Configuration    
    PB13     ------> TIM21_CH1
    PB14     ------> TIM21_CH2 
    */
    GPIO_InitStruct.Pin = dchg_ctrl_3_Pin|dchg_ctrl_4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM21;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM21_MspInit 1 */

  /* USER CODE END TIM21_MspInit 1 */
  }

}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{

  if(htim_pwm->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM2_CLK_DISABLE();
  
    /**TIM2 GPIO Configuration    
    PA0     ------> TIM2_CH1
    PA1     ------> TIM2_CH2
    PB10     ------> TIM2_CH3
    PB11     ------> TIM2_CH4 
    */
    HAL_GPIO_DeInit(GPIOA, fet_ctrl_3a_Pin|fet_ctrl_3b_Pin);

    HAL_GPIO_DeInit(GPIOB, fet_ctrl_4a_Pin|fet_ctrl_4b_Pin);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(htim_pwm->hdma[TIM_DMA_ID_CC1]);
    HAL_DMA_DeInit(htim_pwm->hdma[TIM_DMA_ID_CC2]);
    HAL_DMA_DeInit(htim_pwm->hdma[TIM_DMA_ID_CC3]);
    HAL_DMA_DeInit(htim_pwm->hdma[TIM_DMA_ID_CC4]);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(htim_pwm->Instance==TIM21)
  {
  /* USER CODE BEGIN TIM21_MspDeInit 0 */

  /* USER CODE END TIM21_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM21_CLK_DISABLE();
  
    /**TIM21 GPIO Configuration    
    PB13     ------> TIM21_CH1
    PB14     ------> TIM21_CH2 
    */
    HAL_GPIO_DeInit(GPIOB, dchg_ctrl_3_Pin|dchg_ctrl_4_Pin);

  /* USER CODE BEGIN TIM21_MspDeInit 1 */

  /* USER CODE END TIM21_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
