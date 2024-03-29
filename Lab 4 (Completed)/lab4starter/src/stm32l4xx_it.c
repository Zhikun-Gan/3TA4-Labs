/**
  ******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/stm32l4xx_it.c
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#include "main.h"
#include "stm32l4xx_it.h"

/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef    Adc_Handle;
extern TIM_HandleTypeDef    Adc_Tim_Handle, Pwm_Tim_Handle;



/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/*                 STM32L4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles external line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
	
	HAL_GPIO_EXTI_IRQHandler(SEL_JOY_PIN);  //PA0
}

void EXTI1_IRQHandler (void)
{
 
	HAL_GPIO_EXTI_IRQHandler(LEFT_JOY_PIN);  
}


/**
  * @brief  This function handles EXTI2 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI2_IRQHandler (void)
{

	HAL_GPIO_EXTI_IRQHandler(RIGHT_JOY_PIN);  
}

/**
  * @brief  This function handles EXTI3 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI3_IRQHandler (void)
{
	HAL_GPIO_EXTI_IRQHandler(UP_JOY_PIN); 
}

/**
  * @brief  This function handles EXTI9_5 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler (void)
//void EXTI5_IRQHandler (void)  //will not work!
{
	HAL_GPIO_EXTI_IRQHandler(DOWN_JOY_PIN);  //PA5
}




void ADC1_IRQHandler(void){     //there is no ADC_IRQHandler.
	  HAL_ADC_IRQHandler(&Adc_Handle);
}

void DMA2_Channel3_IRQHandler(void)
{
		HAL_DMA_IRQHandler(Adc_Handle.DMA_Handle);
}



void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&Adc_Tim_Handle);
}

void TIM4_IRQHandler(void)
{	
		HAL_TIM_IRQHandler(&Pwm_Tim_Handle);
}






/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
