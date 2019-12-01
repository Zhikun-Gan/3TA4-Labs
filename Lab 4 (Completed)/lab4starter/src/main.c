/**

******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32L4xx HAL API.
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

#define  PERIOD_VALUE       (uint32_t)(666 - 1)  /* Period Value  */
#define  PULSE_OFF       (uint32_t)(PERIOD_VALUE*12.5/100)        /* Capture Compare 1 Value  */
#define  PULSE_LOW       (uint32_t)(PERIOD_VALUE*25.0/100) /* Capture Compare 2 Value  */
#define  PULSE_MED       (uint32_t)(PERIOD_VALUE*50.0/100)        /* Capture Compare 3 Value  */
#define  PULSE_HIGH       (uint32_t)(PERIOD_VALUE*75.0/100) /* Capture Compare 4 Value  */


#define adcToTempCoeff (double)((3.0/4095)/3/0.01)

__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 


TIM_HandleTypeDef    Adc_Tim_Handle, Pwm_Tim_Handle;
TIM_OC_InitTypeDef Adc_Tim_OCInitStructure, Pwm_Tim_OCInitStructure;
uint16_t Adc_Tim_PrescalerValue,Pwm_Tim_PrescalerValue;
__IO uint16_t Pwm_Pulse_Value;


ADC_HandleTypeDef    Adc_Handle;
ADC_HandleTypeDef             AdcHandle;
ADC_ChannelConfTypeDef        sConfig;
uint32_t                      aResultDMA;


GPIO_InitTypeDef GPIO_ADC_Handle, GPIO_PWM_Handle;



enum programStateTypeDef {
	displayTemp,
	setRefTemp
};

enum programStateTypeDef programState = displayTemp;


__IO uint16_t ADC1ConvertedValue;   //if declare it as 16t, it will not work.


volatile double  setPoint=23.5;
double measuredTemp;
double tempDifference;



char lcd_buffer[6];    // LCD display buffer


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

void ADC_TIM_Config(void);
void PWM_TIM_Config(void);

static void ADC_Config(void);
static void GPIO_Config(void);


//static void EXTILine14_Config(void); // configure the exti line4, for exterrnal button, WHICH BUTTON?


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4 
       - Low Level Initialization
     */

	HAL_Init();

	SystemClock_Config();   

	HAL_InitTick(0x0000); // set systick's priority to the highest.

	
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);


	BSP_LCD_GLASS_Init();
	
	BSP_JOY_Init(JOY_MODE_EXTI);  
	
	ADC_TIM_Config();
	PWM_TIM_Config();
	
	
	GPIO_Config();
	
	ADC_Config();
	
 	BSP_LCD_GLASS_Clear();
	
	
	HAL_ADC_PollForConversion(&AdcHandle,10);
	measuredTemp = (double)(HAL_ADC_GetValue(&AdcHandle)*adcToTempCoeff);
	//measuredTemp = 30.0;
	
  while (1)
  {	
		tempDifference = measuredTemp - setPoint;
		if (tempDifference < 0) {
			BSP_LED_Off(LED5);
			BSP_LED_Off(LED4);
			Pwm_Pulse_Value = PULSE_OFF;
			PWM_TIM_Config();
		}
		else if (tempDifference>0 && tempDifference<2.5) {
			BSP_LED_On(LED5);
			BSP_LED_Off(LED4);
			Pwm_Pulse_Value = PULSE_LOW;
			PWM_TIM_Config();
		}
		else if (tempDifference>2.5 && tempDifference<5) {
			BSP_LED_On(LED5);
			BSP_LED_On(LED4);
			Pwm_Pulse_Value = PULSE_MED;
			PWM_TIM_Config();
		}
		else if (tempDifference>5) {
			BSP_LED_On(LED4);
			BSP_LED_Off(LED5);
			Pwm_Pulse_Value = PULSE_HIGH;
			PWM_TIM_Config();
		}
		
		switch(programState)	{
			case displayTemp:
				sprintf(lcd_buffer,"%2.1f",measuredTemp);
				BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
				break;
			case setRefTemp:
				sprintf(lcd_buffer,"%2.1f",setPoint);
				BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
				break;
			default:
				break;
		}
		
			
	} //end of while 1

}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */


void SystemClock_Config(void)
{ 
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};                                            

 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;            
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;  
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;

	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;   //PLL source: either MSI, or HSI or HSE, but can not make HSE work.
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40; 
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4;   //2, 4,6, 0r 8  
	//the PLL will be MSI (4Mhz)*N /M/R = 

	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  // Set 0 Wait State flash latency for 4Mhz 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; //the freq of pllclk is MSI (4Mhz)*N /M/R = 80Mhz 
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
	
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)   //???
  {
    // Initialization Error 
    while(1);
  }

  // The voltage scaling allows optimizing the power consumption when the device is
  //   clocked below the maximum system frequency, to update the voltage scaling value
  //   regarding system frequency refer to product datasheet.  

  // Enable Power Control clock 
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Disable Power Control clock   //why disable it?
  __HAL_RCC_PWR_CLK_DISABLE();      
}




/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
			case GPIO_PIN_0: 		               //SELECT button					
						if (programState == displayTemp) {
							BSP_LCD_GLASS_Clear();
							programState++;
						}
						else if (programState == setRefTemp) {
							BSP_LCD_GLASS_Clear();
							programState = displayTemp;
						}
						break;	
			case GPIO_PIN_1:     //left button						
							
							break;
			case GPIO_PIN_2:    //right button						  to play again.
						
							break;
			case GPIO_PIN_3:    //up button							
						if (programState == setRefTemp) setPoint+=0.5;
							break;
			
			case GPIO_PIN_5:    //down button						
						if (programState == setRefTemp) setPoint-=0.5;
							break;
			
			default://
						//default
						break;
	  } 
}




void  ADC_TIM_Config(void)
{  
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  Adc_Tim_PrescalerValue = (uint32_t) (SystemCoreClock/ 10000) - 1;
  
  /* Set TIM3 instance */
  Adc_Tim_Handle.Instance = TIM2; //TIM3 is defined in stm32f429xx.h
 
  Adc_Tim_Handle.Init.Period = 1000;
  Adc_Tim_Handle.Init.Prescaler = Adc_Tim_PrescalerValue;
  Adc_Tim_Handle.Init.ClockDivision = 0;
  Adc_Tim_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&Adc_Tim_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    /* Initialization Error */
    Error_Handler();
  }
  
	
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&Adc_Tim_Handle) != HAL_OK)   //the TIM_XXX_Start_IT function enable IT, and also enable Timer
																											//so do not need HAL_TIM_BASE_Start() any more.
  {
    /* Starting Error */
    Error_Handler();
  }
	
	
	
}


// configure Pwm Timer
void  PWM_TIM_Config(void)
{
  Pwm_Tim_Handle.Instance = TIM4;

  Pwm_Tim_Handle.Init.Prescaler         = (uint16_t)(SystemCoreClock / 16000000) - 1;
  Pwm_Tim_Handle.Init.Period            = PERIOD_VALUE;
  Pwm_Tim_Handle.Init.ClockDivision     = 0;
  Pwm_Tim_Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  Pwm_Tim_Handle.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&Pwm_Tim_Handle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the PWM channels #########################################*/
  /* Common configuration for all channels */
  Pwm_Tim_OCInitStructure.OCMode       = TIM_OCMODE_PWM1;
  Pwm_Tim_OCInitStructure.OCPolarity   = TIM_OCPOLARITY_HIGH;
  Pwm_Tim_OCInitStructure.OCFastMode   = TIM_OCFAST_DISABLE;
  Pwm_Tim_OCInitStructure.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  Pwm_Tim_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  Pwm_Tim_OCInitStructure.OCIdleState  = TIM_OCIDLESTATE_RESET;

  /* Set the pulse value for channel 1 */
  Pwm_Tim_OCInitStructure.Pulse = Pwm_Pulse_Value;
  if (HAL_TIM_PWM_ConfigChannel(&Pwm_Tim_Handle, &Pwm_Tim_OCInitStructure, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
	if (HAL_TIM_PWM_Start(&Pwm_Tim_Handle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }

}







void ADC_Config(void)
{
	/* ### - 1 - Initialize ADC peripheral #################################### */
  AdcHandle.Instance          = ADC1;
  if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK)
  {
    /* ADC de-initialization Error */
    Error_Handler();
  }

  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;          /* Asynchronous clock mode, input ADC clock not divided */
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;             /* 12-bit resolution for converted data */
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
  AdcHandle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
  AdcHandle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
  AdcHandle.Init.ContinuousConvMode    = ENABLE;                        /* Continuous mode enabled (automatic conversion restart after each conversion) */
  AdcHandle.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
  AdcHandle.Init.DMAContinuousRequests = DISABLE;                        /* DMA circular mode selected */
  AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
  AdcHandle.Init.OversamplingMode      = DISABLE;                       /* No oversampling */

  /* Initialize ADC peripheral according to the passed parameters */
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    Error_Handler();
  }
  
  
  /* ### - 2 - Start calibration ############################################ */
  if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED) !=  HAL_OK)
  {
    Error_Handler();
  }
  
  /* ### - 3 - Channel configuration ######################################## */
  sConfig.Channel      = ADC_CHANNEL_7;                /* Sampled channel number */
  sConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;    /* Sampling time (number of clock cycles unit) */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
  sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
	
  /* ### - 4 - Start conversion in DMA mode ################################# */
  if (HAL_ADC_Start(&AdcHandle) != HAL_OK)
  {
    Error_Handler();
  }
}




void GPIO_Config(void)
{
	GPIO_ADC_Handle.Pin = GPIO_PIN_2;
	GPIO_ADC_Handle.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	GPIO_ADC_Handle.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA,&GPIO_ADC_Handle);
	
	GPIO_PWM_Handle.Pin = GPIO_PIN_6;
	GPIO_PWM_Handle.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOB,&GPIO_PWM_Handle);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   //see  stm32fxx_hal_tim.c for different callback function names. 
																															//for timer 3 , Timer 3 use update event initerrupt
{
	if ((*htim).Instance==TIM2) {
		measuredTemp = (double)(HAL_ADC_GetValue(&AdcHandle)*adcToTempCoeff);
	}
		
	
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32XXX_hal_tim.c for different callback function names. 
{																																//for timer4 

}
 
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef * htim){  //this is for TIM4_pwm
	
	__HAL_TIM_SET_COUNTER(htim, 0x0000);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
	
}



static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}





#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
