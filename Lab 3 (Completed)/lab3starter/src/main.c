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


this program: 

1. This project needs the libraray file i2c_at2464c.c and its header file. 
2. in the i2c_at2464c.c, the I2C SCL and SDA pins are configured as PULLUP. so do not need to pull up resistors (even do not need the 100 ohm resisters).
NOTE: students can also configure the TimeStamp pin 	

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
I2C_HandleTypeDef  pI2c_Handle;

RTC_HandleTypeDef RTCHandle;
RTC_DateTypeDef RTC_DateStructure;
RTC_TimeTypeDef RTC_TimeStructure;

__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

//memory location to write to in the device
__IO uint16_t memLocation = 0x000A; //pick any location within range

  

char lcd_buffer[9];    // LCD display buffer
char timestring[12]={0};  // LCD display buffer for displaying time
char datestring[13]={0};  // LCD display buffer for displaying date

enum programStateTypeDef { // struct for states of the program
	displayDate,
	saveTime,
	displayTime,
	readTime,
	setting,
	yearSetting,
	monthSetting,
	daySetting,
	weekDaySetting,
	hourSetting,
	minuteSetting,
	secondSetting,
	dataInc
};
enum programStateTypeDef programState = displayTime; // sets initial state to displaying current time
enum programStateTypeDef prevState; // variable for remembering previous state

uint8_t wd=0x01, dd=0x01, mo=0x01, yy, ss, mm, hh; // for weekday, day, month, year, second, minute, hour

uint8_t saved_sec1=0x00, saved_min1=0x00, saved_hr1=0x00; // Last saved time
uint8_t saved_sec2, saved_min2, saved_hr2; // Second last saved time
char * wdString; // LCD display buffer for displaying week day

__IO uint32_t SEL_Pressed_StartTick;   //sysTick when the User button is pressed

__IO uint8_t leftpressed, rightpressed, uppressed, downpressed, selpressed;  // button pressed 
__IO uint8_t  sel_held;   // if the selection button is held for a while (>800ms)

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

void RTC_Config(void);
void RTC_AlarmAConfig(void);

void updateRTC(); // Updates RTC with modified values

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

	leftpressed=0;
	rightpressed=0;
	uppressed=0;
	downpressed=0;
	selpressed=0;
	sel_held=0;
	

	HAL_Init();
	
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
  
	SystemClock_Config();   
											
	
	HAL_InitTick(0x0000); //set the systick interrupt priority to the highest, !!!This line need to be after systemClock_config()

	
	BSP_LCD_GLASS_Init();
	
	BSP_JOY_Init(JOY_MODE_EXTI);

	BSP_LCD_GLASS_DisplayString((uint8_t*)"MT3TA4");	
	HAL_Delay(500);


//configure real-time clock
	RTC_Config();
	
	RTC_AlarmAConfig();
	
	I2C_Init(&pI2c_Handle);


//*********************Testing I2C EEPROM------------------

	//the following variables are for testging I2C_EEPROM
	uint8_t data1 =0x67,  data2=0x68;
	uint8_t readData=0x00;
	uint16_t EE_status;


	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation, data1);

  
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
  }
	
	
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 X");

	HAL_Delay(500);
	
	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1 , data2);
	
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
  }
	
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 X");

	HAL_Delay(500);
	
	readData=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation);

	BSP_LCD_GLASS_Clear();
	if (data1 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 X");
	}	
	
	HAL_Delay(500);
	
	readData=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1);

	BSP_LCD_GLASS_Clear();
	if (data2 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 2 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t *)"r 2 X");
	}	

	HAL_Delay(500);
	
	HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BCD);
	HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BCD);


//******************************testing I2C EEPROM*****************************	
	
	updateRTC();

  /* Infinite loop */
  while (1)
  {
			//the joystick is pulled down. so the default status of the joystick is 0, when pressed, get status of 1. 
			//while the interrupt is configured at the falling edge---the moment the pressing is released, the interrupt is triggered.
			//therefore, the variable "selpressed==1" can not be used to make choice here.
			if (BSP_JOY_GetState() == JOY_SEL) {
					SEL_Pressed_StartTick=HAL_GetTick(); 
					while(BSP_JOY_GetState() == JOY_SEL) {  //while the selection button is pressed)	
						if ((HAL_GetTick()-SEL_Pressed_StartTick)>800) {
								if (programState == displayTime) programState = displayDate; // If selection button is held at displaying current time state,
																																						 // switches to displaying date state
						} 
					}
			}					
//==============================================================			

//==============================================================					
			if (selpressed==1)  {
					selpressed=0;
				
					if (programState == displayTime) programState = saveTime; // saves current time into EEPROM
				
					else if (programState >= yearSetting && programState <= secondSetting) { // while in setting state, goes into selected time parameter
							prevState = programState;
							programState = dataInc;
					}
			} 
//==============================================================			

//==============================================================		 
			if (leftpressed==1) {						
					leftpressed=0;
				
					if (programState == displayTime) programState = readTime; // switches from displaying current time to reading saved times from EEPROM
					else if (programState == readTime) programState = displayTime; // switches from reading saved times to displaying current time
					else if (programState == displayDate) programState = displayTime; // switches from displaying date to displaying current time
				
					else if (programState >= setting && programState < secondSetting) programState++; // while in setting state, cycles through time parameters
					else if (programState == secondSetting) programState = yearSetting;
			}			
//==============================================================			

//==============================================================							
			if (rightpressed==1) {
					rightpressed=0;
				
					if (programState == displayDate || programState == saveTime || programState == displayTime) // goes into setting state
						programState = setting;
					else if (programState >= setting && programState <= secondSetting) programState = displayTime; // switches from setting state back to displaying current time
					else if (programState == dataInc){ // switches from data changing mode back to setting state
						programState = prevState;						
						
					}
			}
//==============================================================			

//==============================================================						
			switch (programState) { 
				case displayDate: // State for displaying date
					// sets LCD buffer strings for different week days
					switch(wd){
						case 1:
							wdString="SUN";
							break;
						case 2:
							wdString="MON";
							break;
						case 3:
							wdString="TUE";
							break;
						case 4:
							wdString="WED";
							break;
						case 5:
							wdString="THU";
							break;
						case 6:
							wdString="FRI";
							break;
						case 7:
							wdString="SAT";
							break;
						default:
							break;
					}
					
					// Cycles between week day display and date display
					sprintf(datestring, "%02u%02u%02u", RTC_DateStructure.Month, RTC_DateStructure.Date, RTC_DateStructure.Year);
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)datestring);
					HAL_Delay(1000);
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)wdString);
					HAL_Delay(1000);
					
					break;
				case saveTime: // Saves current time when selection is pressed
					sprintf(timestring, "%02u%02u%02u", RTC_TimeStructure.Hours, RTC_TimeStructure.Minutes, RTC_TimeStructure.Seconds);
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)timestring);
					
					// In EEPROM, moves last saved time data into slots for second last saved time
					I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS,memLocation+3,I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,memLocation));
					I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS,memLocation+4,I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,memLocation+1));
					I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS,memLocation+5,I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,memLocation+2));
										
					// Saves current time into slots for last saved time
					I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS,memLocation,RTC_TimeStructure.Seconds);
					I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS,memLocation+1,RTC_TimeStructure.Minutes);
					I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS,memLocation+2,RTC_TimeStructure.Hours);
				
					// Goes back to displaying current time state without needing button press
					programState = displayTime;
					break;
				case displayTime: // State for displaying current time
					sprintf(timestring, "%02u%02u%02u", RTC_TimeStructure.Hours, RTC_TimeStructure.Minutes, RTC_TimeStructure.Seconds);
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)timestring);
					break;
				case readTime: // Reads the last two saved times from EEPROM
					// Reads last saved time
					saved_sec1 = I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,memLocation);
					saved_min1 = I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,memLocation+1);
					saved_hr1 = I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,memLocation+2);
					
					//Reads second last saved time
					saved_sec2 = I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,memLocation+3);
					saved_min2 = I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,memLocation+4);
					saved_hr2 = I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,memLocation+5);
					
					// Displays last saved time
					sprintf(timestring, "%02u%02u%02u", saved_hr1, saved_min1, saved_sec1);
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)timestring);
					BSP_LCD_GLASS_DisplayBar(1);
				
					// Displays second last saved time for one second if up button is pressed
					if(uppressed == 1){
						uppressed=0;
						sprintf(timestring, "%02u%02u%02u", saved_hr2, saved_min2, saved_sec2);
						BSP_LCD_GLASS_Clear();
						BSP_LCD_GLASS_DisplayString((uint8_t*)timestring);
						BSP_LCD_GLASS_DisplayBar(2);
						HAL_Delay(1000);
					}
					break;
				case setting: // Transition state between the states for displaying and states for modifying
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"set");
					break;
				case yearSetting: // Indication for changing year
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"setYr");
					break;
				case monthSetting: // Indication for changing month
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"setMo");
					break;
				case daySetting: // Indication for changing day
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"setDay");
					break;
				case weekDaySetting: // Indication for changing week day
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"setWkD");
					break;
				case hourSetting: // Indication for changing hour
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"setHr");
					break;
				case minuteSetting: // Indication for changing minute
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"setMin");
					break;
				case secondSetting: // Indication for changing second
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"setSec");
					break;
				case dataInc: // State for modifying date/time data
					BSP_LCD_GLASS_Clear();
					
					// Checks which date/time parameter is selected
					switch(prevState){
						
						// Modifying year
						case yearSetting:
							BSP_LCD_GLASS_Clear();
							sprintf(lcd_buffer,"%02u",yy);
							BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
							if (uppressed == 1){
								uppressed = 0;
								if (yy<99) yy++;
								else if (yy==99) yy=0;
							}
							else if (downpressed == 1){
								downpressed = 0;
								if (yy>0) yy--;
								else if (yy==0) yy=99;
							}
							break;
							
						// Modifying month	
						case monthSetting:
							BSP_LCD_GLASS_Clear();
							sprintf(lcd_buffer,"%02u",mo);
							BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
							if (uppressed == 1){
								uppressed = 0;
								if (mo<12) mo++;
								else if (mo==12) mo=1;
							}
							else if (downpressed == 1){
								downpressed = 0;
								if (mo>1) mo--;
								else if (mo==1) mo=12;
							}
							break;
							
						// Modifying day
						case daySetting:
							BSP_LCD_GLASS_Clear();
							sprintf(lcd_buffer,"%02u",dd);
							BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
							if (uppressed == 1){
								uppressed = 0;
								if (dd<31) dd++;
								else if (dd==31) dd=1;
							}
							else if (downpressed == 1){
								downpressed = 0;
								if (dd>1) dd--;
								else if (dd==1) dd=31;
							}
							break;
							
						// Modifying week day
						case weekDaySetting:
							switch(wd){
								case 1:
									wdString="SUN";
									break;
								case 2:
									wdString="MON";
									break;
								case 3:
									wdString="TUE";
									break;
								case 4:
									wdString="WED";
									break;
								case 5:
									wdString="THU";
									break;
								case 6:
									wdString="FRI";
									break;
								case 7:
									wdString="SAT";
									break;
								default:
									break;
							}
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)wdString);
							if (uppressed == 1){
								uppressed = 0;
								if (wd<7) wd++;
								else if (wd==7) wd=1;
							}
							else if (downpressed == 1){
								downpressed = 0;
								if (wd>1) wd--;
								else if (wd==1) wd=7;
							}
							break;
							
						// Modifying hour	
						case hourSetting:
							BSP_LCD_GLASS_Clear();
							sprintf(lcd_buffer,"%02u",hh);
							BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
							if (uppressed == 1){
								uppressed = 0;
								if (hh<24) hh++;
								else if (hh==24) hh=0;
							}
							else if (downpressed == 1){
								downpressed = 0;
								if (hh>0) hh--;
								else if (hh==0) hh=24;
							}
							break;
							
						// Modifying minute
						case minuteSetting:
							BSP_LCD_GLASS_Clear();
							sprintf(lcd_buffer,"%02u",mm);
							BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
							if (uppressed == 1){
								uppressed = 0;
								if (mm<59) mm++;
								else if (mm==59) mm=0;
							}
							else if (downpressed == 1){
								downpressed = 0;
								if (mm>0) mm--;
								else if (mm==0) mm=59;
							}
							break;
							
						// Modifying second
						case secondSetting:
							BSP_LCD_GLASS_Clear();
							sprintf(lcd_buffer,"%02u",ss);
							BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
							if (uppressed == 1){
								uppressed = 0;
								if (ss<59) ss++;
								else if (ss==59) ss=0;
							}
							else if (downpressed == 1){
								downpressed = 0;
								if (ss>0) ss--;
								else if (ss==0) ss=59;
							}
							break;
						
						default:
							break;
					} // end of dataInc switch
					
					updateRTC(); // Updates RTC after values are modified
					break;
				
				default:
					break;
				
				
			} //end of switch					
		


	}
}

// Updates RTC
void updateRTC()
{
	RTC_DateStructure.Year=yy;
	RTC_DateStructure.Month=mo;
	RTC_DateStructure.Date=dd;
	RTC_DateStructure.WeekDay=wd;
	
	RTC_TimeStructure.Hours=hh;
	RTC_TimeStructure.Minutes=mm;
	RTC_TimeStructure.Seconds=ss;
	
	HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
	HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN);
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

  // RTC requires to use HSE (or LSE or LSI, suspect these two are not available)
	//reading from RTC requires the APB clock is 7 times faster than HSE clock, 
	//so turn PLL on and use PLL as clock source to sysclk (so to APB)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;     //RTC need either HSE, LSE or LSI           
  
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;  
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;
  
	//RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;//RCC_PLL_NONE;

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
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz


void RTC_Config(void) {
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	

	//****1:***** Enable the RTC domain access (enable wirte access to the RTC )
			//1.1: Enable the Power Controller (PWR) APB1 interface clock:
        __HAL_RCC_PWR_CLK_ENABLE();    
			//1.2:  Enable access to RTC domain 
				HAL_PWR_EnableBkUpAccess();    
			//1.3: Select the RTC clock source
				__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);    
				//RCC_RTCCLKSOURCE_LSI is defined in hal_rcc.h
	       // according to P9 of AN3371 Application Note, LSI's accuracy is not suitable for RTC application!!!! 
				
			//1.4: Enable RTC Clock
			__HAL_RCC_RTC_ENABLE();   //enable RTC --see note for the Macro in _hal_rcc.h---using this Marco requires 
																//the above three lines.
			
	
			//1.5  Enable LSI
			__HAL_RCC_LSI_ENABLE();   //need to enable the LSI !!!
																//defined in _rcc.c
			while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)==RESET) {}    //defind in rcc.c
	
			// for the above steps, please see the CubeHal UM1725, p616, section "Backup Domain Access" 	
				
				
				
	//****2.*****  Configure the RTC Prescaler (Asynchronous and Synchronous) and RTC hour 
        
		
		
				RTCHandle.Instance = RTC;
				RTCHandle.Init.HourFormat = RTC_HOURFORMAT_24;
				
				RTCHandle.Init.AsynchPrediv = 127; 
				RTCHandle.Init.SynchPrediv = 255; 
				
				
				RTCHandle.Init.OutPut = RTC_OUTPUT_ALARMA;
				RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
				RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
				
			
				if(HAL_RTC_Init(&RTCHandle) != HAL_OK)
				{
					BSP_LCD_GLASS_Clear(); 
					BSP_LCD_GLASS_DisplayString((uint8_t *)"RT I X"); 	
				}
	
	
	
	//****3.***** init the time and date
				
				
				RTC_DateStructure.Year = yy;
				RTC_DateStructure.Month = mo;
				RTC_DateStructure.Date = dd;
				RTC_DateStructure.WeekDay = wd;
				
				if(HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BCD) != HAL_OK)   //BIN format is better 
															//before, must set in BCD format and read in BIN format!!
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t *)"D I X");
				} 
  
  
				RTC_TimeStructure.Hours = hh;  
				RTC_TimeStructure.Minutes = mm; 
				RTC_TimeStructure.Seconds = ss;
				//RTC_TimeStructure.TimeFormat = ???;
				RTC_TimeStructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				RTC_TimeStructure.StoreOperation = RTC_STOREOPERATION_RESET;
				
				if(HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BCD) != HAL_OK)   //BIN format is better
																																					//before, must set in BCD format and read in BIN format!!
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t *)"T I X");
				}	
	  



				
			__HAL_RTC_TAMPER1_DISABLE(&RTCHandle);
			__HAL_RTC_TAMPER2_DISABLE(&RTCHandle);	
				//Optionally, a tamper event can cause a timestamp to be recorded. ---P802 of RM0090
				//Timestamp on tamper event
				//With TAMPTS set to ‘1 , any tamper event causes a timestamp to occur. In this case, either
				//the TSF bit or the TSOVF bit are set in RTC_ISR, in the same manner as if a normal
				//timestamp event occurs. The affected tamper flag register (TAMP1F, TAMP2F) is set at the
				//same time that TSF or TSOVF is set. ---P802, about Tamper detection
				//-------that is why need to disable this two tamper interrupts. Before disable these two, when program start, there is always a timestamp interrupt.
				//----also, these two disable function can not be put in the TSConfig().---put there will make  the program freezed when start. the possible reason is
				//-----one the RTC is configured, changing the control register again need to lock and unlock RTC and disable write protection.---See Alarm disable/Enable 
				//---function.
				
			HAL_RTC_WaitForSynchro(&RTCHandle);	
			//To read the calendar through the shadow registers after Calendar initialization,
			//		calendar update or after wake-up from low power modes the software must first clear
			//the RSF flag. The software must then wait until it is set again before reading the
			//calendar, which means that the calendar registers have been correctly copied into the
			//RTC_TR and RTC_DR shadow registers.The HAL_RTC_WaitForSynchro() function
			//implements the above software sequence (RSF clear and RSF check).	
}


void RTC_AlarmAConfig(void)
{
	RTC_AlarmTypeDef RTC_Alarm_Structure;


	
	RTC_Alarm_Structure.Alarm = RTC_ALARM_A;
  RTC_Alarm_Structure.AlarmMask = RTC_ALARMMASK_ALL;
	
			
  
  if(HAL_RTC_SetAlarm_IT(&RTCHandle,&RTC_Alarm_Structure,RTC_FORMAT_BCD) != HAL_OK)
  {
			BSP_LCD_GLASS_Clear(); 
			BSP_LCD_GLASS_DisplayString((uint8_t *)"A S X");
  }

	__HAL_RTC_ALARM_CLEAR_FLAG(&RTCHandle, RTC_FLAG_ALRAF); //without this line, sometimes(SOMETIMES, when first time to use the alarm interrupt)
																			//the interrupt handler will not work!!! 		

		//need to set/enable the NVIC for RTC_Alarm_IRQn!!!!
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);   
	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 3, 0);  //not important ,but it is better not use the same prio as the systick
	
}

//You may need to disable and enable the RTC Alarm at some moment in your application
HAL_StatusTypeDef  RTC_AlarmA_IT_Disable(RTC_HandleTypeDef *hrtc) 
{ 
 	// Process Locked  
	__HAL_LOCK(hrtc);
  
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_DISABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_DISABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  
}


HAL_StatusTypeDef  RTC_AlarmA_IT_Enable(RTC_HandleTypeDef *hrtc) 
{	
	// Process Locked  
	__HAL_LOCK(hrtc);	
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_ENABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_ENABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  

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
						selpressed=1;	
						break;	
			case GPIO_PIN_1:     //left button						
							leftpressed=1;
							break;
			case GPIO_PIN_2:    //right button
							rightpressed=1;			
							break;
			case GPIO_PIN_3:    //up button							
							uppressed = 1;
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)"up");
							break;
			
			case GPIO_PIN_5:    //down button			
							downpressed = 1;
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)"down");
							break;
			case GPIO_PIN_14:    //? button						
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)"PE14");
							break;			
			default://
						//default
						break;
	  } 
}



void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  BSP_LED_Toggle(LED5);
	//RTC_TimeShow();
	HAL_RTC_GetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN); // Updates RTC time every second
	HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
	
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
