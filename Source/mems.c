/**
  ******************************************************************************
  * @file    BSP/Src/mems.c 
  * @author  MCD Application Team
  * @version V1.2.7
  * @date    04-November-2016
  * @brief   This example code shows how to use MEMS features.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#include "mems.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup BSP
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Init af threshold to detect acceleration on MEMS */
/* Typical value: 
      - No  acceleration: X, Y inferior to 100 (positive or negative)
      - Max acceleration: X, Y around 2000 (positive or negative) */
int16_t ThresholdHigh = 200; //Compare with the high threshholds
int16_t ThresholdLow = -200; //compare with the low threshholds

/* Private function prototypes -----------------------------------------------*/
static void ACCELERO_ReadAcc(void);
void delayLoop(int16_t);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Test ACCELERATOR MEMS Hardware.
  *         The main objective of this test is to check acceleration on 2 axes X and Y
  * @param  None
  * @retval None
  */
void ACCELERO_MEMS_Test(void)
{
  /* Init Accelerometer MEMS */
  if(BSP_ACCELERO_Init() != ACCELERO_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }

    ACCELERO_ReadAcc();
}

/**
  * @brief  Read Acceleration data.
  * @param  None
  * @retval None
  */
static void ACCELERO_ReadAcc(void)
{  
  /* Accelerometer variables */
  int16_t buffer[3] = {0};
  int16_t xval, yval = 0x00;
  /* Motor variables */
  int16_t turnOn_time = 0;
	
  /* Read Acceleration */
  BSP_ACCELERO_GetXYZ(buffer);
  
  xval = buffer[0];
  yval = buffer[1];
  
 if((ABS(xval))>(ABS(yval)))
 {
	if(xval > ThresholdHigh)//If x is greater than threshhold
	{ 
		/*values are not exact.....as the servo motor
		was not available for extended period of time*/
		/* LED5 On right red: Angle : 0 to 90 degree*/
		/*Time calculated to turn motor on: time needed: 2250 to 700*/
		turnOn_time = 2420 - (xval*1550)/1800;
		BSP_LED_On(LED5);
		delayLoop(turnOn_time);
		BSP_LED_Off(LED5);
		delayLoop(20000 - turnOn_time);
	}
	else if(xval < ThresholdLow)
	{ 
		/* LED4 On left green: Angle : 90 to 180 degree*/
		/*Time calculated to turn motor on: time needed: 2250 to 3800*/
		turnOn_time = 2070 + (ABS(xval)*1550)/1800;
		BSP_LED_On(LED5);      
		delayLoop(turnOn_time);
		BSP_LED_Off(LED5);
		delayLoop(20000 - turnOn_time);
	}
	else//Delay made as the angle was uneven
	{ 
		BSP_LED_On(LED5);
		delayLoop(2250);
		BSP_LED_Off(LED5);
		delayLoop(20000 - 2250);
	}
  }
  else
  {
    if(yval < ThresholdLow)
    {
		/* LED6 On below blue: Angle : 0 to 90 degree*/
		/*Time calculated to turn motor on: time needed: 2250 to 700*/
		turnOn_time = 2420 - (yval*1550)/1800;
		BSP_LED_On(LED4);
		delayLoop(turnOn_time);
		BSP_LED_Off(LED4);
		delayLoop(20000 - turnOn_time);
    }
    else if(yval > ThresholdHigh)
    {
		/*LED3 On up orange: Angle : 90 to 180 degree*/
		/*Time calculated to turn motor on: time needed: 2250 to 3800*/
		turnOn_time = 2070 + (ABS(yval)*1550)/1800;	
		BSP_LED_On(LED4);
		delayLoop(turnOn_time);
		BSP_LED_Off(LED4);
		delayLoop(20000 - turnOn_time); 
    } 
    else//Delay made as the angle was uneven
    { 
		 BSP_LED_On(LED4);
		 delayLoop(2250);
		 BSP_LED_Off(LED4);
		 delayLoop(20000 - 2250);
    }
  } 
  
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);
  BSP_LED_Off(LED6);
}
//For loop used for microseconds.
void delayLoop(int16_t timeDelay)
{
	int16_t i,j;
	for(i = 0; i < timeDelay;i=i+1){
		for(j = 0; j <= 25; j=j+1);
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
