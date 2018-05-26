/**
  ******************************************************************************
  * @file    Templates_LL/Src/stm32l0xx.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
#include "stm32l0xx_it.h"
#include "sx12xxEiger.h"
#include "uartPort.h"
#include "GPS.h"
/** @addtogroup STM32L0xx_LL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0+ Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
    if (LL_RCC_IsActiveFlag_HSECSS() != 0)
  {
    /* Clear the flag */
    LL_RCC_ClearFlag_HSECSS();
    
    /* Handle the HSE failure directly in main.c */
  //  HSEFailureDetection_Callback(); 
  }
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
    SysTick_ISR();
    TickCounter++;
}

/**
  * Brief   This function handles RCC interrupt request 
  *         and switch the system clock to HSE.
  * Retval  None
  */
//void RCC_IRQHandler(void)
//{
//  /* Check the flag HSE ready */
//  if (LL_RCC_IsActiveFlag_HSERDY() != 0)
//  {
//    /* Clear the flag HSE ready */
//    LL_RCC_ClearFlag_HSERDY();
//    
//    /* Switch the system clock to HSE */
//    HSEReady_Callback();
//  }
//}




//串口1中断
void USART1_IRQHandler(void)
{
    static u8 tail[2]={0};
	static u8 count=0;
	u8 c;
	extern u8 cmd_flag;
	
    if(LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1))
    {		
		 c =LL_USART_ReceiveData8(USART1);		
         uartPutCharIntoRcvBuffer(0,c);
		
		if(count>0)
		  count++;
		
		if(c==0x0D)
        {			
		  tail[0]=c;
		  count++;
		} 
		
		
		if(count==2)
		{
			tail[1]=c;
			count=0;
			if(tail[0]==0x0D && tail[1]==0x0A)
			cmd_flag=1;	
		}		
		
    }
}

//串口2中断
void USART2_IRQHandler(void)
{
    extern u8 GPS_DATA_BUF[];
	uint8_t Res;
	static u16 count = 0;
    
	if(LL_USART_IsActiveFlag_ORE(USART2))
    {
        LL_USART_ReceiveData8(USART2);
        LL_USART_ClearFlag_ORE(USART2);
    }
    
    if(LL_USART_IsActiveFlag_RXNE(USART2))
	{ 	
	    //ch = USART2->DR;
        Res = LL_USART_ReceiveData8(USART2);
       // printf("%c",Res);
      //  USART_SendData(USART1,Res);
      //  uartPutCharIntoRcvBuffer(1,Res);
        
       	if(Res == '$' &&  Save_Data.isGetData == false)
        {
            count = 0;	   
            GPS_DATA_BUF[count++] = Res;            
        }
        else if(Save_Data.isGetData == false)
		{
            GPS_DATA_BUF[count++] = Res;
            
            if(GPS_DATA_BUF[0] == '$' && GPS_DATA_BUF[4] == 'M' && GPS_DATA_BUF[5] == 'C')			//确定是否收到"GPRMC/GNRMC"这一帧数据
            {
                if(Res == '\n')									   
                {           
                    Save_Data.GetDataLen=count;             
                    Save_Data.isGetData = true;
                    count = 0;                           		
                }	
                        
            }  
        }
              
        if(count >= GPS_REC_LEN_MAX)
           count = GPS_REC_LEN_MAX;
        
	}
}




void ADC1_COMP_IRQHandler(void)
{
  /* Check whether ADC group regular overrun caused the ADC interruption */
  if(LL_ADC_IsActiveFlag_OVR(ADC1) != 0)
  {
    /* Clear flag ADC group regular overrun */
    LL_ADC_ClearFlag_OVR(ADC1);
    LL_ADC_DisableIT_OVR(ADC1);
    while(1)
    LED_Blinking(LED_BLINK_ERROR);
    
  }
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
