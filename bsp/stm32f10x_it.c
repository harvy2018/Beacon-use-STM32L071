/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTI
  
  AL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include <stdio.h>

#include "sx12xxEiger.h"
#include "bsp_timer.h"
#include "uartPort.h"
#include "GPS.h"
//中断服务函数接口可以在汇编启动文件中找到！！！

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

// From Joseph Yiu, minor edits by FVH
// hard fault handler in C,
// with stack frame location as input parameter
// called from HardFault_Handler in file xxx.s
void HardFault_Handler_C( unsigned int *args )
{
#if 0
    int8_t s[100];
    unsigned int stacked_r0;
    unsigned int stacked_r1;
    unsigned int stacked_r2;
    unsigned int stacked_r3;
    unsigned int stacked_r12;
    unsigned int stacked_lr;
    unsigned int stacked_pc;
    unsigned int stacked_psr;

    stacked_r0 = ( ( unsigned long) args[0] );
    stacked_r1 = ( ( unsigned long) args[1] );
    stacked_r2 = ( ( unsigned long) args[2] );
    stacked_r3 = ( ( unsigned long) args[3] );

    stacked_r12 = ( ( unsigned long) args[4] );
    stacked_lr = ( ( unsigned long) args[5] );
    stacked_pc = ( ( unsigned long) args[6] );
    stacked_psr = ( ( unsigned long) args[7] );

    sprintf( s, "\n\n[Hard fault handler - all numbers in hex]\n" );
    sprintf( s, "R0 = %x\n", stacked_r0 );
    sprintf( s, "R1 = %x\n", stacked_r1 );
    sprintf( s, "R2 = %x\n", stacked_r2 );
    sprintf( s, "R3 = %x\n", stacked_r3 );
    sprintf( s, "R12 = %x\n", stacked_r12 );
    sprintf( s, "LR [R14] = %x  subroutine call return address\n", stacked_lr );
    sprintf( s, "PC [R15] = %x  program counter\n", stacked_pc );
    sprintf( s, "PSR = %x\n", stacked_psr );
    sprintf( s, "BFAR = %x\n", ( *( ( volatile unsigned long * )( 0xE000ED38 ) ) ) );
    sprintf( s, "CFSR = %x\n", ( *( ( volatile unsigned long * )( 0xE000ED28 ) ) ) );
    sprintf( s, "HFSR = %x\n", ( *( ( volatile unsigned long * )( 0xE000ED2C ) ) ) );
    sprintf( s, "DFSR = %x\n", ( *( ( volatile unsigned long * )( 0xE000ED30 ) ) ) );
    sprintf( s, "AFSR = %x\n", ( *( ( volatile unsigned long * )( 0xE000ED3C ) ) ) );
    sprintf( s, "SCB_SHCSR = %x\n", SCB->SHCSR );
#endif
    /* Go to infinite loop when Hard Fault exception occurs */
    while( 1 )
    {
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
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	
	SysTick_ISR();
	TickCounter++;

}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
//串口1中断
void USART1_IRQHandler(void)
{
    static u8 tail[2]={0};
	static u8 count=0;
	u8 c;
	extern u8 cmd_flag;
	
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {		
		 c =USART_ReceiveData(USART1);		
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
    
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{ 	
	    //ch = USART2->DR;
        Res = USART_ReceiveData(USART2);
      //  printf("%c",Res);
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

//串口2中断
void USART3_IRQHandler(void)
{
	uint8_t ch;
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{ 	
	    //ch = USART2->DR;
        ch = USART_ReceiveData(USART3);
      //   uartPutCharIntoRcvBuffer(2,ch);
	  	//printf( "%c", ch );    //将接收到的数据直接返回打印
	}
}
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
