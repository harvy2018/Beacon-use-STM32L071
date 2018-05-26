/**
  ******************************************************************************
  * @file    Templates_LL/Src/main.c 
  * @author  MCD Application Team
  * @brief   Main program body through the LL API
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
#include <stdio.h>
#include "main.h"
#include "sx12xxEiger.h"
#include "sys_config.h"
#include "info_source.h"
#include "GPS.h"

#define  LOW_SPEED
#define SAMP_COUNT     20
#define BUFFER_SIZE     128    
#define FLASH_SIZE_ADDR  0x1FF8007C
#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)
#define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)

BeaconCoordinate  gpsresp;
CheckIn checkin;

static uint16_t BufferSize = BUFFER_SIZE;			// RF buffer size
static uint8_t  Buffer[BUFFER_SIZE];				// RF buffer

u8  FW_VER[4]={1,0,0,0};
u32 HW_ID[3];
u8 MY_TEST_Msg[] = "SX1278_TEST!!!";

u32 HW_ID[3];//cpu id
u32 SecTick=0;
u16 HistoryRXCnt=0,FlashErrCnt=0;
u8  Flash_Full_Flag=0;
u16 RX_cnt=0,RX_cntT=0,seqno0=0,seqno1=0,seqno2=0,seq_index;
u16 usValue,Voltage,g_usAdcValue;
u16 year;
u8  month,day,hour,minute,sec;
u8  Buzz_Flag=0;
u8  BT_Flag=1;
u8  CHRG_Flag=0;
u8  ScreenTest_Flag=0;
u8  LowPwr_Flag=0;
u8  SOS_Flag=0;
u8  GPS_Live=0;

u32 count100ms=0;
u32 count1s=0;

u32 TXState_Count,RXState_Count;


void SystemClock_Config(void);
void Get_SerialNum(unsigned int* SerialID);
void PrintfLogo(void);

void Configure_ADC(void);
void Activate_ADC(void);
void ConversionStartPoll_ADC_GrpRegular(void);

void LED_Blinking(uint32_t Period);
/* Private functions ---------------------------------------------------------*/
 uint32_t frequency = 0;

tRadioDriver *Radio = NULL;

u16 uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE;


int main(void)
{
    u8 i;
    extern  int8_t RxPacketSnrEstimate;
    extern float RxPacketRssiValue;

    extern int temperaure;   
    extern u8 button_state[3];
    u8 buf[50]={0};
    u8 cc;
    ConcentratorRequst *dptr=(ConcentratorRequst *)Buffer;
  
  
  /* Configure Systick to 1 ms with the current frequency which should be MSI */
  frequency = __LL_RCC_CALC_MSI_FREQ(LL_RCC_MSI_GetRange());
    
  SystemClock_Config();
  uartBufferInitialize();  
  BoardInit();
  Get_SerialNum(HW_ID);
  PrintfLogo(); 

    Radio = RadioDriverInit();   
    Radio->Init();   	
    Radio->StartRx();   //RFLR_STATE_RX_INIT
    
  //  Radio->SetTxPacket(MY_TEST_Msg, sizeof(MY_TEST_Msg)); 
  /* Infinite loop */
 
  bsp_StartTimer(3,100);
    
  while (1)
  {
         
       if(bsp_CheckTimer(3)==1)
        {
            bsp_StartTimer(3,100);      
            count100ms++;
            
            Button_Detect();           
            Button_Process(button_state);              
            parseGpsBuffer();
                                 
            if(GPS_Live==1 && bsp_CheckTimer(1)==1)
            {
                GPS_PWR_OFF;
                GPS_Live=0;
                Save_Data.isParseData=false;
                Save_Data.isUsefull = false;
                bsp_StartTimer(2,120000);
            }
            
            if(GPS_Live==0 && bsp_CheckTimer(2)==1)
            {
                GPS_PWR_ON;
                GPS_Live=0;

            }
            
                
            if(count100ms % 10 == 0)
            {		    
                count1s++;
                
                           
                if((Save_Data.isParseData == true) && (Save_Data.isUsefull == false))
                {
                    LED_G_Blink;
                }
                else if((Save_Data.isParseData == true) && (Save_Data.isUsefull == true))
                {
                   if(GPS_Live==0)
                   {                       
                      bsp_StartTimer(1,10000);
                       GPS_Live=1;
                   }
                }
                
                if(count1s % 3 == 0)
                {                              
                   //  LED_R_Blink;
                    if(SOS_Flag==1)
                    {
                         MakeGpsPack(&gpsresp,RxPacketSnrEstimate,RxPacketRssiValue,SOS_Flag); 
                         
                         UART1_Send_Buf((u8*)&gpsresp, sizeof(gpsresp));                        
                         Radio->SetTxPacket((u8*)&gpsresp, sizeof(gpsresp));   //RFLR_STATE_TX_INIT 
                         LED_B_ON;
                       //   BUZZ_ON;  
                    }   
                            
                   
                }           
                                                                                                 
                if(count1s % 5 == 0)
                {
                    if(LowPwr_Flag==1)
                       LED_R_Blink;
                    
                    //  Battery_Led_Mgr();         
                }
                
                if(count1s % 25 == 0)
                {                              
                    Battery_Led_Mgr();                                    
                } 
                            
                                                                                         
            //    TickCounter =0;            
                 
            }    
      
        }
      
        switch(Radio->Process())
          {			
                case RF_RX_DONE:
                {                   
                                     
                      Radio->GetRxPacket(Buffer, (uint16_t* )&BufferSize);
                    
                    if(dptr->head.Preamble==0xFF && dptr->tail==0xF0)
                    {                                            
                        cc=CalcSum((u8*)&(dptr->head.len),dptr->head.len-1);
                        if(cc!=dptr->cc)
                        {
                           printf("cc err!\r\n"); 
                        }
                        else
                        {
                            if(dptr->head.cmd == Upload_Coordinate)
                            {
                                if(!memcmp((u8*)(dptr->id),(u8*)HW_ID,12))
                                {
                                                               
                                   MakeGpsPack(&gpsresp,RxPacketSnrEstimate,RxPacketRssiValue,1);
                                    
                                   Radio->SetTxPacket((u8*)&gpsresp, sizeof(gpsresp));   //RFLR_STATE_TX_INIT 
                                    
                                   LED_B_ON;
                                }
                                
                            }
                            else if(dptr->head.cmd == Report_BeaconID)
                            {
                                                         
                                UploadIDPack(&checkin);                                                      
                                Radio->SetTxPacket((u8*)&checkin, sizeof(checkin));   //RFLR_STATE_TX_INIT                               
                                LED_B_ON;
                                
                            }
                            
                          printf("SNR:%ddB, RSSI:%0.2fdBm\r\n",RxPacketSnrEstimate,RxPacketRssiValue);
                        }                       
                    }
                                                          
                    memset(Buffer,0,BUFFER_SIZE);			
                   
                }
                break;
                
                case RF_TX_DONE:
                {                    
                   if(SOS_Flag==0)
                     Radio->StartRx();  
                    
                     LED_B_OFF;
                   //  BUZZ_OFF;  
                    
                }
                break;
                
                default:
                break;
                               
            }
       debugCmdHandler();          
       bsp_DelayMS(1);
  }
}

/* ==============   BOARD SPECIFIC CONFIGURATION CODE BEGIN    ============== */
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 2097000
  *            HCLK(Hz)                       = 2097000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            Flash Latency(WS)              = 0
  *            Main regulator output voltage  = Scale3 mode
  * @retval None
  */

void SystemClock_Config(void)
{
  /* MSI configuration and activation */
  LL_RCC_PLL_Disable();
  /* Set new latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
     
   #ifdef LOW_SPEED
     LL_RCC_MSI_Enable();
     while(LL_RCC_MSI_IsReady() != 1) ;
    
     LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_5);  
     LL_RCC_MSI_SetCalibTrimming(0x0);
   #else
     LL_RCC_HSE_Enable();
     while(LL_RCC_HSE_IsReady() != 1) ;
   #endif

  
   /* Configure NVIC for RCC */ 

  /* Sysclk activation  */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  
   #ifdef LOW_SPEED
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_MSI) ;
   #else
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);
     while(LL_RCC_GetSysClkSource()!= LL_RCC_SYS_CLKSOURCE_STATUS_HSE) ;
   #endif

  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  
  SystemCoreClockUpdate();

  /* 1ms config with HSE 8MHz*/
//  LL_Init1msTick(SystemCoreClock/1000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(SystemCoreClock);  

  /* Enable Power Control clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE3);
  /* Disable Power Control clock */
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_PWR);
}


void Configure_ADC(void)
{
   LL_ADC_REG_InitTypeDef ADC_REG_InitStruct;
    
    /*## Configuration of GPIO used by ADC channels ############################*/
  
  /* Note: On this STM32 device, ADC1 channel 4 is mapped on GPIO pin PA.04 */ 
  
  /* Enable GPIO Clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  
  /* Configure GPIO in analog mode to be used as ADC input */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_ANALOG);
  
  /*## Configuration of NVIC #################################################*/
  /* Configure NVIC to enable ADC1 interruptions */
    
  NVIC_SetPriority(ADC1_COMP_IRQn, 2);
  NVIC_EnableIRQ(ADC1_COMP_IRQn);
  
  /*## Configuration of ADC ##################################################*/
  
  /*## Configuration of ADC hierarchical scope: common to several ADC ########*/
  
  /* STM32L0xx ADC is using a dedicated asynchronous clock derived from HSI RC oscillator 16MHz */
 // LL_RCC_HSI_Enable();

  /* Enable ADC clock (core clock) */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  
  
  /*## Configuration of ADC hierarchical scope: ADC instance #################*/
  
  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled.                                              */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Note: Call of the functions below are commented because they are       */
    /*       useless in this example:                                         */
    /*       setting corresponding to default configuration from reset state. */
    
    /* Set ADC clock (conversion clock) */
    LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV2);
//    
//    /* Set ADC data resolution */
//     LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
//    
//    /* Set ADC conversion data alignment */
//     LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
//    
//    /* Set ADC low power mode */
    LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_AUTOWAIT_AUTOPOWEROFF);
    
    /* Set ADC channels sampling time */
    /* Note: On this STM32 serie, sampling time is common to all channels     */
    /*       of the entire ADC instance.                                      */
    /*       Therefore, sampling time is configured here under ADC instance   */
    /*       scope (not under channel scope as on some other STM32 devices    */
    /*       on which sampling time is channel wise).                         */
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_39CYCLES_5);
    
  }
  
  
  /*## Configuration of ADC hierarchical scope: ADC group regular ############*/
  
  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled or enabled without conversion on going        */
  /*       on group regular.                                                  */
  if ((LL_ADC_IsEnabled(ADC1) == 0)               ||
      (LL_ADC_REG_IsConversionOngoing(ADC1) == 0)   )
  {
     LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    
    /* Set ADC group regular trigger polarity */
    // LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
    
    /* Set ADC group regular continuous mode LL_ADC_REG_CONV_CONTINUOUS*/
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_CONTINUOUS);
    
    /* Set ADC group regular conversion data transfer */
    // LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
    
    /* Set ADC group regular overrun behavior */
    LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
    
    /* Set ADC group regular sequencer */
    /* Note: On this STM32 serie, ADC group regular sequencer is              */
    /*       not fully configurable: sequencer length and each rank           */
    /*       affectation to a channel are fixed by channel HW number.         */
    /*       Refer to description of function                                 */
    /*       "LL_ADC_REG_SetSequencerChannels()".                             */
    
    /* Set ADC group regular sequencer discontinuous mode */
    // LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
    
    
    /* Set ADC group regular sequence: channel on rank corresponding to       */
    /* channel number.                                                        */
    LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_9);
  }
  
  
  /*## Configuration of ADC hierarchical scope: ADC group injected ###########*/
  
  /* Note: Feature not available on this STM32 serie */ 
    LL_ADC_EnableIT_OVR(ADC1);
  
}


void Activate_ADC(void)
{
  __IO uint32_t wait_loop_index = 0;
//  #if (USE_TIMEOUT == 1)
//  uint32_t Timeout = 0; /* Variable used for timeout management */
//  #endif /* USE_TIMEOUT */
  
  /*## Operation on ADC hierarchical scope: ADC instance #####################*/
  
  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled.                                              */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Run ADC self calibration */
    LL_ADC_StartCalibration(ADC1);
    
    /* Poll for ADC effectively calibrated */
//    #if (USE_TIMEOUT == 1)
//    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
//    #endif /* USE_TIMEOUT */
    
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
    {
//    #if (USE_TIMEOUT == 1)
//      /* Check Systick counter flag to decrement the time-out value */
//      if (LL_SYSTICK_IsActiveCounterFlag())
//      {
//        if(Timeout-- == 0)
//        {
//        /* Time-out occurred. Set LED to blinking mode */
//        LED_Blinking(LED_BLINK_ERROR);
//        }
//      }
//    #endif /* USE_TIMEOUT */
    }
    
    /* Delay between ADC end of calibration and ADC enable.                   */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }
    
    /* Note: With ADC low power mode "auto power off" enabled, operation of   */
    /*       ADC enable is managed automatically by hardware.                 */
    
    /* Enable ADC */
    if(LL_ADC_GetLowPowerMode(ADC1)==LL_ADC_LP_AUTOWAIT_AUTOPOWEROFF)
       return;
    
     LL_ADC_Enable(ADC1);    
     while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);

  }
  
  /*## Operation on ADC hierarchical scope: ADC group regular ################*/
  /* Note: No operation on ADC group regular performed here.                  */
  /*       ADC group regular conversions to be performed after this function  */
  /*       using function:                                                    */
  /*       "LL_ADC_REG_StartConversion();"                                    */
  
  /*## Operation on ADC hierarchical scope: ADC group injected ###############*/
  /* Note: Feature not available on this STM32 serie */ 
  
}

/**
  * @brief  Perform ADC group regular conversion start, poll for conversion
  *         completion.
  *         (ADC instance: ADC1).
  * @note   This function does not perform ADC group regular conversion stop:
  *         intended to be used with ADC in single mode, trigger SW start
  *         (only 1 ADC conversion done at each trigger, no conversion stop
  *         needed).
  *         In case of continuous mode or conversion trigger set to 
  *         external trigger, ADC group regular conversion stop must be added.
  * @param  None
  * @retval None
  */


void ConversionStartPoll_ADC_GrpRegular(void)
{

  
  /* Start ADC group regular conversion */
  /* Note: Hardware constraint (refer to description of the function          */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of this feature is conditioned to     */
  /*       ADC state:                                                         */
  /*       ADC must be enabled without conversion on going on group regular,  */
  /*       without ADC disable command on going.                              */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  /* Note: With ADC low power mode "auto power off" enabled, operation of     */
  /*       ADC enable is managed automatically by hardware.                   */
  /*       Check of ADC enable state using function "LL_ADC_IsEnabled()"      */
  /*       before performing ADC conversion start is not relevant.            */
  if ((LL_ADC_IsDisableOngoing(ADC1) == 0)        &&
      (LL_ADC_REG_IsConversionOngoing(ADC1) == 0)   )
  {
    LL_ADC_REG_StartConversion(ADC1);
  }
  else
  {
    /* Error: ADC conversion start could not be performed */
    LED_Blinking(LED_BLINK_ERROR);
  }
  

  
  while (LL_ADC_IsActiveFlag_EOC(ADC1) == 0)
  {
//  #if (USE_TIMEOUT == 1)
//    /* Check Systick counter flag to decrement the time-out value */
//    if (LL_SYSTICK_IsActiveCounterFlag())
//    {
//      if(Timeout-- == 0)
//      {
//      /* Time-out occurred. Set LED to blinking mode */
//      LED_Blinking(LED_BLINK_SLOW);
//      }
//    }
//  #endif /* USE_TIMEOUT */
  }
  
  /* Clear flag ADC group regular end of unitary conversion */
  /* Note: This action is not needed here, because flag ADC group regular   */
  /*       end of unitary conversion is cleared automatically when          */
  /*       software reads conversion data from ADC data register.           */
  /*       Nevertheless, this action is done anyway to show how to clear    */
  /*       this flag, needed if conversion data is not always read          */
  /*       or if group injected end of unitary conversion is used (for      */
  /*       devices with group injected available).                          */
  LL_ADC_ClearFlag_EOC(ADC1);
  
}

void LED_Blinking(uint32_t Period)
{
  /* Turn LED2 on */
  LL_GPIO_SetOutputPin(GPIOB,LED_R_PIN);
  
  /* Toggle IO in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(GPIOB,LED_R_PIN);  
    LL_mDelay(Period);
  }
}
//void HSEReady_Callback(void)
//{
//  /* Switch the system clock to HSE */
//  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE); 
//  
//  /* 1ms config with HSE 8MHz*/
//  LL_Init1msTick(HSE_VALUE);
//}
/* ==============   BOARD SPECIFIC CONFIGURATION CODE END      ============== */

void Get_SerialNum(unsigned int* SerialID)
{
    SerialID[0] = *(unsigned int*)(0x1FF80050);
    SerialID[2] = *(unsigned int*)(0x1FF80054);	
    SerialID[1] = *(unsigned int*)(0x1FF80064);

   // SerialID[3] = 0; //预留
}

void PrintfLogo(void)
{	
	
	
	printf("**************************************************\r\n");
	printf("* Release Date : %s %s\r\n", __DATE__,__TIME__);	
	printf("* FW Rev: %d.%d.%d.%d\r\n", FW_VER[0],FW_VER[1],FW_VER[2],FW_VER[3]);	
	printf("* HW ID: %08X %08X %08X\r\n",(HW_ID[0]),(HW_ID[1]),(HW_ID[2]));	
	printf("**************************************************\r\n");
    
}
/*
*********************************************************************************************************
*	函 数 名: AdcPro
*	功能说明: ADC采样处理，插入1ms systick中断进行调用
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void AdcPro(void)
{
	static uint16_t buf[SAMP_COUNT]={0};
	static u16 write=0;
	uint32_t sum,max=0,min=0xFFFFFFFF;
	u16 i;
    

	buf[write] = LL_ADC_REG_ReadConversionData12(ADC1);
      
  
  /* Computation of ADC conversions raw data to physical values               */
  /* using LL ADC driver helper macro.                                        */
  
    
    
	if (++write >= SAMP_COUNT)
	{
		write = 0;
	}

	/* 下面这段代码采用求平均值的方法进行滤波
		也可以改善下，选择去掉最大和最下2个值，使数据更加精确
	*/
	 sum = 0; 
//    for (i = 0; i < SAMP_COUNT; i++)
//    {
//         if(buf[i]>max)
//            max=buf[i];
//    }
//    
//    for (i = 0; i < SAMP_COUNT; i++)
//    {
//          if(buf[i]<min)
//            min=buf[i];
//    }
    
	for (i = 0; i < SAMP_COUNT; i++)
	{
		sum += buf[i];            
	}
//    sum-=max;
//    sum-=min;
    
	g_usAdcValue = sum / (SAMP_COUNT);	/* ADC采样值由若干次采样值平均 */
	/* 软件启动下次ADC转换 */
 //   ConversionStartPoll_ADC_GrpRegular();
}


u16 GetADC(void)
{
	uint16_t ret;

	/* 因为	g_AdcValue 变量在systick中断中改写，为了避免主程序读变量时被中断程序打乱导致数据错误，因此需要
	关闭中断进行保护 */

	/* 进行临界区保护，关闭中断 */
	__set_PRIMASK(1);  /* 关中断 */

	ret = g_usAdcValue;

	__set_PRIMASK(0);  /* 开中断 */

	return ret;
    
}



u16 getVoltage(void)
{    
    //__LL_ADC_CALC_DATA_TO_VOLTAGE(3300, uhADCxConvertedData, LL_ADC_RESOLUTION_12B);
    u16 tValue,tVoltage;
    tValue = GetADC();
  //  Voltage=((u32)usValue * 3300)*14/4095/11;
    tVoltage=__LL_ADC_CALC_DATA_TO_VOLTAGE(3300, tValue, LL_ADC_RESOLUTION_12B)*14/11;
    return tVoltage;
    
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
