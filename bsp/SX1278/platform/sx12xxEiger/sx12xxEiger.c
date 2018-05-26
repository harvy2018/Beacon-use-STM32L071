/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       sx12xxEiger.c
 * \brief        
 *
 * \version    1.0
 * \date       Nov 21 2012
 * \author     Miguel Luis
 */
#include <stdint.h> 
#include "spi.h"
#include "sx12xxEiger.h"
#include "uartPort.h"
#include "bsp_spi_flash.h"
#include "bsp_usart.h"
#include "bsp_timer.h"
#include "sx1276-LoRa.h"
#include "OLED_I2C.h"
#include "device.h"
#include "flashMgr.h"
#include "info_source.h"
#include "GPS.h"

extern u8 Battery0[];
extern u8 Battery1[];
extern u8 Battery2[];
extern u8 Battery3[];
extern u8 Battery4[];
extern u8 BatteryC[];


u8 BT_SLEEP[]="AT+SLEEP\r\n";
u8 BT_WAKE[]="AT+WAKE\r\n";
u8 BT_AT[]="AT\r\n";

u8 BT_STATUS=0;//0:WAKE;1:SLEEP

u8 button_state[3]={0};

// System tick (1ms)
volatile uint32_t TickCounter = 0;

void BoardInit( void )
{
		
    	bsp_InitTimer();
        GPIO_Configuration(); 
		USART1_Config();
		USART2_Config();
		//USART3_Config();

		SpiInit();//lora spi	
        Configure_ADC();
        Activate_ADC();
        ConversionStartPoll_ADC_GrpRegular(); 
	   // sf_InitHard();	/* ≥ı ºªØSPI flash */
      //  I2C_Configuration();
	  //  OLED_Init();   
       // ADC_Configuration();
	  //  TIMX_Init(CALC_TYPE_US);  
      //  NVIC_Configuration();    
}

void Delay (uint32_t delay)
{
    // Wait delay ms
    uint32_t startTick = TickCounter;
    while( ( TickCounter - startTick ) < delay ); 
    
}

//void LongDelay (uint8_t delay)
//{
//    uint32_t longDelay;
//    uint32_t startTick;

//    longDelay = delay * 1000;

//    // Wait delay s
//    startTick = TickCounter;
//    while( ( TickCounter - startTick ) < longDelay );   
//}


void GPIO_Configuration(void)
{ 
    
        LL_GPIO_InitTypeDef gpio_initstruct;       
    
        LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
        LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
        LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
    
        gpio_initstruct.Pin        = BUZZ_PIN|LED_G_PIN|LED_B_PIN|LED_R_PIN;
        gpio_initstruct.Mode       = LL_GPIO_MODE_OUTPUT;
        gpio_initstruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;//LL_GPIO_SPEED_FREQ_HIGH;
        gpio_initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        gpio_initstruct.Pull       = LL_GPIO_PULL_NO;//LL_GPIO_PULL_NO; 
        LL_GPIO_Init(GPIOB, &gpio_initstruct);
    
    
        gpio_initstruct.Pin        = GPS_PWR_PIN|BT_PWR_PIN;
        gpio_initstruct.Mode       = LL_GPIO_MODE_OUTPUT;
        gpio_initstruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
        gpio_initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        gpio_initstruct.Pull       = LL_GPIO_PULL_NO;   
        LL_GPIO_Init(GPIOA, &gpio_initstruct);
        
    
        gpio_initstruct.Pin        = CHRG_PIN|BUTTON_DOWN_PIN;
        gpio_initstruct.Mode       = LL_GPIO_MODE_INPUT;
        gpio_initstruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
        gpio_initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        gpio_initstruct.Pull       = LL_GPIO_PULL_NO;   
        LL_GPIO_Init(GPIOA, &gpio_initstruct);
        
    
        gpio_initstruct.Pin        =  BUTTON_UP_PIN|BUTTON_MID_PIN;
        gpio_initstruct.Mode       = LL_GPIO_MODE_INPUT;
        gpio_initstruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
        gpio_initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        gpio_initstruct.Pull       = LL_GPIO_PULL_NO; 
        LL_GPIO_Init(GPIOB, &gpio_initstruct);
    
        BUZZ_ON;
		LED_ON;
        bsp_DelayMS(200);
        
        LED_OFF;       
        
		BUZZ_OFF;
      //  GPS_PWR_OFF;
        GPS_PWR_ON;
        
        BT_PWR_OFF;

}
/*
void NVIC_Configuration()
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
	
	
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;  // Tax uart port 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;  // Tax uart port 1
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		

 
}
*/
u8 CHRG_Detect(void)
{
    u8 ret;   
    //ret = GPIO_ReadInputDataBit(CHRG_PORT, CHRG_PIN);     
    ret =LL_GPIO_IsInputPinSet(CHRG_PORT, CHRG_PIN);
}



void Button_Detect(void)
{
        
    u8 button[3]={0};

 
    for(u8 i=0;i<7;i++)
    {   
         button[0]+= LL_GPIO_IsInputPinSet( BUTTON_UP_PORT,   BUTTON_UP_PIN );     
         button[1]+= LL_GPIO_IsInputPinSet( BUTTON_MID_PORT,  BUTTON_MID_PIN );
         button[2]+= LL_GPIO_IsInputPinSet( BUTTON_DOWN_PORT, BUTTON_DOWN_PIN );
       //  bsp_DelayMS(10);
    }
    
    for(u8 i=0;i<3;i++)
    {
        if(button[i]<=2)
           button_state[i]=1; 
        else
           button_state[i]=0;  
    }
    
}
    

void Button_Process(u8 *ButtonState)  
{
    extern u32 SecTick;
    extern int temperaure;  
    extern u16 seq_index,RX_cnt;    
    extern u16 Voltage,HistoryRXCnt,FlashErrCnt;
    extern u8 SOS_Flag;   
    u8 dlen;
    u8 buf[128]={0};
    static u8 GPSFlag=0;
    u8 PageNum=4;
  //  static u8 ButtonStatePre[3]={0};
    u8 RFStatus=0;
    extern tRadioDriver *Radio; 

    RFStatus=SX1276LoRaGetRFState();
    
    if(ButtonState[OK]==1 && ButtonState[BACK]==0 && ButtonState[NEXT]==0)
    {
            
           SOS_Flag=!SOS_Flag;
            BUZZ_ON;   
            bsp_DelayMS(200);
            //Delay(200);
            BUZZ_OFF;  
       
    }
    else if((ButtonState[BACK]==1 || ButtonState[NEXT]==1) && (ButtonState[OK]==0) && (RFStatus!=RFLR_STATE_TX_RUNNING))
    {
        CheckIn idpack; 
        UploadIDPack(&idpack);
        Radio->SetTxPacket((u8*)&idpack, sizeof(idpack));
         LED_B_ON;
              
    }
    else if(ButtonState[BACK]==1 && ButtonState[NEXT]==1 && ButtonState[OK]==1)
    {
         if(GPSFlag==0)
         {
           GPS_PWR_OFF;
           GPSFlag=1;             
         }
         else
         {
             GPS_PWR_ON;
             GPSFlag=0;    
         }
    }         
    
}

void Battery_Led_Mgr(void)
{
     u16 Voltage;
    
    extern u8 LowPwr_Flag;
    
    Voltage=getVoltage();
  
            
    if(Voltage>=Battery_WARN && Voltage<Battery_0BAR)
    {
        LowPwr_Flag=1;
    }
//    else if(Voltage<Battery_WARN)
//    {       
//        LED_OFF;
//        SX1276LoRaSetRFState(RFLR_STATE_IDLE);
//        SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP);
//    }
    else
        LowPwr_Flag=0;
          
    printf("Voltage:%d\r\n",Voltage);
}


