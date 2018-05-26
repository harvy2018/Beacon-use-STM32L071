/**
  ******************************************************************************
  * @file    main.c
  * @author  wangwei
  * @version V1.0
  * @date    2016年10月17日00:27:02
  * @brief   串口中断接收测试
  ******************************************************************************
  */ 
#include <stdlib.h>
#include "time.h"
#include "sys_config.h"
#include "sx12xxEiger.h"
#include "18B20.h"
#include "bsp_spi_flash.h"
#include "info_source.h"
#include "OLED_I2C.h"
#include "bsp_timer.h"
#include "uartPort.h"
#include "sx1276-LoRa.h"
//#include "codetab.h"
#include "flashmgr.h"
#include "menu.h"
#include "GPS.h"

#define BUFFER_SIZE     128                          // Define the payload size here
#define SAMP_COUNT     20
//#define SX1278_RX  1
//#define SX1278_TX   1




u32 E=1163789050;
u32 N=400279950;
u32 Unix_Time=0;

u8 FW_VER[4]={1,1,0,0};	


static uint16_t BufferSize = BUFFER_SIZE;			// RF buffer size
static uint8_t  Buffer[BUFFER_SIZE];				// RF buffer
static uint8_t EnableMaster = true; 				// Master/Slave selection

tRadioDriver *Radio = NULL;
uint16_t g_usAdcValue;	/* ADC 采样值的平均值 */


uint8_t MY_TEST_Msg[] = "SX1278_TEST!!!";

u32 HW_ID[3];//cpu id
u32 SecTick=0;
u16 HistoryRXCnt=0,FlashErrCnt=0;
u8  Flash_Full_Flag=0;
u16 RX_cnt=0,RX_cntT=0,seqno0=0,seqno1=0,seqno2=0,seq_index;
u16 usValue,Voltage;
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

BeaconCoordinate  gpsresp;
CheckIn checkin;

void PrintfLogo(void);
void Get_SerialNum(unsigned int* SerialID);
u16 GetADC(void);
u16 getVoltage(void);

void BT_Rx_Process(void);
void update_time(u32 clock);

int main(void)
{
	 
    u8 i;
    extern  int8_t RxPacketSnrEstimate;
    extern float RxPacketRssiValue;

    extern int temperaure;   
    extern u8 button_state[3];

    u8 buf[50]={0};

    u8 header[3]={0xAA,0x0F,1};
    u8 tail[2]={0,0x16};

   
    u8 cc=0;
    u8 sx1276_state=0;
    u8 CntS=0;
    

    ConcentratorRequst *dptr=(ConcentratorRequst *)Buffer;


    uartBufferInitialize();
    BoardInit();
    
    Get_SerialNum(HW_ID);
    PrintfLogo();	
             
   // PositionData_Check();
        
    BUZZ_ON;   
    bsp_DelayMS(200);
    BUZZ_OFF;  

    Radio = RadioDriverInit();   
    Radio->Init();   	
    Radio->StartRx();   //RFLR_STATE_RX_INIT


	while(1)
	{
               	       
        if(TickCounter % 100 == 0)
        {
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
                bsp_StartTimer(2,60000);
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
                    if(LowPwr_Flag==1)
                       LED_R_Blink;
                    
                      Battery_Led_Mgr();         
                }
                
                if(count1s % 25 == 0)
                {                              
                   // Battery_Led_Mgr();                                    
                } 
                
                if(count1s % 10 == 0)
                {                              
                    if(SOS_Flag==1)
                    {
                         MakeGpsPack(&gpsresp,RxPacketSnrEstimate,RxPacketRssiValue,SOS_Flag);                               
                         Radio->SetTxPacket((u8*)&gpsresp, sizeof(gpsresp));   //RFLR_STATE_TX_INIT 
                         LED_B_ON;
                    }                        
                   
                } 
                                                                                         
                TickCounter =0;            
                 
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
                                                               
                                   MakeGpsPack(&gpsresp,1,RxPacketSnrEstimate,RxPacketRssiValue);
                                    
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
                    
                }
                break;
                
                default:
                break;
                               
            }
         
            debugCmdHandler();                             
            bsp_DelayMS(1);
            
      }
	
    }
    
}

void Get_SerialNum(unsigned int* SerialID)
{
    SerialID[0] = *(unsigned int*)(0x1FFFF7E8);
    SerialID[2] = *(unsigned int*)(0x1FFFF7F0);	
    SerialID[1] = *(unsigned int*)(0x1FFFF7EC);

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


void ADC_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	__IO uint16_t ADCConvertedValue;

    /* 使能 ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOB, ENABLE);

	/* 配置PC4为模拟输入(ADC Channel9) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 配置ADC1, 不用DMA, 用软件触发 */
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* 配置ADC1 规则通道14 channel14 configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_55Cycles5);

	/* 使能ADC1 DMA功能 */
	ADC_DMACmd(ADC1, ENABLE);

	/* 使能 ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* 使能ADC1 复位校准寄存器 */
	ADC_ResetCalibration(ADC1);
	/* 检查ADC1的复位寄存器 */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* 启动ADC1校准 */
	ADC_StartCalibration(ADC1);
	/* 检查校准是否结束 */
	while(ADC_GetCalibrationStatus(ADC1));

	/* 软件启动ADC转换 */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
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
    

	buf[write] = ADC_GetConversionValue(ADC1);
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
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	/* 软件启动下次ADC转换 */
    
}

/*
*********************************************************************************************************
*	函 数 名: GetADC
*	功能说明: 读取ADC采样的平均值
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/


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
    u16 usValue,Voltage;
    usValue = GetADC();
    Voltage=((u32)usValue * 3300)*14/4095/11;
    return Voltage;
    
}



void BT_Resp(u8 flag)
{
	 
	 u8 buf[3]={0xFF,0,0xF0};
	 
	 buf[1]=flag;
	 uartWriteBuffer(UART_BT,buf,3);
	 uartSendBufferOut(UART_BT);
}



void BT_Rx_Process(void)
{
	u8 dlen;
	u8 buf[50];
	RealTime *time;
	dlen=uartGetAvailBufferedDataNum(UART_BT);
	if(dlen>=6)
	{
		uartRead(UART_BT,buf,dlen);
		time=(RealTime *)buf;
		if(time->head.Preamble==0xFF && time->head.cmd==BT_Sync_Time_CMD)
		{
		   Unix_Time=__REV(time->timestamp)+LOCAL_TIME_ZONE_SECOND_OFFSET;
		   BT_Resp(0);
		}
		
	}
	
}

void update_time(u32 clock)
{
	struct tm *t;
    u32 timestick;
    timestick=clock;
    t = localtime((time_t *)&timestick);
	
	year=t->tm_year+REFERENCE_YEAR_BASE;
    month=t->tm_mon+1;
    day=t->tm_mday;
    hour=t->tm_hour;
    minute=t->tm_min;
    sec=t->tm_sec;
	
}

/*********************************************END OF FILE**********************/
