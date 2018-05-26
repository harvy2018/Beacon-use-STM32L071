

#include <stdio.h>
#include <time.h>
#include "menu.h"
#include "device.h"
#include "sx12xxEiger.h"
#include "uartPort.h"
#include "bsp_spi_flash.h"
#include "bsp_usart.h"
#include "bsp_timer.h"
#include "sx1276-LoRa.h"
#include "OLED_I2C.h"
#include "flashmgr.h"
#include "info_source.h"


extern u8 BT_SLEEP[];
extern u8 BT_WAKE[];
extern u8 BT_AT[];
extern u8 Buzz_Flag;
extern u8 BT_BMP[];
extern u8 Wireless_BMP[];
extern u8 BT_Flag;
u8 Erase_Flag=0;

void RFSwitchON(void)
{
   SX1276LoRaSetRFState(RFLR_STATE_RX_INIT); 
   OLED_ShowStr(0,0,"RF ON",2);    
   OLED_DrawBMP(68,0,84,2,Wireless_BMP); 
}

void RFSwitchOFF(void)
{
   SX1276LoRaSetRFState(RFLR_STATE_IDLE); 
   OLED_ShowStr(0,0,"RF OFF",2);  
   OLED_ClearBMP(68,0,84,2);    
}

void BUZZSwitchON(void)
{
    Buzz_Flag=1;
    OLED_ShowStr(0,0,"BUZZ ON",2);    
    BUZZ_ON;   
    bsp_DelayMS(100);
    BUZZ_OFF;  
}

void BUZZSwitchOFF(void)
{
    Buzz_Flag=0;
    OLED_ShowStr(0,0,"BUZZ OFF",2);    
    BUZZ_ON;   
    bsp_DelayMS(100);
    BUZZ_OFF; 
}

void BTSwitchON(void)
{
  u8 dlen;
  u8 buf[50];
 
    
  for(u8 t=0;t<60;t++) 
  {
      uartWriteBuffer(UART_BT,BT_WAKE,strlen(BT_WAKE));
      uartSendBufferOut(UART_BT);
  }
   bsp_DelayMS(500);

  dlen=uartGetAvailBufferedDataNum(UART_BT);
  if(dlen>=2)  
  {                      
    uartRead(UART_BT,buf,dlen);
    if(buf[dlen-5]=='O' && buf[dlen-4]=='K')
    {
         OLED_ShowStr(0,0,"BT ON",2); 
         
         OLED_DrawBMP(87,0,95,2,BT_BMP);
         BT_Flag=1;
    }
        
  }
}

void BTSwitchOFF(void)
{
      uartWriteBuffer(UART_BT,BT_SLEEP,strlen(BT_SLEEP));
      uartSendBufferOut(UART_BT);
      OLED_ShowStr(0,0,"BT OFF",2); 
      OLED_ClearBMP(87,0,95,2);
      BT_Flag=0;
}

void DeviceInfoShow(u8 *ButtonState)
{
    extern u32 SecTick;
    extern int temperaure;  
    extern u16 seq_index,RX_cnt;    
    extern u16 Voltage,HistoryRXCnt,FlashErrCnt;
    extern u8 FW_VER[4];
    
    u8 dlen;
    u8 buf[50]={0};
    static u8 sysFlag=0;
    u8 PageNum=4;
    
    
    if(ButtonState[NEXT]==1)
        sysFlag=!sysFlag;
   
        
    if(sysFlag==0)
    {
       if(temperaure<-1000000 || temperaure>1000000)
       {
           OLED_ShowStr(0,2,"Temp:---",2); 
       }           
       else
       {
        sprintf(buf,"Temp:%0.2f'",(float)temperaure/10000);
        OLED_ShowStr(0,2,buf,2); 
       }
       
        sprintf(buf,"RunTime:%dmin",SecTick/60); 
        OLED_ShowStr(0,4,buf,2); 
        
        sprintf(buf,"Voltage:%dmV",Voltage);             
        OLED_ShowStr(0,6,buf,2);
    }
    else
    {
        sprintf(buf,"FW Rev:%d.%d.%d.%d", FW_VER[0],FW_VER[1],FW_VER[2],FW_VER[3]);	   
        OLED_ShowStr(0,2,buf,2);
//        OLED_ShowStr(0,4,"Release Date:",2);      
//        sprintf(buf,"%s", __DATE__);
//        OLED_ShowStr(0,6,buf,2);
        
          Erase_Flag=0;
       
    }
    
}

void ClearData(u8 *ButtonState)
{
     FlashSectionStruct *pFlash;
     DataStorage_Info datastorage_info;
     if(Erase_Flag==0)
     {
         OLED_ShowStr(0,2,"Erase Data...",2); 
         
         pFlash=&flashSectionStruct[FLASH_SECTION_DATA_INFO];
         
         uf_EraseChip(FLASH_EX);
         
         uf_WriteBuffer(FLASH_EX,FlashVer,pFlash->base,sizeof(FlashVer));
         OLED_CLS_B();
         OLED_ShowStr(0,2,"All Record Data Cleared!",2); 
         Erase_Flag=1;
     }
     else
         OLED_ShowStr(0,2,"All Record Data Cleared!",2);  
}


void Record_Review(u8 *ButtonState)
{
    
    FlashSectionStruct *pFlash,*pFlash1;
    Watch_Info_Data Wdata;
    
    u8 buf[128]={0};
    u8 j=0;   
	struct tm *t;
    u32 timestick;
    static u32 read_addr=0;
   
     pFlash=uf_RetrieveFlashSectionInfo(FLASH_SECTION_POSITION_DATA);
    
     if(ButtonState[OK]==1)
         read_addr=(pFlash->wptr-sizeof(Watch_Info_Data));
     else if(ButtonState[NEXT]==1)
         read_addr-=sizeof(Watch_Info_Data);
     
    
    uf_ReadBuffer(FLASH_EX,(u8*)&Wdata,read_addr,sizeof(Watch_Info_Data));
		
	timestick=Wdata.timestamp;
	t = localtime((time_t *)&timestick);
     
	if(t->tm_year+REFERENCE_YEAR_BASE!=2106)	
	{
        sprintf(buf,"%d-%02d-%02d",t->tm_year+REFERENCE_YEAR_BASE,t->tm_mon+1,t->tm_mday);	
        OLED_ShowStr(0,0,buf,1);
        
        sprintf(buf," %02d:%02d:%02d",t->tm_hour,t->tm_min,t->tm_sec);	
        OLED_ShowStr(0,1,buf,1);
        
        memset(buf,0,sizeof(buf));  

        for(u8 i=0;i<9;i++) 
        {    
          sprintf(buf+j,"%02X",Wdata.ID[i]);
          j+=2;   
        }
        OLED_ShowStr(0,2,"ID:",1);
        OLED_ShowStr(18,2,buf,1);
                                 
        sprintf(buf,"RxCnt: %d",Wdata.RxCnt);     
        OLED_ShowStr(0,3,buf,1);    

        sprintf(buf,"SNR: %ddB",Wdata.Snr);                            
        OLED_ShowStr(0,4,buf,1);  

        sprintf(buf,"RSSI:%ddBm",Wdata.Rssi);
        OLED_ShowStr(0,5,buf,1);
        memset(buf,0,sizeof(buf));

        sprintf(buf,"E: %f",(float)Wdata.longitude/10000000);
        OLED_ShowStr(0,6,buf,1);
        sprintf(buf,"N: %f",(float)Wdata.latitude/10000000);            
        OLED_ShowStr(0,7,buf,1);
    }
    else
    {
          OLED_ShowStr(32,4,"No Data!",2);
    }
  
}

void Record_Review_B(u8 *ButtonState)
{
    
    FlashSectionStruct *pFlash,*pFlash1;
    Watch_Info_Data Wdata;
    
    u8 buf[128]={0};
    u8 j=0;   
	struct tm *t;
    u32 timestick;
    static u32 read_addr=0;
   
     pFlash=uf_RetrieveFlashSectionInfo(FLASH_SECTION_POSITION_DATA);
    
     if(pFlash->wptr==pFlash->base)
     {       
         OLED_ShowStr(32,4,"No Data!",2); 
         return;
     }
    
     
     if(ButtonState[BACK]==1 && read_addr==0)
     {
         read_addr=(pFlash->wptr-sizeof(Watch_Info_Data));
     }
     else if(ButtonState[BACK]==1)
     {
         read_addr-=sizeof(Watch_Info_Data);
         if(read_addr<pFlash->base)
             read_addr+=sizeof(Watch_Info_Data);
         
     }
     else if(ButtonState[NEXT]==1 && read_addr==0)
     {
         read_addr=(pFlash->wptr-sizeof(Watch_Info_Data));
     }
     else if(ButtonState[NEXT]==1)
     {
        
         if(read_addr==(pFlash->wptr-sizeof(Watch_Info_Data)))
         {
             read_addr=read_addr;
         }
         else
         {
              read_addr+=sizeof(Watch_Info_Data);
         }
                         
     }
        
     if(ButtonState[BACK]==1 || ButtonState[NEXT]==1)
     {
       
        uf_ReadBuffer(FLASH_EX,(u8*)&Wdata,read_addr,sizeof(Watch_Info_Data));
            
        timestick=Wdata.timestamp;
        t = localtime((time_t *)&timestick);
         
        if(t->tm_year+REFERENCE_YEAR_BASE!=2106)	
        {
            sprintf(buf,"%d-%02d-%02d",t->tm_year+REFERENCE_YEAR_BASE,t->tm_mon+1,t->tm_mday);	
            OLED_ShowStr(0,0,buf,1);
            
            sprintf(buf," %02d:%02d:%02d",t->tm_hour,t->tm_min,t->tm_sec);	
            OLED_ShowStr(0,1,buf,1);
            
            memset(buf,0,sizeof(buf));  

            for(u8 i=0;i<9;i++) 
            {    
              sprintf(buf+j,"%02X",Wdata.ID[i]);
              j+=2;   
            }
            OLED_ShowStr(0,2,"ID:",1);
            OLED_ShowStr(18,2,buf,1);
                                     
            sprintf(buf,"RxCnt: %d",Wdata.RxCnt);     
            OLED_ShowStr(0,3,buf,1);    

            sprintf(buf,"SNR: %ddB",Wdata.Snr);                            
            OLED_ShowStr(0,4,buf,1);  

            sprintf(buf,"RSSI:%ddBm",Wdata.Rssi);
            OLED_ShowStr(0,5,buf,1);
            memset(buf,0,sizeof(buf));

            sprintf(buf,"E: %f",(float)Wdata.longitude/10000000);
            OLED_ShowStr(0,6,buf,1);
            sprintf(buf,"N: %f",(float)Wdata.latitude/10000000);            
            OLED_ShowStr(0,7,buf,1);
        }
        else
        {
              OLED_ShowStr(32,4,"No Data!",2);
              read_addr+=sizeof(Watch_Info_Data);
        }
    }
  
}

void Record_Statis(u8 *ButtonState)
{
    u8 buf[50]={0};  
    extern u16 HistoryRXCnt,FlashErrCnt,RX_cnt;
    
    sprintf(buf,"H_RXCnt:%d",HistoryRXCnt);
    OLED_ShowStr(0,2,buf,2); 
    memset(buf,0,50);
    sprintf(buf,"C_RXCnt:%d",RX_cnt);             
    OLED_ShowStr(0,4,buf,2);
    memset(buf,0,50);
    sprintf(buf,"FlashErrCnt:%d",FlashErrCnt);
    OLED_ShowStr(0,6,buf,2);
}

void ScreenTest(u8 *ButtonState)
{
    extern u8 ScreenTest_Flag;
    static u8 dispFlag=0;
    
    ScreenTest_Flag=1;  
    
    if(ButtonState[NEXT]==1)
        dispFlag=!dispFlag;
    
    
    if(dispFlag==0)
        OLED_Fill(0xFF);    
    else
        OLED_CLS();//ÇåÆÁ   

    
}
