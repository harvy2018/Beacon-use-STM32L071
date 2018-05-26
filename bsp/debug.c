/***************************************************************************************
File name: debugr.c
Date: 2016-9-27
Author: Wangwei/GuokuYan
Description:
     This file contain all the debug command handler which capture the keyboad input and process it. 
***************************************************************************************/
#include "stm32l0xx.h"
#include <stdio.h>
#include "flashMgr.h"
#include "uartPort.h"
#include "debug.h"
#include "bsp_spi_flash.h"


#define Comx 0

u8 cmd_flag;

void debugCmdHandler()
{
    u8 dlen;
	u8 buf[12];
	u8 flashdata[4096];
	u32 raddr;
	u8 txmode;
	u8 metermode;
	
	DebugCmd *cmd;
	
    if(cmd_flag==1)
	{
		cmd_flag=0;
		dlen=uartGetAvailBufferedDataNum(Comx);
		if(dlen>10)
		{
		    uartClearRcvBuffer(Comx);
			printf("cmd buf overflow!\r\n");
		}
		else
		   uartRead(Comx,buf,dlen);
		
		cmd=(DebugCmd *)buf;
		uartClearRcvBuffer(Comx);
		
        if(cmd->cmd_tail==0x0A0D)
		{
						
			switch  (cmd->cmd_id)
			{
				case EraseFlash:           // Erase historic _total_data and _transaction_data flash space and reset system
				{
					unsigned int addr;
			
				    printf("debugCmdHandler: Erased the external flash\r\n");
				//	uf_EraseChip(FLASH_EX);
										
					printf("debugCmdHandler: Reset CPU, waiting .......\r\n");
					
					
					NVIC_SystemReset();
				}
				break;
				
				case SystemReset:
				{
					printf("debugCmdHandler: Reset CPU, waiting .......\r\n");
					bsp_DelayMS(1000);
					NVIC_SystemReset();
				}
				break;
				
				case LogReport:
				{
					
				
					
				}
				break;				
				case TxModeSet:
				{															
				   
				 
				}
				break;
				case ReadFlash:
				{															
				    extern u16 HistoryRXCnt,RX_cnt;
                    s16 RemainCnt;
					raddr=atoi(cmd->cmd_data);
				//	uf_ReadBuffer(FLASH_EX,flashdata,raddr<<12,SF_PAGE_SIZE);
                    if(raddr>=2)
                    {
                         if(raddr==2)
                         {
                             if(HistoryRXCnt+RX_cnt>128)
                             {
                                printf("HistoryRXCnt>128.Display Record:128\r\n");         
                          //      Data_Analyse(flashdata,128);
                             }
                             else
                             {
                                 printf("Display Record:%d\r\n",HistoryRXCnt+RX_cnt);         
                         //        Data_Analyse(flashdata,HistoryRXCnt+RX_cnt);
                             }
                         }
                         else
                         {

                             RemainCnt=HistoryRXCnt+RX_cnt-(raddr-2)*128;
                             
                             if(RemainCnt>128)
                             {
                                 printf("RemainCnt>128.Display Record:128\r\n");         
                          //       Data_Analyse(flashdata,128);                               
                               
                             }
                             else if(RemainCnt>0)
                             {
                                 printf("Display Record:%d\r\n", RemainCnt);         
                        //         Data_Analyse(flashdata,RemainCnt);
                                 
                             }
                             else
                                 printf("This sector hasn't data!\r\n");   
                             
                         }
                       
                    }
                    else
                    {
                        for (u16 i=0; i<(SF_PAGE_SIZE); i++) 
                        { 
                            printf("%02X ", flashdata[i]);                   
                            
                            if (!((i+1)%16) && i>0)
                            {
                                printf(": %d\r\n", i+1);
                                
                            }
                        }
                    }

				 
				}
				break;
				
				case MeterModeSet:
				{
					
				}
				break;
				
				case Upgrade:
				{
				//	USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
				//	Main_Menu();
				//	printf("Unsupported command: %04X\r\n",cmd->cmd_id);
					
				}	
				break;
				
				case TestMode:
				{
				//	Main_Menu();
				}
				break;
				
				default: 	
					printf("Unkonw supported command: %04X\r\n",cmd->cmd_id);
				break;
			
			}     
	    }
        else
		{
			 printf("cmd tail err!\r\n");
			 uartClearRcvBuffer(Comx);
		}			
	}

}        
