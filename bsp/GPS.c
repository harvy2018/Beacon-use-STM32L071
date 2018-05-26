

#include <stdlib.h>
#include <time.h>
#include "GPS.h"
#include "device.h"
#include "sx12xxEiger.h"
#include "uartPort.h"
#include "bsp_spi_flash.h"
#include "bsp_usart.h"
#include "bsp_timer.h"

#include "info_source.h"

u8 GPS_DATA_BUF[GPS_REC_LEN_MAX];

SaveData Save_Data;


void ClrGpsData(void)
{
	Save_Data.isGetData = false;
	Save_Data.isParseData = false;
	Save_Data.isUsefull = false;
    
	memset(Save_Data.GPS_Buffer, 0, GPS_Buffer_Length);      //清空
	memset(Save_Data.UTCTime, 0, UTCTime_Length);
	memset(Save_Data.latitude, 0, latitude_Length);
	memset(Save_Data.N_S, 0, N_S_Length);
	memset(Save_Data.longitude, 0, longitude_Length);
	memset(Save_Data.E_W, 0, E_W_Length);
	
}

void errorLog(int num)
{
	
	while (1)
	{
	  	printf("ERROR%d\r\n",num);
	}
}

void parseGpsBuffer()
{
	char *subString;
	char *subStringNext;
	char i = 0;   
    
	if (Save_Data.isGetData)
	{
        
        memcpy(Save_Data.GPS_Buffer,GPS_DATA_BUF,Save_Data.GetDataLen);
        Save_Data.isGetData = false;
		
		printf(Save_Data.GPS_Buffer);
		
		for (i = 0 ; i <= 6 ; i++)
		{
			if (i == 0)
			{
				if ((subString = strstr(Save_Data.GPS_Buffer, ",")) == NULL)
					errorLog(1);	//解析错误
			}
			else
			{
				subString++;
				if ((subStringNext = strstr(subString, ",")) != NULL)
				{
					char usefullBuffer[2]; 
					switch(i)
					{
						case 1:memcpy(Save_Data.UTCTime, subString, subStringNext - subString);break;	//获取UTC时间
						case 2:memcpy(usefullBuffer, subString, subStringNext - subString);break;	//获取UTC时间
						case 3:memcpy(Save_Data.latitude, subString, subStringNext - subString);break;	//获取纬度信息
						case 4:memcpy(Save_Data.N_S, subString, subStringNext - subString);break;	//获取N/S
						case 5:memcpy(Save_Data.longitude, subString, subStringNext - subString);break;	//获取经度信息
						case 6:memcpy(Save_Data.E_W, subString, subStringNext - subString);break;	//获取E/W

						default:break;
					}

					subString = subStringNext;
					Save_Data.isParseData = true;
                    
					if(usefullBuffer[0] == 'A')
						Save_Data.isUsefull = true;
					else if(usefullBuffer[0] == 'V')
						Save_Data.isUsefull = false;

				}
				else
				{
					errorLog(2);	//解析错误
				}
			}


		}
	}
}

void GetGpsData(u32 *latitude,u32 *longitude,u8 sos)
{
	
    float lat_f,lot_f;
    
    if (Save_Data.isParseData || sos)
	{
		Save_Data.isParseData = false;
		
		if(Save_Data.isUsefull || sos)
		{
			Save_Data.isUsefull = false;
            
			lat_f=atof(Save_Data.latitude);
            *latitude=(u32)(lat_f*10000);
            
            lot_f=atof(Save_Data.longitude);
            *longitude=(u32)(lot_f*10000);
                                         
		}
		else
		{
			printf("GPS DATA is not usefull!\r\n");
            *latitude=0;
            *longitude=0;
		}
        
   //    ClrGpsData();
		
	}
    else
    {
         *latitude=0;
         *longitude=0;
    }
    
}

void MakeGpsPack(BeaconCoordinate* gpsresptr,s8 SNR,s16 RSSI,u8 sos)
{
    extern u32 HW_ID[3];
    
    gpsresptr->head.Preamble=0xFF;
    gpsresptr->head.cmd=Upload_Coordinate;
    gpsresptr->head.len=sizeof(BeaconCoordinate)-2;
    
    memcpy((u8*)(gpsresptr->id),(u8*)HW_ID,12);
    GetGpsData(&(gpsresptr->latitude),&(gpsresptr->longitude),sos);
    gpsresptr->voltage=getVoltage();
    gpsresptr->Rssi=(u16)RSSI;
    gpsresptr->Snr=SNR;
          
    gpsresptr->cc=CalcSum((u8*)&(gpsresptr->head.len),sizeof(BeaconCoordinate)-3);
       
    gpsresptr->tail=0xF0;
    
}


void UploadIDPack(CheckIn* idptr)
{
    extern u32 HW_ID[3];
    
    idptr->head.Preamble=0xFF;
    idptr->head.cmd=Report_BeaconID;
    idptr->head.len=sizeof(CheckIn)-2;
    memcpy((u8*)(idptr->id),(u8*)HW_ID,12);
    idptr->voltage=getVoltage();  
  
    idptr->cc=CalcSum((u8*)&(idptr->head.len),sizeof(CheckIn)-3);
    
    idptr->tail=0xF0;
}

void printGpsBuffer()
{
	if (Save_Data.isParseData)
	{
		Save_Data.isParseData = false;
		
		printf("Save_Data.UTCTime = ");
		printf(Save_Data.UTCTime);
		printf("\r\n");

		if(Save_Data.isUsefull)
		{
			Save_Data.isUsefull = false;
			printf("Save_Data.latitude = ");
			printf(Save_Data.latitude);
			printf("\r\n");


			printf("Save_Data.N_S = ");
			printf(Save_Data.N_S);
			printf("\r\n");

			printf("Save_Data.longitude = ");
			printf(Save_Data.longitude);
			printf("\r\n");

			printf("Save_Data.E_W = ");
			printf(Save_Data.E_W);
			printf("\r\n");
		}
		else
		{
			printf("GPS DATA is not usefull!\r\n");
		}
		
	}
}

u8 CalcSum(u8* ptr,u8 len)
{
    u8 cc=0;
    for(u8 i=0;i<len;i++)
    {
        cc+=*ptr;
        ptr++;
    }
    
    return cc;
}




