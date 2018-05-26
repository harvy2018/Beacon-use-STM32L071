#ifndef __GPS_H
#define __GPS_H
#include "stdio.h"	
#include "string.h"
#include "stm32l0xx.h"	
#include "info_source.h"
//�������鳤��
#define GPS_Buffer_Length 80
#define UTCTime_Length 11
#define latitude_Length 11
#define N_S_Length 2
#define longitude_Length 12
#define E_W_Length 2 

#define GPS_REC_LEN_MAX  200

typedef struct _SaveData 
{
	char GPS_Buffer[GPS_Buffer_Length];
	char isGetData;		//�Ƿ��ȡ��GPS����
	char isParseData;	//�Ƿ�������
	char UTCTime[UTCTime_Length];		//UTCʱ��
	char latitude[latitude_Length];		//γ��
	char N_S[N_S_Length];		//N/S
	char longitude[longitude_Length];		//����
	char E_W[E_W_Length];		//E/W
	char isUsefull;		//��λ��Ϣ�Ƿ���Ч
    u8   GetDataLen;//�յ����������ݳ���
}SaveData;

extern u8 GPS_DATA_BUF[];
extern SaveData Save_Data;
void parseGpsBuffer(void);
void GetGpsData(u32 *latitude,u32 *longitude,u8 sos);
void MakeGpsPack(BeaconCoordinate* gpsresptr,s8 SNR,s16 RSSI,u8 sos);
void UploadIDPack(CheckIn* idptr);
u8 CalcSum(u8* ptr,u8 len);
#endif