
#ifndef _INFOSOURCE_H_
#define _INFOSOURCE_H_

#include "sys_config.h"
#define BT_Sync_Time_CMD            0x01
#define BT_BeaconID_Notice_CMD      0x02
#define BT_BeaconID_Dispatch_CMD    0x03
#define BT_BeaconInfo_Upload_CMD    0x04
#define BT_Beacon_CheckIn_CMD       0x05
#define BT_Beacon_CheckOut_CMD      0x06
#define BT_Beacon_CheckOutAll_CMD   0x07

#define Upload_Coordinate  0x11
#define Report_BeaconID    0x12

typedef enum _Watch_Type
{
    WATCH_15,
    WATCH_WA,
    WATCH_X,
    WATCH_MAX   
    
}Watch_Type;


#pragma pack(4)


typedef struct _Watch_15
{
     u16 Header;
	 u8 Length;
	 u8 Mask[9];
	 u8 Random[2];
	 u8 CMD;
	 u8 SensorType;
	 u32 longitude;
	 u32 latitude;
	 u8 HeartRatio;
	 u16 CRC16; 
	
}Watch_15;

typedef struct _Watch_WangAn
{
     u8 Header;
	 u32 ID;
	 u32 longitude;
	 u32 latitude;
	 u8  HeartRatio;
	 u16 Tail; 
	
}Watch_WangAn;

typedef struct _Watch_Info_TX
{
     u8  Preamble;
	 u8  ID[9];
	 u32 longitude;//����
	 u32 latitude;//γ��
	 s8  Snr;
	 s16 Rssi; 
     u16 RxCnt; //��ID��Ϣ���ռ���
	 u16 TotalRxCnt;//�ܽ��ռ���
     u8  cc;
     u8  Tail;
	
}Watch_Info_TX;

typedef struct _Watch_Info_Data
{
	 u8  Head[3];
     u8  ID[9];	 
	 u32 longitude;//����
	 u32 latitude;//γ��
	 s8  Snr;
	 s16 Rssi; 
     u16 RxCnt; //��ID��Ϣ���ռ���
	 u16 TotalRxCnt;//�ܽ��ռ���
     u32 timestamp;
     u8  cc;
    
}Watch_Info_Data;


typedef struct _DataStorage_Info
{
    
    u8   FlashVer[4];
  //  u16  TotalRxCnt;//�ܽ��ռ���
    
}DataStorage_Info;

/*------------------------------------------------------------------------------------------------*/


typedef struct _BTHeader
{
    u8  Preamble;
    u8  len;
    u8  cmd; 
    u8  ver;
   
}BTHeader;


typedef struct _RealTime
{
    
    BTHeader head;
	u32 timestamp;
    u8 cc;
	u8 Tail;
  
}RealTime;


typedef struct _ConcentratorRequst
{
    
    BTHeader head;
	u32 id[3];
    u8  cc;
	u8 tail;
    
}ConcentratorRequst;


typedef struct _BeaconCoordinate
{
    
    BTHeader head;
	u32 id[3];
    u32 longitude;//����
	u32 latitude;//γ�� 
    u16 voltage; //��ѹ   
    s16 Rssi; //�ź�ǿ��
	s8  Snr; //�����
    u8  resver;
	u8  cc;    
	u8  tail;
    
}BeaconCoordinate;


typedef struct _CheckIn //��������
{    
    BTHeader head;
	u32 id[3];
    u16 voltage; //��ѹ   
    u8  cc;
	u8 tail;
    
}CheckIn;

#endif

