
#ifndef _FLASHMGR_H_
#define _FLASHMGR_H_


#include "stm32l0xx.h"	
#include "stdint.h"	





#pragma pack(1)
typedef enum _Flash_Sector_ID
{
	FLASH_SECTION_DATA_INFO=0,
    FLASH_SECTION_DATA_INDEX,
	FLASH_SECTION_POSITION_DATA,
	
	FLASH_SECTION_MAX
	
}Flash_Sector_ID;


typedef enum _FLASH_TYPE
{
	FLASH_IN,			// 片内FLASH
	FLASH_EX,			// 片外FLASH
	FLASH_MAX
	
}FLASH_TYPE;	

//设计环形数据结构以满足不同的使用需求
typedef struct _FlashSectionStruct
{
    FLASH_TYPE type;     //  Flash 类型
    u32     base;      //  分区的起始地址
    u32     size;	 //  分区的大小
    u32     wptr;     //  写指针
    u32     rptr;      //   读指针
	
}FlashSectionStruct;

#pragma pack()

void uf_InitHard(FLASH_TYPE type);
uint32_t uf_ReadID(FLASH_TYPE type);

void uf_EraseChip(FLASH_TYPE type);
void uf_EraseSector(FLASH_TYPE type,uint32_t _uiSectorAddr);
void uf_EraseBlock(FLASH_TYPE type,uint8_t blocksize, uint32_t _uiSectorAddr);
//void uf_PageWrite(FLASH_TYPE type,uint8_t * _pBuf, uint32_t _uiWriteAddr, uint16_t _usSize);
uint8_t uf_WriteBuffer(FLASH_TYPE type,uint8_t* _pBuf, uint32_t _uiWriteAddr, uint16_t _usWriteSize);
void uf_ReadBuffer(FLASH_TYPE type,uint8_t * _pBuf, uint32_t _uiReadAddr, uint32_t _uiSize);
FlashSectionStruct  *uf_RetrieveFlashSectionInfo(Flash_Sector_ID sectionID);


extern  FlashSectionStruct flashSectionStruct[];
extern u8 Area_Used;
extern u8 Area_Unused;
extern u8 Area_Info;
extern u8 FlashVer[4];


#endif

