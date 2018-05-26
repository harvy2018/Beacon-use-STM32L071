
#include <stdbool.h>
#include "stm32f10x.h"	
#include "flashmgr.h"	
#include "bsp_spi_flash.h"
#include "stm32f10x_flash.h"

//#include "download.h"	
u8 FlashVer[4]={1,0,0,3};
u8 Area_Used=0xFF; 
u8 Area_Unused=0; 
u8 Area_Info;//
////系统上电后做如下初始化：FLASH_SECTION_MAX
FlashSectionStruct flashSectionStruct[FLASH_SECTION_MAX]=
{
	{ FLASH_EX,    0x000000,   4*1024,     0x000000,  	0x000000}, //FLASH_SECTION_DATA_INFO
    { FLASH_EX,    0x001000,   4*1024,     0x001000,  	0x001000}, //FLASH_SECTION_DATA_INDEX
	{ FLASH_EX,    0x002000,   128*1024,   0x002000,  	0x002000} //FLASH_SECTION_POSITION_DATA
	
//    { FLASH_EX,    0x1FF000,   4096, 	   0x1FF000,  	0x1FF000}, //FLASH_SECTION_STATISTICS_DATA
//    { FLASH_EX,    0x1FE000,   4096, 	   0x1FE000,  	0x1FE000}, //FLASH_SECTION_PLCSETUP_DATA
//    { FLASH_EX,    0x1FD000,   4096, 	   0x1FD000,  	0x1FD000}, //FLASH_SECTION_TAXCODE_DATA


//    { FLASH_IN,	0x8000000, 	0x3000,    	0x8000000, 	0x8000000},//FLASH_SECTION_FW_BOOT

//    { FLASH_IN,	0x8003000, 	0x800,    	0x8003000, 	0x8003000},//FLASH_SECTION_PINGPONG_SEL
//    { FLASH_IN,  0x8003800, 0x800,      0x8003800, 	0x8003800},//FLASH_SECTION_FLASH_MANAGER_VER

//    { FLASH_IN,	0x8004000, 	0x800,      0x8004000, 	0x8004000},//FLASH_SECTION_SYSTEM_INFO
//    { FLASH_IN,	0x8004800, 	0x800,      0x8004800, 	0x8004800},//FLASH_SECTION_CONFIG_PARA

//    { FLASH_IN,	0x8005000, 	0x1000,     0x8005000, 	0x8005000},//FLASH_SECTION_SPI_TEST
//    { FLASH_IN,	0x8006000, 	240*1024, 	0x8006000, 	0x8006000},//FLASH_SECTION_FW_PING
//    { FLASH_IN,	0x8042000, 	240*1024,  	0x8042000,  0x8042000}//FLASH_SECTION_FW_PONG
	
	
};

void uf_InitHard(FLASH_TYPE type);
uint32_t uf_ReadID(FLASH_TYPE type);

void uf_EraseChip(FLASH_TYPE type);
void uf_EraseSector(FLASH_TYPE type,uint32_t _uiSectorAddr);
void uf_EraseBlock(FLASH_TYPE type,uint8_t blocksize, uint32_t _uiSectorAddr);
void uf_PageWrite(FLASH_TYPE type,uint8_t * _pBuf, uint32_t _uiWriteAddr, uint16_t _usSize);
uint8_t uf_WriteBuffer(FLASH_TYPE type,uint8_t* _pBuf, uint32_t _uiWriteAddr, uint16_t _usWriteSize);
void uf_ReadBuffer(FLASH_TYPE type,uint8_t * _pBuf, uint32_t _uiReadAddr, uint32_t _uiSize);

FLASH_Status FLASH_ProgramStart(u32 ADDR,vu32 ApplicationSize);
static u32 FLASH_PagesMask(vu32 Size);
u32 FLASH_WriteBank(u8 *pData, u32 addr, u32 size);
void FLASH_ProgramDone(void);


FlashSectionStruct  *uf_RetrieveFlashSectionInfo(Flash_Sector_ID sectionID)
{
	return (FlashSectionStruct *)((unsigned char *)flashSectionStruct + sectionID * sizeof(FlashSectionStruct));
}


void ExFlash_Lock(void)
{
     GPIO_ResetBits(SF_WP_GPIO,SF_WP_PIN);
}

void ExFlash_Unlock(void)
{
     GPIO_SetBits(SF_WP_GPIO,SF_WP_PIN);
}

void uf_EraseChip(FLASH_TYPE type)
{
	if(type==FLASH_EX)
    {
	    ExFlash_Unlock();
        sf_EraseChip();
         ExFlash_Lock(); 
    }


}

void uf_EraseSector(FLASH_TYPE type,uint32_t _uiSectorAddr)
{
	 if(type==FLASH_EX)
     {
	     ExFlash_Unlock();
         sf_EraseSector(_uiSectorAddr);
         ExFlash_Lock();
     }
	 else if(type==FLASH_IN)
	 {
		 FLASH_Unlock();
		 FLASH_ErasePage(_uiSectorAddr);
		 FLASH_Lock();
	 }
		 	 
}


void uf_EraseBlock(FLASH_TYPE type,uint8_t blocksize, uint32_t _uiSectorAddr)
{
	 if(type==FLASH_EX)
     {
         ExFlash_Unlock();
		 sf_EraseBlock(blocksize, _uiSectorAddr);
         ExFlash_Lock();
     }
	  
}


//void uf_PageWrite(FLASH_TYPE type,uint8_t * _pBuf, uint32_t _uiWriteAddr, uint16_t _usSize)
//{
//	if(type==FLASH_EX)
//	  sf_AutoWritePage(_pBuf,_uiWriteAddr, _usSize);
//	else if(type==FLASH_IN)
//	  FLASH_WriteBank(_pBuf, _uiWriteAddr, _usSize);
//	
//	
//}


//成功返回1，失败返回0
uint8_t uf_WriteBuffer(FLASH_TYPE type,uint8_t* _pBuf, uint32_t _uiWriteAddr, uint16_t _usWriteSize)
{
	uint8_t res;
	
	if(type==FLASH_EX)
    {
	    ExFlash_Unlock();
        res=sf_WriteBuffer(_pBuf,  _uiWriteAddr,  _usWriteSize);
        ExFlash_Lock();
    }
	else if(type==FLASH_IN)
	{
		 res=FLASH_ProgramStart(_uiWriteAddr,_usWriteSize);
		 if(res==FLASH_COMPLETE)		 
		    res=FLASH_WriteBank(_pBuf, _uiWriteAddr, _usWriteSize);
		 else
			 res=0;
		 
		  FLASH_ProgramDone();
	}
	
	return res;
	
}

void uf_ReadBuffer(FLASH_TYPE type,uint8_t * _pBuf, uint32_t _uiReadAddr, uint32_t _uiSize)
{
	 if(type==FLASH_EX)       
	   sf_ReadBuffer(_pBuf,  _uiReadAddr,  _uiSize);
     
	 
	
}


/*******************************************************************************
* Function Name :unsigned int FLASH_PagesMask(volatile unsigned int Size)
* Description   :计算所要擦除 的页数
* Input         :
* Output        :
* Other         :
*******************************************************************************/
static u32 FLASH_PagesMask(vu32 Size)
{
    u32 pagenumber = 0x0;
    u32 size = Size;

    if ((size % PAGE_SIZE) != 0)
    {
        pagenumber = (size / PAGE_SIZE) + 1;
    }
    else
    {
        pagenumber = size / PAGE_SIZE;
    }
    return pagenumber;
}


/*******************************************************************************
* Function Name :u32 FLASH_WriteBank(u32 *pData, u32 addr, u32 size)
* Description   :写入一块数据
* Input         :pData:数据；addr:数据的地址；size:长度
* Output        :TRUE:成功，FALSE:失败。
* Other         :
*******************************************************************************/
u32 FLASH_WriteBank(u8 *pData, u32 addr, u32 size)
{
    vu16 *pDataTemp = (vu16 *)pData;
    vu32 temp = addr;
//    FLASH_Status FLASHStatus = FLASH_COMPLETE;
//    u32 NbrOfPage = 0;
//
//    NbrOfPage = FLASH_PagesMask(addr + size - ApplicationAddress);
//    for (; (m_EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); m_EraseCounter++)
//    {
//        FLASHStatus = FLASH_ErasePage(ApplicationAddress + (PAGE_SIZE * m_EraseCounter));
//    }

    for (; temp < (addr + size); pDataTemp++, temp += 2)
    {
        FLASH_ProgramHalfWord(temp, *pDataTemp);
        if (*pDataTemp != *(vu16 *)temp)
        {
            return false;
        }
    }

    return true;
}

/*******************************************************************************
* Function Name :void FLASH_ProgramDone(void)
* Description   :烧写启动
* Input         :
* Output        :
* Other         :
*******************************************************************************/
FLASH_Status FLASH_ProgramStart(u32 ADDR,vu32 ApplicationSize)
{
    FLASH_Status FLASHStatus = FLASH_COMPLETE;
    u32 NbrOfPage = 0;
    vu32 EraseCounter = 0;

    FLASH_Unlock();
//    m_EraseCounter = 0;

    NbrOfPage = FLASH_PagesMask(ApplicationSize);
    for (; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
    {
        FLASHStatus = FLASH_ErasePage(ADDR + (PAGE_SIZE * EraseCounter));
    }
		return FLASHStatus;
}


/*******************************************************************************
* Function Name :void FLASH_ProgramDone(void)
* Description   :烧写结束
* Input         :
* Output        :
* Other         :
*******************************************************************************/
void FLASH_ProgramDone(void)
{
    FLASH_Lock();
}


/********************************* END ****************************************/



