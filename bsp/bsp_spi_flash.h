/*
*********************************************************************************************************
*
*	模块名称 : 串行FLASH  SST25VF016B驱动模块
*	文件名称 :  bsp_SST25VF016B.h
*	版    本 : V1.0
*	说    明 : 头文件
*	修改记录 :
*		版本号  日期       作者    说明
*		v1.0    2011-08-21 armfly  ST固件库升级到V3.5.0版本。
*
*	Copyright (C), 2010-2011, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_SST25VF016B_H
#define _BSP_SST25VF016B_H

#include <inttypes.h>

#if defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)
 #define PAGE_SIZE                         (0x400)    /* 1 Kbyte */
 #define FLASH_SIZE                        (0x20000)  /* 128 KBytes */
#elif defined STM32F10X_CL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x40000)  /* 256 KBytes */
#elif defined STM32F10X_HD
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x80000)  /* 512 KBytes */
#endif


#define SPI_FLASH          SPI2
#define SF_CLK             RCC_APB1Periph_SPI2
#define SF_GPIO            GPIOB
#define SF_GPIO_CLK        RCC_APB2Periph_GPIOB
#define SF_PIN_SCK         GPIO_Pin_13
#define SF_PIN_MISO        GPIO_Pin_14
#define SF_PIN_MOSI        GPIO_Pin_15
#define SF_WP_PIN          GPIO_Pin_3

#define SF_WP_GPIO         GPIOB

#define SF_CS             GPIO_Pin_12
#define SF_CS_GPIO        GPIOB
#define SF_CS_GPIO_CLK    RCC_APB2Periph_GPIOB

/* 片选口线置低选中  */
#define SF_CS_LOW()       GPIO_ResetBits(SF_CS_GPIO, SF_CS)

/* 片选口线置高不选中 */
#define SF_CS_HIGH()      GPIO_SetBits(SF_CS_GPIO, SF_CS)

#define CMD_AAI       0xAD  	/* AAI 连续编程指令 */
#define CMD_DISWR	  0x04		/* 禁止写, 退出AAI状态 */
#define CMD_EWRSR	  0x50		/* 允许写状态寄存器的命令 */
#define CMD_WRSR      0x01  	/* 写状态寄存器命令 */
#define CMD_WREN      0x06		/* 写使能命令 */
#define CMD_READ      0x03  	/* 读数据区命令 */
#define CMD_RDSR      0x05		/* 读状态寄存器命令 */
#define CMD_RDID      0x9F		/* 读器件ID命令 */
#define CMD_SE        0x20		/* 擦除扇区命令 */
#define CMD_BE        0xC7		/* 批量擦除命令 */
#define DUMMY_BYTE    0xA5		/* 哑命令，可以为任意值，用于读操作 */

#define WIP_FLAG      0x01		/* 状态寄存器中的正在编程标志（WIP) */


/* 定义串行Flash ID */
enum
{
	SST25VF016B_ID = 0xBF2541,
	MX25L1606E_ID = 0xC22015
};

#define SF_UNKNOW_NAME	"Unknow"



typedef struct
{
	uint32_t ChipID;		/* 芯片ID */
	char ChipName[16];		/* 芯片型号字符串，主要用于显示 */
	uint32_t TotalSize;		/* 总容量 */
	uint16_t PageSize;		/* 页面大小 */
}SFLASH_T;

extern SFLASH_T g_tSF;

#define SF_TOTAL_SIZE		(2*1024*1024)	/* 总容量，2M字节 */
#define SF_PAGE_SIZE    	(4*1024)		/* 页面大小，4K字节 */

void sf_InitHard(void);
uint32_t sf_ReadID(void);
void sf_EraseChip(void);
void sf_EraseSector(uint32_t _uiSectorAddr);
void sf_PageWrite(uint8_t * _pBuf, uint32_t _uiWriteAddr, uint16_t _usSize);

uint8_t sf_WriteBuffer(uint8_t* _pBuf, uint32_t _uiWriteAddr, uint16_t _usWriteSize);
void sf_ReadBuffer(uint8_t * _pBuf, uint32_t _uiReadAddr, uint32_t _uiSize);
void sf_EraseBlock(uint8_t blocksize, uint32_t _uiSectorAddr);


#endif
