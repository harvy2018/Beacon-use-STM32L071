#ifndef __5110LCD_H__
#define __5110LCD_H__

#include "stm32f10x_gpio.h"



#define RES_PIN  GPIO_Pin_8

#define CE_PIN  GPIO_Pin_15
#define DC_PIN  GPIO_Pin_14
#define DIN_PIN  GPIO_Pin_13
#define CLK_PIN  GPIO_Pin_12
#define BACKLIGHT_PIN  GPIO_Pin_1




#define res0   GPIO_ResetBits(GPIOA, RES_PIN)  //复位,0复位
#define sce0   GPIO_ResetBits(GPIOB, CE_PIN)  //片选

#define dc0    GPIO_ResetBits(GPIOB, DC_PIN)   //1写数据，0写指令

#define sdin0  GPIO_ResetBits(GPIOB, DIN_PIN)   //数据
#define sclk0  GPIO_ResetBits(GPIOB, CLK_PIN)   //时钟
#define backled0  GPIO_ResetBits(GPIOB, BACKLIGHT_PIN)   //背光


#define res1   GPIO_SetBits(GPIOA, RES_PIN)  //复位,0复位
#define sce1   GPIO_SetBits(GPIOB, CE_PIN)  //片选

#define dc1    GPIO_SetBits(GPIOB, DC_PIN)   //1写数据，0写指令
#define sdin1  GPIO_SetBits(GPIOB, DIN_PIN)   //数据
#define sclk1  GPIO_SetBits(GPIOB, CLK_PIN)   //时钟
#define backled1  GPIO_SetBits(GPIOB, BACKLIGHT_PIN)   //背光

  
/*函数名称 功能描述
GPIO_DeInit 重新初始化外围设备GPIOx相关寄存器到它的默认复位值
GPIO_AFIODeInit 初始化交错功能(remap, event control和 EXTI 配置) 寄存器
GPIO_Init 根据GPIO_初始化结构指定的元素初始化外围设备GPIOx
GPIO_StructInit 填充GPIO_初始化结构（GPIO_InitStruct）内的元素为复位值
GPIO_ReadInputDataBit 读指定端口引脚输入数据
GPIO_ReadInputData 读指定端口输入数据
GPIO_ReadOtputDataBit 读指定端口引脚输出数据
GPIO_ReadOtputData 读指定端口输出数据
GPIO_SetBits 置1指定的端口引脚
GPIO_ResetBits 清0指定的端口引脚
GPIO_WriteBit 设置或清除选择的数据端口引脚
GPIO_Write 写指定数据到GPIOx端口寄存器
GPIO_ANAPinConfig 允许或禁止 GPIO 4 模拟输入模式
GPIO_PinLockConfig 锁定GPIO引脚寄存器
GPIO_EventOutputConfig 选择GPIO引脚作为事件输出
GPIO_EventOutputCmd 允许或禁止事件输出
GPIO_PinRemapConfig 改变指定引脚的影射
GPIO_EMIConfig 允许或禁止GPIO 8 和 9 的EMI 模式
GPIO_InitTypeDef GPIO_InitStructure;
//键盘定义
#define   KEY_OK    GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) 
#define   KEY_DOWN   GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)
#define   KEY_UP    GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2)
#define   KEY_ESC   GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3)	  
*/


void LCD_init(void);
void LCD_clear(void);
void LCD_write_String(unsigned char X,unsigned char Y,char *s);
void LCD_write_shu(unsigned char row, unsigned char page,unsigned char c); //row:列 page:页 dd:字符
void LCD_write_hanzi(unsigned char row, unsigned char page,unsigned char c) ;//row:列 page:页 dd:字符
#endif


