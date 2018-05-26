#ifndef __USART1_H
#define	__USART1_H

//#include "sys_config.h"
#include <stdio.h>
#include "main.h"

             
#define USART1_CLK_ENABLE()           LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1)
#define USART1_CLK_SOURCE()           LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2)
#define USART1_GPIO_CLK_ENABLE()      LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA)   /* Enable the peripheral clock of GPIOA */
#define USART1_TX_PIN                 LL_GPIO_PIN_9
#define USART1_TX_GPIO_PORT           GPIOA
#define USART1_SET_TX_GPIO_AF()       LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_4)
#define USART1_RX_PIN                 LL_GPIO_PIN_10
#define USART1_RX_GPIO_PORT           GPIOA
#define USART1_SET_RX_GPIO_AF()       LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_4)



#define USART2_CLK_ENABLE()           LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2)
#define USART2_CLK_SOURCE()           LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1)
#define USART2_GPIO_CLK_ENABLE()      LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA)   /* Enable the peripheral clock of GPIOA */
#define USART2_TX_PIN                 LL_GPIO_PIN_2
#define USART2_TX_GPIO_PORT           GPIOA
#define USART2_SET_TX_GPIO_AF()       LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_4)
#define USART2_RX_PIN                 LL_GPIO_PIN_3
#define USART2_RX_GPIO_PORT           GPIOA
#define USART2_SET_RX_GPIO_AF()       LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_4)


void USART1_Config(void);
void USART2_Config(void);
void USART3_Config(void);

void UsartSend(u16 ch);
void UART1_Send_Buf(u8* c, u8 len);
void USART_putchar(USART_TypeDef* USART_x, unsigned char ch);

int fputc(int ch, FILE *f);
int fgetc(FILE *f);

#endif /* __USART1_H */
