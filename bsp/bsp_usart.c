/**
  ******************************************************************************
  * @file    bsp_usart1.c
  * @author  wangwei
  * @version V1.0
  * @date    2016年10月17日00:27:02
  * @brief   usart应用bsp
  ******************************************************************************
  ******************************************************************************
  */
  
#include "bsp_usart.h"

 /**
  * @brief  USART1 GPIO 配置,工作模式配置。115200 8-N-1
  * @param  无
  * @retval 无
  */

void USART1_Config(void)
{
	
  LL_USART_InitTypeDef usart_initstruct;

  /* (1) Enable GPIO clock and configures the USART pins *********************/
  /* Enable the peripheral clock of GPIO Port */
  USART1_GPIO_CLK_ENABLE();
    
  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(USART1_TX_GPIO_PORT, USART1_TX_PIN, LL_GPIO_MODE_ALTERNATE);
  USART1_SET_TX_GPIO_AF();
  LL_GPIO_SetPinSpeed(USART1_TX_GPIO_PORT, USART1_TX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(USART1_TX_GPIO_PORT, USART1_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(USART1_TX_GPIO_PORT, USART1_TX_PIN, LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(USART1_RX_GPIO_PORT, USART1_RX_PIN, LL_GPIO_MODE_ALTERNATE);
  USART1_SET_RX_GPIO_AF();
  LL_GPIO_SetPinSpeed(USART1_RX_GPIO_PORT, USART1_RX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(USART1_RX_GPIO_PORT, USART1_RX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(USART1_RX_GPIO_PORT, USART1_RX_PIN, LL_GPIO_PULL_UP);

  /* (2) NVIC Configuration for USART interrupts */
  /*  - Set priority for USARTx_IRQn */
  /*  - Enable USARTx_IRQn */
  NVIC_SetPriority(USART1_IRQn, 7);  
  NVIC_EnableIRQ(USART1_IRQn);

  /* (3) Enable USART peripheral clock and clock source ***********************/
  USART1_CLK_ENABLE();

  /* Set clock source */
  USART1_CLK_SOURCE();

  /* (4) Configure USART functional parameters ********************************/
  
  /* Disable USART prior modifying configuration registers */
  /* Note: Commented as corresponding to Reset value */
  // LL_USART_Disable(USART1);

  /* Set fields of initialization structure                   */
  /*  - BaudRate            : 115200                          */
  /*  - DataWidth           : LL_USART_DATAWIDTH_8B           */
  /*  - StopBits            : LL_USART_STOPBITS_1             */
  /*  - Parity              : LL_USART_PARITY_NONE            */
  /*  - TransferDirection   : LL_USART_DIRECTION_TX_RX        */
  /*  - HardwareFlowControl : LL_USART_HWCONTROL_NONE         */
  /*  - OverSampling        : LL_USART_OVERSAMPLING_16        */
  usart_initstruct.BaudRate            = 115200;
  usart_initstruct.DataWidth           = LL_USART_DATAWIDTH_8B;
  usart_initstruct.StopBits            = LL_USART_STOPBITS_1;
  usart_initstruct.Parity              = LL_USART_PARITY_NONE;
  usart_initstruct.TransferDirection   = LL_USART_DIRECTION_TX_RX;
  usart_initstruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  usart_initstruct.OverSampling        = LL_USART_OVERSAMPLING_16;
  
  /* Initialize USART instance according to parameters defined in initialization structure */
  LL_USART_Init(USART1, &usart_initstruct);

  /* (5) Enable USART *********************************************************/
  LL_USART_Enable(USART1);

  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  { 
  }

  /* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USART1);
//  LL_USART_EnableIT_ERROR(USART1);
  
}

 /**
  * @brief  USART2 GPIO 配置,工作模式配置。115200 8-N-1
  * @param  无
  * @retval 无
  */
void USART2_Config(void)
{
  LL_USART_InitTypeDef usart_initstruct;

  /* (1) Enable GPIO clock and configures the USART pins *********************/

  /* Enable the peripheral clock of GPIO Port */
  USART2_GPIO_CLK_ENABLE();
    
  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(USART2_TX_GPIO_PORT, USART2_TX_PIN, LL_GPIO_MODE_ALTERNATE);
  USART2_SET_TX_GPIO_AF();
  LL_GPIO_SetPinSpeed(USART2_TX_GPIO_PORT, USART2_TX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(USART2_TX_GPIO_PORT, USART2_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(USART2_TX_GPIO_PORT, USART2_TX_PIN, LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(USART2_RX_GPIO_PORT, USART2_RX_PIN, LL_GPIO_MODE_ALTERNATE);
  USART2_SET_RX_GPIO_AF();
  LL_GPIO_SetPinSpeed(USART2_RX_GPIO_PORT, USART2_RX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(USART2_RX_GPIO_PORT, USART2_RX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(USART2_RX_GPIO_PORT, USART2_RX_PIN, LL_GPIO_PULL_UP);

  /* (2) NVIC Configuration for USART interrupts */
  /*  - Set priority for USART2_IRQn */
  /*  - Enable USART2_IRQn */
  NVIC_SetPriority(USART2_IRQn,4);  
  NVIC_EnableIRQ(USART2_IRQn);

  /* (3) Enable USART peripheral clock and clock source ***********************/
  USART2_CLK_ENABLE();

  /* Set clock source */
  USART2_CLK_SOURCE();

  /* (4) Configure USART functional parameters ********************************/
  
  /* Disable USART prior modifying configuration registers */
  /* Note: Commented as corresponding to Reset value */
  // LL_USART_Disable(USART1);

  /* Set fields of initialization structure                   */
  /*  - BaudRate            : 115200                          */
  /*  - DataWidth           : LL_USART_DATAWIDTH_8B           */
  /*  - StopBits            : LL_USART_STOPBITS_1             */
  /*  - Parity              : LL_USART_PARITY_NONE            */
  /*  - TransferDirection   : LL_USART_DIRECTION_TX_RX        */
  /*  - HardwareFlowControl : LL_USART_HWCONTROL_NONE         */
  /*  - OverSampling        : LL_USART_OVERSAMPLING_16        */
  usart_initstruct.BaudRate            = 9600;
  usart_initstruct.DataWidth           = LL_USART_DATAWIDTH_8B;
  usart_initstruct.StopBits            = LL_USART_STOPBITS_1;
  usart_initstruct.Parity              = LL_USART_PARITY_NONE;
  usart_initstruct.TransferDirection   = LL_USART_DIRECTION_TX_RX;
  usart_initstruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  usart_initstruct.OverSampling        = LL_USART_OVERSAMPLING_16;
  
  /* Initialize USART instance according to parameters defined in initialization structure */
  LL_USART_Init(USART2, &usart_initstruct);

  /* (5) Enable USART *********************************************************/
  LL_USART_Enable(USART2);

  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  { 
  }

  /* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USART2);
  //LL_USART_EnableIT_ERROR(USART2);
  
}




/**
  * @brief  发送字符
  * @param  len   字符长度， *c   指向待发送字符串的指针
  * @retval 无
  */
void UART1_Send_Buf(u8* c, u8 len)
{
    while(len--)
    {
        LL_USART_TransmitData8(USART1,*c);
        while(LL_USART_IsActiveFlag_TC(USART1)==RESET); 
		c++;
    }        

}



void PrintChar(char *s)
{
	char *p;
	p=s;
	while(*p != '\0')
	{
		LL_USART_TransmitData8(USART1,*p);
		p++;
	}
}


void USART_putchar(USART_TypeDef* USART_x, unsigned char ch)
{
/* Write a character to the USART */
    LL_USART_TransmitData8(USART_x, (unsigned char) ch);
      while(LL_USART_IsActiveFlag_TC(USART1)==RESET); 
}



/// 重定向c库函数printf到USARTx
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到USARTx */
		LL_USART_TransmitData8(USART1, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while(LL_USART_IsActiveFlag_TC(USART1)==RESET); 	
	
		return (ch);
}

/// 重定向c库函数scanf到USARTx
int fgetc(FILE *f)
{
		/* 等待串口x输入数据 */
		while (LL_USART_IsActiveFlag_RXNE(USART1)  == RESET);

		return (int)LL_USART_ReceiveData8(USART1);
}
/*********************************************END OF FILE**********************/
