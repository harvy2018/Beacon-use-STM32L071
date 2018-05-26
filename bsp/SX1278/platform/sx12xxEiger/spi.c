/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       spi.c
 * \brief      SPI hardware driver
 *
 * \version    1.0
 * \date       Feb 12 2010
 * \author     Miguel Luis
 */
 
//#include "stm32f10x_spi.h"

#include "spi.h"


//ÐÞ¸Ä³ÉSPI1
#define SPI_INTERFACE                               SPI1
#define SPI_CLK                                     LL_APB2_GRP1_PERIPH_SPI1

#define SPI1_PORT                                   GPIOA

#define SPI1_PORT_CLK                               LL_IOP_GRP1_PERIPH_GPIOA

#define SPI_PIN_SCK                                 LL_GPIO_PIN_5
#define SPI_PIN_MISO                                LL_GPIO_PIN_6
#define SPI_PIN_MOSI                                LL_GPIO_PIN_7

#define SPI_PIN_NSS                                 LL_GPIO_PIN_4

void SpiInit( void )
{
    /* (1) Enables GPIO clock and configures the SPI1 pins ********************/
  /* Enable the peripheral clock of GPIOB */
  LL_IOP_GRP1_EnableClock(SPI1_PORT_CLK);

  /* Configure SCK Pin connected to pin 31 of CN10 connector */
  LL_GPIO_SetPinMode(SPI1_PORT, SPI_PIN_SCK, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(SPI1_PORT, SPI_PIN_SCK, LL_GPIO_AF_0);
  LL_GPIO_SetPinSpeed(SPI1_PORT, SPI_PIN_SCK, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(SPI1_PORT, SPI_PIN_SCK, LL_GPIO_PULL_DOWN);

  /* Configure MOSI Pin connected to pin 29 of CN10 connector */
  LL_GPIO_SetPinMode(SPI1_PORT, SPI_PIN_MOSI, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(SPI1_PORT, SPI_PIN_MOSI, LL_GPIO_AF_0);
  LL_GPIO_SetPinSpeed(SPI1_PORT, SPI_PIN_MOSI, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(SPI1_PORT, SPI_PIN_MOSI, LL_GPIO_PULL_DOWN);
    
  /* Configure MISO Pin connected to pin 27 of CN10 connector */
  LL_GPIO_SetPinMode(SPI1_PORT, SPI_PIN_MISO, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(SPI1_PORT, SPI_PIN_MISO, LL_GPIO_AF_0);
  LL_GPIO_SetPinSpeed(SPI1_PORT, SPI_PIN_MISO, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(SPI1_PORT, SPI_PIN_MISO, LL_GPIO_PULL_DOWN);
  
  LL_GPIO_SetPinMode(SPI1_PORT, SPI_PIN_NSS, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetAFPin_0_7(SPI1_PORT, SPI_PIN_NSS, LL_GPIO_AF_0);
  LL_GPIO_SetPinSpeed(SPI1_PORT, SPI_PIN_NSS, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(SPI1_PORT, SPI_PIN_NSS, LL_GPIO_PULL_DOWN);
  
  
  LL_GPIO_SetOutputPin(SPI1_PORT,SPI_PIN_NSS);

  /* (2) Configure NVIC for SPI1 transfer complete/error interrupts **********/
  /* Set priority for SPI1_IRQn */
//  NVIC_SetPriority(SPI1_IRQn, 0);
  /* Enable SPI1_IRQn           */
//  NVIC_EnableIRQ(SPI1_IRQn);

  /* (3) Configure SPI1 functional parameters ********************************/
  /* Enable the peripheral clock of GPIOB */
  LL_APB2_GRP1_EnableClock(SPI_CLK);

  /* Configure SPI1 communication */
  LL_SPI_SetBaudRatePrescaler(SPI_INTERFACE, LL_SPI_BAUDRATEPRESCALER_DIV4);
  LL_SPI_SetTransferDirection(SPI_INTERFACE,LL_SPI_FULL_DUPLEX);
  LL_SPI_SetClockPhase(SPI_INTERFACE, LL_SPI_PHASE_1EDGE);
  LL_SPI_SetClockPolarity(SPI_INTERFACE, LL_SPI_POLARITY_LOW);
  
  /* Reset value is LL_SPI_MSB_FIRST */
  LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
  LL_SPI_SetDataWidth(SPI_INTERFACE, LL_SPI_DATAWIDTH_8BIT);
  LL_SPI_SetNSSMode(SPI_INTERFACE, LL_SPI_NSS_SOFT);
  LL_SPI_SetMode(SPI_INTERFACE, LL_SPI_MODE_MASTER);
  LL_SPI_SetCRCPolynomial(SPI_INTERFACE,7);
  LL_SPI_EnableCRC(SPI_INTERFACE);

  LL_SPI_Enable(SPI_INTERFACE);
  
  /* Configure SPI1 transfer interrupts */
  /* Enable TXE   Interrupt */
//  LL_SPI_EnableIT_TXE(SPI1);
  /* Enable SPI1 Error Interrupt */
//  LL_SPI_EnableIT_ERR(SPI1);
  
  
}


uint8_t SpiInOut( uint8_t outData )
{
    
    while(LL_SPI_IsActiveFlag_TXE(SPI_INTERFACE) == RESET);
	  LL_SPI_TransmitData8(SPI_INTERFACE, outData);
    
    while( LL_SPI_IsActiveFlag_RXNE(SPI_INTERFACE) == RESET);
    return LL_SPI_ReceiveData8( SPI_INTERFACE );

}



