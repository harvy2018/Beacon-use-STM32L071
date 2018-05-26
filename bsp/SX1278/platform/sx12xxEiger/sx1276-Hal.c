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
 * \file       sx1276-Hal.c
 * \brief      SX1276 Hardware Abstraction Layer
 *
 * \version    2.0.B2 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
//#include <stdint.h>
//#include <stdbool.h> 

#include "platform.h"

#if defined( USE_SX1276_RADIO )

//#include "ioe.h"
#include "spi.h"
#include "../../radio/sx1276-Hal.h"

/*!
 * SX1276 RESET I/O definitions
 */
#define RESET_IOPORT                                GPIOB
#define RESET_PIN                                   LL_GPIO_PIN_5

/*!
 * SX1276 SPI NSS I/O definitions
 */
#define NSS_IOPORT                                  GPIOA
#define NSS_PIN                                     LL_GPIO_PIN_4    //ԭGPIO_Pin_15

/*!
 * SX1276 DIO pins  I/O definitions
 */
#define DIO0_IOPORT                                 GPIOC
#define DIO0_PIN                                    LL_GPIO_PIN_13

#define DIO1_IOPORT                                 GPIOC
#define DIO1_PIN                                    LL_GPIO_PIN_14

#define DIO2_IOPORT                                 GPIOC
#define DIO2_PIN                                    LL_GPIO_PIN_15

#define DIO3_IOPORT                                 GPIOA
#define DIO3_PIN                                    LL_GPIO_PIN_0

#define DIO4_IOPORT                                 GPIOA
#define DIO4_PIN                                    LL_GPIO_PIN_1

#define DIO5_IOPORT                                 GPIOB
#define DIO5_PIN                                    LL_GPIO_PIN_4

#define RXTX_IOPORT                                 //GPIOB
#define RXTX_PIN                                    //GPIO_Pin_0


#define RXE_PORT       		    GPIOB  //VB
#define RXE_PIN  			    LL_GPIO_PIN_0

//#define RXE_CLOCK  				RCC_APB2Periph_GPIOB

#define RXE_HIGH()         		LL_GPIO_SetOutputPin(RXE_PORT,RXE_PIN)
#define RXE_LOW()          		LL_GPIO_ResetOutputPin(RXE_PORT,RXE_PIN)
#define RXE_STATE()        		LL_GPIO_IsOutputPinSet(RXE_PORT,RXE_PIN)

#define TXE_PORT       		    GPIOB  //VA
#define TXE_PIN  				LL_GPIO_PIN_9
//#define TXE_CLOCK  				RCC_APB2Periph_GPIOB
#define TXE_HIGH()         		LL_GPIO_SetOutputPin(TXE_PORT,TXE_PIN)
#define TXE_LOW()          		LL_GPIO_ResetOutputPin(TXE_PORT,TXE_PIN)
#define TXE_STATE()        		LL_GPIO_IsOutputPinSet(TXE_PORT,TXE_PIN)

void Set_RF_Switch_RX(void)
{
    RXE_HIGH();
    TXE_LOW();
}

void Set_RF_Switch_TX(void)
{
	RXE_LOW();
	TXE_HIGH();
}


void SX1276InitIo( void )
{
   
    LL_GPIO_InitTypeDef gpio_initstruct;  
    
    gpio_initstruct.Pin = RXE_PIN | TXE_PIN;
    gpio_initstruct.Mode       = LL_GPIO_MODE_INPUT;
    gpio_initstruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    gpio_initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_initstruct.Pull       = LL_GPIO_PULL_NO; 
    LL_GPIO_Init(GPIOB, &gpio_initstruct);
  
 
	Set_RF_Switch_RX();	
	
    // Configure DIO0
    gpio_initstruct.Pin = DIO0_PIN ;
    LL_GPIO_Init( DIO0_IOPORT, &gpio_initstruct);
    
    // Configure DIO1
    gpio_initstruct.Pin = DIO1_PIN ;
    LL_GPIO_Init( DIO1_IOPORT, &gpio_initstruct);
    
    // Configure DIO2
    gpio_initstruct.Pin = DIO2_PIN ;
    LL_GPIO_Init( DIO2_IOPORT, &gpio_initstruct);
    
    // REAMARK: DIO3/4/5 configured are connected to IO expander

    // Configure DIO3 as input
    gpio_initstruct.Pin = DIO3_PIN ;
    LL_GPIO_Init( DIO3_IOPORT, &gpio_initstruct);
    // Configure DIO4 as input
    gpio_initstruct.Pin = DIO4_PIN ;
    LL_GPIO_Init( DIO4_IOPORT, &gpio_initstruct);
    // Configure DIO5 as input
	gpio_initstruct.Pin = DIO5_PIN ;
    LL_GPIO_Init( DIO5_IOPORT, &gpio_initstruct);
}

void SX1276SetReset( uint8_t state )
{
    LL_GPIO_InitTypeDef gpio_initstruct;  

    if( state == RADIO_RESET_ON )
    {
        // Configure RESET as output

        gpio_initstruct.Pin        = RESET_PIN ;
        gpio_initstruct.Mode       = LL_GPIO_MODE_OUTPUT;
        gpio_initstruct.Speed      = LL_GPIO_SPEED_FREQ_MEDIUM;
        gpio_initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        gpio_initstruct.Pull       = LL_GPIO_PULL_NO; 
        LL_GPIO_Init(RESET_IOPORT, &gpio_initstruct);
		
		// Set RESET pin to 0     
        LL_GPIO_ResetOutputPin(RESET_IOPORT,RESET_PIN);
    }
    else
    {
		gpio_initstruct.Pin        = RESET_PIN ;
        gpio_initstruct.Mode       = LL_GPIO_MODE_OUTPUT;
        gpio_initstruct.Speed      = LL_GPIO_SPEED_FREQ_MEDIUM;
        gpio_initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        gpio_initstruct.Pull       = LL_GPIO_PULL_NO; 
        LL_GPIO_Init(RESET_IOPORT, &gpio_initstruct);
		
		// Set RESET pin to 1
        LL_GPIO_SetOutputPin( RESET_IOPORT, RESET_PIN);

    }
}

void SX1276Write( uint8_t addr, uint8_t data )
{
    SX1276WriteBuffer( addr, &data, 1 );
}

void SX1276Read( uint8_t addr, uint8_t *data )
{
    SX1276ReadBuffer( addr, data, 1 );
}

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
    LL_GPIO_ResetOutputPin( NSS_IOPORT, NSS_PIN);

    SpiInOut( addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        SpiInOut( buffer[i] );
    }

    //NSS = 1;
    LL_GPIO_SetOutputPin( NSS_IOPORT, NSS_PIN);
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
     LL_GPIO_ResetOutputPin( NSS_IOPORT, NSS_PIN);
	 //GPIO_ResetBits(NSS_IOPORT,NSS_PIN);

    SpiInOut( addr & 0x7F );

    for( i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( 0 );
    }

    //NSS = 1;
     LL_GPIO_SetOutputPin( NSS_IOPORT, NSS_PIN);
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1276WriteBuffer( 0, buffer, size );
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1276ReadBuffer( 0, buffer, size );
}

inline uint8_t SX1276ReadDio0( void )
{
    return LL_GPIO_IsInputPinSet( DIO0_IOPORT, DIO0_PIN );
}

inline uint8_t SX1276ReadDio1( void )
{
    return LL_GPIO_IsInputPinSet( DIO1_IOPORT, DIO1_PIN );
}

inline uint8_t SX1276ReadDio2( void )
{
    return LL_GPIO_IsInputPinSet( DIO2_IOPORT, DIO2_PIN );
}

inline uint8_t SX1276ReadDio3( void )
{
    return LL_GPIO_IsInputPinSet( DIO3_IOPORT, DIO3_PIN );
}

inline uint8_t SX1276ReadDio4( void )
{
    return LL_GPIO_IsInputPinSet( DIO4_IOPORT, DIO4_PIN );
}

inline uint8_t SX1276ReadDio5( void )
{
    return LL_GPIO_IsInputPinSet( DIO5_IOPORT, DIO5_PIN );
}


//射频芯片收发切换
inline void SX1276WriteRxTx( uint8_t txEnable )
{
    if( txEnable != 0 )
    {
		Set_RF_Switch_TX(); //单片机将射频开关芯片切换成发射状态
//        IoePinOn( FEM_CTX_PIN );
//        IoePinOff( FEM_CPS_PIN );
    }
    else
    {
		Set_RF_Switch_RX();  //单片机将射频开关芯片切换成接收状态
//        IoePinOff( FEM_CTX_PIN );
//        IoePinOn( FEM_CPS_PIN );
    }
}

#endif // USE_SX1276_RADIO
