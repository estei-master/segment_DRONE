#include "debug.h"

/** @file debug.c */

/** We use the USART6 as it corresponds the RS232 port on the Discovery board */
void vDebugUARTInit( void )
{
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );

	/* Enable UART clock */
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART6, ENABLE );

	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig( GPIOC, GPIO_PinSource6, GPIO_AF_USART6 );

	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig( GPIOC, GPIO_PinSource7, GPIO_AF_USART6 );

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	USART_InitStructure.USART_BaudRate = 230400;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	/* Rx not necessary... */
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init( USART6, &USART_InitStructure );

	/* Enable USART */
	USART_Cmd( USART6, ENABLE );

	printf( "\r\n\r\n------------------------------------------------------------------------------------------------\r\n" );
	printf( "\r\nSysTick     Tag    Task   Prio  Module        Function                        Message" );
	printf( "\r\n         0  INFO   ---    0     debug         vDebugUARTInit()                UART6 initialized" );
}

/** Preformatted debug printf function */
uint32_t ulDebugMsg( unsigned long ulTickCnt, char * pcTag,
		signed char * pcTask, unsigned long ulTaskPrio, char * pcModule,
		char * pcFunc, char * pcMessage )
{
		return printf( "\r\n%10d  %6s  %5s  %4d  %12s  %30s  %s",
				( int )ulTickCnt, pcTag, pcTask, ( int )ulTaskPrio,
				pcModule, pcFunc, pcMessage );
}

/** Retarget printf() to USART6 function */
void vDebugUARTPutchar( uint8_t ucChar )
{
	/** @todo : Consider using an interrupt instead */
	/* Wait for previous character transmission */
	while( USART_GetFlagStatus( USART6, USART_FLAG_TC ) == RESET )
	{
		/* Wait */
	}

	/* Send character */
	USART_SendData( USART6, ucChar );
}
