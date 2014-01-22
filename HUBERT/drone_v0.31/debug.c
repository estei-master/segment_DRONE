#include "debug.h"

#define debugUART_MAX_STRING_LENGHT 200

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
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	//USART_InitStructure.USART_BaudRate = 115200;
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
	printf( "\r\nSysTick   \tTag  \tTask \tPrio\tModule      \tFunction                      \tMessage" );
	printf( "\r\n         0\tINFO\t-\t   0\tdebug       \tvDebugUARTInit()              \tUART6 initialized" );
}

/** Preformatted debug printf function */
uint32_t ulDebugMsg( unsigned long ulTickCnt, char * pcTag,
		signed char * pcTask, unsigned long ulTaskPrio, char * pcModule,
		char * pcFunc, char * pcMessage )
{
		return printf( "\r\n%10d\t%5s\t%5s\t%4d\t%12s\t%30s\t%s",
				( int )ulTickCnt, pcTag, pcTask, ( int )ulTaskPrio,
				pcModule, pcFunc, pcMessage );
}

/** Retarget printf() to USART6 function */
void vDebugUARTPutchar( uint8_t ucChar )
{
	/* TODO : Use an interrupt instead */
	/* Wait for previous character transmission */
	while( USART_GetFlagStatus( USART6, USART_FLAG_TC ) == RESET )
	{
		/* Wait */
	}

	/* Send character */
	USART_SendData( USART6, ucChar );
}
