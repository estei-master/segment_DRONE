/** @file telemeter.c */
/** @todo Test integration of telemters */

#include "telemeter.h"
#include "task.h"

/** Used to fill the pcModule argument of the ulDebugMsg() call. */
#define MODULE		"telemeter   "

/** Buffer for converted values */
static volatile uint16_t pusADCConvertedValues[ 8 ];

/** Range : 300 à 5000 (mm). If distance is lower/higher, values stick to the
extremes. */
static inline uint8_t prvTelemeterDistValid( const uint16_t * const pusDistance )
{
uint16_t usHighMarginMm = 500;
uint16_t usLowMarginMm = 50;

	if( ( *pusDistance > 5000 + usHighMarginMm )
			|| ( *pusDistance < 300 - usLowMarginMm ) )
	{
		ulDebugMsg( xTaskGetTickCount(), "ERROR",
			pcTaskGetTaskName( NULL ), uxTaskPriorityGet( NULL ), MODULE,
			"prvTelemeterDistValid()", "Data outside of valid range !" );

		return 0;
	}
	else
	{
		ulDebugMsg( xTaskGetTickCount(), "INFO ",
			pcTaskGetTaskName( NULL ), uxTaskPriorityGet( NULL ), MODULE,
			"prvTelemeterDistValid()", "Data inside valid range" );

		return 1;
	}
}

inline void vTelemeterInit( void )
{
DMA_InitTypeDef DMA_InitStructure;
GPIO_InitTypeDef GPIO_AdcInit;
ADC_InitTypeDef	ADC_InitStructure;

	ulDebugMsg( xTaskGetTickCount(), "INFO ", ( signed char * ) "----", 0,
			MODULE, "vTelemeterInit()", "Initializing telemeters" );

	/* DMA initialization */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2, ENABLE );
	DMA_DeInit( DMA2_Stream0 );
	DMA_InitStructure.DMA_Channel			= DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = ( uint32_t ) &ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr 	= ( uint32_t ) pusADCConvertedValues;
	DMA_InitStructure.DMA_DIR				= DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize 		= 8;
	DMA_InitStructure.DMA_PeripheralInc 	= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc			= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize	= DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode				= DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority			= DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode			= DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold 	= DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst		= DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst	= DMA_PeripheralBurst_Single;
	DMA_Init( DMA2_Stream0, &DMA_InitStructure );
	/* DMA2_Stream0 enable */
	DMA_Cmd( DMA2_Stream0, ENABLE );

	/* ADC initialization */
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1, ENABLE );

	/* Configure the  PA5 pin (ADC123_IN1)obst1
	                  PA6 pin (ADC123_IN2)obts2
	                  PA7 pin (ADC123_IN3)obst3
	                  PC4 pin (ADC123_IN1)obst4
	                  PC5 pin (ADC123_IN2)obst5
	                  PC3 pin (ADC123_IN3)obst7
	                  PC0 pin (ADC123_IN1)captcourant
	                  PB0 pin (ADC123_IN2)obst6 */
	/* PA port */
	GPIO_AdcInit.GPIO_Pin	=  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_AdcInit.GPIO_Mode 	= GPIO_Mode_AN;
	GPIO_AdcInit.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOA, &GPIO_AdcInit );
	/* PB */
	GPIO_AdcInit.GPIO_Pin 	=  GPIO_Pin_0;
	GPIO_AdcInit.GPIO_Mode 	= GPIO_Mode_AN;
	GPIO_AdcInit.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOB, &GPIO_AdcInit );
	/* PC */
	GPIO_AdcInit.GPIO_Pin 	=  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_3 | GPIO_Pin_0;
	GPIO_AdcInit.GPIO_Mode 	= GPIO_Mode_AN;
	GPIO_AdcInit.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOB, &GPIO_AdcInit );
	/* ADC Common configuration -- ADC_CCR Register */
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent ;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit( &ADC_CommonInitStructure );
	/* Input configuration */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_10b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 8;
	ADC_Init( ADC1, &ADC_InitStructure );
	/* ADC1 regular channel6 configuration -- ADCx->SMPR1, SMPR2 and ADCx->SQR1, SQR2, SQR3
	                channel8 */
	ADC_RegularChannelConfig( ADC1, ADC_Channel_5,  1, ADC_SampleTime_480Cycles );
	ADC_RegularChannelConfig( ADC1, ADC_Channel_6,  2, ADC_SampleTime_480Cycles );
	ADC_RegularChannelConfig( ADC1, ADC_Channel_7,  3, ADC_SampleTime_480Cycles );
	ADC_RegularChannelConfig( ADC1, ADC_Channel_14, 4, ADC_SampleTime_480Cycles );
	ADC_RegularChannelConfig( ADC1, ADC_Channel_15, 5, ADC_SampleTime_480Cycles );
	ADC_RegularChannelConfig( ADC1, ADC_Channel_8,  6, ADC_SampleTime_480Cycles );
	ADC_RegularChannelConfig( ADC1, ADC_Channel_13, 7, ADC_SampleTime_480Cycles );
	ADC_RegularChannelConfig( ADC1, ADC_Channel_10, 8, ADC_SampleTime_480Cycles );
	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd( ADC1, ENABLE );
	/* Enable ADC1 DMA */
	ADC_DMACmd( ADC1, ENABLE );
	/* Enable ADC1 */
	ADC_Cmd( ADC1, ENABLE );
	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConv( ADC1 );
}

/** Tests the correct initialization of the telemeters
@return 0 if test successful, 1 otherwise */
inline uint8_t ucTelemeterInitTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO ", ( signed char * ) "----", 0,
		MODULE, "ucTelemeterInitTest()", "Does nothing ATM" );

	return 0;
}

inline uint8_t ucTelemeterTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO ", pcTaskGetTaskName( NULL ),
		uxTaskPriorityGet( NULL ),
		MODULE, "ucTelemeterTest()", "Does nothing ATM" );

	return 0;
}

inline uint16_t usTelemeterGetDist( const enum telemeterId eTelemeterId )
{
uint16_t usTlmDist;

	if( !ucTelemeterIdValid( eTelemeterId ) )
	{
		usTlmDist = 0;
	}

	ulDebugMsg( xTaskGetTickCount(), "INFO ", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), MODULE, "usTelemeterGetDist()",
			"Measuring distance" );

	switch( eTelemeterId )
	{
	case TLM_FRT:
		/* PA5 */
		usTlmDist = 5 * pusADCConvertedValues[ 0 ];
	case TLM_BCK:
		/* PA6 */
		usTlmDist = 5 * pusADCConvertedValues[ 1 ];
	case TLM_RGT_FRT:
		/* PA7 */
		usTlmDist = 5 * pusADCConvertedValues[ 2 ];
	case TLM_RGT_BCK:
		/* PC4 */
		usTlmDist = 5 * pusADCConvertedValues[ 3 ];
	case TLM_LFT_FRT:
		/* PC5 */
		usTlmDist = 5 * pusADCConvertedValues[ 4 ];
	case TLM_LFT_BCK:
		/* PB0 */
		usTlmDist = 5 * pusADCConvertedValues[ 5 ];
	case TLM_BOT:
		/* PC3 */
		usTlmDist = 5 * pusADCConvertedValues[ 6 ];
	default:
		usTlmDist = 0;
	}

	return usTlmDist;
}

/** TLM_NONE is not considered a valid telemeterId */
inline uint8_t ucTelemeterIdValid( const enum telemeterId eTelemeterId )
{
	if( ( eTelemeterId == TLM_FRT )
			|| ( eTelemeterId == TLM_LFT_FRT ) || ( eTelemeterId == TLM_LFT_BCK )
			|| ( eTelemeterId == TLM_RGT_FRT ) || ( eTelemeterId == TLM_RGT_BCK )
			|| ( eTelemeterId == TLM_BCK ) || ( eTelemeterId == TLM_BOT ) )
	{
		ulDebugMsg( xTaskGetTickCount(), "INFO ", pcTaskGetTaskName( NULL ),
				uxTaskPriorityGet( NULL ), MODULE, "ucTelemeterIdValid()",
				"Identifier is valid" );
		return 1;
	}
	else
	{
		ulDebugMsg( xTaskGetTickCount(), "ERROR", pcTaskGetTaskName( NULL ),
				uxTaskPriorityGet( NULL ), MODULE, "ucTelemeterIdValid()",
				"Identifier is not valid !" );
		return 0;
	}
}

/** Doesn't tell which values are invalid.
@return 1 if every field is valid, 0 otherwise. */
inline uint8_t ucTelemeterDataValid( const struct telemeterData * const pxTelemeterData )
{
	if( prvTelemeterDistValid( &( pxTelemeterData->usFrontDist ) )
			&& prvTelemeterDistValid( &( pxTelemeterData->usLeftFrontDist ) )
			&& prvTelemeterDistValid( &( pxTelemeterData->usLeftBackDist ) )
			&& prvTelemeterDistValid( &( pxTelemeterData->usRightFrontDist ) )
			&& prvTelemeterDistValid( &( pxTelemeterData->usRightBackDist ) )
			&& prvTelemeterDistValid( &( pxTelemeterData->usBackDist ) )
			&& prvTelemeterDistValid( &( pxTelemeterData->usBottomDist ) ) )
	{
		ulDebugMsg( xTaskGetTickCount(), "INFO ", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), MODULE, "ucTelemeterDataValid()",
			"Data is valid" );
		return 1;
	}
	else
	{
		ulDebugMsg( xTaskGetTickCount(), "ERROR", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), MODULE, "ucTelemeterDataValid()",
			"Data is not valid !" );
		return 0;
	}
}

/** Updates each telemeter measurement */
inline void vTelemeterGetData( struct telemeterData * const pxNewTlmData )
{
	pxNewTlmData->usFrontDist		= usTelemeterGetDist( TLM_FRT );
	pxNewTlmData->usLeftFrontDist	= usTelemeterGetDist( TLM_LFT_FRT );
	pxNewTlmData->usLeftBackDist	= usTelemeterGetDist( TLM_LFT_BCK );
	pxNewTlmData->usRightFrontDist	= usTelemeterGetDist( TLM_RGT_FRT );
	pxNewTlmData->usRightBackDist	= usTelemeterGetDist( TLM_RGT_BCK );
	pxNewTlmData->usBackDist		= usTelemeterGetDist( TLM_BCK );
	pxNewTlmData->usBottomDist		= usTelemeterGetDist( TLM_BOT );
}
