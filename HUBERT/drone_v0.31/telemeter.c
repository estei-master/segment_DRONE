#include "telemeter.h"
#include "task.h"

/** Range : 300 à 5000 (mm). If distance is lower/higher, values stick to the
extremes. */
static inline uint8_t prvTelemeterIsDistValid( const uint16_t * const pusDistance )
{
uint16_t usHighMarginMm = 500;
uint16_t usLowMarginMm = 50;

	if( ( *pusDistance > 5000 + usHighMarginMm )
			|| ( *pusDistance < 300 + usLowMarginMm ) )
	{
		ulDebugMsg( xTaskGetTickCount(), "ERROR",
			pcTaskGetTaskName( NULL ), uxTaskPriorityGet( NULL ), "telemeter",
			"prvTelemeterIsDistValid()", "Data outside of valid range !" );

		return 0;
	}
	else
	{
		ulDebugMsg( xTaskGetTickCount(), "INFO",
			pcTaskGetTaskName( NULL ), uxTaskPriorityGet( NULL ), "telemeter",
			"prvTelemeterIsDistValid()", "Data inside valid range !" );

		return 1;
	}
}

inline void vTelemeterInit( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", ( signed char * ) "-", 0,
		"telemeter", "vTelemeterInit()", "Does nothing ATM" );
}

inline uint8_t ucTelemeterInitTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", ( signed char * ) "-", 0,
		"telemeter", "ucTelemeterInitTest()", "Does nothing ATM" );

	return 0;
}

inline uint8_t ucTelemeterTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
		uxTaskPriorityGet( NULL ),
		"telemeter", "ucTelemeterTest()", "Does nothing ATM" );

	return 0;
}

inline uint16_t usTelemeterGetDist( const enum telemeterId eTelemeterId )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
		uxTaskPriorityGet( NULL ), "telemeter", "usTelemeterGetDist()",
		"Does nothing ATM" );

	return 0;
}

/** TLM_NONE is not considered a valid telemeterId */
inline uint8_t ucTelemeterIsIdValid( const enum telemeterId eTelemeterId )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
		uxTaskPriorityGet( NULL ), "telemeter", "ucTelemeterIsIdValid()",
		"Does nothing ATM" );

	if( ( eTelemeterId == TLM_FRT )
			|| ( eTelemeterId == TLM_LFT_FRT ) || ( eTelemeterId == TLM_LFT_BCK )
			|| ( eTelemeterId == TLM_RGT_FRT ) || ( eTelemeterId == TLM_RGT_BCK )
			|| ( eTelemeterId == TLM_BCK ) || ( eTelemeterId == TLM_BOT ) )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/** Doesn't tell which values are invalid, return 1 if every field is valid,
0 otherwise. */
inline uint8_t ucTelemeterIsDataValid( const struct telemeterData * const pxTelemeterData )
{
	if( prvTelemeterIsDistValid( &( pxTelemeterData->usFrontDist ) )
			&& prvTelemeterIsDistValid( &( pxTelemeterData->usLeftFrontDist ) )
			&& prvTelemeterIsDistValid( &( pxTelemeterData->usLeftBackDist ) )
			&& prvTelemeterIsDistValid( &( pxTelemeterData->usRightFrontDist ) )
			&& prvTelemeterIsDistValid( &( pxTelemeterData->usRightBackDist ) )
			&& prvTelemeterIsDistValid( &( pxTelemeterData->usBackDist ) )
			&& prvTelemeterIsDistValid( &( pxTelemeterData->usBottomDist ) ) )
	{
		ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "telemeter", "ucTelemeterIsDataValid()",
			"Data valid" );
		return 1;
	}
	else
	{
		ulDebugMsg( xTaskGetTickCount(), "ERROR", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "telemeter", "ucTelemeterIsDataValid()",
			"Data invalid" );
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
