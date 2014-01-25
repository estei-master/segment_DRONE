#include "telemeter.h"
#include "task.h"

/** Used to fill the pcModule argument of the ulDebugMsg() call. */
#define MODULE		"telemeter   "

/** Range : 300 à 5000 (mm). If distance is lower/higher, values stick to the
extremes. */
static inline uint8_t prvTelemeterDistValid( const uint16_t * const pusDistance )
{
uint16_t usHighMarginMm = 500;
uint16_t usLowMarginMm = 50;

	if( ( *pusDistance > 5000 + usHighMarginMm )
			|| ( *pusDistance < 300 + usLowMarginMm ) )
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
	ulDebugMsg( xTaskGetTickCount(), "TODO ", ( signed char * ) "---", 0,
		MODULE, "vTelemeterInit()", "Does nothing ATM" );
}

/** Tests the correct initialization of the telemeters
@return 0 if test successful, 1 otherwise */
inline uint8_t ucTelemeterInitTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO ", ( signed char * ) "---", 0,
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
	ulDebugMsg( xTaskGetTickCount(), "TODO ", pcTaskGetTaskName( NULL ),
		uxTaskPriorityGet( NULL ), MODULE, "usTelemeterGetDist()",
		"Does nothing ATM" );

	return 0;
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
