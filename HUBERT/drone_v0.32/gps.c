#include "gps.h"
#include "task.h"

/******************************************************************************
 **	 	 DEBUGGING MACROS
 *****************************************************************************/

/** Used to fill the pcModule argument of the ulDebugMsg() call. */
#define MODULE		"gps         "

/******************************************************************************
 **	 	 PUBLIC FUNCTIONS DEFINITION
 *****************************************************************************/

inline void vGPSInit( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO ", ( signed char * ) "---", 0,
			MODULE, "vGPSInit", "Does nothing ATM" );
}

/** Tests the correct initialization of the GPS
@return 0 if test successful, 1 otherwise */
inline uint8_t ucGPSInitTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO ", ( signed char * ) "---", 0,
			MODULE, "ucGPSInitTest", "Does nothing ATM" );

	return 0;
}

inline uint8_t ucGPSTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO ", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), MODULE, "ucGPSTest",
			"Does nothing ATM" );

	return 0;
}

/** Receives a GPS frame and updates its parameter. */
inline void vGPSGetData( struct GPSData *pxGPSData )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO ", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), MODULE, "vGPSGetData",
			"Does nothing ATM" );
}

/** Tests whether its parameter is valid GPS data */
inline enum GPSErrorMask eGPSDataValid( const struct GPSData * const pxNewGPSData )
{
	/* TODO : Implement it */
	return GPS_ERR_NONE;
}
