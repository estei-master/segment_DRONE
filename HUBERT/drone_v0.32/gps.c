#include "gps.h"
#include "task.h"

inline void vGPSInit( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", ( signed char * ) "---", 0, "gps",
			"vGPSInit", "Does nothing ATM" );
}

/** Tests the correct initialization of the GPS
@return 0 if test successful, 1 otherwise */
inline uint8_t ucGPSInitTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", ( signed char * ) "---", 0, "gps",
			"ucGPSInitTest", "Does nothing ATM" );

	return 0;
}

inline uint8_t ucGPSTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "gps", "ucGPSTest", "Does nothing ATM" );

	return 0;
}

inline void vGPSGetData( struct GPSData *pxGPSData )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "gps", "vGPSGetData", "Does nothing ATM" );
}

inline enum GPSErrorMask eGPSIsDataValid( const struct GPSData * const pxNewGPSData )
{
	/* TODO : Implement it */
	return GPS_ERR_NONE;
}
