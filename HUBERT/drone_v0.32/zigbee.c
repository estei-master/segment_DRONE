/** @file zigbee.c */

#include "zigbee.h"
#include "task.h"

/** Used to fill the pcModule argument of the ulDebugMsg() call. */
#define MODULE		"zigbee      "

inline void vZigbeeInit( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO ", ( signed char * ) "----", 0,
			MODULE, "vZigbeeInit()", "Does nothing ATM" );
}

/** Tests the correct initialization of the zigbee communication
@return 0 if test successful, 1 otherwise */
inline uint8_t ucZigbeeInitTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO ", ( signed char * ) "----", 0,
			MODULE, "ucZigbeeInitTest()", "Does nothing ATM" );

	return 0;
}

inline uint8_t ucZigbeeTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO ", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), MODULE, "ucZigbeeTest()",
			"Does nothing ATM" );

	return 0;
}

inline void vZigbeeSendData( const uint8_t * const pucData )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO ", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), MODULE, "vZigbeeSend()",
			"Does nothing ATM" );
}

inline void vZigbeeReceiveData( struct zigbeeData * const pxZigbeeData )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO ", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), MODULE, "vZigbeeReceive()",
			"Does nothing ATM" );
}
