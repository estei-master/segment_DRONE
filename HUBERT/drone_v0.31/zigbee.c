#include "zigbee.h"
#include "task.h"

inline void vZigbeeInit( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", ( signed char * ) "-", 0,
			"zigbee", "vZigbeeInit()", "Does nothing ATM" );
}

inline uint8_t ucZigbeeInitTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", ( signed char * ) "-", 0,
			"zigbee", "ucZigbeeInitTest()", "Does nothing ATM" );

	return 0;
}

inline uint8_t ucZigbeeTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "zigbee", "ucZigbeeTest()",
			"Does nothing ATM" );

	return 0;
}

inline void vZigbeeSendData( const uint8_t * const pucData )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "zigbee", "vZigbeeSend()",
			"Does nothing ATM" );
}

inline void vZigbeeReceiveData( struct zigbeeData * const pxZigbeeData )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "zigbee", "vZigbeeReceive()",
			"Does nothing ATM" );
}
