/** @file pwm.c */

#include "pwm.h"
#include "task.h"

/** Used to fill the pcModule argument of the ulDebugMsg() call. */
#define MODULE		"pwm         "

inline void vPWMInit( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", ( signed char * ) "---", 0,
			MODULE, "vPWMInit", "Does nothing ATM" );
}

inline uint8_t ucPWMInitTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", ( signed char * ) "---", 0,
			MODULE, "ucPWMInitTest", "Does nothing ATM" );

	return 0;
}

inline uint8_t ucPWMTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), MODULE, "ucPWMTest",
			"Does nothing ATM" );

	return 0;
}

inline void vPWMSet( enum PWMId ePWMId )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), MODULE, "vPWMSet", "Does nothing ATM" );
}
