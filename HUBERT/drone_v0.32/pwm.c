#include "pwm.h"
#include "task.h"

inline void vPWMInit( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", ( signed char * ) "---", 0, "pwm",
		"vPWMInit", "Does nothing ATM" );
}

inline uint8_t ucPWMInitTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", ( signed char * ) "---", 0, "pwm",
		"ucPWMInitTest", "Does nothing ATM" );

	return 0;
}

inline uint8_t ucPWMTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
		uxTaskPriorityGet( NULL ), "pwm", "ucPWMTest", "Does nothing ATM" );

	return 0;
}

inline void vPWMSet( enum PWMId ePWMId )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
		uxTaskPriorityGet( NULL ), "pwm", "vPWMSet", "Does nothing ATM" );
}
