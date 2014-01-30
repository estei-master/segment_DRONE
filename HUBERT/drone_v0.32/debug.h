#ifndef DEBUG_H
#define DEBUG_H

/** @file debug.h */

#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include "stdio.h"
#include "semihosting.h"

/******************************************************************************
 **	 	 CUSTOM DEBUG MACROS
 *****************************************************************************/

/* These macros are designed to be always visible by the compiler, so they
don't introduce weird bugs. Always false conditions are optimized out anyway */

/** Used to fill the pcModule argument of the ulDebugMsg() call. */
#define debugMODULE	"FreeRTOS    "

/* Debug macros - UART overrides semihosting if both are defined */
/** Debug mode output selection : UART6 (for stm32f4 Discovery board) */
#define debugUSE_DEBUG_UART				1
/** Debug mode output selection : Semihosting. Is overriden by UART if
debugUSE_DEBUG_UART is set. */
#define debugUSE_DEBUG_SEMIHOSTING		0

/** Redirects printf() to USART6 or to semihosting according to previous
macros. */
#define debugPUTCHAR( chr ) \
do { \
	if ( debugUSE_DEBUG_UART ) \
	{ \
		vDebugUARTPutchar( ( uint8_t ) ( chr ) ); \
	} \
	else if( debugUSE_DEBUG_SEMIHOSTING ) \
	{ \
		SH_SendChar( chr ); \
	} \
} while( 0 )

/** Initializes UART6 if debugUSE_DEBUG_UART is set. */
#define debugUART_INIT() \
do { \
	if ( debugUSE_DEBUG_UART ) \
	{ \
		vDebugUARTInit(); \
	} \
} while( 0 )

/******************************************************************************
 **	 	 FREERTOS TRACE MACROS
 *****************************************************************************/

/** Define the traceTASK_SWITCHED_IN() macro to output the voltage
associated with the task being selected to run on DAC1. */
#define traceTASK_SWITCHED_IN() { \
		/* DAC->DHR12L1 = (unsigned)pxCurrentTCB->pxTaskTag; */ \
		ulDebugMsg( xTaskGetTickCount(), "INFO ", pxCurrentTCB->pcTaskName, \
				uxTaskPriorityGet( pxCurrentTCB ), debugMODULE, \
				"traceTASK_SWITCHED_IN()", "Task running" ); \
}

#define traceTASK_SWITCHED_OUT() \
		ulDebugMsg( xTaskGetTickCount(), "INFO ", pxCurrentTCB->pcTaskName, \
				uxTaskPriorityGet( pxCurrentTCB ), debugMODULE, \
				"traceTASK_SWITCHED_OUT()", \
				"Task switched out of running state" )

/* For queues (messages/semaphores/mutexes) */
#define traceQUEUE_CREATE( pxNewQueue ) \
		ulDebugMsg( xTaskGetTickCount(), "INFO ", ( signed char * ) "----", 0, \
				debugMODULE, "traceQUEUE_CREATE()", \
				"Queue created successfully" )

/* For queues (messages/semaphores/mutexes) */
#define traceQUEUE_CREATE_FAILED() \
		ulDebugMsg( xTaskGetTickCount(), "ERROR", ( signed char * ) "----", 0, \
				debugMODULE, "traceQUEUE_CREATE_FAILED()", \
				"Queue creation failed !" )

/* For queues (messages/semaphores/mutexes take) */
#define traceQUEUE_RECEIVE( pxQueue ) \
		ulDebugMsg( xTaskGetTickCount(), "INFO ", ( signed char * ) "----", 0, \
				debugMODULE, "traceQUEUE_RECEIVE()", "Queue received" )

#define traceBLOCKING_ON_QUEUE_RECEIVE( pxQueue ) \
		ulDebugMsg( xTaskGetTickCount(), "INFO ", ( signed char * ) "----", 0, \
				debugMODULE, "traceBLOCKING_ON_QUEUE_RECEIVE()", \
				"Task blocking on queue (receive)" )

/* For queues (messages/semaphores/mutexes give) */
#define traceQUEUE_SEND( pxQueue ) \
		ulDebugMsg( xTaskGetTickCount(), "INFO ", ( signed char * ) "----", 0, \
				debugMODULE, "traceQUEUE_SEND()", "Sending to queue" )

#define traceBLOCKING_ON_QUEUE_SEND( pxQueue ) \
		ulDebugMsg( xTaskGetTickCount(), "INFO ", ( signed char * ) "----", 0, \
				debugMODULE, "traceBLOCKING_ON_QUEUE_SEND()", \
				"Task blocking on queue (send)" )

#define traceTASK_CREATE( pxNewTCB ) \
		ulDebugMsg( xTaskGetTickCount(), "INFO ", pxNewTCB->pcTaskName, \
				uxTaskPriorityGet( pxNewTCB ), debugMODULE, \
				"traceTASK_CREATE()", "Task created successfully" )

#define traceTASK_CREATE_FAILED() \
		ulDebugMsg( xTaskGetTickCount(), "ERROR", ( signed char * ) "----", 0, \
				debugMODULE, "traceTASK_CREATE_FAILED()", \
				"Task creation failed !" )

#define traceTASK_DELAY_UNTIL() \
		ulDebugMsg( xTaskGetTickCount(), "INFO ", pxCurrentTCB->pcTaskName, \
				uxTaskPriorityGet( pxCurrentTCB ), debugMODULE, \
				"traceTASK_DELAY_UNTIL()", "Task delayed (until)" )

#define traceTASK_DELAY() \
		ulDebugMsg( xTaskGetTickCount(), "INFO ", pxCurrentTCB->pcTaskName, \
				uxTaskPriorityGet( pxCurrentTCB ), debugMODULE, \
				"traceTASK_DELAY()", "Task delayed" )

/** Macro used to check parameter consistency inside FreeRTOS */
/* Triggered at tasks.c line 506 (alignment of the initialized stack) */
#define configASSERT( x ) if( ( x ) == 0 ) \
		ulDebugMsg( xTaskGetTickCount(), "ERROR", ( signed char * ) "----", 0, \
				debugMODULE, "configASSERT()", "failed : " #x " == 0" )

/******************************************************************************
 **	 	 EXPORTED FUNCTIONS DECLARATION
 *****************************************************************************/

void vDebugUARTInit( void );
void vDebugUARTPutchar( uint8_t ucChar );
uint32_t ulDebugMsg( unsigned long ulTickCnt, char * pcTag,
		signed char * pcTask, unsigned long ulTaskPrio, char * pcModule,
		char * pcFunc, char * pcMessage );

#endif /* DEBUG_H */
