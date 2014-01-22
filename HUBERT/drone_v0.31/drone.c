/******************************************************************************
 ** 	DRONE CONTROL PROGRAM - USING FREERTOS REAL-TIME KERNEL
 **
 **		TODO : Detailed description here
 **
 **		1 tabulation = 4 spaces
 **
 **		Disclaimer : The coding style used herein is the one of FreeRTOS, any
 **		ocular injury resulting from reading this code is the sole
 **		responsibility of the reader. You've been warned.
 *****************************************************************************/

/* TODO : watchdog on prvFlightCtrlTask() in non-ground mode
TODO : test FreeRTOS API function return value
TODO : measure maximum critical section time
TODO : Adapt variables type to their size */

/******************************************************************************
 **	 	 INCLUDES
 *****************************************************************************/

/*****  Standard includes  ***************************************************/

/*****  Kernel includes  *****************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/*****  Project includes  ****************************************************/

#include "drone.h"

/** Peripheral includes */
#include "zigbee.h"
#include "telemeter.h"
//#include "battery.h"	/* Doesn't exist : no specific file needed */
#include "gps.h"
#include "imu.h"
/** Motor control libraries */
#include "pwm.h"
#include "pid.h"
#include "flight.h"

/******************************************************************************
 **		TASK PRIORITY MACROS
 *****************************************************************************/

#define droneBATTERY_MONITORING_PRIO		( tskIDLE_PRIORITY + 4 )
#define droneGPS_RECEIVE_PRIO				( tskIDLE_PRIORITY + 2 )
#define droneZIGBEE_RECEIVE_PRIO			( tskIDLE_PRIORITY + 2 )
#define droneVIDEO_TOGGLE_PRIO				( tskIDLE_PRIORITY + 1 )
#define droneDETECT_OBSTACLE_PRIO			( tskIDLE_PRIORITY + 4 )
#define droneFLIGHT_CTRL_PRIO				( tskIDLE_PRIORITY + 3 )

/******************************************************************************
 **	 	 INTERNAL STRUCTURES AND ENUMERATIONS DECLARATION
 *****************************************************************************/

/** Mutually exclusive drone states behavior. */
enum droneFltState
{
	/** Initialization phase */
	STATE_STARTING,
	/** Initialization successful */
	STATE_GROUND_RDY,
	/** Error while landed */
	STATE_GROUND_ERR,
	/** Must be landed */
	STATE_TAKEOFF,
	/** Normal flight, stationary flight if no flight command received */
	STATE_FLIGHT,
	/** PID parameters autotuning */
	STATE_AUTOTUNING,
	/** Error while flying - triggers emergency landing */
	STATE_FLIGHT_ERR,
	/** Must be flying */
	STATE_LANDING,
};

/** Mainly used for status transmission - Used as a mask */
enum droneError
{
	/** Everything is shiny */
	DRN_ERR_NONE		= 0x00,
	/** Timeout of telemeter data */
	DRN_ERR_TLM_TOUT	= 0x01,
	/** Data out of valid range */
	DRN_ERR_TLM_INVAL	= 0x02,
	/** Timeout of IMU data */
	DRN_ERR_IMU_TOUT	= 0x04,
	/** Data out of valid range */
	DRN_ERR_IMU_INVAL	= 0x08,
	/** Timeout of GPS data */
	DRN_ERR_GPS_TOUT	= 0x10,
	/** Data out of valid range */
	DRN_ERR_GPS_INVAL	= 0x20,
	/** Timeout of telemeter data */
	DRN_ERR_CMD_TOUT	= 0x40,
	/** Timeout of battery level */
	DRN_ERR_BATT_TOUT	= 0x80,
	/** Data out of valid range */
	DRN_ERR_BATT_INVAL	= 0x100,
	/** Timeout of base-drone communication data */
	DRN_ERR_RX_TOUT		= 0x200,
	/** Initialization errors */
//	Covered by STATE_STARTING and normal errors
	/** Configuration invalid */
	DRN_ERR_CONFIG		= 0x400,
	/** Invalid command */
	DRN_ERR_CMD			= 0x800,
//	DRN_ERR_INIT_RX		= 0x1000,
//	DRN_ERR_INIT_BATT	= 0x2000,
//	DRN_ERR_INIT_TLM	= 0x4000,
	DRN_ERR_INIT		= 0x8000,
	/** No video error because we don't have much to do with it */
};

struct droneState
{
	/** Drone state */
	enum droneFltState eFltState;
	/** Error codes for transmission */
	enum droneError eErrorMask;
};

/*****  Monitored drone parameters structs  **********************************/

struct droneIMUWrapper
{
	struct IMUData xIMUData;
	/** Time of last update */
	portTickType xUpdateTime;
};

struct droneZigbeeWrapper
{
	/** Data received from the base used by other tasks */
	struct flightCommand xFlightCmd;
	/** Signal level */
	uint32_t ulSignalLvl;
	/** Time of last Zigbee reception */
	portTickType xZigbeeUpdateTime;
	/** Time of last command update */
	portTickType xCmdUpdateTime;
};

struct droneGPSWrapper
{
	/** GPS data */
	struct GPSData xGPSData;
	/** Time of last update */
	portTickType xUpdateTime;
};

/** Telemeter distances, identifiers of telemeters having detected an obstacle
and time of last update */
struct droneTelemeterWrapper
{
	/** Measured distances */
	struct telemeterData xTelemeterData;
	/** Time of last update */
	portTickType xUpdateTime;
};

struct droneBatteryWrapper
{
	uint32_t ulPowerLvl;
	/** Time of last update */
	portTickType xUpdateTime;
};

/******************************************************************************
 **	 	 GLOBAL VARIABLES DECLARATION
 *****************************************************************************/

/*****  Read-only data structures  *******************************************/

/** Defined in flight.h */
struct flightMovement xStationaryFlightMvt;
struct flightCommand xStationaryFlightCmd;

/** Default drone configuration */
const struct droneConfig xDfltDroneConfig = {
	/** flight limits */
	.ulMaxAngle					= 25,	/* TODO : sensible value */
	.ulTakeoffAltitude			= 1000,	/* TODO : sensible value */
	.usMinAltitude				= 2000,	/* TODO : sensible value */
	.ulCritBatteryLvl			= 15,	/* TODO : sensible value */
	.usCritObstacleDist			= 750,	/* TODO : sensible value */
	.ulCritZigbeeSignalLvl		= 5,	/* TODO : sensible value */
	.ulRefAltitude				= 0,
	/** task periods (ms) */
	.xBatteryMonitoringPeriod	= 30000 / portTICK_RATE_MS,
	.xDetectObstaclePeriod		= 50 / portTICK_RATE_MS,
	.xFlightCtrlPeriod			= 50 / portTICK_RATE_MS,
	.xGPSReceivePeriod			= 10000 / portTICK_RATE_MS,
	.xZigbeeReceivePeriod		= 500 / portTICK_RATE_MS,
	/** Timeouts for data validity */
	.xIMUDataTimeout			= 3,
	.xZigbeeCmdTimeout 			= 2,
	.xZigbeeReceiveTimeout		= 10,
	.xTelemeterTimeout 			= 2,
	.xBatteryTimeout			= 2,
	.xGPSTimeout				= 10,
};

/** Initial drone state */
const struct droneState xInitDroneState = {
	.eFltState = STATE_STARTING,
	.eErrorMask = DRN_ERR_NONE,
};

/*****  Message queues  ******************************************************/

static xSemaphoreHandle xFlightCtrlSem = NULL;

/*****  Task Handles  ********************************************************/

/** Used to associate an analog voltage "tag" to each task - see main() */
xTaskHandle xFlightCtrlHandle;
xTaskHandle xDetectObstacleHandle;
xTaskHandle xZigbeeReceiveHandle;
xTaskHandle xBatteryMonitoringHandle;
xTaskHandle xGPSReceiveHandle;
/* The idle task handle is retrieved from a FreeRTOS API function */

/*****  Monitored drone parameters  ******************************************/

/* TODO : think of an initialization value (Magic number ?) */
/** GPS data and time of last update */
static struct droneGPSWrapper xGPSWrapper;
/** Flight command received from the base, time of last command update and of
last reception. */
static struct droneZigbeeWrapper xZigbeeWrapper;
/** Attitude measured by the IMU module and time of last update */
static struct droneIMUWrapper xIMUWrapper;
/** Battery level and time of last update */
static struct droneBatteryWrapper xBatteryWrapper;
/** Telemeter measures and time of last update */
static struct droneTelemeterWrapper xTelemeterWrapper;
/** Must be updated through prvSafeSetDroneConfig() */
static struct droneConfig xDroneConfig;
/** Used to specify the current behavior of each task, must be updated through
prvSafeSetDroneState() for preemption safety. */
struct droneState xDroneState;

/******************************************************************************
 **	 	 FUNCTIONS DECLARATION
 *****************************************************************************/

/*****  Real-time tasks  *****************************************************/

static void prvDetectObstacleTask( void *pvParam );
static void prvFlightCtrlTask( void *pvParam );
static void prvZigbeeReceiveTask( void *pvParam );
static void prvBatteryMonitoringTask( void *pvParam );
static void prvGPSReceiveTask( void *pvParam );

/*****  Utility routines  ****************************************************/

/** Initialization routines */
static inline void prvSetupHardware( void );
static inline void prvInitDroneConfig( void );
static inline enum droneError prvInitTestPeriph( void );
static inline void prvInitAltitude( const struct droneIMUWrapper * const pxCurrIMUWrapper,
		struct droneConfig * const pxCurrDroneConfig );

/** Thread-safe global variable update routines */
static inline void prvSafeSetIMUData( const struct IMUData * const pxNewIMUData );
static inline void prvSafeSetGPSData( const struct GPSData * const pxNewGPSData );
static inline void prvSafeSetZigbeeData( const struct flightCommand * const pxNewFltCmd );
static inline void prvSafeSetBatteryData( const uint32_t * const pulNewBatteryLvl );
static inline void prvSafeSetDroneConfig( const struct droneConfig * const pxNewDroneConfig );
static inline void prvSafeSetDroneState( const struct droneState * const xNewDroneState );
static inline void prvSafeSetTelemeterData( const struct telemeterData * const pxNewTelemeterData );

/** Thread-safe global variable read routines */
static inline void prvSafeGetIMUData( struct droneIMUWrapper * const pxNewIMUWrapper );
static inline void prvSafeGetGPSData( struct droneGPSWrapper * const pxNewGPSWrapper );
static inline void prvSafeGetZigbeeData( struct droneZigbeeWrapper * const pxNewZigbeeWrapper );
static inline void prvSafeGetBatteryData( struct droneBatteryWrapper * const pxNewBattWrapper );
static inline void prvSafeGetTelemeterData( struct droneTelemeterWrapper * const pxNewTlmWrapper );
static inline void prvSafeGetDroneConfig( struct droneConfig * const pxNewConfig );
static inline void prvSafeGetDroneState( struct droneState * const pxNewState );

/** Video control routines */
static inline void prvEnableVideo( void );
static inline void prvDisableVideo( void );
//static inline void prvToggleVideo( void );

/** Tests routines */
static inline uint8_t prvTakeoffDone( const struct IMUData * const pxCurrIMUData,
		const struct droneConfig * const pxCurrDroneConfig );
static inline uint8_t prvLandingDone( const struct IMUData * const pxCurrIMUData,
		const struct droneConfig * const pxCurrDroneConfig );
static inline uint8_t prvObstacleDetected( const struct droneTelemeterWrapper * const pxCurrTelemeterWrapper );
static inline uint8_t prvBatteryCritical( const uint32_t * const pulPowerLvl,
		const struct droneConfig * const pxCurrDroneConfig );
static inline uint8_t prvZigbeeSignalCritical( const struct droneZigbeeWrapper * const pxCurrZigbeeWrapper,
		const struct droneConfig * const pxCurrDroneConfig );
static inline uint8_t prvTelemeterTimedout( const struct droneTelemeterWrapper * const pxCurrTelemeterWrapper,
		const struct droneConfig * const pxCurrDroneConfig );
static inline uint8_t prvBatteryTimedout( const struct droneBatteryWrapper * const pxCurrBatteryWrapper,
		const struct droneConfig * const pxCurrDroneConfig );
static inline uint8_t prvIMUTimedout( const struct droneIMUWrapper * const pxCurrIMUWrapper,
		const struct droneConfig * const pxCurrDroneConfig );
static inline uint8_t prvGPSTimedout( const struct droneGPSWrapper * const pxCurrGPSWrapper,
		const struct droneConfig * const pxCurrDroneConfig );
static inline uint8_t prvZigbeeTimedout( const struct droneZigbeeWrapper * const pxCurrZigbeeWrapper,
		const struct droneConfig * const pxCurrDroneConfig );
static inline uint8_t prvFltCmdTimedout( const struct droneZigbeeWrapper * const pxCurrZigbeeWrapper,
		const struct droneConfig * const pxCurrDroneConfig );
static inline uint8_t ucBatteryInitTest( void );
static inline void prvHandleTimouts( const struct droneTelemeterWrapper * const pxCurrTelemeterWrapper,
		const struct droneBatteryWrapper * const pxCurrBatteryWrapper,
		const struct droneIMUWrapper * const pxCurrIMUWrapper,
		const struct droneGPSWrapper * const pxCurrGPSWrapper,
		struct droneZigbeeWrapper * const pxCurrZigbeeWrapper,
		const struct droneConfig * const pxCurrDroneConfig,
		struct droneState * const pxCurrDroneState );
static inline uint8_t prvDroneConfigValid( struct droneConfig *pxNewDroneConfig );
static inline uint8_t prvBatteryValid( const uint32_t * const pulNewBattLvl );

static inline void prvResetDroneConfig( void );
static inline void prvSelectErrorState( struct droneState * const pxCurrState );
static inline void prvGetBatteryLvl( uint32_t *pulBatteryLvl );
static inline void prvSendStatus( void );
static inline void prvSendError( const struct droneState * const xCurrDroneState );
static inline void prvSendConfig( const struct droneConfig * const pxDroneConfig );
static inline void prvReboot( void );
static inline void prvShutdown( void );

/******************************************************************************
 **	 	 MAIN FUNCTION
 *****************************************************************************/

int main( void )
{
/* Local variable for commands on error */
struct zigbeeData xLocZigbeeData;

	/* Clock and NVIC setup */
	prvSetupHardware();

	/* Initialize UART6 if debugUSE_DEBUG_UART is set */
	debugUART_INIT();

	/* Initialize xDroneState and xDroneConfig global variable to default
	behavior */
	prvInitDroneConfig();

	/* Setup and check correct behavior of each peripheral */
	xDroneState.eErrorMask = prvInitTestPeriph();

	/* Error on GPS only is ignored altogether */
	if( ( xDroneState.eErrorMask != DRN_ERR_NONE )
			&& ( xDroneState.eErrorMask != DRN_ERR_GPS_TOUT ) )
	{
		ulDebugMsg( xTaskGetTickCount(), "ERROR",
				 ( signed char * ) "-", 0, "drone", "main()",
				"Bad initialization of some critical peripheral !" );

		/* We should add some tests there in case the peripheral eventually
		responds correctly, to not get stuck rebooting. */
		xDroneState.eFltState = STATE_GROUND_ERR;

		do
		{
			prvSendError( &xDroneState );
			vZigbeeReceiveData( &xLocZigbeeData );
		}
		while( ( xLocZigbeeData.eCmdId != CMD_REBOOT )
				|| ( xLocZigbeeData.eCmdId != CMD_SHUTDOWN ) );

		/* Execute the requested action */
		switch( xLocZigbeeData.eCmdId )
		{
		case CMD_REBOOT:
			prvReboot();
			break;
		case CMD_SHUTDOWN:
			prvShutdown();
			break;
		default:
			/* Just in case… */
			prvShutdown();
			break;
		}
	}

	/* Binary semaphore used to wake up the flight control task */
	ulDebugMsg( xTaskGetTickCount(), "INFO", ( signed char * ) "-", 0, "drone",
			"main()", "Creating xFlightCtrlSem" );
	/* Same as vSemaphoreCreateBinary( xFlightCtrlSem ); with a semaphore
	initialized as taken instead of free. */
	xFlightCtrlSem = xQueueCreate( ( unsigned portBASE_TYPE ) 1,
			semSEMAPHORE_QUEUE_ITEM_LENGTH );
	if( xFlightCtrlSem == NULL )
	{
		ulDebugMsg( xTaskGetTickCount(), "ERROR", ( signed char * ) "-", 0,
				"drone", "main()", "xFlightCtrlSem creation failed !" );

		/* Anonymous initialization error */
		xDroneState.eFltState = STATE_GROUND_ERR;
		xDroneState.eErrorMask |= DRN_ERR_INIT;

		do
		{
			prvSendError( &xDroneState );
			vZigbeeReceiveData( &xLocZigbeeData );
		}
		while( ( xLocZigbeeData.eCmdId != CMD_REBOOT )
				|| ( xLocZigbeeData.eCmdId != CMD_SHUTDOWN ) );

		/* Execute the requested action */
		switch( xLocZigbeeData.eCmdId )
		{
		case CMD_REBOOT:
			prvReboot();
			break;
		case CMD_SHUTDOWN:
			prvShutdown();
			break;
		default:
			/* Just in case… */
			prvShutdown();
			break;
		}
	}

	ulDebugMsg( xTaskGetTickCount(), "INFO", ( signed char * ) "-", 0, "drone",
			"main()", "Creating prvDetectObstacleTask" );
	xTaskCreate( prvDetectObstacleTask, ( signed char * ) "Obs",
			configMINIMAL_STACK_SIZE, NULL,
			droneDETECT_OBSTACLE_PRIO, &xDetectObstacleHandle );

	ulDebugMsg( xTaskGetTickCount(), "INFO", ( signed char * ) "-", 0, "drone",
			"main()", "Creating prvBatteryMonitoringTask" );
	xTaskCreate( prvBatteryMonitoringTask,
			( signed char * ) "Bat",
			configMINIMAL_STACK_SIZE, NULL,
			droneBATTERY_MONITORING_PRIO, &xBatteryMonitoringHandle );

	ulDebugMsg( xTaskGetTickCount(), "INFO", ( signed char * ) "-", 0, "drone",
			"main()", "Creating prvFlightCtrlTask" );
	xTaskCreate( prvFlightCtrlTask, ( signed char * ) "Flt",
			configMINIMAL_STACK_SIZE, NULL,
			droneFLIGHT_CTRL_PRIO, &xFlightCtrlHandle );

	ulDebugMsg( xTaskGetTickCount(), "INFO", ( signed char * ) "-", 0, "drone",
			"main()", "Creating prvZigbeeReceiveTask" );
	xTaskCreate( prvZigbeeReceiveTask, ( signed char * ) "Zig",
			configMINIMAL_STACK_SIZE, NULL,
			droneZIGBEE_RECEIVE_PRIO, &xZigbeeReceiveHandle );

	ulDebugMsg( xTaskGetTickCount(), "INFO", ( signed char * ) "-", 0, "drone",
			"main()", "Creating prvGPSReceiveTask" );

	/* At this point, the drone is hopefully correctly initialized and waiting
	for orders */
	xDroneState.eFltState = STATE_GROUND_RDY;
	/* TODO : buzzer ready signal */

	xTaskCreate( prvGPSReceiveTask, ( signed char * ) "GPS",
			configMINIMAL_STACK_SIZE, NULL,
			droneGPS_RECEIVE_PRIO, &xGPSReceiveHandle );

	/* TODO : check validity */
	/* Test whether tasks where correctly created */
	if( ( xDetectObstacleHandle == NULL )
			|| ( xFlightCtrlHandle == NULL )
			|| ( xZigbeeReceiveHandle == NULL )
			|| ( xBatteryMonitoringHandle == NULL )
			|| ( xGPSReceiveHandle == NULL ) )
	{
		ulDebugMsg( xTaskGetTickCount(), "ERROR", ( signed char * ) "-", 0, "drone",
				"main()", "Tasks creation failed !" );

		/* TODO : Refactor into a routine */
		/* Anonymous initialization error */
		xDroneState.eFltState = STATE_GROUND_ERR;
		xDroneState.eErrorMask |= DRN_ERR_INIT;

		do
		{
			prvSendError( &xDroneState );
			vZigbeeReceiveData( &xLocZigbeeData );
		}
		while( ( xLocZigbeeData.eCmdId != CMD_REBOOT )
				|| ( xLocZigbeeData.eCmdId != CMD_SHUTDOWN ) );

		/* Execute the requested action */
		switch( xLocZigbeeData.eCmdId )
		{
		case CMD_REBOOT:
			prvReboot();
			break;
		case CMD_SHUTDOWN:
			prvShutdown();
			break;
		default:
			/* Just in case… */
			prvShutdown();
			break;
		}
	}

	/* Tasks are associated with a voltage on whatever pin is defined in the
	traceTASK_SWITCHED_IN() macro (in FreeRTOSConfig.h).
	Sorted by increasing priority then frequency */
//	vTaskSetApplicationTaskTag( xTaskGetIdleTaskHandle(), ( void * ) 1 );
//	vTaskSetApplicationTaskTag( xGPSReceiveHandle, ( void * ) 2 );
//	vTaskSetApplicationTaskTag( xZigbeeReceiveHandle, ( void * ) 3 );
//	vTaskSetApplicationTaskTag( xFlightCtrlHandle, ( void * ) 4 );
//	vTaskSetApplicationTaskTag( xBatteryMonitoringHandle, ( void * ) 5 );
//	vTaskSetApplicationTaskTag( xDetectObstacleHandle, ( void * ) 6 );

	ulDebugMsg( xTaskGetTickCount(), "INFO", ( signed char * ) "-", 0, "drone",
			"main()", "Starting scheduler" );
	vTaskStartScheduler();
	ulDebugMsg( xTaskGetTickCount(), "ERROR", ( signed char * ) "-", 0,
			"drone", "main()", "Scheduler returned !" );

	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was insufficient FreeRTOS heap memory available for the idle
	and/or timer tasks to be created. */

	xDroneState.eFltState = STATE_GROUND_ERR;
	xDroneState.eErrorMask |= DRN_ERR_INIT;

	do
	{
		prvSendError( &xDroneState );
		vZigbeeReceiveData( &xLocZigbeeData );
	}
	while( ( xLocZigbeeData.eCmdId != CMD_REBOOT )
			|| ( xLocZigbeeData.eCmdId != CMD_SHUTDOWN ) );

	/* Execute the requested action */
	switch( xLocZigbeeData.eCmdId )
	{
	case CMD_REBOOT:
		prvReboot();
		break;
	case CMD_SHUTDOWN:
		prvShutdown();
		break;
	default:
		/* Just in case… */
		prvShutdown();
		break;
	}

	while( 1 )
	{
		/* Nothing to do */
	}
}

/******************************************************************************
 **	 	 FUNCTION DEFINITIONS
 *****************************************************************************/

/*****  Real-time tasks  *****************************************************/

/** Task to update xTelemeterWrapper global variable, sets
eDroneState.eErrorMask with DRN_ERR_TLM_INVAL if measured telemeter distances
are invalid, and wakes prvFlightCtrlTask() if an obstacle is detected */
static void prvDetectObstacleTask( void *pvParam )
{
portTickType xNextWakeTime = xTaskGetTickCount();
struct telemeterData xNewTlmData;
struct droneConfig xCurrConfig;
struct droneState xCurrDroneState;

	while( 1 )
	{
		/* Secure xCurrDroneConfig update */
		prvSafeGetDroneConfig( &xCurrConfig );
		/* Update local copy of xDroneState */
		prvSafeGetDroneState( &xCurrDroneState );

		/* TODO :	- Check them all with a telemeter specific function,
					  returning a mask for invalid measures.
					- Update xDroneState.eErrorMask (set DRN_ERR_TLM_INVAL or
					  reset it) */

		/* Update each telemeter measurement */
		vTelemeterGetData( &xNewTlmData );
		/* Check their validity */
		if( ucTelemeterIsDataValid( &xNewTlmData ) )
		{
			/* Register success */
			xCurrDroneState.eErrorMask &= ~DRN_ERR_TLM_INVAL;
			prvSafeSetDroneState( &xCurrDroneState );

			if( xNewTlmData.usFrontDist < xCurrConfig.usCritObstacleDist )
			{
				/* Register obstacle on this telemeter */
				xNewTlmData.eIdMask |= TLM_FRT;
			}
			else
			{
				/* Register no obstacle on this telemeter */
				xNewTlmData.eIdMask &= ~TLM_FRT;
			}

			if( xNewTlmData.usBackDist < xCurrConfig.usCritObstacleDist )
			{
				/* Register obstacle on this telemeter */
				xNewTlmData.eIdMask |= TLM_BCK;
			}
			else
			{
				/* Register no obstacle on this telemeter */
				xNewTlmData.eIdMask &= ~TLM_BCK;
			}

			if( xNewTlmData.usLeftBackDist < xCurrConfig.usCritObstacleDist )
			{
				/* Register obstacle on this telemeter */
				xNewTlmData.eIdMask |= TLM_LFT_BCK;
			}
			else
			{
				/* Register no obstacle on this telemeter */
				xNewTlmData.eIdMask &= ~TLM_LFT_BCK;
			}

			if( xNewTlmData.usLeftFrontDist < xCurrConfig.usCritObstacleDist )
			{
				/* Register obstacle on this telemeter */
				xNewTlmData.eIdMask |= TLM_LFT_FRT;
			}
			else
			{
				/* Register no obstacle on this telemeter */
				xNewTlmData.eIdMask &= ~TLM_LFT_FRT;
			}

			if( xNewTlmData.usRightBackDist < xCurrConfig.usCritObstacleDist )
			{
				/* Register obstacle on this telemeter */
				xNewTlmData.eIdMask |= TLM_RGT_BCK;
			}
			else
			{
				/* Register no obstacle on this telemeter */
				xNewTlmData.eIdMask &= ~TLM_RGT_BCK;
			}

			if( xNewTlmData.usRightFrontDist < xCurrConfig.usCritObstacleDist )
			{
				/* Register obstacle on this telemeter */
				xNewTlmData.eIdMask |= TLM_RGT_FRT;
			}
			else
			{
				/* Register no obstacle on this telemeter */
				xNewTlmData.eIdMask &= ~TLM_RGT_FRT;
			}

			/* Ignore bottom "obstacles" while landed and landing;
			usBottomDist is still updated. */
			if( ( xDroneState.eFltState != STATE_GROUND_RDY )
					|| (xDroneState.eFltState != STATE_LANDING ) )
			{
				if( xNewTlmData.usBottomDist < xCurrConfig.usMinAltitude )
				{
					/* Register obstacle on this telemeter */
					xNewTlmData.eIdMask |= TLM_BOT;
				}
				else
				{
					/* Register no obstacle on this telemeter */
					xNewTlmData.eIdMask &= ~TLM_BOT;
				}
			}
			else
			{
				/* Register no obstacle on this telemeter */
				xNewTlmData.eIdMask &= ~TLM_BOT;
			}

			/* Update xTelemeterWrapper global variable */
			prvSafeSetTelemeterData( &xNewTlmData );

			/* Set xDroneMode to MODE_AVOIDANCE to and immediately wake up
			prvFlightCtrlTask() for better reactivity. */
			if( xNewTlmData.eIdMask != TLM_NONE )
			{
				/* Set FlightCtrlTask in obstacle avoidance mode */
				//prvSafeGetDroneState( &xNewDroneState );
				/* FIXME */
				//xNewDroneState.eErrorMask = DRN_ERR_TLM;
				//prvSafeSetDroneState( &xNewDroneState );
				xSemaphoreGive( xFlightCtrlSem );
			}
		}
		else
		{
			/* Register measurement invalidity */
			xCurrDroneState.eErrorMask |= DRN_ERR_TLM_INVAL;
			prvSafeSetDroneState( &xCurrDroneState );
			/* The global variable is not updated */
		}

		vTaskDelayUntil( &xNextWakeTime, xCurrConfig.xDetectObstaclePeriod );
	}

	/* Should never get there */
	ulDebugMsg( xTaskGetTickCount(), "ERROR", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone",  "prvDetectObstacleTask()",
			"Got out of infinite loop !" );
}

/** Task to compute the desired flight behavior according to the current flight
mode/state (set point for PID in prvMotorCtrlTask()).
This task is periodic but can be woken by an event. This is possible using a
timeout on a binary semaphore. The task repeats at each timeout, but reacts
to the semaphore being given.
This task checks the global variables for measurement timeouts and sets
xDroneState.* accordingly. */
static void prvFlightCtrlTask( void *pvParam )
{
struct droneIMUWrapper xNewIMUWrapper;
struct droneState xCurrDroneState;
struct droneTelemeterWrapper xCurrTlmWrapper;
struct droneConfig xCurrConfig;
struct droneBatteryWrapper xCurrBatteryWrapper;
struct droneGPSWrapper xCurrGPSWrapper;
struct droneZigbeeWrapper xCurrZigbeeWrapper;
struct flightMovement xNextFltMvt = xStationaryFlightMvt;
enum IMUErrorMask eIMUError;
//uint8_t ucIMUMeasurementCnt = 0;

	/* Condition already tested in main() */
	if( xFlightCtrlSem == NULL )
	{
		ulDebugMsg( xTaskGetTickCount(), "ERROR", pcTaskGetTaskName( NULL ),
				uxTaskPriorityGet( NULL ), "drone",  "prvFlightCtrlTask()",
				"xFlightCtrlSem is NULL !" );
	}

	while( 1 )
	{
		ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
				uxTaskPriorityGet( NULL ), "drone",  "prvFlightCtrlTask()",
				"Woken up" );

		/* Update of local copy of xDroneState */
		prvSafeGetDroneState( &xCurrDroneState );
		/* Update of local copy of xDroneConfig */
		prvSafeGetDroneConfig( &xCurrConfig );

		/* Attitude measurement */
		vIMUGetData( &( xNewIMUWrapper.xIMUData ) );
		/* Is data within valid range ? */
		eIMUError = eIMUIsDataValid( &( xNewIMUWrapper.xIMUData ) );

		/* TODO : Attitude measurement, retry if measurement is invalid */
//		do
//		{
//			ucIMUMeasurementCnt++;
//
//			/* Attitude measurement */
//			vIMUGetData( pxNewIMUData );
//
//			/* Is data within valid range ? */
//			eIMUError = eIMUIsDataValid( pxNewIMUData );
//		}
//		while( ( ucIMUMeasurementCnt < 3 )
//				&&  ( eIMUError != IMU_ERR_NONE ) );

		/* At this point, xIMUData may or may not be valid */
		if( eIMUError != IMU_ERR_NONE )
		{
			/* We work with the last valid value - better than nothing */
			prvSafeGetIMUData( &xNewIMUWrapper );
			/* Update xDroneState.eErrorMask */
			xCurrDroneState.eErrorMask |= DRN_ERR_IMU_INVAL;
			prvSafeSetDroneState( &xCurrDroneState );
//			/* Check IMU timeout, as IMUDataWrapper was not updated */
//			if( prvIMUTimedout( &xNewIMUWrapper, &xCurrConfig ) )
//			{
//				/* Select relevant error state based on current state */
//				prvSelectErrorState( &xCurrDroneState );
//				/* Add IMU timeout error to eErrorMask */
//				xCurrDroneState.eErrorMask |= DRN_ERR_IMU_TOUT;
//				/* Apply the changes to xDroneState global variable */
//				prvSafeSetDroneState( &xCurrDroneState );
//			}
		}
		else
		{
			/* Update xIMUWrapper global variable */
			prvSafeSetIMUData( &( xNewIMUWrapper.xIMUData ) );
			/* Clear DRN_ERR_IMU_INVAL out of xDroneState.eErrorMask to reflect
			no IMUData invalid error. */
			xCurrDroneState.eErrorMask &= ~DRN_ERR_IMU_INVAL;
			prvSafeSetDroneState( &xCurrDroneState );
		}

		/* Update local data for centralized timeout checking */
		prvSafeGetTelemeterData( &xCurrTlmWrapper );
		prvSafeGetBatteryData( &xCurrBatteryWrapper );
		prvSafeGetGPSData( &xCurrGPSWrapper );
		prvSafeGetZigbeeData( &xCurrZigbeeWrapper );

		/* Checks every peripheral timeout and updates both local and global
		droneState variable */
		prvHandleTimouts( &xCurrTlmWrapper, &xCurrBatteryWrapper,
				&xNewIMUWrapper, &xCurrGPSWrapper, &xCurrZigbeeWrapper,
				&xCurrConfig, &xCurrDroneState );

		/* At this point, the xDroneState reflects any timeout of peripheral
		measurements */

		/* PID set point computation based on state/orders (some transition
		orders are translated into states) */
		switch( xCurrDroneState.eFltState )
		{
		case STATE_GROUND_RDY:
			/* Nothing to do, the drone is idle on the ground. */
			if( prvObstacleDetected( &xCurrTlmWrapper ) )
			{
				/* TODO : some led blinking */
			}
			break;
		case STATE_TAKEOFF:
			/* If some serious error fires while taking off, the state is
			switched to STATE_FLIGHT_ERR and takeoff is aborted */
			if( !prvTakeoffDone( &( xNewIMUWrapper.xIMUData ),
					&( xCurrConfig ) ) )
			{
				/* Set xNextFlightMvt for takeoff */
				vFlightTakeoffMvt( &xNextFltMvt, &( xNewIMUWrapper.xIMUData ),
						&( xCurrTlmWrapper.xTelemeterData ),
						xDroneConfig.ulTakeoffAltitude );
			}
			else
			{
				/* When takeoff has finished, switch to flying state.
				Does nothing for this period, STATE_FLIGHT will only be
				handled on next task period. */
				xCurrDroneState.eFltState = STATE_FLIGHT;
				prvSafeSetDroneState( &xCurrDroneState );
			}
			break;
		case STATE_AUTOTUNING:
			/* Update flightMvt if autotune not finished, switch state
			otherwise */
			if( !ucFlightAutotuneMvt( &xNextFltMvt,
					&( xNewIMUWrapper.xIMUData ),
					&( xCurrTlmWrapper.xTelemeterData ) ) )
			{
				/* TODO : Consider updating the PID parameters as part of
				xDroneConfig */
				/* xNextFltMvt is updated in the condition */
			}
			else
			{
				/* Autotune done, switch to normal flight state */
				xCurrDroneState.eFltState = STATE_FLIGHT;
				prvSafeSetDroneState( &xCurrDroneState );
			}
			break;
		case STATE_FLIGHT:
			/* Normal flight, the drone reacts to movement orders from the base
			and saved in xZigbeeData.xFlightCmd, if it didn't timeout. */
			vFlightCommandMvt( &xNextFltMvt, &( xNewIMUWrapper.xIMUData ),
					&( xCurrTlmWrapper.xTelemeterData ),
					&( xCurrZigbeeWrapper.xFlightCmd ) );
			break;
		case STATE_LANDING:
			if ( !prvLandingDone( &( xNewIMUWrapper.xIMUData ),
					&( xCurrConfig ) ) )
			{
				vFlightLandMvt( &xNextFltMvt, &( xNewIMUWrapper.xIMUData ),
						&( xCurrTlmWrapper.xTelemeterData ) );
			}
			else
			{
				/* Landing finished */
				xCurrDroneState.eFltState = STATE_GROUND_RDY;
				prvSafeSetDroneState( &xCurrDroneState );
			}
			break;
		case STATE_GROUND_ERR:
			/* Nothing to do */
			break;
		case STATE_FLIGHT_ERR:
			/* Emergency landing. In this state, IMUData, TelemeterData might
			be outdated */
			if ( !prvLandingDone( &( xNewIMUWrapper.xIMUData ),
					&( xCurrConfig ) ) )
			{
				vFlightLandMvt( &xNextFltMvt, &( xNewIMUWrapper.xIMUData ),
						&( xCurrTlmWrapper.xTelemeterData ) );
			}
			else
			{
				/* Landing finished */
				xCurrDroneState.eFltState = STATE_GROUND_ERR;
				prvSafeSetDroneState( &xCurrDroneState );
			}
			break;
		default:
			/* STATE_STARTING, the scheduler has not yet started. */
			break;
		} /* switch( eDroneState ) */

		/* Execution of computed movement, if we aren't in STATE_GROUND_RDY,
		STATE_STARTING or STATE_GROUND_ERR. */
		if( ( xDroneState.eFltState != STATE_STARTING )
				&& ( xDroneState.eFltState != STATE_GROUND_RDY )
				&& ( xDroneState.eFltState != STATE_GROUND_ERR ) )
		{
			vFlightExecuteMvt( &xNextFltMvt );
		}

		ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
				uxTaskPriorityGet( NULL ), "drone",  "prvFlightCtrlTask()",
				"Trying to take xFlightCtrlSem" );
		xSemaphoreTake( xFlightCtrlSem, xCurrConfig.xFlightCtrlPeriod );
	}

	/* Should never get there */
	ulDebugMsg( xTaskGetTickCount(), "ERROR", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone", "prvFlightCtrlTask()",
			"Got out of infinite loop !" );
}

/** Periodically receives commands from the base. Updates xZigbeeWrapper
global variable, which is checked by other tasks for commands.
Manages Rx from the base and status Tx to it, along with video toggle. */
static void prvZigbeeReceiveTask( void *pvParam )
{
portTickType xNextWakeTime = xTaskGetTickCount();
struct zigbeeData xNewZigbeeData;
struct droneConfig xCurrConfig;
struct droneState xCurrDroneState;
struct droneIMUWrapper xCurrIMUWrapper;

	while( 1 )
	{
		ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
				uxTaskPriorityGet( NULL ), "drone",  "prvZigbeeReceiveTask()",
				"Woken up" );

		/* Receive drone orders */
		vZigbeeReceiveData( &xNewZigbeeData );

		/* Update local copy of xDroneConfig */
		prvSafeGetDroneConfig( &xCurrConfig );

		/* Update local copy of xDroneState */
		prvSafeGetDroneState( &xCurrDroneState );

		switch ( xNewZigbeeData.eCmdId )
		{
		case CMD_GET_STATUS:
			/* Register command validity */
			xCurrDroneState.eErrorMask &= ~DRN_ERR_CMD;
			/* Update the global variable */
			prvSafeSetDroneState( &xCurrDroneState );
			/* Send status after updating it */
			prvSendStatus();
			break;
		case CMD_SET_CONFIG:
			if( xCurrDroneState.eFltState == STATE_GROUND_RDY )
			{
				if( prvDroneConfigValid( &( xNewZigbeeData.xCmdParam.xDroneConfig ) ) )
				{
					/* Apply new configuration */
					prvSafeSetDroneConfig( &( xNewZigbeeData.xCmdParam.xDroneConfig ) );
					/* Register success */
					xCurrDroneState.eErrorMask &= ~DRN_ERR_CONFIG;
				}
				else
				{
					/* Register error */
					xCurrDroneState.eErrorMask |= DRN_ERR_CONFIG;
				}
				/* Register command validity */
				xCurrDroneState.eErrorMask &= ~DRN_ERR_CMD;
			}
			else
			{
				/* Register error */
				xCurrDroneState.eErrorMask |= DRN_ERR_CMD;
			}
			prvSafeSetDroneState( &xCurrDroneState );
			break;
		case CMD_VIDEO_ENABLE:
			prvEnableVideo();
			/* Register command validity */
			xCurrDroneState.eErrorMask &= ~DRN_ERR_CMD;
			/* Update the global variable */
			prvSafeSetDroneState( &xCurrDroneState );
			break;
		case CMD_VIDEO_DISABLE:
			prvDisableVideo();
			/* Register command validity */
			xCurrDroneState.eErrorMask &= ~DRN_ERR_CMD;
			/* Update the global variable */
			prvSafeSetDroneState( &xCurrDroneState );
			break;
		case CMD_FLT_TAKEOFF:
			/* Check if takeoff is possible */
			if( xCurrDroneState.eFltState == STATE_GROUND_RDY )
			{
				/* Set reference altitude */
				prvSafeGetIMUData( &xCurrIMUWrapper );
				prvInitAltitude( &xCurrIMUWrapper, &xCurrConfig );
				/* Change its mode to STATE_TAKEOFF */
				xCurrDroneState.eFltState = STATE_TAKEOFF;
				/* Register command validity */
				xCurrDroneState.eErrorMask &= ~DRN_ERR_CMD;
				/* Update the global variable */
				prvSafeSetDroneState( &xCurrDroneState );
			}
			else
			{
				/* Register error */
				xCurrDroneState.eErrorMask |= DRN_ERR_CMD;
				prvSafeSetDroneState( &xCurrDroneState );
			}
			break;
		case CMD_FLT_LANDING:
			if( xCurrDroneState.eFltState == STATE_FLIGHT )
			{
				/* Change its mode to STATE_LANDING */
				xCurrDroneState.eFltState = STATE_LANDING;
				/* Register command validity */
				xCurrDroneState.eErrorMask &= ~DRN_ERR_CMD;
				/* Update the global variable */
				prvSafeSetDroneState( &xCurrDroneState );
			}
			else
			{
				/* Register error */
				xCurrDroneState.eErrorMask |= DRN_ERR_CMD;
				prvSafeSetDroneState( &xCurrDroneState );
			}
			break;
		case CMD_FLT_AUTOTUNING:
			if( xCurrDroneState.eFltState == STATE_FLIGHT )
			{
				/* Change its mode to STATE_AUTOTUNING */
				xCurrDroneState.eFltState = STATE_AUTOTUNING;
				/* Register command validity */
				xCurrDroneState.eErrorMask &= ~DRN_ERR_CMD;
				/* Update the global variable */
				prvSafeSetDroneState( &xCurrDroneState );
			}
			else
			{
				/* Register error */
				xCurrDroneState.eErrorMask = DRN_ERR_CMD;
				prvSafeSetDroneState( &xCurrDroneState );
			}
			break;
		case CMD_RESET_AUTOTUNING:
			/* TODO : Implement autotuning reset */
			/* Register command validity */
			xCurrDroneState.eErrorMask &= ~DRN_ERR_CMD;
			prvSafeSetDroneState( &xCurrDroneState );
			break;
		case CMD_RESET_CONFIG:
			if( xCurrDroneState.eFltState == STATE_GROUND_RDY )
			{
				/* Reset xDroneConfig to its default value */
				prvResetDroneConfig();
				/* Register command validity */
				xCurrDroneState.eErrorMask &= ~DRN_ERR_CMD;
				prvSafeSetDroneState( &xCurrDroneState );
			}
			else
			{
				/* Register error */
				xCurrDroneState.eErrorMask = DRN_ERR_CMD;
				prvSafeSetDroneState( &xCurrDroneState );
			}
			break;
		case CMD_GET_CONFIG:
			prvSendConfig( &xCurrConfig );
			/* Register command validity */
			xCurrDroneState.eErrorMask &= ~DRN_ERR_CMD;
			prvSafeSetDroneState( &xCurrDroneState );
			break;
		case CMD_FLT_MANEUVER:
			if( ucFlightCmdValid( &( xNewZigbeeData.xCmdParam.xFlightCmd ) ) )
			{
				/* Register command for next flight control task run */
				prvSafeSetZigbeeData( &( xNewZigbeeData.xCmdParam.xFlightCmd ) );
				/* Register command validity */
				xCurrDroneState.eErrorMask &= ~DRN_ERR_CMD;
				prvSafeSetDroneState( &xCurrDroneState );
			}
			else
			{
				/* Register error */
				xCurrDroneState.eErrorMask |= DRN_ERR_CMD;
				prvSafeSetDroneState( &xCurrDroneState );
			}
			break;
		default:
			/* Register error */
			xCurrDroneState.eErrorMask |= DRN_ERR_CMD;
			prvSafeSetDroneState( &xCurrDroneState );
			break;
		}

		/* Checks whether the communication was successful */
		if( !( xCurrDroneState.eErrorMask &= DRN_ERR_CMD ) )
		{
			/* Register successful communication */
			taskENTER_CRITICAL();
			xZigbeeWrapper.xZigbeeUpdateTime = xTaskGetTickCount();
			taskEXIT_CRITICAL();
		}

		ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
				uxTaskPriorityGet( NULL ), "drone",  "prvZigbeeReceiveTask()",
				"Delayed" );

		vTaskDelayUntil( &xNextWakeTime, xCurrConfig.xZigbeeReceivePeriod );
	}

	/* Should never get there */
	ulDebugMsg( xTaskGetTickCount(), "ERROR", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone", "prvZigbeeReceiveTask()",
			"Got out of infinite loop !" );
}

/* TODO : Add comment here */
static void prvBatteryMonitoringTask( void *pvParam )
{
portTickType xNextWakeTime = xTaskGetTickCount();
struct droneState xCurrDroneState;
struct droneConfig xCurrConfig;
struct droneBatteryWrapper xNewBatteryWrapper;

	while( 1 )
	{
		ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
				uxTaskPriorityGet( NULL ), "drone",
				"prvBatteryMonitoringTask()", "Woken up" );

		/* Measure battery level */
		prvGetBatteryLvl( &( xNewBatteryWrapper.ulPowerLvl ) );
		/* Update local copy of xDroneState */
		prvSafeGetDroneState( &xCurrDroneState );
		/* Check measurement consistency */
		if( prvBatteryValid( &( xNewBatteryWrapper.ulPowerLvl ) ) )
		{
			/* Update global battery level */
			prvSafeSetBatteryData( &( xNewBatteryWrapper.ulPowerLvl ) );
			/* Register measurement success */
			xCurrDroneState.eErrorMask &= ~DRN_ERR_BATT_INVAL;
		}
		else
		{
			/* Register error */
			xCurrDroneState.eErrorMask |= DRN_ERR_BATT_INVAL;
			/* Discard this measure altogether */
			prvSafeGetBatteryData( &xNewBatteryWrapper );
		}
		/* Update global xDroneState */
		prvSafeSetDroneState( &xCurrDroneState );

		/* Update local copy of xDroneConfig */
		prvSafeGetDroneConfig( &xCurrConfig );
		/* Land if battery is critical */
		if( prvBatteryCritical( &( xNewBatteryWrapper.ulPowerLvl ),
				&xCurrConfig ) )
		{
			ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
					uxTaskPriorityGet( NULL ), "drone",
					"prvBatteryMonitoringTask()",
					"Critical battery level detected" );
			xCurrDroneState.eFltState = STATE_FLIGHT_ERR;
			prvSafeSetDroneState( &xCurrDroneState );
		}

		ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
				uxTaskPriorityGet( NULL ), "drone",
				"prvBatteryMonitoringTask()", "Delayed" );
		vTaskDelayUntil( &xNextWakeTime,
				xCurrConfig.xBatteryMonitoringPeriod );
	}

	/* TODO : send error to base if gets out of infinite loop */
	ulDebugMsg( xTaskGetTickCount(), "ERROR", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone",
			"prvBatteryMonitoringTask()",
			"Got out of infinite loop !" );
}

/* TODO : Add comment here */
static void prvGPSReceiveTask( void *pvParam )
{
/* Updated by each call to vTaskDelayUntil() */
portTickType xNextWakeTime = xTaskGetTickCount();
struct droneGPSWrapper xNewGPSWrapper;
struct droneConfig xCurrConfig;
struct droneState xCurrDroneState;

	while( 1 )
	{
		ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
				uxTaskPriorityGet( NULL ), "drone",  "prvGPSReceiveTask()",
				"Woken up" );

		/* Receive data from GPS module */
		vGPSGetData( &( xNewGPSWrapper.xGPSData ) );
		/* Update local copy of xDroneState */
		prvSafeSetDroneState( &xCurrDroneState );
		if( eGPSIsDataValid( &( xNewGPSWrapper.xGPSData ) ) )
		{
			/* Update global GPS data */
			prvSafeSetGPSData( &( xNewGPSWrapper.xGPSData ) );
			/* Register measurement success */
			xCurrDroneState.eErrorMask &= ~DRN_ERR_GPS_INVAL;
		}
		else
		{
			/* Register error */
			xCurrDroneState.eErrorMask |= DRN_ERR_GPS_INVAL;
		}
		/* Update global xDroneState */
		prvSafeSetDroneState( &xCurrDroneState );

		/* Update local copy of xDroneConfig */
		prvSafeGetDroneConfig( &xCurrConfig );

		ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
				uxTaskPriorityGet( NULL ), "drone",  "prvGPSReceiveTask()",
				"Delayed" );

		vTaskDelayUntil( &xNextWakeTime,
				xCurrConfig.xGPSReceivePeriod );
	}

	ulDebugMsg( xTaskGetTickCount(), "ERROR", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone", "prvGPSReceiveTask()",
			"Got out of infinite loop !" );
}

/*****  Hook functions  ******************************************************/

/** Executed when no other task is runnable. Could be used to simulate events */
void vApplicationIdleHook( void )
{
	ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone",  "vApplicationIdleHook()",
			"Entering idle task" );

	ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone",  "vApplicationIdleHook()",
			"Going into sleep mode" );
	/* Wait for interrupt to save power (sleeping mode) */
	__WFI();
}

/*****  Utility routines  ****************************************************/

/** Transmits current configuration to the base */
static inline void prvSendConfig( const struct droneConfig * const pxDroneConfig )
{
	/* TODO : Configuration IBIT transmission */
}

/** Initialize the drone configuration and state */
static inline void prvInitDroneConfig( void )
{
	/* Initialization of base-configurable parameters */
	xDroneConfig = xDfltDroneConfig;

	/* Drone state initialization */
	xDroneState = xInitDroneState;
}

/** Hardware specific clock configuration that was not already performed before
main() was called. */
static inline void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits
    if using a ARM Cortex-M microcontroller. */
	NVIC_SetPriorityGrouping( 0 );

	SystemInit();
	SystemCoreClockUpdate();
}

/** Executes the PBITs (power-on built-in tests) and returns a code to identify
the failed tests, or 0 if no test failed. */
static inline enum droneError prvInitTestPeriph( void )
{
enum droneError eErrorMask = DRN_ERR_NONE;

	/* Taking the most serious error codes to ensure the drone doesn't ignore
	them. */
	if( !ucGPSInitTest() )
		eErrorMask |= DRN_ERR_GPS_TOUT;
	if( !ucIMUInitTest() )
		eErrorMask |= DRN_ERR_IMU_TOUT;
	if( !ucZigbeeInitTest() )
		eErrorMask |= DRN_ERR_RX_TOUT;
	if( !ucTelemeterInitTest() )
		eErrorMask |= DRN_ERR_TLM_TOUT;
	if( !ucBatteryInitTest() )
		eErrorMask |= DRN_ERR_BATT_TOUT;

	return eErrorMask;
}

/* Executes the CBITs/IBITs (continuous/initiated built-in tests) and returns a
code to identify the failed tests, or 0 if no test failed. */
//static inline uint8_t prvTest( void )
//{
//uint8_t ucErrorMask = 0x00;
//
//	/* TODO : Video ? error code macros ? */
//	if( !ucGPSTest() )
//		ucErrorMask |= 0x01;
//	if( !ucIMUTest() )
//		ucErrorMask |= 0x02;
//	if( !ucZigbeeTest() )
//		ucErrorMask |= 0x04;
//	if( !ucPWMTest() )
//		ucErrorMask |= 0x08;
//	if( !ucTelemeterTest() )
//		ucErrorMask |= 0x10;
//
//	return ucErrorMask;
//}

/** Checks xDroneConfig, returns 1 if valid, 0 otherwise */
static inline uint8_t prvDroneConfigValid( struct droneConfig *pxNewDroneConfig )
{
	/* TODO : implement it */

	return 1;
}

static inline uint8_t prvBatteryValid( const uint32_t * const pulNewBattLvl )
{
	/* TODO : Implement it */
//	if(  )
//	{
		return 1;
//	}
//	else
//	{
//		return 0;
//	}
}

/** Resets the global drone configuration */
static inline void prvResetDroneConfig( void )
{
	prvSafeSetDroneConfig( &xDfltDroneConfig );
}

/** Selects STATE_FLIGHT_ERR if already flying, to STATE_GROUND_ERR
otherwise, and updates pxCurrState. Does NOT update global xDroneState */
static inline void prvSelectErrorState( struct droneState * const pxCurrState )
{
	switch( pxCurrState->eFltState )
	{
	case STATE_AUTOTUNING:
	case STATE_FLIGHT:
	case STATE_LANDING:
	case STATE_TAKEOFF:
		pxCurrState->eFltState = STATE_FLIGHT_ERR;
		break;
	case STATE_GROUND_RDY:
	case STATE_STARTING:
	default:
		pxCurrState->eFltState = STATE_GROUND_ERR;
		break;
	}
}

/** Safely copies from pxNewIMUData buffer to xIMUWrapper global variable
and updates its xIMUUpdateTime field. Doesn't check parameter consistency
(every field must be defined). */
static inline void prvSafeSetIMUData( const struct IMUData * const pxNewIMUData )
{
	taskENTER_CRITICAL();
	xIMUWrapper.xIMUData = *pxNewIMUData;
	xIMUWrapper.xUpdateTime = xTaskGetTickCount();
	taskEXIT_CRITICAL();

	ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone",  "prvSafeSetIMUData()",
			"xIMUData updated" );
}

/** Safely copies from pxNewGPSData buffer to xGPSWrapper global variable
and updates its xGPSUpdateTime field. Doesn't check parameter consistency
(every field must be defined). */
static inline void prvSafeSetGPSData( const struct GPSData * const pxNewGPSData )
{
	taskENTER_CRITICAL();
	xGPSWrapper.xGPSData = *pxNewGPSData;
	xGPSWrapper.xUpdateTime = xTaskGetTickCount();
	taskEXIT_CRITICAL();

	ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone",  "prvSafeSetGPSData()",
			"xGPSData updated" );
}

/** Safely copies from pxNewFltCmd buffer to xZigbeeWrapper global variable and
updates its xZigbeeUpdateTime and xCmdUpdateTime fields. Doesn't check
parameter consistency (every field must be defined). */
static inline void prvSafeSetZigbeeData( const struct flightCommand * const pxNewFltCmd )
{
	taskENTER_CRITICAL();
	xZigbeeWrapper.xFlightCmd = *pxNewFltCmd;
	xZigbeeWrapper.xCmdUpdateTime = xTaskGetTickCount();
	xZigbeeWrapper.xZigbeeUpdateTime = xTaskGetTickCount();
	taskEXIT_CRITICAL();

	ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone",  "prvSafeSetFltCmd()",
			"xFlightCmd updated" );
}

/** Safely copies from plNewBatteryLvl buffer to xBatteryWrapper global
variable and updates its xBatteryUpdateTime field. Doesn't check parameter
consistency (every field must be defined). */
static inline void prvSafeSetBatteryData( const uint32_t * const pulNewBatteryLvl )
{
	taskENTER_CRITICAL();
	xBatteryWrapper.ulPowerLvl = *pulNewBatteryLvl;
	xBatteryWrapper.xUpdateTime = xTaskGetTickCount();
	taskEXIT_CRITICAL();

	ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone",  "prvSafeSetBatteryData()",
			"ulPowerLvl updated" );
}

/** Safely copies from pxNewDroneConfig buffer to xDroneConfig global variable.
Doesn't check parameter consistency (every field must be defined). */
static inline void prvSafeSetDroneConfig( const struct droneConfig * const pxNewDroneConfig )
{
	taskENTER_CRITICAL();
	xDroneConfig = *pxNewDroneConfig;
	taskEXIT_CRITICAL();

	ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone",  "prvSafeSetDroneConfig()",
			"xDroneConfig updated" );
}

/** Safely copies from pxNewTelemeterData buffer to xTelemeterWrapper global
variable and updates its xTelemeterUpdateTime field. Doesn't check parameter
consistency (every field must be defined). */
static inline void prvSafeSetTelemeterData( const struct telemeterData * const pxNewTelemeterData )
{
	taskENTER_CRITICAL();
	xTelemeterWrapper.xTelemeterData = *pxNewTelemeterData;
	xTelemeterWrapper.xUpdateTime = xTaskGetTickCount();
	taskEXIT_CRITICAL();

	ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone",  "prvSafeSetTelemeterData()",
			"xTelemeterData updated" );
}

/** Safely copies eNewDroneState to the xDroneState global variable. Updates
eErrorMask by overwriting it */
static inline void prvSafeSetDroneState( const struct droneState * const pxNewDroneState )
{
	taskENTER_CRITICAL();
	xDroneState.eFltState = pxNewDroneState->eFltState;
	xDroneState.eErrorMask = pxNewDroneState->eErrorMask;
	taskEXIT_CRITICAL();

	ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone",  "prvSafeSetDroneState()",
			"xDroneState updated" );
}

/** Safely copies from xIMUWrapper global variable to pxNewIMUData buffer. */
static inline void prvSafeGetIMUData( struct droneIMUWrapper * const pxNewIMUWrapper )
{
	taskENTER_CRITICAL();
	*pxNewIMUWrapper = xIMUWrapper;
	taskEXIT_CRITICAL();
}

/** Safely copies from xGPSWrapper global variable to pxNewGPSData buffer . */
static inline void prvSafeGetGPSData( struct droneGPSWrapper * const pxNewGPSWrapper )
{
	taskENTER_CRITICAL();
	*pxNewGPSWrapper = xGPSWrapper;
	taskEXIT_CRITICAL();
}

/** Safely copies from xZigbeeWrapper global variable to pxNewZigbeeData
buffer. */
static inline void prvSafeGetZigbeeData( struct droneZigbeeWrapper * const pxNewZigbeeWrapper )
{
	taskENTER_CRITICAL();
	*pxNewZigbeeWrapper = xZigbeeWrapper;
	taskEXIT_CRITICAL();
}

/** Safely copies from xBatteryWrapper global variable to plNewBatteryLvl
buffer. */
static inline void prvSafeGetBatteryData( struct droneBatteryWrapper * const pxNewBattWrapper )
{
	taskENTER_CRITICAL();
	*pxNewBattWrapper = xBatteryWrapper;
	taskEXIT_CRITICAL();
}

/** Safely copies from xDroneConfig global variable to pxNewDroneConfig
buffer. */
static inline void prvSafeGetDroneConfig( struct droneConfig * const pxNewConfig )
{
	taskENTER_CRITICAL();
	*pxNewConfig = xDroneConfig;
	taskEXIT_CRITICAL();
}

/** Safely copies from xTelemeterWrapper global variable to pxNewTelemeterData
buffer. */
static inline void prvSafeGetTelemeterData( struct droneTelemeterWrapper * const pxNewTlmWrapper )
{
	taskENTER_CRITICAL();
	*pxNewTlmWrapper = xTelemeterWrapper;
	taskEXIT_CRITICAL();
}

/** Safely copies the eDroneState global variable to peNewFltState. */
static inline void prvSafeGetDroneState( struct droneState * const pxNewState )
{
	taskENTER_CRITICAL();
	*pxNewState = xDroneState;
	taskEXIT_CRITICAL();
}

static inline void prvEnableVideo( void )
{
	/* TODO */
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone",  "prvEnableVideo()",
			"Does nothing ATM" );
	/* GPIO_SetBits(GPIOx, GPIO_Pin) */
}

static inline void prvDisableVideo( void )
{
	/* TODO */
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone",  "prvDisableVideo()",
			"Does nothing ATM" );
}

/** Returns 0 if xDroneConfig.ulTakeoffAltitude is not yet reached,
!0 otherwise */
static inline uint8_t prvTakeoffDone( const struct IMUData * const pxCurrIMUData,
		const struct droneConfig * const pxCurrDroneConfig )
{
	return xIMUWrapper.xIMUData.lAltitude < xDroneConfig.ulTakeoffAltitude;
}

/** Returns 0 if the drone has landed, !0 otherwise */
static inline uint8_t prvLandingDone( const struct IMUData * const pxCurrIMUData,
		const struct droneConfig * const pxCurrDroneConfig )
{
	/* TODO */
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone",  "prvLandingDone()",
			"Does nothing ATM" );
	return 0;
}

/** Returns 0 if no obstacle has been detected. Checks whether
xTelemeterData.eIdMask is TLM_NONE */
static inline uint8_t prvObstacleDetected( const struct droneTelemeterWrapper * const pxCurrTelemeterWrapper)
{
	return ( pxCurrTelemeterWrapper->xTelemeterData.eIdMask != TLM_NONE );
}

static inline uint8_t prvTelemeterTimedout( const struct droneTelemeterWrapper * const pxCurrTelemeterWrapper,
		const struct droneConfig * const pxCurrDroneConfig )
{
/* Since last update */
portTickType ulTimeElapsed;

	/* TODO : take into account systick overflow (probably not an issue for low
	SysTick frequency and short periods). */
	ulTimeElapsed = xTaskGetTickCount() - pxCurrTelemeterWrapper->xUpdateTime;
	return ulTimeElapsed > ( pxCurrDroneConfig->xTelemeterTimeout * pxCurrDroneConfig->xDetectObstaclePeriod );
}

static inline uint8_t prvBatteryCritical( const uint32_t * const pulPowerLvl,
		const struct droneConfig * const pxCurrDroneConfig )
{
	return ( *pulPowerLvl < pxCurrDroneConfig->ulCritBatteryLvl );
}

static inline uint8_t prvZigbeeSignalCritical( const struct droneZigbeeWrapper * const pxCurrZigbeeWrapper,
		const struct droneConfig * const pxCurrDroneConfig )
{
	return ( pxCurrZigbeeWrapper->ulSignalLvl < pxCurrDroneConfig->ulCritZigbeeSignalLvl );
}

static inline uint8_t prvBatteryTimedout( const struct droneBatteryWrapper * const pxCurrBatteryWrapper,
		const struct droneConfig * const pxCurrDroneConfig )
{
/* Since last update */
portTickType ulTimeElapsed;

	/* TODO : take into account systick overflow (probably not an issue for low
	SysTick frequency and short periods). */
	ulTimeElapsed = xTaskGetTickCount() - pxCurrBatteryWrapper->xUpdateTime;
	return ulTimeElapsed > ( pxCurrDroneConfig->xBatteryTimeout * pxCurrDroneConfig->xBatteryMonitoringPeriod );
}

static inline uint8_t prvIMUTimedout( const struct droneIMUWrapper * const pxCurrIMUWrapper,
		const struct droneConfig * const pxCurrDroneConfig )
{
/* Since last update */
portTickType ulTimeElapsed;

	/* TODO : take into account systick overflow (probably not an issue for low
	SysTick frequency and short periods). */
	ulTimeElapsed = xTaskGetTickCount() - pxCurrIMUWrapper->xUpdateTime;
	return ulTimeElapsed > ( pxCurrDroneConfig->xIMUDataTimeout * pxCurrDroneConfig->xFlightCtrlPeriod );
}

static inline uint8_t prvGPSTimedout( const struct droneGPSWrapper * const pxCurrGPSWrapper,
		const struct droneConfig * const pxCurrDroneConfig )
{
/* Since last update */
portTickType ulTimeElapsed;

	/* TODO : take into account systick overflow (probably not an issue for low
	SysTick frequency and short periods). */
	ulTimeElapsed = xTaskGetTickCount() - pxCurrGPSWrapper->xUpdateTime;
	return ulTimeElapsed > ( pxCurrDroneConfig->xGPSTimeout * pxCurrDroneConfig->xGPSReceivePeriod );
}

static inline uint8_t prvZigbeeTimedout( const struct droneZigbeeWrapper * const pxCurrZigbeeWrapper,
		const struct droneConfig * const pxCurrDroneConfig )
{
/* Since last update */
portTickType ulTimeElapsed;

	/* TODO : take into account systick overflow (probably not an issue for low
	SysTick frequency and short periods). */
	ulTimeElapsed = xTaskGetTickCount() - pxCurrZigbeeWrapper->xZigbeeUpdateTime;
	return ulTimeElapsed > ( pxCurrDroneConfig->xZigbeeReceiveTimeout * pxCurrDroneConfig->xZigbeeReceivePeriod );
}

static inline uint8_t prvFltCmdTimedout( const struct droneZigbeeWrapper * const pxCurrZigbeeWrapper,
		const struct droneConfig * const pxCurrDroneConfig )
{
/* Since last update */
portTickType ulTimeElapsed;

	/* TODO : take into account systick overflow (probably not an issue for low
	SysTick frequency and short periods). */
	ulTimeElapsed = xTaskGetTickCount() - pxCurrZigbeeWrapper->xCmdUpdateTime;
	return ulTimeElapsed > ( pxCurrDroneConfig->xZigbeeCmdTimeout * pxCurrDroneConfig->xZigbeeReceivePeriod );
}

static inline uint8_t ucBatteryInitTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ), uxTaskPriorityGet( NULL ), "drone", "ucBatteryInitTest()", "Does nothing ATM" );

	return 0;
}

/** Updates droneState global AND local variables based on timeouts for every
peripheral. If xFlightCmd is outdated, reverts it stationary command */
static inline void prvHandleTimouts( const struct droneTelemeterWrapper * const pxCurrTelemeterWrapper,
		const struct droneBatteryWrapper * const pxCurrBatteryWrapper,
		const struct droneIMUWrapper * const pxCurrIMUWrapper,
		const struct droneGPSWrapper * const pxCurrGPSWrapper,
		struct droneZigbeeWrapper * const pxCurrZigbeeWrapper,
		const struct droneConfig * const pxCurrDroneConfig,
		struct droneState * const pxCurrDroneState )
{
	if( prvTelemeterTimedout( pxCurrTelemeterWrapper, pxCurrDroneConfig ) )
	{
		/* Register error */
		pxCurrDroneState->eErrorMask |= DRN_ERR_TLM_TOUT;
		/* Select relevant error state */
		prvSelectErrorState( pxCurrDroneState );
	}
	else
	{
		/* Unregister error in case it was registered */
		pxCurrDroneState->eErrorMask &= ~DRN_ERR_TLM_TOUT;
	}

	if( prvBatteryTimedout( pxCurrBatteryWrapper, pxCurrDroneConfig ) )
	{
		/* Register error */
		pxCurrDroneState->eErrorMask |= DRN_ERR_BATT_TOUT;
		/* Select relevant error state */
		prvSelectErrorState( pxCurrDroneState );
	}
	else
	{
		/* Unregister error in case it was registered */
		pxCurrDroneState->eErrorMask &= ~DRN_ERR_BATT_TOUT;
	}

	if( prvIMUTimedout( pxCurrIMUWrapper, pxCurrDroneConfig ) )
	{
		/* Register error */
		pxCurrDroneState->eErrorMask |= DRN_ERR_IMU_TOUT;
		/* Select relevant error state */
		prvSelectErrorState( pxCurrDroneState );
	}
	else
	{
		/* Unregister error in case it was registered */
		pxCurrDroneState->eErrorMask &= ~DRN_ERR_IMU_TOUT;
	}

	if( prvGPSTimedout( pxCurrGPSWrapper, pxCurrDroneConfig ) )
	{
		/* Register error */
		pxCurrDroneState->eErrorMask |= DRN_ERR_GPS_TOUT;
		/* No switching to an error state : this is a minor error */
	}
	else
	{
		/* Unregister error in case it was registered */
		pxCurrDroneState->eErrorMask &= ~DRN_ERR_GPS_TOUT;
	}

	if( prvZigbeeTimedout( pxCurrZigbeeWrapper, pxCurrDroneConfig) )
	{
		/* Register error */
		pxCurrDroneState->eErrorMask |= DRN_ERR_RX_TOUT;
		/* Select relevant error state */
		prvSelectErrorState( pxCurrDroneState );
	}
	else
	{
		/* Unregister error in case it was registered */
		pxCurrDroneState->eErrorMask &= ~DRN_ERR_RX_TOUT;
	}

	if( prvFltCmdTimedout( pxCurrZigbeeWrapper, pxCurrDroneConfig) )
	{
		/* Register error */
		pxCurrDroneState->eErrorMask |= DRN_ERR_CMD_TOUT;
		/* This is not a critical error : the movement ordered has not been
		updated for a while, we just disable it to ensure the drone doesn't
		get stuck with an outdated order, ie. we put it into stationary
		movement again. */
		pxCurrZigbeeWrapper->xFlightCmd = xStationaryFlightCmd;
		prvSafeSetZigbeeData( &( pxCurrZigbeeWrapper->xFlightCmd ) );
	}
	else
	{
		/* Unregister error in case it was registered */
		pxCurrDroneState->eErrorMask &= ~DRN_ERR_CMD_TOUT;
	}

	/* Set the drone to the resulting state */
	prvSafeSetDroneState( pxCurrDroneState );
}

/** Initializes the reference altitude in drone configuration, called at takeoff. */
static inline void prvInitAltitude( const struct droneIMUWrapper * const pxCurrIMUWrapper,
		struct droneConfig * const pxCurrDroneConfig )
{
	pxCurrDroneConfig->ulRefAltitude = pxCurrIMUWrapper->xIMUData.lAltitude;
	prvSafeSetDroneConfig( pxCurrDroneConfig );
}

static inline void prvGetBatteryLvl( uint32_t *pulBatteryLvl )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "drone",  "prvGetBatteryData()",
			"Does nothing ATM" );
	/* TODO : Implement it */
}

/** Sends drone status to the base via the zigbee module */
static inline void prvSendStatus( void )
{
	/* TODO : implement prvSendStatus() */
}

static inline void prvSendError( const struct droneState * const xCurrDroneState )
{
	/* TODO : Implement */
}

/** Reboot the MCU. DO NOT DO THIS WHILE FLYING ! */
static inline void prvReboot( void )
{
	NVIC_SystemReset();
}

static inline void prvShutdown( void )
{
	/* TODO : Implement */
	__WFI();
}
