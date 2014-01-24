#include "imu.h"
#include "task.h"

/******************************************************************************
 **	 	 STATIC FUNCTIONS DECLARATION
 *****************************************************************************/

static inline enum IMUErrorMask prvIsAltitudeValid( const struct IMUData * const pxIMUData );
static inline enum IMUErrorMask prvIsAngleValid( const struct IMUData * const pxIMUData );
static inline enum IMUErrorMask prvIsSpeedValid( const struct IMUData * const pxIMUData );
static inline enum IMUErrorMask prvIsAccelValid( const struct IMUData * const pxIMUData );

/******************************************************************************
 **	 	 PUBLIC FUNCTIONS DEFINITION
 *****************************************************************************/

/** Initializes the IMU */
inline void vIMUInit( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", ( signed char * ) "-", 0, "imu", "vIMUInit()",
			"Does nothing ATM" );

	/* TODO : implement during integration */
}

/** Tests correct Initialization of the IMU */
inline uint8_t ucIMUInitTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", ( signed char * ) "-", 0, "imu", "ucIMUInitTest()",
			"Does nothing ATM" );

	/* TODO : implement during integration */
	return 0;
}

/** Tests correct behavior of the IMU */
inline uint8_t ucIMUTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "imu", "ucIMUTest()", "Does nothing ATM" );

	return 0;
}

/** Gets IMU attitude measurements */
inline void vIMUGetData( struct IMUData * const pxIMUData )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "imu", "vIMUGetData()", "Does nothing ATM" );

	/* TODO : implement during integration */
}

/** Tests validity of pointed data */
inline enum IMUErrorMask eIMUIsDataValid( const struct IMUData * const pxIMUData )
{
enum IMUErrorMask eErrorMask = IMU_ERR_NONE;

	eErrorMask |= prvIsAltitudeValid( pxIMUData )
			| prvIsAngleValid( pxIMUData )
			| prvIsSpeedValid( pxIMUData )
			| prvIsAccelValid( pxIMUData );

	if( eErrorMask == IMU_ERR_NONE )
	{
		ulDebugMsg( xTaskGetTickCount(), "INFO", pcTaskGetTaskName( NULL ),
				uxTaskPriorityGet( NULL ), "imu", "eIMUIsDataValid()", "IMU measure is in valid range" );
	}
	else
	{
		ulDebugMsg( xTaskGetTickCount(), "ERROR", pcTaskGetTaskName( NULL ),
				uxTaskPriorityGet( NULL ), "imu", "eIMUIsDataValid()", "IMU measure is not in valid range !" );
	}

	return eErrorMask;

}

/******************************************************************************
 **	 	 STATIC FUNCTIONS DEFINITION
 *****************************************************************************/

static inline enum IMUErrorMask prvIsAltitudeValid( const struct IMUData * const pxIMUData )
{
enum IMUErrorMask eErrorMask = IMU_ERR_NONE;

	if( pxIMUData->lAltitude > imuMAX_VALID_ALTITUDE
			|| pxIMUData->lAltitude < imuMIN_VALID_ALTITUDE )
	{
		eErrorMask = IMU_ERR_ALTITUDE;
	}

	return eErrorMask;
}

static inline enum IMUErrorMask prvIsAngleValid( const struct IMUData * const pxIMUData )
{
enum IMUErrorMask eErrorMask = IMU_ERR_NONE;

	if( pxIMUData->plAngle[ IMU_AXIS_X ] > imuMAX_VALID_ANGLE
			|| pxIMUData->plAngle[ IMU_AXIS_X ] < imuMIN_VALID_ANGLE )
	{
		eErrorMask |= IMU_ERR_XANGLE;
	}

	if( pxIMUData->plAngle[ IMU_AXIS_Y ] > imuMAX_VALID_ANGLE
			|| pxIMUData->plAngle[ IMU_AXIS_Y ] < imuMIN_VALID_ANGLE )
	{
		eErrorMask |= IMU_ERR_YANGLE;
	}

	if( pxIMUData->plAngle[ IMU_AXIS_Z ] > imuMAX_VALID_ANGLE
			|| pxIMUData->plAngle[ IMU_AXIS_Z ] < imuMIN_VALID_ANGLE )
	{
		eErrorMask |= IMU_ERR_ZANGLE;
	}

	return eErrorMask;
}

static inline enum IMUErrorMask prvIsSpeedValid( const struct IMUData * const pxIMUData )
{
enum IMUErrorMask eErrorMask = IMU_ERR_NONE;

	if( pxIMUData->plSpeed[ IMU_AXIS_X ] > imuMAX_VALID_SPEED
			|| pxIMUData->plSpeed[ IMU_AXIS_X ] < imuMIN_VALID_SPEED )
	{
		eErrorMask |= IMU_ERR_XSPEED;
	}

	if( pxIMUData->plSpeed[ IMU_AXIS_Y ] > imuMAX_VALID_SPEED
			|| pxIMUData->plSpeed[ IMU_AXIS_Y ] < imuMIN_VALID_SPEED )
	{
		eErrorMask |= IMU_ERR_YSPEED;
	}

	if( pxIMUData->plSpeed[ IMU_AXIS_Z ] > imuMAX_VALID_SPEED
			|| pxIMUData->plSpeed[ IMU_AXIS_Z ] < imuMIN_VALID_SPEED )
	{
		eErrorMask |= IMU_ERR_ZSPEED;
	}

	return eErrorMask;
}

static inline enum IMUErrorMask prvIsAccelValid( const struct IMUData * const pxIMUData )
{
enum IMUErrorMask eErrorMask = IMU_ERR_NONE;

	if( pxIMUData->plAccel[ IMU_AXIS_X ] > imuMAX_VALID_ACCEL
			|| pxIMUData->plAccel[ IMU_AXIS_X ] < imuMIN_VALID_ACCEL )
	{
		eErrorMask |= IMU_ERR_XACCEL;
	}

	if( pxIMUData->plAccel[ IMU_AXIS_Y ] > imuMAX_VALID_ACCEL
			|| pxIMUData->plAccel[ IMU_AXIS_Y ] < imuMIN_VALID_ACCEL )
	{
		eErrorMask |= IMU_ERR_YACCEL;
	}

	if( pxIMUData->plAccel[ IMU_AXIS_Z ] > imuMAX_VALID_ACCEL
			|| pxIMUData->plAccel[ IMU_AXIS_Z ] < imuMIN_VALID_ACCEL )
	{
		eErrorMask |= IMU_ERR_ZACCEL;
	}

	return eErrorMask;
}

