#include "flight.h"
#include "pid.h"
#include "task.h"

/* TODO : fix it (angles ? speeds ? reference ?) */
/** Default movement : stationary flight */
const struct flightMovement xStationaryFlightMvt = {
		.lRotX		= 0,
		.lRotY		= 0,
		.lRotZ		= 0,
		.lTransX	= 0,
		.lTransY	= 0,
		.lTransZ	= 0,
};

/** Default command : stationary flight */
const struct flightCommand xStationaryFlightCmd = {
		.cTransX	= 0,
		.cTransY	= 0,
		.cTransZ	= 0,
		.cRotZ		= 0,
};

/* FIXME : check validity of this test */
/** Tests whether the argument is the stationary movement. Returns 1 if it is,
0 otherwise */
inline uint8_t ucFlightIsStationary( const struct flightMovement * const pxFlightMvt )
{
	if ( pxFlightMvt->lRotX == xStationaryFlightMvt.lRotX
			&& pxFlightMvt->lRotY == xStationaryFlightMvt.lRotY
			&& pxFlightMvt->lRotZ == xStationaryFlightMvt.lRotZ
			&& pxFlightMvt->lTransX == xStationaryFlightMvt.lTransX
			&& pxFlightMvt->lTransY == xStationaryFlightMvt.lTransY
			&& pxFlightMvt->lTransZ == xStationaryFlightMvt.lTransZ )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

inline uint8_t ucFlightCmdValid( const struct flightCommand * const pxFlightCmd )
{
	if( ( pxFlightCmd->cTransX >= -2 && pxFlightCmd->cTransX <= +2 )
			&& ( pxFlightCmd->cTransY >= -2 && pxFlightCmd->cTransY <= +2 )
			&&( pxFlightCmd->cTransZ >= -2 && pxFlightCmd->cTransZ <= +2 )
			&&( pxFlightCmd->cRotZ >= -2 && pxFlightCmd->cRotZ <= +2 ) )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/** Updates pxMovement as xStationaryFlightMvt with target altitude of
ulTakeoffAltitude */
inline void vFlightTakeoffMvt( struct flightMovement *pxMovement,
		const struct IMUData *pxIMUData,
		const struct telemeterData * const pxTelemeterData,
		uint32_t ulTakeoffAltitude )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "flight", "vFlightTakeoffMvt()",
			"Does nothing ATM" );
}

/** Slowly descend until ground is detected, then proceed even more slowly. May
be useful to take into account the altitude measured by the IMU before
takeoff. */
inline void vFlightLandMvt( struct flightMovement * const pxMovement,
		const struct IMUData * const pxIMUData,
		const struct telemeterData * const pxTelemeterData )
{
	/* Use reference altitude as set point for the PID, or measured altitude
	from the bottom telemeter (ie. landing Altitude = current Altitude - bottom
	telemeter obstacle distance, as an absolute altitude). */
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "flight", "vFlightLandMvt()",
			"Does nothing ATM" );
}

/* TODO : implementation */
/** Returns 1 if autotuning is done; PID parameters are considered static in
either flight.c or pid.c. */
inline uint8_t ucFlightAutotuneMvt( struct flightMovement * const pxMovement,
		const struct IMUData * const pxIMUData,
		const struct telemeterData * const pxTelemeterData )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "flight", "vFlightAutotuneMvt()",
			"Does nothing ATM" );

	return 0;
}

/** Computes xMovement according to both pxIMUData and the base command from
pxZigbeeData, and stores it in pxMovement */
inline void vFlightCommandMvt( struct flightMovement * const pxMovement,
		const struct IMUData * const pxIMUData,
		const struct telemeterData * const pxTelemeterData,
		const struct flightCommand * const pxFlightCmd )
{
	/* TODO : Should check xZigbeeData last update (and take into account timer
	overflow). */
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "flight", "vFlightCommandMvt()",
			"Does nothing ATM" );
}

/** Computes xMovement according to both pxIMUData and the base command from
pxZigbeeData, ensures the drone doesn't come any closer to the obstacle and
stores xMovement in pxMovement */
inline void vFlightAvoidMvt( struct flightMovement *pxMovement,
		const struct IMUData *pxIMUData,
		const struct telemeterData *pxtelemeterData )
{
	/* TODO : Should check xZigbeeData last update (and take into account timer
	overflow). */
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "flight", "vFlightAvoidMvt()",
			"Does nothing ATM" );
}

/** Executes computed xMovement using the PID algorithm for each
degree of freedom (each with its own PID coefficients) */
inline void vFlightExecuteMvt( const struct flightMovement *pxMovement )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "flight", "vFlightExecuteMvt()",
			"Does nothing ATM" );
}
