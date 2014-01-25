#ifndef FLIGHT_H
#define FLIGHT_H

#include "common.h"
#include "imu.h"
#include "telemeter.h"

/* TODO : absolute or relative ? Angle or angular rate ? */
/** Desired drone attitude */
struct flightMovement
{
	/** Target rotation */
	int32_t lRotX;		/** Roll (right when +) */
	int32_t lRotY;		/** Pitch (forward when +) */
	int32_t lRotZ;		/** Yaw (counter-clockwise when +) */
	/** Target translation */
	int32_t lTransX;	/** Forward translation (when +) */
	int32_t lTransY;	/** Lateral translation (left when +) */
	int32_t lTransZ;	/** Vertical translation (up when +) */
};

/** Command as received from the base */
struct flightCommand
{
	/** Translations */
	int8_t cTransX;	/** Forward translation (backwards when -) */
	int8_t cTransY;	/** Lateral translation (left when +) */
	int8_t cTransZ;	/** Vertical translation (up when +) */
	/** Yaw */
	int8_t cRotZ;	/** Yaw (counter-clockwise when +) */
};

inline uint8_t ucFlightStationary( const struct flightMovement * const pxFlightMvt );
inline uint8_t ucFlightCmdValid( const struct flightCommand * const pxFlightCmd );

inline void vFlightTakeoffMvt( struct flightMovement * const pxMovement,
		const struct IMUData * const pxIMUData,
		const struct telemeterData * const pxTelemeterData,
		uint32_t ulTakeoffAltitude );
inline void vFlightLandMvt( struct flightMovement * const pxMovement,
		const struct IMUData * const pxIMUData,
		const struct telemeterData * const pxTelemeterData );
inline uint8_t ucFlightAutotuneMvt( struct flightMovement * const pxMovement,
		const struct IMUData * const pxIMUData,
		const struct telemeterData * const pxTelemeterData );
inline void vFlightCommandMvt( struct flightMovement * const pxMovement,
		const struct IMUData * const pxIMUData,
		const struct telemeterData * const pxTelemeterData,
		const struct flightCommand * const pxFlightCmd );
inline void vFlightExecuteMvt( const struct flightMovement * const pxMovement );
//inline void vFlightAvoidMvt( struct flightMovement *pxMovement,
//		const struct IMUData *pxIMUData,
//		const struct telemeterData *pxTelemeterData );

#endif /* FLIGHT_H */
