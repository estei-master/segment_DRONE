/** @file imu.h */

#ifndef IMU_H
#define IMU_H

#include "common.h"

/** Minimum angle received from the IMU to be considered valid */
#define imuMIN_VALID_ANGLE			-180
/** Maximum angle received from the IMU to be considered valid */
#define imuMAX_VALID_ANGLE			180
//#define imuMIN_VALID_SPEED			-100		/* @todo sensible value */
//#define imuMAX_VALID_SPEED			100			/* @todo sensible value */
//#define imuMIN_VALID_ACCEL			-100		/* @todo sensible value */
//#define imuMAX_VALID_ACCEL			100			/* @todo sensible value */
/** Minimum altitude received from the IMU to be considered valid */
#define imuMIN_VALID_ALTITUDE		-2147483647	/** @todo sensible value */
/** Maximum altitude received from the IMU to be considered valid */
#define imuMAX_VALID_ALTITUDE		2147483647 	/** @todo sensible value */

/** Used as a mask to describe the fields of an IMUData that are out of
consistent measurement range. */
enum IMUErrorMask
{
	/** No error on IMU measurements. This value is mutually exclusive with every
	others, and their combinations. */
	IMU_ERR_NONE = 0x00,
	/** Altitude out of valid range */
	IMU_ERR_ALTITUDE = 0x01,
	/** Roll angle out of valid range */
	IMU_ERR_XANGLE = 0x02,
	/** Pitch angle out of valid range */
	IMU_ERR_YANGLE = 0x04,
	/** Yaw angle out of valid range */
	IMU_ERR_ZANGLE = 0x08,
	/* Speed out of valid range */
//	IMU_ERR_XSPEED = 0x10,
//	IMU_ERR_YSPEED = 0x20,
//	IMU_ERR_ZSPEED = 0x40,
	/* Acceleration out of valid range */
//	IMU_ERR_XACCEL = 0x80,
//	IMU_ERR_YACCEL = 0x100,
//	IMU_ERR_ZACCEL = 0x200,
};

/** Used as indexes for Angle/speed/acceleration data arrays */
enum IMUAxis
{
	/** Roll axis */
	IMU_AXIS_X = 0,
	/** Pitch axis */
	IMU_AXIS_Y = 1,
	/** Yaw axis */
	IMU_AXIS_Z = 2,
};

/** Attitude measurement from the IMU module. */
struct IMUData
{
	/** Drone altitude, calculated from air pressure. */
	int32_t lAltitude;
	/** Values between -180 and +180, indexes specified in enum IMUAxis */
	int16_t plAngle[3];
	/* @todo valid range ? */
	//int32_t plSpeed[3];
	/* @todo valid range ? */
	//int32_t plAccel[3];
};

inline void vIMUInit( void );
inline uint8_t ucIMUInitTest( void );
inline uint8_t ucIMUTest( void );
inline enum IMUErrorMask eIMUDataValid( const struct IMUData * const pxIMUData );
inline void vIMUGetData( struct IMUData * const pxIMUData );

#endif /* IMU_H */
