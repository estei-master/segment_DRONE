/** @file telemeter.h */

#ifndef TELEMETER_H
#define TELEMETER_H

#include "common.h"

/** Telemeter identifiers. Values can be used as non-exclusive masks. These are
also used to save the telemeters having detected an obstacle. */
enum telemeterId
{
	/** Null value, for mask initialization */
	TLM_NONE = 0x00,
	/** Front telemeter identifier */
	TLM_FRT = 0x01,
	/** Left front telemeter identifier */
	TLM_LFT_FRT = 0x02,
	/** Left back telemeter identifier */
	TLM_LFT_BCK = 0X04,
	/** Right front telemeter identifier */
	TLM_RGT_FRT = 0x08,
	/** Right back telemeter identifier */
	TLM_RGT_BCK = 0x10,
	/** Back telemeter identifier */
	TLM_BCK = 0x20,
	/** Bottom telemeter identifier */
	TLM_BOT	= 0x40
};

/** Measured distance for each 7 telemeter. Values are between 300 and
5000 mm. */
struct telemeterData
{
	/** Distance measured on front telemeter (mm) */
	uint16_t usFrontDist;
	/** Distance measured on left front telemeter (mm) */
	uint16_t usLeftFrontDist;
	/** Distance measured on left back telemeter (mm) */
	uint16_t usLeftBackDist;
	/** Distance measured on right front telemeter (mm) */
	uint16_t usRightFrontDist;
	/** Distance measured on right back telemeter (mm) */
	uint16_t usRightBackDist;
	/** Distance measured on back telemeter (mm) */
	uint16_t usBackDist;
	/** Distance measured on bottom telemeter (mm) */
	uint16_t usBottomDist;
	/** Mask to register telemeters with critical distance measured, as per
	droneConfig */
	enum telemeterId eIdMask;
};

/** Telemeter initialization routine */
inline void vTelemeterInit( void );
/** Telemeter initialization test routine */
inline uint8_t ucTelemeterInitTest( void );
/** Telemeter initialization test routine */
inline uint8_t ucTelemeterTest( void );
/** Telemeter measurement routine. */
inline uint16_t usTelemeterGetDist( const enum telemeterId eTelemeterId );
/** Telemeter identifier validity testing routine */
inline uint8_t ucTelemeterIdValid( const enum telemeterId eTelemeterId );
/** Telemeter data validity testing routine */
inline uint8_t ucTelemeterDataValid( const struct telemeterData * const pxTelemeterData );
/** Telemeter measurement routine. Gets measures for all 7 telemeter. */
inline void vTelemeterGetData( struct telemeterData * const pxNewTlmData );

#endif /* TELEMETER_H */
