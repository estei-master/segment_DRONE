/** @file gps.h */

#ifndef GPS_H
#define GPS_H

#include "common.h"

/** GPS frame data structure */
struct GPSData
{
	/** Message identifier */
	int8_t messageId[7];
	/** UTC time */
	int8_t UTCTime[11];
	/** Latitude, first character is the sign */
	int8_t lattitude[11];
	/** North/south indicator. Issue if \0 not used as 2nd character */
	int8_t latIndicator[2];
	/** Longitude, first character is the sign */
	int8_t longitude[12];
	/** West/east indicator. Issue if \0 not used as 2nd character */
	int8_t longIndicator[2];
	/** Signal stability */
	int8_t fixIndicator[2];
	/** Number of satellites */
	int8_t satellitesUsed[3];
	/** Mean sea level altitude */
	int8_t MSLAltitude[5];
	/** Power level of the GPS signal */
	uint32_t ulSignalLvl;
};

/** GPS Errors, unused yet. */
enum GPSErrorMask
{
	GPS_ERR_NONE		= 0x00,
	/** @todo Add flags for each field of a GPSData struct */
};

/** GPS initialization routine */
inline void vGPSInit( void );
/** GPS initial testing routine */
inline uint8_t ucGPSInitTest( void );
/** GPS testing routine */
inline uint8_t ucGPSTest( void );
/** GPS reception routine */
inline void vGPSGetData( struct GPSData *pxGPSData );
/** GPS frame testing routine */
inline enum GPSErrorMask eGPSDataValid( const struct GPSData * const pxNewGPSData );

#endif /* GPS_H */
