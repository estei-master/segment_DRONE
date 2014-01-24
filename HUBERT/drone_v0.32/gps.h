#ifndef GPS_H
#define GPS_H

#include "common.h"

/** GPS frame data structure */
struct GPSData
{
	/* TODO : add relevant fields */
	/** Power level of the GPS signal */
	uint32_t ulSignalLvl;
};

/** GPS Errors */
enum GPSErrorMask
{
	GPS_ERR_NONE		= 0x00,
	/* TODO : Add flags for each field of a GPSData struct */
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
inline enum GPSErrorMask eGPSIsDataValid( const struct GPSData * const pxNewGPSData );

#endif /* GPS_H */
