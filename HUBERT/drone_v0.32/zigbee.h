/** @file zigbee.h */

#ifndef ZIGBEE_H
#define ZIGBEE_H

#include "common.h"
#include "drone.h"
#include "flight.h"

struct droneBatteryWrapper;

/** @todo Expand enum zigbeeCommandId as necessary */
/** Identifier for commands received from the base. Treated internally to the
zigbee task. */
enum zigbeeCommandId
{
	/** Reboot command */
	CMD_REBOOT,
	/** Shutdown command */
	CMD_SHUTDOWN,
	/** Video activation command */
	CMD_VIDEO_ENABLE,
	/** Video deactivation command */
	CMD_VIDEO_DISABLE,
	/** Takeoff command */
	CMD_FLT_TAKEOFF,
	/** Landing command */
	CMD_FLT_LANDING,
	/** PID parameters automatic tuning command */
	CMD_FLT_AUTOTUNING,
	/** Reset to default PID parameters command */
	CMD_RESET_AUTOTUNING,
	/** Drone configuration command */
	CMD_SET_CONFIG,
	/** Reset to default drone configuration command */
	CMD_RESET_CONFIG,
	/** Send current drone configuration command */
	CMD_GET_CONFIG,
	/** Send current drone status command */
	CMD_GET_STATUS,
	/** Execute a specific maneuver command */
	CMD_FLT_MANEUVER,
	/** Ignore minor error command. Used when a non critical error
	occured. */
	CMD_ERR_IGNORE,
};

/** Parameters associated with a zigbeeCommandId, more precisely CMD_SET_CONFIG
and CMD_FLT_MANEUVER. As this is a union, both cannot be used simultaneously.
The drone configuration doesn't need to make it to the global variable. */
union zigbeeCommandParam
{
	/** Drone configuration, matching the CMD_SET_CONFIG zigbeeCommandId */
	struct droneConfig xDroneConfig;
	/** Movement to execute, matching the CMD_FLT_MANEUVER
	zigbeeCommandId */
	struct flightCommand xFlightCmd;
};

/** Data and status from the zigbee module, processed by the zigbee task */
struct zigbeeData
{
	/** Command identifier */
	enum zigbeeCommandId eCmdId;
	/** Command parameters */
	union zigbeeCommandParam xCmdParam;
	/** Signal power level, between 26 and 88 (negative dBm) */
	uint8_t ulSignalLvl;
};

/** Zigbee module initialization routine, sets up the UART and DMA for zigbee
communication. */
inline void vZigbeeInit( void );
/** Zigbee initialization test routine */
inline uint8_t ucZigbeeInitTest( void );
/** Zigbee normal test routine. Yet unused and undefined. */
inline uint8_t ucZigbeeTest( void );
/* Zigbee data emission routine */
//inline void vZigbeeSendData( const uint8_t * const pucData );
/** Zigbee data reception routine */
inline void vZigbeeReceiveData( struct zigbeeData * const pxZigbeeData );

#endif /* ZIGBEE_H */
