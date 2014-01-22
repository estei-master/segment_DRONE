#ifndef DRONE_H
#define DRONE_H

#include "common.h"

/*****  Base-configurable parameters struct  *********************************/

/* Shared with zigbee.* */
/* TODO : add macros to define maximum and minimum possible configuration */
/** General drone configuration structure, holds limits, tasks periods and
parameter timouts */
struct droneConfig
{
	/** Maximum pitch/roll angle (horizontal reference) - akin to maximum
	horizontal speed. */
	uint32_t ulMaxAngle;
	/** Preset altitude for automatic takeoff */
	uint32_t ulTakeoffAltitude;
	/** Minimal flight height (based on telemeters) */
	uint16_t usMinAltitude;
	/** Reference altitude, set at takeoff */
	uint32_t ulRefAltitude;
	/** Battery level to activate automatic landing */
	uint32_t ulCritBatteryLvl;
	/** Distance to activate obstacle avoidance */
	uint16_t usCritObstacleDist;
	/** Critical drone-base link level - yet unused */
	uint32_t ulCritZigbeeSignalLvl;
	/** Task periods */
	portTickType xBatteryMonitoringPeriod;
	portTickType xDetectObstaclePeriod;
	portTickType xZigbeeReceivePeriod;
	portTickType xFlightCtrlPeriod;
	portTickType xGPSReceivePeriod;
	portTickType xVideoTogglePeriod;
	/** Timeouts for data validity, as a multiplier to its corresponding
	task period (timeout = N => timeout after N task periods elapsed) */
	uint8_t xIMUDataTimeout;
	uint8_t xZigbeeCmdTimeout;
	uint8_t xZigbeeReceiveTimeout;
	uint8_t xTelemeterTimeout;
	uint8_t xBatteryTimeout;
	uint8_t xGPSTimeout;
};

#endif /* DRONE_H */
