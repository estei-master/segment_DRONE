#ifndef DRONE_H
#define DRONE_H

/** @file drone.h */

#include "common.h"

/*****  Base-configurable parameters struct  *********************************/

/** @todo Add macros to define maximum and minimum possible configuration */
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
	uint8_t ulCritZigbeeSignalLvl;
	/* Task periods */
	/** Period of the battery monitoring task (ms) */
	portTickType xBatteryMonitoringPeriod;
	/** Period of the obstacle detection task (ms) */
	portTickType xDetectObstaclePeriod;
	/** Period of the flight control task (ms) */
	portTickType xFlightCtrlPeriod;
	/** Period of the GPS reception task (ms) */
	portTickType xGPSReceivePeriod;
	/** Period of the ZigBee base-drone communication task (ms) */
	portTickType xZigbeeReceivePeriod;
	/** Timeout for the IMU data validity, defined as a multiplier to
	xFlightCtrlPeriod. After this delay, a DRN_ERR_IMU_TOUT error is
	triggered. */
	uint8_t xIMUDataTimeout;
	/** Timeout for the flight command validity, defined as a multiplier to
	xZigbeeReceivePeriod. After this delay, a DRN_ERR_CMD_TOUT error is
	triggered. */
	uint8_t xZigbeeCmdTimeout;
	/** Timeout for the ZigBee communication, defined as a multiplier to
	xZigbeeReceivePeriod. After this delay, a DRN_ERR_RX_TOUT error is
	triggered. */
	uint8_t xZigbeeReceiveTimeout;
	/** Timeout for the telemeter data validity, defined as a multiplier to
	xDetectObstaclePeriod. After this delay, a DRN_ERR_TLM_TOUT error is
	triggered. */
	uint8_t xTelemeterTimeout;
	/** Timeout for the battery power level validity, defined as a multiplier
	to xBatteryMonitoringPeriod. After this delay, a DRN_ERR_BATT_TOUT error is
	triggered. */
	uint8_t xBatteryTimeout;
	/** Timeout for the GPS data validity, defined as a multiplier to
	xGPSReceivePeriod. After this delay, a DRN_ERR_GPS_TOUT error is
	triggered. */
	uint8_t xGPSTimeout;
};

#endif /* DRONE_H */
