#ifndef PID_H
#define PID_H

#include "common.h"

/** PID parameters Ki, Kp, Kd. One PIDParam structure per degree of freedom */
struct PIDParam
{
	uint32_t ulKprop;	/** Proportional factor */
	uint32_t ulKint;	/** Integral factor */
	uint32_t ulKder;	/** Derived factor */
};

#endif /* PID_H */
