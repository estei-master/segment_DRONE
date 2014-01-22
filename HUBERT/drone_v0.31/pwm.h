#ifndef PWM_H
#define PWM_H

#include "common.h"

/** Identifiers for each motor PWM control */
enum PWMId
{
	PWM_LFT,		/** Left motor PWM */
	PWM_LFT_FRT,	/** Left-front motor PWM */
	PWM_LFT_BCK,	/** Left-back motor PWM */
	PWM_RGT_FRT,	/** Right-front motor PWM */
	PWM_RGT_BCK,	/** Right-back motor PWM */
	PWM_RGT,		/** Right motor PWM */
};

/** PWM initialization routine */
inline void vPWMInit( void );
/** PWM initial testing routine */
inline uint8_t ucPWMInitTest( void );
/** PWM testing routine */
inline uint8_t ucPWMTest( void );
/** PWM affectation routine */
inline void vPWMSet( enum PWMId ePWMId );

#endif /* PWM_H */
