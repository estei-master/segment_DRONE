/******************************************************************************
 **		COMMON INCLUSIONS
 **
 **		Includes headers used throughout the project.
 **
 **		1 tabulation = 4 spaces
 *****************************************************************************/

#ifndef COMMON_H
#define COMMON_H

/*****  Manufacturer supplied header files for CMSIS functions  **************/

#include "stm32f4xx.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"

/*****  Custom debugging header file, for UART and semihosting output ********/

#include "debug.h"

/*****  FreeRTOS kernel basic includes   *************************************/

#include "FreeRTOS.h"

#endif /* COMMON_H */
