/*
 * Driver_Common.h
 *
 *  Created on: Sep 21, 2025
 *      Author: nhduong
 */

#ifndef INC_DRIVER_COMMON_H_
#define INC_DRIVER_COMMON_H_


#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include<string.h>
#define ARM_DRIVER_VERSION_MAJOR_MINOR(major,minor) (((major) << 8) | (minor))

/**
\brief Driver Version
*/
typedef struct _ARM_DRIVER_VERSION {
  uint16_t api;                         ///< API version
  uint16_t drv;                         ///< Driver version
} ARM_DRIVER_VERSION;

/* General return codes */
#define ARM_DRIVER_OK                 0 ///< Operation succeeded
#define ARM_DRIVER_ERROR             -1 ///< Unspecified error
#define ARM_DRIVER_ERROR_BUSY        -2 ///< Driver is busy
#define ARM_DRIVER_ERROR_TIMEOUT     -3 ///< Timeout occurred
#define ARM_DRIVER_ERROR_UNSUPPORTED -4 ///< Operation not supported
#define ARM_DRIVER_ERROR_PARAMETER   -5 ///< Parameter error
#define ARM_DRIVER_ERROR_SPECIFIC    -6 ///< Start of driver specific errors

/**
\brief General power states
*/
typedef enum _ARM_POWER_STATE {
  ARM_POWER_OFF,                        ///< Power off: no operation possible
  ARM_POWER_LOW,                        ///< Low Power mode: retain state, detect and signal wake-up events
  ARM_POWER_FULL                        ///< Power on: full operation at maximum performance
} ARM_POWER_STATE;


#endif /* INC_DRIVER_COMMON_H_ */
