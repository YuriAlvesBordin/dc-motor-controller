/**
 * @file    dc_motor_types.h
 * @brief   Common types and enumerations shared across the DC motor library.
 *
 * @details No global variables and no dynamic memory allocation are used
 *          anywhere in this header or its consumers. All state is kept
 *          inside host-declared structures.
 */

#ifndef DC_MOTOR_TYPES_H
#define DC_MOTOR_TYPES_H

#include <stdint.h>

/**
 * @brief Status codes returned by the library API.
 */
typedef enum
{
    DC_MOTOR_OK = 0,           /**< Operation succeeded.                          */
    DC_MOTOR_ERR_NULL,         /**< A NULL pointer was passed to the function.    */
    DC_MOTOR_ERR_MODE,         /**< Requested operation is invalid for current mode. */
    DC_MOTOR_ERR_DISABLED,     /**< The requested feature is disabled at compile time. */
    DC_MOTOR_ERR_RANGE,        /**< A numerical argument is out of the allowed range. */
    DC_MOTOR_ERR_WATCHDOG,     /**< The watchdog has tripped and the motor is frozen. */
} dc_motor_status_t;

/**
 * @brief Operating mode of a motor instance.
 */
typedef enum
{
    DC_MOTOR_MODE_IDLE = 0,    /**< Motor stopped, output forced to zero.         */
#if DC_MOTOR_ENABLE_OPENLOOP
    DC_MOTOR_MODE_OPENLOOP,    /**< Acceleration-ramped direct command mode.      */
#endif
#if DC_MOTOR_ENABLE_CLOSEDLOOP
    DC_MOTOR_MODE_CLOSEDLOOP,  /**< PID closed-loop tracking a measured variable. */
#endif
} dc_motor_mode_t;

/**
 * @brief Direction of rotation reported by the host's feedback sensor.
 */
typedef enum
{
    DC_MOTOR_DIR_NONE  = 0,    /**< No motion (or below the dead-band threshold). */
    DC_MOTOR_DIR_FWD   = 1,    /**< Forward rotation.                             */
    DC_MOTOR_DIR_REV   = -1,   /**< Reverse rotation.                             */
} dc_motor_direction_t;

#endif /* DC_MOTOR_TYPES_H */
