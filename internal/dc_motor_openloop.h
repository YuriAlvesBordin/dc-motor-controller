/**
 * @file    dc_motor_openloop.h
 * @brief   Open-loop (ramped command) mode for the DC motor library.
 *
 * @details In open-loop mode the host commands a target output level
 *          expressed in percent of the maximum (e.g. 50.0 for half-throttle
 *          forward, -25.0 for quarter-throttle reverse). The library then
 *          slews the actual applied output towards that target using the
 *          acceleration / deceleration ramps provided by dc_motor_ramp_t.
 *
 *          When DC_MOTOR_ENABLE_RAMP is undefined the target is applied
 *          instantaneously (the ramp struct is replaced by an empty
 *          placeholder so no memory is consumed).
 *
 *          This module is enabled only when DC_MOTOR_ENABLE_OPENLOOP is
 *          defined in dc_motor_config.h.
 */

#ifndef DC_MOTOR_OPENLOOP_H
#define DC_MOTOR_OPENLOOP_H

#include "dc_motor_config.h"
#include "dc_motor_types.h"
#include "dc_motor_ramp.h"

#if DC_MOTOR_ENABLE_OPENLOOP

/**
 * @brief Open-loop controller state.
 */
typedef struct
{
    float target;    /**< Last commanded target level, in percent.           */
    float output;    /**< Most recent applied output, in percent.            */
#if DC_MOTOR_ENABLE_RAMP
    dc_motor_ramp_t ramp;  /**< Slew-rate generator.                         */
#endif
} dc_motor_openloop_t;

/**
 * @brief Initialise an open-loop controller with default ramp limits.
 *
 * @param ol Pointer to the open-loop state. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if ol is NULL.
 */
dc_motor_status_t dc_motor_openloop_init(dc_motor_openloop_t *ol);

/**
 * @brief Command a new target output level.
 *
 * @details The target is clamped to
 *          [DC_MOTOR_OPENLOOP_OUT_MIN, DC_MOTOR_OPENLOOP_OUT_MAX].
 *          The actual output will slew towards the clamped target on the
 *          next calls to dc_motor_openloop_update().
 *
 * @param ol     Pointer to the open-loop state. Must not be NULL.
 * @param target Desired output level in percent (signed for direction).
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if ol is NULL.
 */
dc_motor_status_t dc_motor_openloop_set_target(dc_motor_openloop_t *ol,
                                               float target);

/**
 * @brief Force the open-loop output to a specific value without slewing.
 *
 * @param ol    Pointer to the open-loop state. Must not be NULL.
 * @param value Output value to inject (will be clamped to the limits).
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if ol is NULL.
 */
dc_motor_status_t dc_motor_openloop_reset(dc_motor_openloop_t *ol,
                                          float value);

/**
 * @brief Compute one open-loop output sample.
 *
 * @details If a ramp is enabled the output advances towards the target
 *          using dc_motor_ramp_update(). Otherwise the output is set
 *          directly to the target. A configurable dead-band forces the
 *          output to zero when its absolute value is below
 *          DC_MOTOR_OPENLOOP_DEADBAND.
 *
 * @param ol Pointer to the open-loop state. Must not be NULL.
 * @param dt Time elapsed since the last update, in seconds. Must be > 0.
 * @return The updated output level in percent. Returns 0.0f if ol is NULL
 *         or dt <= 0.
 */
float dc_motor_openloop_update(dc_motor_openloop_t *ol, float dt);

/**
 * @brief Return the last commanded target.
 *
 * @param ol Pointer to the open-loop state.
 * @return The target level, or 0.0f if ol is NULL.
 */
float dc_motor_openloop_get_target(const dc_motor_openloop_t *ol);

/**
 * @brief Return the most recently applied output.
 *
 * @param ol Pointer to the open-loop state.
 * @return The output level, or 0.0f if ol is NULL.
 */
float dc_motor_openloop_get_output(const dc_motor_openloop_t *ol);

#else

typedef struct dc_motor_openloop_t_empty { uint8_t _unused; } dc_motor_openloop_t;

#endif /* DC_MOTOR_ENABLE_OPENLOOP */

#endif /* DC_MOTOR_OPENLOOP_H */
