/**
 * @file    dc_motor_ramp.h
 * @brief   Slew-rate limited setpoint generator.
 *
 * @details Produces a smoothly transitioning reference value that moves
 *          towards a target at a configurable acceleration when speeding up
 *          (|output| growing) and a configurable deceleration when braking
 *          (|output| shrinking). Used by both the open-loop and the
 *          closed-loop modes so the setpoint fed to the PID or to the PWM
 *          output is always physically achievable.
 *
 *          The module is enabled only when DC_MOTOR_ENABLE_RAMP is defined
 *          in dc_motor_config.h. When disabled, setpoints are applied
 *          instantaneously by the calling code.
 */

#ifndef DC_MOTOR_RAMP_H
#define DC_MOTOR_RAMP_H

#include "dc_motor_config.h"
#include "dc_motor_types.h"

#if DC_MOTOR_ENABLE_RAMP

/**
 * @brief Ramp generator state.
 */
typedef struct
{
    float current;  /**< Current (slewed) value of the generated setpoint. */
    float target;   /**< Target value the ramp is converging towards.       */
    float accel;    /**< Acceleration limit, in units-per-second.           */
    float decel;    /**< Deceleration limit, in units-per-second.           */
} dc_motor_ramp_t;

/**
 * @brief Initialise a ramp generator with explicit limits.
 *
 * @param ramp   Pointer to the ramp state. Must not be NULL.
 * @param accel  Acceleration limit (> 0) in units-per-second.
 * @param decel  Deceleration limit (> 0) in units-per-second.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if ramp is NULL,
 *         DC_MOTOR_ERR_RANGE if accel or decel is <= 0.
 */
dc_motor_status_t dc_motor_ramp_init(dc_motor_ramp_t *ramp,
                                     float accel, float decel);

/**
 * @brief Initialise a ramp generator using the default limits defined in
 *        dc_motor_config.h.
 *
 * @param ramp Pointer to the ramp state. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if ramp is NULL.
 */
dc_motor_status_t dc_motor_ramp_init_default(dc_motor_ramp_t *ramp);

/**
 * @brief Set the ramp target.
 *
 * @details The current value is not changed; it will slew towards the new
 *          target on subsequent calls to dc_motor_ramp_update().
 *
 * @param ramp   Pointer to the ramp state. Must not be NULL.
 * @param target New target value.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if ramp is NULL.
 */
dc_motor_status_t dc_motor_ramp_set_target(dc_motor_ramp_t *ramp,
                                           float target);

/**
 * @brief Force the current value of the ramp without slewing.
 *
 * @details Useful when the motor is re-enabled after an idle period or
 *          when the host wants to synchronise the ramp with an external
 *          measured value. The target is also set to the supplied value.
 *
 * @param ramp   Pointer to the ramp state. Must not be NULL.
 * @param value  Value to inject as both current and target.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if ramp is NULL.
 */
dc_motor_status_t dc_motor_ramp_reset(dc_motor_ramp_t *ramp, float value);

/**
 * @brief Advance the ramp by one control period.
 *
 * @details The acceleration limit is used whenever |target| > |current|
 *          (the motor is speeding up); the deceleration limit is used
 *          otherwise (braking). When crossing zero, the limit applied is
 *          the one corresponding to the smaller of |current| and |target|.
 *
 * @param ramp Pointer to the ramp state. Must not be NULL.
 * @param dt   Time elapsed since the last update, in seconds. Must be > 0.
 * @return The updated ramp current value. Returns 0.0f if ramp is NULL or
 *         dt <= 0.
 */
float dc_motor_ramp_update(dc_motor_ramp_t *ramp, float dt);

/**
 * @brief Return non-zero if the ramp has reached its target.
 *
 * @param ramp Pointer to the ramp state.
 * @return 1 if |current - target| is below 1e-3, 0 otherwise.
 */
int dc_motor_ramp_is_idle(const dc_motor_ramp_t *ramp);

#else

typedef struct dc_motor_ramp_t_empty { uint8_t _unused; } dc_motor_ramp_t;

#endif /* DC_MOTOR_ENABLE_RAMP */

#endif /* DC_MOTOR_RAMP_H */
