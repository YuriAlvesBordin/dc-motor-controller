/**
 * @file    dc_motor_ramp.c
 * @brief   Slew-rate limited setpoint generator implementation.
 */

#include "dc_motor_ramp.h"

#if DC_MOTOR_ENABLE_RAMP

#include <stddef.h>

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
                                     float accel, float decel)
{
    if (ramp == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    if ((accel <= 0.0f) || (decel <= 0.0f))
    {
        return DC_MOTOR_ERR_RANGE;
    }

    ramp->current = 0.0f;
    ramp->target = 0.0f;
    ramp->accel = accel;
    ramp->decel = decel;
    return DC_MOTOR_OK;
}

/**
 * @brief Initialise a ramp generator using the default limits defined in
 *        dc_motor_config.h.
 *
 * @param ramp Pointer to the ramp state. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if ramp is NULL.
 */
dc_motor_status_t dc_motor_ramp_init_default(dc_motor_ramp_t *ramp)
{
    return dc_motor_ramp_init(ramp,
                              DC_MOTOR_RAMP_DEFAULT_ACCEL,
                              DC_MOTOR_RAMP_DEFAULT_DECEL);
}

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
                                           float target)
{
    if (ramp == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    ramp->target = target;
    return DC_MOTOR_OK;
}

/**
 * @brief Force the current value of the ramp without slewing.
 *
 * @details Useful when the motor is re-enabled after an idle period or
 *          when the host wants to synchronise the ramp with an external
 *          measured value. The target is also set to the supplied value.
 *
 * @param ramp  Pointer to the ramp state. Must not be NULL.
 * @param value Value to inject as both current and target.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if ramp is NULL.
 */
dc_motor_status_t dc_motor_ramp_reset(dc_motor_ramp_t *ramp, float value)
{
    if (ramp == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    ramp->current = value;
    ramp->target = value;
    return DC_MOTOR_OK;
}

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
float dc_motor_ramp_update(dc_motor_ramp_t *ramp, float dt)
{
    float diff;
    float step;
    float limit;
    int speeding_up;

    if ((ramp == NULL) || (dt <= 0.0f))
    {
        return 0.0f;
    }

    diff = ramp->target - ramp->current;
    if (diff == 0.0f)
    {
        return ramp->current;
    }

    if (ramp->current >= 0.0f)
    {
        speeding_up = (ramp->target > ramp->current);
    }
    else
    {
        speeding_up = (ramp->target < ramp->current);
    }

    limit = speeding_up ? ramp->accel : ramp->decel;
    step = limit * dt;

    if (diff > step)
    {
        ramp->current += step;
    }
    else if (diff < -step)
    {
        ramp->current -= step;
    }
    else
    {
        ramp->current = ramp->target;
    }

    return ramp->current;
}

/**
 * @brief Return non-zero if the ramp has reached its target.
 *
 * @param ramp Pointer to the ramp state.
 * @return 1 if |current - target| is below 1e-3, 0 otherwise.
 */
int dc_motor_ramp_is_idle(const dc_motor_ramp_t *ramp)
{
    float diff;
    const float threshold = 1e-3f;

    if (ramp == NULL)
    {
        return 1;
    }

    diff = ramp->target - ramp->current;
    if (diff < 0.0f)
    {
        diff = -diff;
    }
    return (diff <= threshold) ? 1 : 0;
}

#endif