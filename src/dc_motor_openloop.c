/**
 * @file    dc_motor_openloop.c
 * @brief   Open-loop (ramped command) mode implementation.
 */

#include "dc_motor_openloop.h"

#if DC_MOTOR_ENABLE_OPENLOOP

#include <stddef.h>

static float dc_motor_openloop_clamp(float v)
{
    if (v > DC_MOTOR_OPENLOOP_OUT_MAX)
    {
        return DC_MOTOR_OPENLOOP_OUT_MAX;
    }
    if (v < DC_MOTOR_OPENLOOP_OUT_MIN)
    {
        return DC_MOTOR_OPENLOOP_OUT_MIN;
    }
    return v;
}

static float dc_motor_openloop_apply_deadband(float v)
{
    float abs_v;
    if (DC_MOTOR_OPENLOOP_DEADBAND <= 0.0f)
    {
        return v;
    }
    abs_v = (v < 0.0f) ? -v : v;
    if (abs_v < DC_MOTOR_OPENLOOP_DEADBAND)
    {
        return 0.0f;
    }
    return v;
}

/**
 * @brief Initialise an open-loop controller with default ramp limits.
 *
 * @param ol Pointer to the open-loop state. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if ol is NULL.
 */
dc_motor_status_t dc_motor_openloop_init(dc_motor_openloop_t *ol)
{
    if (ol == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    ol->target = 0.0f;
    ol->output = 0.0f;
#if DC_MOTOR_ENABLE_RAMP
    dc_motor_ramp_init_default(&ol->ramp);
#endif
    return DC_MOTOR_OK;
}

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
                                               float target)
{
    if (ol == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    ol->target = dc_motor_openloop_clamp(target);
#if DC_MOTOR_ENABLE_RAMP
    dc_motor_ramp_set_target(&ol->ramp, ol->target);
#endif
    return DC_MOTOR_OK;
}

/**
 * @brief Force the open-loop output to a specific value without slewing.
 *
 * @param ol    Pointer to the open-loop state. Must not be NULL.
 * @param value Output value to inject (will be clamped to the limits).
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if ol is NULL.
 */
dc_motor_status_t dc_motor_openloop_reset(dc_motor_openloop_t *ol,
                                          float value)
{
    if (ol == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    ol->target = dc_motor_openloop_clamp(value);
    ol->output = ol->target;
#if DC_MOTOR_ENABLE_RAMP
    dc_motor_ramp_reset(&ol->ramp, ol->output);
#endif
    return DC_MOTOR_OK;
}

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
float dc_motor_openloop_update(dc_motor_openloop_t *ol, float dt)
{
    if ((ol == NULL) || (dt <= 0.0f))
    {
        return 0.0f;
    }

#if DC_MOTOR_ENABLE_RAMP
    ol->output = dc_motor_ramp_update(&ol->ramp, dt);
#else
    ol->output = ol->target;
#endif

    ol->output = dc_motor_openloop_apply_deadband(ol->output);
    return ol->output;
}

/**
 * @brief Return the last commanded target.
 *
 * @param ol Pointer to the open-loop state.
 * @return The target level, or 0.0f if ol is NULL.
 */
float dc_motor_openloop_get_target(const dc_motor_openloop_t *ol)
{
    if (ol == NULL)
    {
        return 0.0f;
    }
    return ol->target;
}

/**
 * @brief Return the most recently applied output.
 *
 * @param ol Pointer to the open-loop state.
 * @return The output level, or 0.0f if ol is NULL.
 */
float dc_motor_openloop_get_output(const dc_motor_openloop_t *ol)
{
    if (ol == NULL)
    {
        return 0.0f;
    }
    return ol->output;
}

#endif /* DC_MOTOR_ENABLE_OPENLOOP */