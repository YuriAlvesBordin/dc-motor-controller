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

float dc_motor_openloop_get_target(const dc_motor_openloop_t *ol)
{
    if (ol == NULL)
    {
        return 0.0f;
    }
    return ol->target;
}

float dc_motor_openloop_get_output(const dc_motor_openloop_t *ol)
{
    if (ol == NULL)
    {
        return 0.0f;
    }
    return ol->output;
}

#endif /* DC_MOTOR_ENABLE_OPENLOOP */
