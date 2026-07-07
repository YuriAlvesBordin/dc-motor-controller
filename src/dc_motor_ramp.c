#include "dc_motor_ramp.h"

#if DC_MOTOR_ENABLE_RAMP

#include <stddef.h>

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
    ramp->target  = 0.0f;
    ramp->accel   = accel;
    ramp->decel   = decel;
    return DC_MOTOR_OK;
}

dc_motor_status_t dc_motor_ramp_init_default(dc_motor_ramp_t *ramp)
{
    return dc_motor_ramp_init(ramp,
                              DC_MOTOR_RAMP_DEFAULT_ACCEL,
                              DC_MOTOR_RAMP_DEFAULT_DECEL);
}

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

dc_motor_status_t dc_motor_ramp_reset(dc_motor_ramp_t *ramp, float value)
{
    if (ramp == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    ramp->current = value;
    ramp->target  = value;
    return DC_MOTOR_OK;
}

float dc_motor_ramp_update(dc_motor_ramp_t *ramp, float dt)
{
    float diff;
    float step;
    float limit;
    int   speeding_up;

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
    step  = limit * dt;

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
