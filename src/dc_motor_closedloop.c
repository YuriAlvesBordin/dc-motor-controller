#include "dc_motor_closedloop.h"

#if DC_MOTOR_ENABLE_CLOSEDLOOP

#include <stddef.h>

dc_motor_status_t dc_motor_closedloop_init(dc_motor_closedloop_t *cl)
{
    if (cl == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    cl->setpoint_raw = 0.0f;
    cl->setpoint_eff = 0.0f;
    cl->measured = 0.0f;
    cl->output = 0.0f;

    dc_motor_pid_init_default(&cl->pid);

#if DC_MOTOR_ENABLE_RAMP
    dc_motor_ramp_init_default(&cl->ramp);
#endif
    return DC_MOTOR_OK;
}

dc_motor_status_t dc_motor_closedloop_set_setpoint(dc_motor_closedloop_t *cl,
                                                   float setpoint)
{
    if (cl == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    cl->setpoint_raw = setpoint;
#if DC_MOTOR_ENABLE_RAMP
    dc_motor_ramp_set_target(&cl->ramp, setpoint);
#else
    cl->setpoint_eff = setpoint;
#endif
    return DC_MOTOR_OK;
}

dc_motor_status_t dc_motor_closedloop_set_measured(dc_motor_closedloop_t *cl,
                                                   float measured)
{
    if (cl == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    cl->measured = measured;
    return DC_MOTOR_OK;
}

float dc_motor_closedloop_update(dc_motor_closedloop_t *cl, float dt)
{
    if ((cl == NULL) || (dt <= 0.0f))
    {
        return 0.0f;
    }

#if DC_MOTOR_ENABLE_RAMP
    cl->setpoint_eff = dc_motor_ramp_update(&cl->ramp, dt);
#else
    cl->setpoint_eff = cl->setpoint_raw;
#endif

    cl->output = dc_motor_pid_update(&cl->pid,
                                     cl->setpoint_eff,
                                     cl->measured,
                                     dt);
    return cl->output;
}

dc_motor_status_t dc_motor_closedloop_reset(dc_motor_closedloop_t *cl)
{
    if (cl == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    dc_motor_pid_reset(&cl->pid);
    cl->setpoint_eff = cl->setpoint_raw;
    cl->output = 0.0f;
#if DC_MOTOR_ENABLE_RAMP
    dc_motor_ramp_reset(&cl->ramp, cl->setpoint_raw);
#endif
    return DC_MOTOR_OK;
}

dc_motor_pid_t *dc_motor_closedloop_get_pid(dc_motor_closedloop_t *cl)
{
    if (cl == NULL)
    {
        return NULL;
    }
    return &cl->pid;
}

float dc_motor_closedloop_get_setpoint(const dc_motor_closedloop_t *cl)
{
    if (cl == NULL)
    {
        return 0.0f;
    }
    return cl->setpoint_eff;
}

float dc_motor_closedloop_get_output(const dc_motor_closedloop_t *cl)
{
    if (cl == NULL)
    {
        return 0.0f;
    }
    return cl->output;
}

#endif /* DC_MOTOR_ENABLE_CLOSEDLOOP */
