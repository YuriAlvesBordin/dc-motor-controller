#include "dc_motor.h"

#include <stddef.h>

dc_motor_status_t dc_motor_init(dc_motor_t *m)
{
    if (m == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }

    m->mode = DC_MOTOR_MODE_IDLE;
    m->output = 0.0f;
    m->measured = 0.0f;
    m->elapsed_ms = 0u;

#if DC_MOTOR_ENABLE_OPENLOOP
    dc_motor_openloop_init(&m->openloop);
#endif

#if DC_MOTOR_ENABLE_CLOSEDLOOP
    dc_motor_closedloop_init(&m->closedloop);
#endif

#if DC_MOTOR_ENABLE_WATCHDOG
    m->watchdog_last_feed_ms = 0u;
    m->watchdog_tripped = 0u;
#endif

    return DC_MOTOR_OK;
}

dc_motor_status_t dc_motor_stop(dc_motor_t *m)
{
    if (m == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }

    m->mode = DC_MOTOR_MODE_IDLE;
    m->output = 0.0f;

#if DC_MOTOR_ENABLE_OPENLOOP
    dc_motor_openloop_reset(&m->openloop, 0.0f);
#endif

#if DC_MOTOR_ENABLE_CLOSEDLOOP
    dc_motor_closedloop_reset(&m->closedloop);
#endif

#if DC_MOTOR_ENABLE_WATCHDOG
    m->watchdog_last_feed_ms = 0u;
    m->watchdog_tripped = 0u;
#endif

    return DC_MOTOR_OK;
}

dc_motor_status_t dc_motor_set_openloop(dc_motor_t *m, float level)
{
#if !DC_MOTOR_ENABLE_OPENLOOP
    (void)m;
    (void)level;
    return DC_MOTOR_ERR_DISABLED;
#else
    if (m == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    m->mode = DC_MOTOR_MODE_OPENLOOP;
    dc_motor_openloop_set_target(&m->openloop, level);
    return DC_MOTOR_OK;
#endif
}

dc_motor_status_t dc_motor_set_closedloop(dc_motor_t *m, float setpoint)
{
#if !DC_MOTOR_ENABLE_CLOSEDLOOP
    (void)m;
    (void)setpoint;
    return DC_MOTOR_ERR_DISABLED;
#else
    if (m == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    m->mode = DC_MOTOR_MODE_CLOSEDLOOP;
    dc_motor_closedloop_set_setpoint(&m->closedloop, setpoint);
    return DC_MOTOR_OK;
#endif
}

dc_motor_status_t dc_motor_set_measured(dc_motor_t *m, float measured)
{
    if (m == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    m->measured = measured;
#if DC_MOTOR_ENABLE_CLOSEDLOOP
    if (m->mode == DC_MOTOR_MODE_CLOSEDLOOP)
    {
        dc_motor_closedloop_set_measured(&m->closedloop, measured);
    }
#endif
    return DC_MOTOR_OK;
}

float dc_motor_update(dc_motor_t *m, float dt)
{
    if ((m == NULL) || (dt <= 0.0f))
    {
        return 0.0f;
    }

#if DC_MOTOR_ENABLE_WATCHDOG
    if (m->watchdog_tripped)
    {
        m->output = 0.0f;
        return 0.0f;
    }
    m->watchdog_last_feed_ms = 0u;
#endif

    switch (m->mode)
    {
        case DC_MOTOR_MODE_IDLE:
            m->output = 0.0f;
            break;

#if DC_MOTOR_ENABLE_OPENLOOP
        case DC_MOTOR_MODE_OPENLOOP:
            m->output = dc_motor_openloop_update(&m->openloop, dt);
            break;
#endif

#if DC_MOTOR_ENABLE_CLOSEDLOOP
        case DC_MOTOR_MODE_CLOSEDLOOP:
            m->output = dc_motor_closedloop_update(&m->closedloop, dt);
            break;
#endif

        default:
            m->output = 0.0f;
            break;
    }

    return m->output;
}

dc_motor_status_t dc_motor_feed_watchdog(dc_motor_t *m)
{
#if !DC_MOTOR_ENABLE_WATCHDOG
    (void)m;
    return DC_MOTOR_ERR_DISABLED;
#else
    if (m == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    m->watchdog_last_feed_ms = 0u;
    return DC_MOTOR_OK;
#endif
}

dc_motor_status_t dc_motor_tick(dc_motor_t *m, uint32_t elapsed_ms)
{
#if !DC_MOTOR_ENABLE_WATCHDOG
    (void)m;
    (void)elapsed_ms;
    return DC_MOTOR_ERR_DISABLED;
#else
    uint32_t limit_ms;

    if (m == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }

    m->elapsed_ms += elapsed_ms;
    m->watchdog_last_feed_ms += elapsed_ms;

    limit_ms = (uint32_t)DC_MOTOR_WATCHDOG_TIMEOUT_MS;
    if (m->watchdog_last_feed_ms > limit_ms)
    {
        m->watchdog_tripped = 1u;
        m->output = 0.0f;
        return DC_MOTOR_ERR_WATCHDOG;
    }
    return DC_MOTOR_OK;
#endif
}

dc_motor_mode_t dc_motor_get_mode(const dc_motor_t *m)
{
    if (m == NULL)
    {
        return DC_MOTOR_MODE_IDLE;
    }
    return m->mode;
}

float dc_motor_get_output(const dc_motor_t *m)
{
    if (m == NULL)
    {
        return 0.0f;
    }
    return m->output;
}

float dc_motor_get_measured(const dc_motor_t *m)
{
    if (m == NULL)
    {
        return 0.0f;
    }
    return m->measured;
}
