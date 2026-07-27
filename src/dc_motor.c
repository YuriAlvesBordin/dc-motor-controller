/**
 * @file    dc_motor.c
 * @brief   Top-level DC motor control API implementation.
 */

#include "dc_motor.h"

#include <stddef.h>

/**
 * @brief Initialise a motor instance.
 *
 * @param m Pointer to the motor instance. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if m is NULL.
 */
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

/**
 * @brief Switch the motor to IDLE mode and zero the output.
 *
 * @details Also resets the PID and ramp sub-controllers so the next
 *          command starts from a clean state.
 *
 * @param m Pointer to the motor instance. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if m is NULL.
 */
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

/**
 * @brief Command the motor in open-loop mode.
 *
 * @details Switches the mode to OPENLOOP (if not already), sets the
 *          commanded level (e.g. 50.0 for half-throttle forward), and lets
 *          the ramp generator bring the output towards it. Available only
 *          when DC_MOTOR_ENABLE_OPENLOOP is defined.
 *
 * @param m      Pointer to the motor instance. Must not be NULL.
 * @param level  Desired output level in percent (signed for direction).
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if m is NULL,
 *         DC_MOTOR_ERR_DISABLED if open-loop is disabled at compile time.
 */
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

/**
 * @brief Command the motor in closed-loop mode.
 *
 * @details Switches the mode to CLOSEDLOOP and sets the target process
 *          value the PID will track. The measured value must be pushed
 *          separately via dc_motor_set_measured(). Available only when
 *          DC_MOTOR_ENABLE_CLOSEDLOOP is defined.
 *
 * @param m         Pointer to the motor instance. Must not be NULL.
 * @param setpoint  Desired process value (units defined by the host).
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if m is NULL,
 *         DC_MOTOR_ERR_DISABLED if closed-loop is disabled at compile time.
 */
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

/**
 * @brief Push the latest measured process value.
 *
 * @details Only used in closed-loop mode. In open-loop mode the call is
 *          accepted but has no effect on the output.
 *
 * @param m        Pointer to the motor instance. Must not be NULL.
 * @param measured Latest reading from the host's sensor.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if m is NULL.
 */
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

/**
 * @brief Run one control iteration.
 *
 * @details Must be called by the host at the period defined by
 *          DC_MOTOR_CONTROL_PERIOD_SEC. The function:
 *            - checks the watchdog (if enabled),
 *            - dispatches to the open-loop or closed-loop sub-controller
 *              depending on the current mode,
 *            - copies the produced output into m->output.
 *
 * @param m  Pointer to the motor instance. Must not be NULL.
 * @param dt Control period in seconds. Must be > 0.
 * @return The updated output command in percent, or 0.0f on error.
 */
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

/**
 * @brief Refresh the watchdog without changing the operating mode.
 *
 * @details Only meaningful when DC_MOTOR_ENABLE_WATCHDOG is defined.
 *
 * @param m Pointer to the motor instance. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if m is NULL,
 *         DC_MOTOR_ERR_DISABLED if the watchdog is disabled.
 */
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

/**
 * @brief Notify the library of elapsed real time for watchdog tracking.
 *
 * @details The host typically calls this from a 1 ms tick with
 *          elapsed_ms = 1. When the elapsed time since the last feed
 *          exceeds DC_MOTOR_WATCHDOG_TIMEOUT_MS the output is forced to
 *          zero and the watchdog_tripped flag is set. The flag is cleared
 *          by calling dc_motor_stop() followed by dc_motor_init().
 *
 * @param m          Pointer to the motor instance. Must not be NULL.
 * @param elapsed_ms Milliseconds elapsed since the last tick call.
 * @return DC_MOTOR_OK on success,
 *         DC_MOTOR_ERR_NULL if m is NULL,
 *         DC_MOTOR_ERR_WATCHDOG if the watchdog has tripped (output forced
 *         to zero),
 *         DC_MOTOR_ERR_DISABLED if the watchdog is disabled.
 */
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

/**
 * @brief Return the current operating mode.
 *
 * @param m Pointer to the motor instance.
 * @return The mode, or DC_MOTOR_MODE_IDLE if m is NULL.
 */
dc_motor_mode_t dc_motor_get_mode(const dc_motor_t *m)
{
    if (m == NULL)
    {
        return DC_MOTOR_MODE_IDLE;
    }
    return m->mode;
}

/**
 * @brief Return the most recent output command.
 *
 * @param m Pointer to the motor instance.
 * @return The output in percent, or 0.0f if m is NULL.
 */
float dc_motor_get_output(const dc_motor_t *m)
{
    if (m == NULL)
    {
        return 0.0f;
    }
    return m->output;
}

/**
 * @brief Return the current measured process value.
 *
 * @param m Pointer to the motor instance.
 * @return The measured value, or 0.0f if m is NULL.
 */
float dc_motor_get_measured(const dc_motor_t *m)
{
    if (m == NULL)
    {
        return 0.0f;
    }
    return m->measured;
}