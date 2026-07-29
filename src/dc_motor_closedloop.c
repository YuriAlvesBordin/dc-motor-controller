/**
 * @file    dc_motor_closedloop.c
 * @brief   Closed-loop PID mode implementation.
 */

#include "dc_motor_closedloop.h"

#if DC_MOTOR_ENABLE_CLOSEDLOOP

#include <stddef.h>

/**
 * @brief Initialise a closed-loop controller with default PID gains and
 *        ramp limits.
 *
 * @param cl Pointer to the closed-loop state. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if cl is NULL.
 */
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

/**
 * @brief Command a new setpoint.
 *
 * @details The setpoint is stored as-is. When a ramp is enabled the
 *          effective setpoint fed to the PID slews towards this value on
 *          subsequent calls to dc_motor_closedloop_update().
 *
 * @param cl       Pointer to the closed-loop state. Must not be NULL.
 * @param setpoint Desired process value.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if cl is NULL.
 */
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

/**
 * @brief Push the latest measured process value into the controller.
 *
 * @param cl       Pointer to the closed-loop state. Must not be NULL.
 * @param measured Latest reading from the host's sensor.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if cl is NULL.
 */
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

/**
 * @brief Compute one closed-loop output sample.
 *
 * @details The ramp is advanced first, then the PID is evaluated against
 *          the (possibly slewed) effective setpoint and the most recently
 *          pushed measured value. The dt argument must match
 *          DC_MOTOR_CONTROL_PERIOD_SEC.
 *
 *          Physical dead-band mapping (snap-to-minimum):
 *
 *            pid_out == 0            -> output = 0   (motor off)
 *            0 < pid_out < DEADBAND  -> output = DEADBAND (snap to min)
 *            pid_out >= DEADBAND     -> output = pid_out  (normal)
 *
 *          Snapping to the minimum instead of zero avoids chatter: if the
 *          steady-state operating point for a low setpoint falls inside the
 *          dead-band the motor would otherwise pulse on/off every cycle.
 *          The PID integrator is left untouched in all cases.
 *
 * @param cl Pointer to the closed-loop state. Must not be NULL.
 * @param dt Time elapsed since the last update, in seconds. Must be > 0.
 * @return The updated output level in percent. Returns 0.0f if cl is NULL
 *         or dt <= 0.
 */
float dc_motor_closedloop_update(dc_motor_closedloop_t *cl, float dt)
{
    float pid_out;

    if ((cl == NULL) || (dt <= 0.0f))
    {
        return 0.0f;
    }

    if (cl->setpoint_raw == 0.0f)
    {
        dc_motor_pid_reset(&cl->pid);
        cl->setpoint_eff = 0.0f;
        cl->output = 0.0f;
#if DC_MOTOR_ENABLE_RAMP
        dc_motor_ramp_reset(&cl->ramp, 0.0f);
#endif
        return 0.0f;
    }

#if DC_MOTOR_ENABLE_RAMP
    cl->setpoint_eff = dc_motor_ramp_update(&cl->ramp, dt);
#else
    cl->setpoint_eff = cl->setpoint_raw;
#endif

    pid_out = dc_motor_pid_update(&cl->pid,
                                  cl->setpoint_eff,
                                  cl->measured,
                                  dt);

    /*
     * Snap-to-minimum dead-band:
     *   - pid_out == 0           : motor off (setpoint is zero, handled above,
     *                              but guard here for safety)
     *   - 0 < pid_out < DEADBAND : snap up to DEADBAND so the motor always
     *                              receives a duty it can execute; prevents
     *                              chatter when the steady-state point sits
     *                              inside the dead-band (common at low RPM).
     *   - pid_out >= DEADBAND    : pass through unchanged.
     *
     * DC_MOTOR_PID_DEADBAND is a float literal — runtime if(), not #if.
     * The compiler folds the constant comparison away at -Os.
     */
    if (DC_MOTOR_PID_DEADBAND > 0.0f)
    {
        if ((pid_out > 0.0f) && (pid_out < DC_MOTOR_PID_DEADBAND))
        {
            cl->output = DC_MOTOR_PID_DEADBAND;
        }
        else
        {
            cl->output = pid_out;
        }
    }
    else
    {
        cl->output = pid_out;
    }

    return cl->output;
}

/**
 * @brief Reset the closed-loop state (PID + ramp).
 *
 * @details Gains and limits are preserved. The effective setpoint is
 *          forced to the current raw setpoint so no transient is triggered.
 *
 * @param cl Pointer to the closed-loop state. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if cl is NULL.
 */
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

/**
 * @brief Read-only accessor for the PID sub-state.
 *
 * @details Allows the host to retune the controller at runtime using
 *          dc_motor_pid_tune() without exposing the full struct.
 *
 * @param cl Pointer to the closed-loop state.
 * @return Pointer to the embedded PID, or NULL if cl is NULL.
 */
dc_motor_pid_t *dc_motor_closedloop_get_pid(dc_motor_closedloop_t *cl)
{
    if (cl == NULL)
    {
        return NULL;
    }
    return &cl->pid;
}

/**
 * @brief Return the effective setpoint (after the ramp).
 *
 * @param cl Pointer to the closed-loop state.
 * @return The effective setpoint, or 0.0f if cl is NULL.
 */
float dc_motor_closedloop_get_setpoint(const dc_motor_closedloop_t *cl)
{
    if (cl == NULL)
    {
        return 0.0f;
    }
    return cl->setpoint_eff;
}

/**
 * @brief Return the most recent PID output.
 *
 * @param cl Pointer to the closed-loop state.
 * @return The output level in percent, or 0.0f if cl is NULL.
 */
float dc_motor_closedloop_get_output(const dc_motor_closedloop_t *cl)
{
    if (cl == NULL)
    {
        return 0.0f;
    }
    return cl->output;
}

#endif /* DC_MOTOR_ENABLE_CLOSEDLOOP */
