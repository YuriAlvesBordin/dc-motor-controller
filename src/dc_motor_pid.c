/**
 * @file    dc_motor_pid.c
 * @brief   PID controller implementation for closed-loop motor control.
 */

#include "dc_motor_pid.h"

#if DC_MOTOR_ENABLE_CLOSEDLOOP

#include <stddef.h>

/**
 * @brief Initialise a PID controller with explicit gains and limits.
 *
 * @param pid     Pointer to the controller state. Must not be NULL.
 * @param kp      Proportional gain.
 * @param ki      Integral gain (per second).
 * @param kd      Derivative gain (per second).
 * @param out_min Minimum output value (clamping lower bound).
 * @param out_max Maximum output value (clamping upper bound).
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if pid is NULL,
 *         or DC_MOTOR_ERR_RANGE if out_min >= out_max.
 */
dc_motor_status_t dc_motor_pid_init(dc_motor_pid_t *pid,
                                    float kp, float ki, float kd,
                                    float out_min, float out_max)
{
    if (pid == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    if (out_min >= out_max)
    {
        return DC_MOTOR_ERR_RANGE;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_setpoint = 0.0f;
#if DC_MOTOR_PID_DERIV_ON_MEASUREMENT
    pid->prev_meas = 0.0f;
#endif
#if DC_MOTOR_ENABLE_PID_D_FILTER
    pid->d_filter = DC_MOTOR_PID_DEFAULT_D_FILTER;
    pid->deriv_filt = 0.0f;
#endif
#if DC_MOTOR_ENABLE_PID_FEEDFORWARD
    pid->kff = DC_MOTOR_PID_DEFAULT_KFF;
    pid->kff_static = DC_MOTOR_PID_DEFAULT_KFF_STATIC;
#endif
    return DC_MOTOR_OK;
}

/**
 * @brief Initialise a PID controller using the default gains and limits
 *        defined in dc_motor_config.h.
 *
 * @param pid Pointer to the controller state. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if pid is NULL.
 */
dc_motor_status_t dc_motor_pid_init_default(dc_motor_pid_t *pid)
{
    return dc_motor_pid_init(pid,
                             DC_MOTOR_PID_DEFAULT_KP,
                             DC_MOTOR_PID_DEFAULT_KI,
                             DC_MOTOR_PID_DEFAULT_KD,
                             DC_MOTOR_PID_DEFAULT_OUT_MIN,
                             DC_MOTOR_PID_DEFAULT_OUT_MAX);
}

/**
 * @brief Reset the controller state (integral and derivative memories).
 *
 * @details Useful when switching setpoint stepwise or when re-enabling the
 *          controller after being idle. Gains and limits are preserved.
 *
 * @param pid Pointer to the controller state. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if pid is NULL.
 */
dc_motor_status_t dc_motor_pid_reset(dc_motor_pid_t *pid)
{
    if (pid == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
#if DC_MOTOR_PID_DERIV_ON_MEASUREMENT
    pid->prev_meas = 0.0f;
#endif
#if DC_MOTOR_ENABLE_PID_D_FILTER
    pid->deriv_filt = 0.0f;
#endif
    return DC_MOTOR_OK;
}

/**
 * @brief Update gains at runtime without resetting the state.
 *
 * @param pid Pointer to the controller state. Must not be NULL.
 * @param kp  New proportional gain.
 * @param ki  New integral gain.
 * @param kd  New derivative gain.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if pid is NULL.
 */
dc_motor_status_t dc_motor_pid_tune(dc_motor_pid_t *pid,
                                    float kp, float ki, float kd)
{
    if (pid == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    return DC_MOTOR_OK;
}

#if DC_MOTOR_ENABLE_PID_FEEDFORWARD
/**
 * @brief Update feed-forward gains at runtime.
 *
 * @param pid        Pointer to the controller state. Must not be NULL.
 * @param kff        Velocity feed-forward gain.
 * @param kff_static Static feed-forward offset (applied when setpoint != 0).
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if pid is NULL.
 */
dc_motor_status_t dc_motor_pid_set_feedforward(dc_motor_pid_t *pid,
                                               float kff, float kff_static)
{
    if (pid == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    pid->kff = kff;
    pid->kff_static = kff_static;
    return DC_MOTOR_OK;
}
#endif

/**
 * @brief Compute one PID output sample.
 *
 * @param pid       Pointer to the controller state. Must not be NULL.
 * @param setpoint  Desired process value.
 * @param measured  Current process value (from the host's sensor).
 * @param dt        Time elapsed since the last update, in seconds. Must be > 0.
 * @return The clamped controller output in the range [out_min, out_max].
 *         If pid is NULL or dt <= 0 the function returns 0.0f.
 */
float dc_motor_pid_update(dc_motor_pid_t *pid,
                          float setpoint, float measured, float dt)
{
    float error;
    float p_term;
    float i_term;
    float d_term;
    float ff_term;
    float raw_output;
    float clamped_output;

    if ((pid == NULL) || (dt <= 0.0f))
    {
        return 0.0f;
    }

    error = setpoint - measured;

    if (setpoint != pid->prev_setpoint)
    {
        pid->prev_error = error;
#if DC_MOTOR_PID_DERIV_ON_MEASUREMENT
        pid->prev_meas = measured;
#endif
    }
    pid->prev_setpoint = setpoint;

    p_term = pid->kp * error;

#if DC_MOTOR_PID_DERIV_ON_MEASUREMENT
    {
        float d_raw = (measured - pid->prev_meas) / dt;
#if DC_MOTOR_ENABLE_PID_D_FILTER
        float a = pid->d_filter;
        pid->deriv_filt = a * d_raw + (1.0f - a) * pid->deriv_filt;
        d_term = -pid->kd * pid->deriv_filt;
#else
        d_term = -pid->kd * d_raw;
#endif
    }
#else
#if DC_MOTOR_ENABLE_PID_D_FILTER
    {
        float d_raw = (error - pid->prev_error) / dt;
        float a = pid->d_filter;
        pid->deriv_filt = a * d_raw + (1.0f - a) * pid->deriv_filt;
        d_term = pid->kd * pid->deriv_filt;
    }
#else
    d_term = pid->kd * (error - pid->prev_error) / dt;
#endif
#endif

#if DC_MOTOR_ENABLE_PID_FEEDFORWARD
    ff_term = pid->kff * setpoint;
    if (setpoint > 0.0f)
    {
        ff_term += pid->kff_static;
    }
    else if (setpoint < 0.0f)
    {
        ff_term -= pid->kff_static;
    }
#else
    ff_term = 0.0f;
#endif

    /*
     * Anti-windup: saturation-freeze.
     *
     * Only block integral accumulation when the output is genuinely
     * saturated at the HIGH end (> out_max) or the LOW end (< -out_max,
     * i.e. deeply negative — not merely near zero).
     *
     * Using out_min (= 0.0f) as the lower freeze threshold is wrong for a
     * unidirectional motor: in steady state with a small positive error the
     * tentative output can momentarily dip to 0 or slightly below due to the
     * D term or FF, which would incorrectly freeze the integral and let the
     * output drop to zero (motor stops).
     *
     * Solution: freeze only on |output| > out_max saturation. The output
     * clamp below still enforces [out_min, out_max] on what is sent to the
     * motor; the integrator is just allowed to go slightly negative
     * internally so it can recover quickly without the motor cutting out.
     */
#if DC_MOTOR_ENABLE_PID_ANTI_WINDUP
    {
        float tentative = p_term + pid->ki * pid->integral + d_term + ff_term;
        if (tentative < pid->out_max)
        {
            if (tentative > -(pid->out_max))
            {
                pid->integral += error * dt;
            }
        }
    }
#else
    pid->integral += error * dt;
#endif

    i_term = pid->ki * pid->integral;
    raw_output = p_term + i_term + d_term + ff_term;

    clamped_output = raw_output;
    if (clamped_output > pid->out_max)
    {
        clamped_output = pid->out_max;
    }
    else if (clamped_output < pid->out_min)
    {
        clamped_output = pid->out_min;
    }

    pid->prev_error = error;
#if DC_MOTOR_PID_DERIV_ON_MEASUREMENT
    pid->prev_meas = measured;
#endif

    return clamped_output;
}

#endif
