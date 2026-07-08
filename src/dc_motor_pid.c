#include "dc_motor_pid.h"

#if DC_MOTOR_ENABLE_CLOSEDLOOP

#include <stddef.h>

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

    pid->kp        = kp;
    pid->ki        = ki;
    pid->kd        = kd;
    pid->out_min   = out_min;
    pid->out_max   = out_max;
    pid->integral  = 0.0f;
    pid->prev_error = 0.0f;
#if DC_MOTOR_PID_DERIV_ON_MEASUREMENT
    pid->prev_meas = 0.0f;
#endif
#if DC_MOTOR_ENABLE_PID_D_FILTER
    pid->d_filter   = DC_MOTOR_PID_DEFAULT_D_FILTER;
    pid->deriv_filt = 0.0f;
#endif
    return DC_MOTOR_OK;
}

dc_motor_status_t dc_motor_pid_init_default(dc_motor_pid_t *pid)
{
    return dc_motor_pid_init(pid,
                             DC_MOTOR_PID_DEFAULT_KP,
                             DC_MOTOR_PID_DEFAULT_KI,
                             DC_MOTOR_PID_DEFAULT_KD,
                             DC_MOTOR_PID_DEFAULT_OUT_MIN,
                             DC_MOTOR_PID_DEFAULT_OUT_MAX);
}

dc_motor_status_t dc_motor_pid_reset(dc_motor_pid_t *pid)
{
    if (pid == NULL)
    {
        return DC_MOTOR_ERR_NULL;
    }
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
#if DC_MOTOR_PID_DERIV_ON_MEASUREMENT
    pid->prev_meas = 0.0f;
#endif
#if DC_MOTOR_ENABLE_PID_D_FILTER
    pid->deriv_filt = 0.0f;
#endif
    return DC_MOTOR_OK;
}

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

float dc_motor_pid_update(dc_motor_pid_t *pid,
                          float setpoint, float measured, float dt)
{
    float error;
    float p_term;
    float i_term;
    float d_term;
    float raw_output;
    float clamped_output;

    if ((pid == NULL) || (dt <= 0.0f))
    {
        return 0.0f;
    }

    error  = setpoint - measured;
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

    pid->integral += error * dt;

#if DC_MOTOR_ENABLE_PID_ANTI_WINDUP
    {
        /* Conditional integration anti-windup: only clamp the integral when
         * the output is saturated AND the integral is pulling in the same
         * direction as the saturation (true windup condition).
         * This prevents the integral from being frozen at out_min when the
         * motor stalls and the controller needs to keep increasing the duty. */
        float i_contribution = pid->ki * pid->integral;
        float unclamped      = p_term + i_contribution + d_term;

        if ((unclamped > pid->out_max) && (i_contribution > 0.0f))
        {
            pid->integral = (pid->out_max - p_term - d_term) / pid->ki;
        }
        else if ((unclamped < pid->out_min) && (i_contribution < 0.0f))
        {
            pid->integral = (pid->out_min - p_term - d_term) / pid->ki;
        }
    }
#endif

    i_term     = pid->ki * pid->integral;
    raw_output = p_term + i_term + d_term;

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
