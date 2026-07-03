#include "dc_motor_pid.h"
#include "dc_motor_types.h"

/* ------------------------------------------------------------------ */
/*  Internal helper                                                    */
/* ------------------------------------------------------------------ */

static float pid_clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void DcMotor_Pid_Reset(DcMotor_PidState_t *st)
{
    if (!st) return;
    st->integral       = 0.0f;
    st->prev_error     = 0.0f;
    st->filtered_deriv = 0.0f;
}

void DcMotor_Pid_DefaultConfig(DcMotor_PidConfig_t *cfg)
{
    if (!cfg) return;
    *cfg = (DcMotor_PidConfig_t){
        .kp                 = DC_MOTOR_DEFAULT_KP,
        .ki                 = DC_MOTOR_DEFAULT_KI,
        .kd                 = DC_MOTOR_DEFAULT_KD,
        .dt_s               = DC_MOTOR_DEFAULT_DT_S,
        .output_min         = 0.0f,
        .output_max         = 1.0f,
        .integral_min       = -0.5f,
        .integral_max       =  0.5f,
        .deriv_filter_alpha = DC_MOTOR_DERIV_FILTER_ALPHA,
    };
}

float DcMotor_Pid_Compute(const DcMotor_PidConfig_t *cfg,
                          DcMotor_PidState_t        *st,
                          float                      dt_s,
                          float                      setpoint,
                          float                      measured)
{
    if (!cfg || !st || dt_s <= 0.0f) return 0.0f;

    float err = setpoint - measured;

    /* Proportional */
    float p = cfg->kp * DC_MOTOR_PID_GAIN_SCALE * err;

    /* Integral with anti-windup clamp */
    st->integral += cfg->ki * DC_MOTOR_PID_GAIN_SCALE * err * dt_s;
    st->integral  = pid_clamp(st->integral, cfg->integral_min, cfg->integral_max);

    /* Derivative with low-pass filter */
    float d_raw          = (err - st->prev_error) / dt_s;
    float alpha          = pid_clamp(cfg->deriv_filter_alpha, 0.0f, 1.0f);
    st->filtered_deriv   = alpha * d_raw + (1.0f - alpha) * st->filtered_deriv;
    float d              = cfg->kd * DC_MOTOR_PID_GAIN_SCALE * st->filtered_deriv;

    st->prev_error = err;

    return p + st->integral + d;
}
