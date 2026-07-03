#include "dc_motor_config.h"

#if DC_MOTOR_ENABLE_PID

#include "internal/dc_motor_pid.h"

void DcMotor_Pid_DefaultConfig(DcMotor_PidConfig_t *cfg)
{
    if (!cfg) return;
    cfg->kp                 = DC_MOTOR_DEFAULT_KP;
    cfg->ki                 = DC_MOTOR_DEFAULT_KI;
    cfg->kd                 = DC_MOTOR_DEFAULT_KD;
    cfg->dt_s               = DC_MOTOR_DEFAULT_DT_S;
    cfg->output_min         = 0.0f;
    cfg->output_max         = 1.0f;
    cfg->integral_min       = -1.0f;
    cfg->integral_max       = 1.0f;
    cfg->deriv_filter_alpha = DC_MOTOR_DERIV_FILTER_ALPHA;
}

void DcMotor_Pid_Reset(DcMotor_PidState_t *st)
{
    if (!st) return;
    st->integral       = 0.0f;
    st->prev_error     = 0.0f;
    st->filtered_deriv = 0.0f;
}

float DcMotor_Pid_Compute(const DcMotor_PidConfig_t *cfg,
                           DcMotor_PidState_t        *st,
                           float                      dt_s,
                           float                      setpoint,
                           float                      measured)
{
    if (!cfg || !st || dt_s <= 0.0f) return 0.0f;

    float error = setpoint - measured;

    st->integral += cfg->ki * error * dt_s;
    if (st->integral > cfg->integral_max) st->integral = cfg->integral_max;
    if (st->integral < cfg->integral_min) st->integral = cfg->integral_min;

    float raw_deriv    = (error - st->prev_error) / dt_s;
    float alpha        = cfg->deriv_filter_alpha;
    float deriv        = alpha * raw_deriv + (1.0f - alpha) * st->filtered_deriv;
    st->filtered_deriv = deriv;
    st->prev_error     = error;

    float out = cfg->kp * error + st->integral + cfg->kd * deriv;
    if (out > cfg->output_max) out = cfg->output_max;
    if (out < cfg->output_min) out = cfg->output_min;
    return out;
}

#endif
