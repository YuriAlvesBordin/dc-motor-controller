#include "dc_motor.h"
#include <string.h>
#include <math.h>

static float s_clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void s_apply_duty(DcMotor_Handle_t *hdm, float duty)
{
    hdm->current_duty = s_clamp(duty, 0.0f, 1.0f);
    hdm->port->set_pwm_duty(hdm->hw, hdm->current_duty);
}

static void s_latch_fault(DcMotor_Handle_t *hdm, DcMotor_State_t s, DcMotor_Status_t st)
{
    hdm->state       = s;
    hdm->last_status = st;
    hdm->target_duty = 0.0f;
    hdm->closed_loop = false;
    s_apply_duty(hdm, 0.0f);
    DcMotor_ResetPid(hdm);
    hdm->stall_active = false;
}

/* ------------------------------------------------------------------ */

static float s_ramp(DcMotor_Handle_t *hdm, float dt_s)
{
    float err   = hdm->target_duty - hdm->current_duty;
    float alpha = s_clamp(hdm->ramp_cfg.smooth_alpha, 0.0f, 1.0f);

    if (err == 0.0f) return hdm->current_duty;

    float rate     = (err > 0.0f) ? hdm->ramp_cfg.accel_rate : hdm->ramp_cfg.decel_rate;
    float step     = err * alpha;
    float max_step = rate * dt_s;

    if (step >  max_step) step =  max_step;
    if (step < -max_step) step = -max_step;

    float next = hdm->current_duty + step;

    /* snap to target to avoid hunting at the end of the ramp */
    if ((err > 0.0f && next > hdm->target_duty) ||
        (err < 0.0f && next < hdm->target_duty))
        next = hdm->target_duty;

    return next;
}

static float s_pid(DcMotor_Handle_t *hdm, float dt_s, float rpm)
{
    if (dt_s <= 0.0f) return hdm->current_duty;

    float err = hdm->rpm_setpoint - rpm;

    float p = hdm->pid_cfg.kp * DC_MOTOR_PID_GAIN_SCALE * err;

    hdm->pid_integral += hdm->pid_cfg.ki * DC_MOTOR_PID_GAIN_SCALE * err * dt_s;
    hdm->pid_integral  = s_clamp(hdm->pid_integral,
                                 hdm->pid_cfg.integral_min,
                                 hdm->pid_cfg.integral_max);

    /* filter the derivative to cut encoder noise — raw d/dt at 10 ms with
       optical encoders is basically a noise amplifier */
    float d_raw  = (err - hdm->pid_prev_error) / dt_s;
    float alpha  = s_clamp(hdm->pid_cfg.deriv_filter_alpha, 0.0f, 1.0f);
    hdm->pid_filtered_deriv = alpha * d_raw + (1.0f - alpha) * hdm->pid_filtered_deriv;
    float d = hdm->pid_cfg.kd * DC_MOTOR_PID_GAIN_SCALE * hdm->pid_filtered_deriv;

    hdm->pid_prev_error = err;

    return p + hdm->pid_integral + d;
}

static void s_stall_watchdog(DcMotor_Handle_t *hdm, uint32_t tick_ms, float rpm)
{
    if (hdm->safety_cfg.stall_timeout_ms == 0U) return;

    /* no point watching if the commanded duty is near zero */
    if (hdm->current_duty < hdm->safety_cfg.min_duty_for_stall) {
        hdm->stall_active = false;
        return;
    }

    if (fabsf(rpm) < hdm->safety_cfg.stall_min_rpm) {
        if (!hdm->stall_active) {
            hdm->stall_start_ms = tick_ms;
            hdm->stall_active   = true;
        } else if ((tick_ms - hdm->stall_start_ms) >= hdm->safety_cfg.stall_timeout_ms) {
            s_latch_fault(hdm, DC_MOTOR_FAULT_STALL, DC_MOTOR_ERR_STALL);
        }
    } else {
        hdm->stall_active = false;
    }
}

/* ------------------------------------------------------------------ */

DcMotor_Status_t DcMotor_Init(DcMotor_Handle_t             *hdm,
                              const DcMotor_Port_t         *port,
                              void                         *hw,
                              const DcMotor_PidConfig_t    *pid,
                              const DcMotor_RampConfig_t   *ramp,
                              const DcMotor_SafetyConfig_t *safety)
{
    if (!hdm || !port)         return DC_MOTOR_ERR_NULL_PTR;
    if (!port->set_pwm_duty)   return DC_MOTOR_ERR_PARAM;

    memset(hdm, 0, sizeof(*hdm));
    hdm->port = port;
    hdm->hw   = hw;

    if (pid) {
        if (pid->dt_s <= 0.0f)                     return DC_MOTOR_ERR_PARAM;
        if (pid->output_min > pid->output_max)     return DC_MOTOR_ERR_PARAM;
        if (pid->integral_min > pid->integral_max) return DC_MOTOR_ERR_PARAM;
        hdm->pid_cfg = *pid;
        hdm->pid_cfg.deriv_filter_alpha = s_clamp(hdm->pid_cfg.deriv_filter_alpha, 0.0f, 1.0f);
    } else {
        hdm->pid_cfg = (DcMotor_PidConfig_t){
            .kp = DC_MOTOR_DEFAULT_KP,
            .ki = DC_MOTOR_DEFAULT_KI,
            .kd = DC_MOTOR_DEFAULT_KD,
            .dt_s = DC_MOTOR_DEFAULT_DT_S,
            .output_min = 0.0f,    .output_max = 1.0f,
            .integral_min = -0.5f, .integral_max = 0.5f,
            .deriv_filter_alpha = DC_MOTOR_DERIV_FILTER_ALPHA,
        };
    }

    if (ramp) {
        if (ramp->accel_rate <= 0.0f || ramp->decel_rate <= 0.0f) return DC_MOTOR_ERR_PARAM;
        hdm->ramp_cfg = *ramp;
        hdm->ramp_cfg.smooth_alpha = s_clamp(hdm->ramp_cfg.smooth_alpha, 0.0f, 1.0f);
    } else {
        hdm->ramp_cfg = (DcMotor_RampConfig_t){
            .accel_rate  = DC_MOTOR_DEFAULT_ACCEL_RATE,
            .decel_rate  = DC_MOTOR_DEFAULT_DECEL_RATE,
            .smooth_alpha = DC_MOTOR_RAMP_SMOOTH_ALPHA,
        };
    }

    if (safety) {
        hdm->safety_cfg = *safety;
    } else {
        hdm->safety_cfg = (DcMotor_SafetyConfig_t){
            .stall_timeout_ms   = DC_MOTOR_DEFAULT_STALL_TIMEOUT_MS,
            .stall_min_rpm      = DC_MOTOR_DEFAULT_STALL_MIN_RPM,
            .min_duty_for_stall = DC_MOTOR_DEFAULT_MIN_DUTY_FOR_STALL,
        };
    }

    hdm->state        = DC_MOTOR_IDLE;
    hdm->last_status  = DC_MOTOR_OK;
    hdm->first_update = true;

    if (port->start_pwm) port->start_pwm(hw);
    s_apply_duty(hdm, 0.0f);

    return DC_MOTOR_OK;
}

DcMotor_Status_t DcMotor_SetDuty(DcMotor_Handle_t *hdm, float duty)
{
    if (!hdm) return DC_MOTOR_ERR_NULL_PTR;
    if (DcMotor_IsFault(hdm)) return DC_MOTOR_ERR_FAULT;

    hdm->closed_loop  = false;
    hdm->rpm_setpoint = 0.0f;
    hdm->target_duty  = s_clamp(duty, 0.0f, 1.0f);

    if (hdm->target_duty > 0.0f && hdm->state == DC_MOTOR_IDLE)
        hdm->state = DC_MOTOR_RAMPING;

    return (hdm->last_status = DC_MOTOR_OK);
}

DcMotor_Status_t DcMotor_SetSpeed(DcMotor_Handle_t *hdm, float speed_pct)
{
    return DcMotor_SetDuty(hdm, s_clamp(speed_pct, 0.0f, 100.0f) / 100.0f);
}

DcMotor_Status_t DcMotor_Stop(DcMotor_Handle_t *hdm)
{
    if (!hdm) return DC_MOTOR_ERR_NULL_PTR;
    if (DcMotor_IsFault(hdm)) return DC_MOTOR_ERR_FAULT;

    hdm->closed_loop  = false;
    hdm->rpm_setpoint = 0.0f;
    hdm->target_duty  = 0.0f;

    return (hdm->last_status = DC_MOTOR_OK);
}

void DcMotor_EmergencyStop(DcMotor_Handle_t *hdm)
{
    if (!hdm) return;
    hdm->target_duty = 0.0f;
    hdm->closed_loop = false;
    s_apply_duty(hdm, 0.0f);
    DcMotor_ResetPid(hdm);
    hdm->stall_active = false;
    if (!DcMotor_IsFault(hdm)) hdm->state = DC_MOTOR_IDLE;
}

void DcMotor_Update(DcMotor_Handle_t *hdm, uint32_t tick_ms, float current_rpm)
{
    if (!hdm || DcMotor_IsFault(hdm)) return;

    float dt_s = hdm->pid_cfg.dt_s;

    if (hdm->closed_loop) {
        float duty = s_clamp(s_pid(hdm, dt_s, current_rpm),
                             hdm->pid_cfg.output_min,
                             hdm->pid_cfg.output_max);
        s_apply_duty(hdm, duty);
        hdm->state = (hdm->current_duty > 0.0f) ? DC_MOTOR_RUNNING : DC_MOTOR_IDLE;
    } else {
        s_apply_duty(hdm, s_ramp(hdm, dt_s));

        if (fabsf(hdm->current_duty - hdm->target_duty) > 1e-5f)
            hdm->state = DC_MOTOR_RAMPING;
        else if (hdm->current_duty > 0.0f)
            hdm->state = DC_MOTOR_RUNNING;
        else
            hdm->state = DC_MOTOR_IDLE;
    }

    s_stall_watchdog(hdm, tick_ms, current_rpm);

    hdm->last_tick_ms = tick_ms;
    hdm->first_update = false;
}

DcMotor_Status_t DcMotor_SetRpmSetpoint(DcMotor_Handle_t *hdm, float rpm_sp)
{
    if (!hdm) return DC_MOTOR_ERR_NULL_PTR;
    if (DcMotor_IsFault(hdm)) return DC_MOTOR_ERR_FAULT;

    if (rpm_sp <= 0.0f) return DcMotor_Stop(hdm);

    hdm->rpm_setpoint = rpm_sp;
    hdm->closed_loop  = true;
    if (hdm->state == DC_MOTOR_IDLE) hdm->state = DC_MOTOR_RUNNING;

    return (hdm->last_status = DC_MOTOR_OK);
}

void DcMotor_SetClosedLoop(DcMotor_Handle_t *hdm, bool enabled)
{
    if (!hdm) return;
    hdm->closed_loop = enabled;
    if (!enabled) { DcMotor_ResetPid(hdm); hdm->rpm_setpoint = 0.0f; }
}

void DcMotor_SetPidConfig(DcMotor_Handle_t *hdm, const DcMotor_PidConfig_t *cfg)
{
    if (!hdm || !cfg) return;
    if (cfg->dt_s <= 0.0f || cfg->output_min > cfg->output_max) return;
    if (cfg->integral_min > cfg->integral_max) return;
    hdm->pid_cfg = *cfg;
    hdm->pid_cfg.deriv_filter_alpha = s_clamp(hdm->pid_cfg.deriv_filter_alpha, 0.0f, 1.0f);
}

void DcMotor_SetRampConfig(DcMotor_Handle_t *hdm, const DcMotor_RampConfig_t *cfg)
{
    if (!hdm || !cfg || cfg->accel_rate <= 0.0f || cfg->decel_rate <= 0.0f) return;
    hdm->ramp_cfg = *cfg;
    hdm->ramp_cfg.smooth_alpha = s_clamp(hdm->ramp_cfg.smooth_alpha, 0.0f, 1.0f);
}

void DcMotor_SetSafetyConfig(DcMotor_Handle_t *hdm, const DcMotor_SafetyConfig_t *cfg)
{
    if (!hdm || !cfg) return;
    hdm->safety_cfg = *cfg;
}

DcMotor_Status_t DcMotor_ClearFault(DcMotor_Handle_t *hdm)
{
    if (!hdm) return DC_MOTOR_ERR_NULL_PTR;
    if (!DcMotor_IsFault(hdm)) return DC_MOTOR_OK;

    hdm->state        = DC_MOTOR_IDLE;
    hdm->last_status  = DC_MOTOR_OK;
    hdm->stall_active = false;
    hdm->target_duty  = 0.0f;
    hdm->closed_loop  = false;
    DcMotor_ResetPid(hdm);

    if (hdm->port->start_pwm) hdm->port->start_pwm(hdm->hw);
    s_apply_duty(hdm, 0.0f);

    return DC_MOTOR_OK;
}

void DcMotor_ResetPid(DcMotor_Handle_t *hdm)
{
    if (!hdm) return;
    hdm->pid_integral       = 0.0f;
    hdm->pid_prev_error     = 0.0f;
    hdm->pid_filtered_deriv = 0.0f;
}
