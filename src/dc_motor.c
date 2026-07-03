#include "../include/dc_motor.h"
#include "internal/dc_motor_pid.h"
#include "internal/dc_motor_ramp.h"
#include "internal/dc_motor_safety.h"
#include <string.h>

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

static void s_on_stall_fault(void *ctx)
{
    DcMotor_Handle_t *hdm = (DcMotor_Handle_t *)ctx;
    hdm->state       = DC_MOTOR_FAULT_STALL;
    hdm->last_status = DC_MOTOR_ERR_STALL;
    hdm->target_duty = 0.0f;
    hdm->closed_loop = false;
    s_apply_duty(hdm, 0.0f);
    DcMotor_ResetPid(hdm);
}

DcMotor_Status_t DcMotor_Init(DcMotor_Handle_t             *hdm,
                              const DcMotor_Port_t         *port,
                              void                         *hw,
                              const DcMotor_PidConfig_t    *pid,
                              const DcMotor_RampConfig_t   *ramp,
                              const DcMotor_SafetyConfig_t *safety)
{
    if (!hdm || !port)       return DC_MOTOR_ERR_NULL_PTR;
    if (!port->set_pwm_duty) return DC_MOTOR_ERR_PARAM;

    memset(hdm, 0, sizeof(*hdm));
    hdm->port = port;
    hdm->hw   = hw;

    if (pid) {
        if (pid->dt_s <= 0.0f)                     return DC_MOTOR_ERR_PARAM;
        if (pid->output_min > pid->output_max)     return DC_MOTOR_ERR_PARAM;
        if (pid->integral_min > pid->integral_max) return DC_MOTOR_ERR_PARAM;
        hdm->pid_cfg = *pid;
        hdm->pid_cfg.deriv_filter_alpha =
            s_clamp(hdm->pid_cfg.deriv_filter_alpha, 0.0f, 1.0f);
    } else {
        DcMotor_Pid_DefaultConfig(&hdm->pid_cfg);
    }

    if (ramp) {
        if (ramp->accel_rate <= 0.0f || ramp->decel_rate <= 0.0f)
            return DC_MOTOR_ERR_PARAM;
        hdm->ramp_cfg = *ramp;
        hdm->ramp_cfg.smooth_alpha =
            s_clamp(hdm->ramp_cfg.smooth_alpha, 0.0f, 1.0f);
    } else {
        DcMotor_Ramp_DefaultConfig(&hdm->ramp_cfg);
    }

    if (safety) {
        hdm->safety_cfg = *safety;
    } else {
        DcMotor_Safety_DefaultConfig(&hdm->safety_cfg);
    }

    DcMotor_Pid_Reset(&hdm->pid_st);
    DcMotor_Safety_Reset(&hdm->safety_st);

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
    DcMotor_Safety_Reset(&hdm->safety_st);
    if (!DcMotor_IsFault(hdm)) hdm->state = DC_MOTOR_IDLE;
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

void DcMotor_Update(DcMotor_Handle_t *hdm, uint32_t tick_ms, float current_rpm)
{
    if (!hdm || DcMotor_IsFault(hdm)) return;

    float dt_s = hdm->pid_cfg.dt_s;

    if (hdm->closed_loop) {
        float duty = DcMotor_Pid_Compute(&hdm->pid_cfg, &hdm->pid_st,
                                         dt_s, hdm->rpm_setpoint, current_rpm);
        duty = s_clamp(duty, hdm->pid_cfg.output_min, hdm->pid_cfg.output_max);
        s_apply_duty(hdm, duty);
        hdm->state = (hdm->current_duty > 0.0f) ? DC_MOTOR_RUNNING : DC_MOTOR_IDLE;
    } else {
        float next = DcMotor_Ramp_Step(&hdm->ramp_cfg,
                                       hdm->current_duty,
                                       hdm->target_duty,
                                       dt_s);
        s_apply_duty(hdm, next);

        if      (hdm->current_duty != hdm->target_duty) hdm->state = DC_MOTOR_RAMPING;
        else if (hdm->current_duty > 0.0f)              hdm->state = DC_MOTOR_RUNNING;
        else                                             hdm->state = DC_MOTOR_IDLE;
    }

    DcMotor_Safety_Update(&hdm->safety_cfg, &hdm->safety_st,
                          tick_ms, hdm->current_duty, current_rpm,
                          s_on_stall_fault, hdm);

    hdm->last_tick_ms = tick_ms;
    hdm->first_update = false;
}

void DcMotor_SetPidConfig(DcMotor_Handle_t *hdm, const DcMotor_PidConfig_t *cfg)
{
    if (!hdm || !cfg) return;
    if (cfg->dt_s <= 0.0f || cfg->output_min > cfg->output_max) return;
    if (cfg->integral_min > cfg->integral_max) return;
    hdm->pid_cfg = *cfg;
    hdm->pid_cfg.deriv_filter_alpha =
        s_clamp(hdm->pid_cfg.deriv_filter_alpha, 0.0f, 1.0f);
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
    hdm->target_duty  = 0.0f;
    hdm->closed_loop  = false;
    DcMotor_ResetPid(hdm);
    DcMotor_Safety_Reset(&hdm->safety_st);

    if (hdm->port->start_pwm) hdm->port->start_pwm(hdm->hw);
    s_apply_duty(hdm, 0.0f);

    return DC_MOTOR_OK;
}

void DcMotor_ResetPid(DcMotor_Handle_t *hdm)
{
    if (!hdm) return;
    DcMotor_Pid_Reset(&hdm->pid_st);
}
