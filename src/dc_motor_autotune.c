#include "dc_motor_autotune.h"
#include "internal/dc_motor_types.h"
#include "internal/dc_motor_pid.h"
#include <string.h>
#include <math.h>

#ifndef DC_AUTOTUNE_DEFAULT_RELAY_AMP
#define DC_AUTOTUNE_DEFAULT_RELAY_AMP       50.0f
#endif
#ifndef DC_AUTOTUNE_DEFAULT_RELAY_DUTY_STEP
#define DC_AUTOTUNE_DEFAULT_RELAY_DUTY_STEP 0.15f
#endif
#ifndef DC_AUTOTUNE_DEFAULT_TIMEOUT_MS
#define DC_AUTOTUNE_DEFAULT_TIMEOUT_MS      30000U
#endif
#ifndef DC_AUTOTUNE_MIN_CYCLES
#define DC_AUTOTUNE_MIN_CYCLES              4U
#endif
#ifndef DC_AUTOTUNE_WARMUP_MS
#define DC_AUTOTUNE_WARMUP_MS               2000U
#endif

void DcMotor_Autotune_DefaultConfig(DcMotor_AutotuneCfg_t *cfg)
{
    if (!cfg) return;
    cfg->rpm_target       = 0.0f;
    cfg->relay_amp        = DC_AUTOTUNE_DEFAULT_RELAY_AMP;
    cfg->relay_duty_step  = DC_AUTOTUNE_DEFAULT_RELAY_DUTY_STEP;
    cfg->relay_timeout_ms = DC_AUTOTUNE_DEFAULT_TIMEOUT_MS;
    cfg->dt_s             = DC_MOTOR_DEFAULT_DT_S;
}

DcMotor_Status_t DcMotor_Autotune_Start(DcMotor_AutotuneCtx_t       *ctx,
                                         DcMotor_Handle_t            *motor,
                                         const DcMotor_AutotuneCfg_t *cfg,
                                         DcMotor_AutotuneDoneCb_t     done_cb,
                                         void                        *user)
{
    if (!ctx || !motor || !cfg || cfg->rpm_target <= 0.0f) return DC_MOTOR_ERR_PARAM;
    if (!done_cb) return DC_MOTOR_ERR_PARAM;

    memset(ctx, 0, sizeof(*ctx));
    ctx->motor   = motor;
    ctx->cfg     = *cfg;
    ctx->done_cb = done_cb;
    ctx->user    = user;
    ctx->state   = DC_AUTOTUNE_WARMUP;
    return DC_MOTOR_OK;
}

static DcMotor_AutotuneState_t s_finish_err(DcMotor_AutotuneCtx_t  *ctx,
                                             DcMotor_AutotuneState_t err)
{
    DcMotor_EmergencyStop(ctx->motor);
    ctx->state = err;
    return err;
}

static DcMotor_AutotuneState_t s_calc(DcMotor_AutotuneCtx_t *ctx)
{
    if (ctx->cycle_count < DC_AUTOTUNE_MIN_CYCLES)
        return s_finish_err(ctx, DC_AUTOTUNE_ERR_NO_OSCILLATION);

    float a  = ctx->amplitude_acc / (float)ctx->cycle_count;
    float d  = ctx->cfg.relay_duty_step;
    float ku = (4.0f * d) / ((float)M_PI * a);
    float tu_s = ctx->cfg.dt_s * 20.0f;

    ctx->result.ku   = ku;
    ctx->result.tu_s = tu_s;

    DcMotor_PidConfig_t *p = &ctx->tuned_pid;
    DcMotor_Pid_DefaultConfig(p);
    p->kp   = ku / 3.2f;
    p->ki   = p->kp / (2.87f * tu_s);
    p->kd   = p->kp * tu_s / 11.4f;
    p->dt_s = ctx->cfg.dt_s;

    ctx->state = DC_AUTOTUNE_DONE;
    ctx->done_cb(p, ctx->user);
    return DC_AUTOTUNE_DONE;
}

DcMotor_AutotuneState_t DcMotor_Autotune_Update(DcMotor_AutotuneCtx_t *ctx,
                                                 uint32_t               tick_ms,
                                                 float                  current_rpm)
{
    if (!ctx) return DC_AUTOTUNE_IDLE;

    switch (ctx->state) {
    case DC_AUTOTUNE_IDLE:
    case DC_AUTOTUNE_DONE:
        return ctx->state;

    case DC_AUTOTUNE_WARMUP:
        if (ctx->warmup_start_ms == 0U) ctx->warmup_start_ms = tick_ms;
        DcMotor_SetDuty(ctx->motor, 0.5f);
        if ((tick_ms - ctx->warmup_start_ms) >= DC_AUTOTUNE_WARMUP_MS) {
            ctx->state          = DC_AUTOTUNE_RELAY;
            ctx->relay_start_ms = tick_ms;
            ctx->peak_rpm       = current_rpm;
            ctx->valley_rpm     = current_rpm;
            ctx->relay_output   = ctx->cfg.relay_duty_step;
        }
        return DC_AUTOTUNE_WARMUP;

    case DC_AUTOTUNE_RELAY: {
        if (DcMotor_IsFault(ctx->motor))
            return s_finish_err(ctx, DC_AUTOTUNE_ERR_STALL);
        if ((tick_ms - ctx->relay_start_ms) >= ctx->cfg.relay_timeout_ms)
            return s_finish_err(ctx, DC_AUTOTUNE_ERR_TIMEOUT);

        float sp  = ctx->cfg.rpm_target;
        float amp = ctx->cfg.relay_amp;

        if (current_rpm > ctx->peak_rpm)   ctx->peak_rpm   = current_rpm;
        if (current_rpm < ctx->valley_rpm) ctx->valley_rpm = current_rpm;

        if (ctx->relay_output > 0.0f && current_rpm > sp + amp) {
            ctx->relay_output = -ctx->cfg.relay_duty_step;
            float half_amp = (ctx->peak_rpm - ctx->valley_rpm) / 2.0f;
            if (half_amp > 0.0f) ctx->amplitude_acc += half_amp;
            ctx->cycle_count++;
            ctx->peak_rpm   = current_rpm;
            ctx->valley_rpm = current_rpm;
        } else if (ctx->relay_output < 0.0f && current_rpm < sp - amp) {
            ctx->relay_output = ctx->cfg.relay_duty_step;
        }

        DcMotor_SetDuty(ctx->motor, 0.5f + ctx->relay_output);

        if (ctx->cycle_count >= 8U)
            return s_calc(ctx);

        return DC_AUTOTUNE_RELAY;
    }

    case DC_AUTOTUNE_CALC:
        return s_calc(ctx);

    default:
        return ctx->state;
    }
}

void DcMotor_Autotune_Abort(DcMotor_AutotuneCtx_t *ctx)
{
    if (!ctx) return;
    DcMotor_EmergencyStop(ctx->motor);
    ctx->state = DC_AUTOTUNE_IDLE;
}

DcMotor_Status_t DcMotor_Autotune_GetResult(const DcMotor_AutotuneCtx_t *ctx,
                                             DcMotor_AutotuneResult_t    *out)
{
    if (!ctx || !out) return DC_MOTOR_ERR_NULL_PTR;
    if (ctx->state != DC_AUTOTUNE_DONE) return DC_MOTOR_ERR_NO_DATA;
    *out = ctx->result;
    return DC_MOTOR_OK;
}
