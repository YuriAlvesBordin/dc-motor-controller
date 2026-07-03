#include "dc_motor_autotune.h"
#include <string.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/*  Private constants                                                  */
/* ------------------------------------------------------------------ */

#define AUTOTUNE_DEFAULT_RELAY_TIMEOUT_MS   30000U
#define AUTOTUNE_DEFAULT_RELAY_AMP          50.0f   /* RPM  */
#define AUTOTUNE_DEFAULT_RELAY_DUTY_STEP    0.15f   /* duty */

/* ------------------------------------------------------------------ */
/*  Internal helpers                                                   */
/* ------------------------------------------------------------------ */

static float at_clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/** Apply relay output directly to the motor (bypass PID and ramp). */
static void s_relay_apply(DcMotor_AutotuneCtx_t *ctx, bool high)
{
    ctx->relay_high = high;
    float base  = at_clamp(ctx->hdm->current_duty, 0.0f, 1.0f);
    float delta = ctx->cfg.relay_duty_step;
    float duty  = high ? at_clamp(base + delta, 0.0f, 1.0f)
                       : at_clamp(base - delta, 0.0f, 1.0f);
    /* Write directly to the HAL port — autotune owns the actuator */
    ctx->hdm->port->set_pwm_duty(ctx->hdm->hw, duty);
    ctx->hdm->current_duty = duty;
}

/** Abort helper — stops the motor and marks error state. */
static DcMotor_AutotuneState_t s_abort(DcMotor_AutotuneCtx_t *ctx,
                                       DcMotor_AutotuneState_t err)
{
    ctx->state = err;
    DcMotor_EmergencyStop(ctx->hdm);
    return err;
}

/* ------------------------------------------------------------------ */
/*  Public API — Start / Abort / DefaultConfig / GetResult            */
/* ------------------------------------------------------------------ */

void DcMotor_Autotune_DefaultConfig(DcMotor_AutotuneCfg_t *cfg)
{
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->relay_amp          = AUTOTUNE_DEFAULT_RELAY_AMP;
    cfg->relay_duty_step    = AUTOTUNE_DEFAULT_RELAY_DUTY_STEP;
    cfg->relay_timeout_ms   = AUTOTUNE_DEFAULT_RELAY_TIMEOUT_MS;
    cfg->dt_s               = DC_MOTOR_DEFAULT_DT_S;
    cfg->output_min         = 0.0f;
    cfg->output_max         = 1.0f;
    cfg->integral_min       = -0.5f;
    cfg->integral_max       =  0.5f;
    cfg->deriv_filter_alpha = DC_MOTOR_DERIV_FILTER_ALPHA;
    /* rpm_target must be set by the caller */
}

DcMotor_Status_t DcMotor_Autotune_Start(DcMotor_AutotuneCtx_t       *ctx,
                                         DcMotor_Handle_t            *hdm,
                                         const DcMotor_AutotuneCfg_t *cfg,
                                         DcMotor_AutotuneDoneCb_t     done_cb,
                                         void                        *user)
{
    if (!ctx || !hdm || !cfg)           return DC_MOTOR_ERR_NULL_PTR;
    if (cfg->rpm_target <= 0.0f)        return DC_MOTOR_ERR_PARAM;
    if (cfg->relay_amp  <= 0.0f)        return DC_MOTOR_ERR_PARAM;
    if (cfg->relay_duty_step <= 0.0f)   return DC_MOTOR_ERR_PARAM;
    if (cfg->dt_s <= 0.0f)              return DC_MOTOR_ERR_PARAM;
    if (DcMotor_IsFault(hdm))           return DC_MOTOR_ERR_FAULT;

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg      = *cfg;
    ctx->hdm      = hdm;
    ctx->done_cb  = done_cb;
    ctx->user_ctx = user;

    if (ctx->cfg.relay_timeout_ms == 0U)
        ctx->cfg.relay_timeout_ms = AUTOTUNE_DEFAULT_RELAY_TIMEOUT_MS;

    /* Kick off warmup — use open-loop ramp to reach 50 % of target RPM */
    float warmup_duty = at_clamp(cfg->rpm_target / 1000.0f, 0.1f, 0.5f);
    DcMotor_SetDuty(hdm, warmup_duty);

    ctx->state          = DC_AUTOTUNE_WARMUP;
    ctx->tracking_peak  = true;
    ctx->peak_rpm       = 0.0f;
    ctx->valley_rpm     = cfg->rpm_target * 2.0f; /* large sentinel */

    return DC_MOTOR_OK;
}

void DcMotor_Autotune_Abort(DcMotor_AutotuneCtx_t *ctx)
{
    if (!ctx) return;
    s_abort(ctx, DC_AUTOTUNE_IDLE);
    ctx->state = DC_AUTOTUNE_IDLE;
}

DcMotor_Status_t DcMotor_Autotune_GetResult(const DcMotor_AutotuneCtx_t *ctx,
                                             DcMotor_PidConfig_t         *out_cfg)
{
    if (!ctx || !out_cfg)               return DC_MOTOR_ERR_NULL_PTR;
    if (ctx->state != DC_AUTOTUNE_DONE) return DC_MOTOR_ERR_NO_DATA;
    *out_cfg = ctx->result;
    return DC_MOTOR_OK;
}

/* ------------------------------------------------------------------ */
/*  State machine                                                      */
/* ------------------------------------------------------------------ */

static DcMotor_AutotuneState_t s_run_warmup(DcMotor_AutotuneCtx_t *ctx,
                                             uint32_t               tick_ms,
                                             float                  rpm)
{
    /* Drive motor with ramp until RPM reaches the warmup fraction */
    DcMotor_Update(ctx->hdm, tick_ms, rpm);

    float target_warmup_rpm = ctx->cfg.rpm_target * DC_MOTOR_AUTOTUNE_WARMUP_FRAC;

    if (rpm >= target_warmup_rpm) {
        /* Motor is warm — switch to relay phase */
        ctx->state          = DC_AUTOTUNE_RELAY;
        ctx->phase_start_ms = tick_ms;
        ctx->relay_high     = true;
        ctx->last_rpm       = rpm;
        ctx->peak_rpm       = rpm;
        ctx->valley_rpm     = rpm;
        ctx->tracking_peak  = true;
        s_relay_apply(ctx, true);
        return DC_AUTOTUNE_RELAY;
    }

    uint32_t elapsed = tick_ms - ctx->phase_start_ms;
    if (elapsed >= DC_MOTOR_AUTOTUNE_WARMUP_TIMEOUT_MS)
        return s_abort(ctx, DC_AUTOTUNE_ERR_TIMEOUT);

    return DC_AUTOTUNE_WARMUP;
}

static DcMotor_AutotuneState_t s_run_relay(DcMotor_AutotuneCtx_t *ctx,
                                            uint32_t               tick_ms,
                                            float                  rpm)
{
    const float hyst  = DC_MOTOR_AUTOTUNE_HYSTERESIS;
    const float amp   = ctx->cfg.relay_amp;
    const float tgt   = ctx->cfg.rpm_target;

    /* Safety: abort if stall was latched by the handle's own watchdog */
    if (DcMotor_IsFault(ctx->hdm))
        return s_abort(ctx, DC_AUTOTUNE_ERR_STALL);

    /* Timeout guard */
    uint32_t elapsed = tick_ms - ctx->phase_start_ms;
    if (elapsed >= ctx->cfg.relay_timeout_ms)
        return s_abort(ctx, DC_AUTOTUNE_ERR_TIMEOUT);

    /* --- Relay switching logic ------------------------------------ */
    if (ctx->relay_high && rpm >= (tgt + amp - hyst)) {
        s_relay_apply(ctx, false);
    } else if (!ctx->relay_high && rpm <= (tgt - amp + hyst)) {
        s_relay_apply(ctx, true);
    }

    /* --- Peak / valley detection ---------------------------------- */
    if (ctx->tracking_peak) {
        if (rpm > ctx->peak_rpm) {
            ctx->peak_rpm      = rpm;
            ctx->peak_tick_ms  = tick_ms;
        } else if (rpm < ctx->peak_rpm - hyst) {
            /* Peak confirmed — start tracking valley */
            float half_amp = (ctx->peak_rpm - ctx->valley_rpm) * 0.5f;
            if (ctx->cycle_count > 0U) {
                /* Accumulate full cycle using previous valley→peak→valley */
                ctx->sum_amplitude += half_amp;
                float period_s = (float)(tick_ms - ctx->last_peak_tick_ms) * 1e-3f;
                ctx->sum_period_s  += period_s;
                ctx->cycle_count++;
            } else {
                ctx->cycle_count = 1U;
            }
            ctx->last_peak_tick_ms = tick_ms;
            ctx->valley_rpm        = rpm;     /* reset for next valley */
            ctx->tracking_peak     = false;
        }
    } else {
        /* Tracking valley */
        if (rpm < ctx->valley_rpm) {
            ctx->valley_rpm      = rpm;
            ctx->valley_tick_ms  = tick_ms;
        } else if (rpm > ctx->valley_rpm + hyst) {
            /* Valley confirmed — restart tracking peak */
            ctx->peak_rpm      = rpm;
            ctx->tracking_peak = true;
        }
    }

    /* Check if we have enough cycles */
    if (ctx->cycle_count >= DC_MOTOR_AUTOTUNE_MIN_CYCLES)
        return (ctx->state = DC_AUTOTUNE_CALC);

    if (ctx->cycle_count >= DC_MOTOR_AUTOTUNE_MAX_CYCLES)
        return s_abort(ctx, DC_AUTOTUNE_ERR_NO_OSCILLATION);

    return DC_AUTOTUNE_RELAY;
}

static DcMotor_AutotuneState_t s_run_calc(DcMotor_AutotuneCtx_t *ctx)
{
    uint32_t n = ctx->cycle_count > 1U ? ctx->cycle_count - 1U : 1U;

    float a  = ctx->sum_amplitude / (float)n;   /* average half-amplitude [RPM] */
    float Tu = ctx->sum_period_s  / (float)n;   /* average period [s]           */

    if (a <= 0.0f || Tu <= 0.0f)
        return s_abort(ctx, DC_AUTOTUNE_ERR_NO_OSCILLATION);

    /* Ultimate gain: Ku = 4d / (pi * a) */
    float d  = ctx->cfg.relay_duty_step;
    float Ku = (4.0f * d) / ((float)M_PI * a);

    /* Tyreus-Luyben rules (low overshoot) */
    float Kp = Ku / 3.2f;
    float Ki = Kp / (2.87f * Tu);
    float Kd = Kp * Tu / 11.4f;

    /* Scale back by PID_GAIN_SCALE (the gain scale is baked into Compute) */
    float sc = DC_MOTOR_PID_GAIN_SCALE;
    Kp /= sc;
    Ki /= sc;
    Kd /= sc;

    ctx->result = (DcMotor_PidConfig_t){
        .kp                 = Kp,
        .ki                 = Ki,
        .kd                 = Kd,
        .dt_s               = ctx->cfg.dt_s,
        .output_min         = ctx->cfg.output_min,
        .output_max         = ctx->cfg.output_max,
        .integral_min       = ctx->cfg.integral_min,
        .integral_max       = ctx->cfg.integral_max,
        .deriv_filter_alpha = ctx->cfg.deriv_filter_alpha,
    };

    ctx->state = DC_AUTOTUNE_DONE;

    /* Return motor to normal idle state */
    DcMotor_EmergencyStop(ctx->hdm);
    DcMotor_ClearFault(ctx->hdm);

    if (ctx->done_cb)
        ctx->done_cb(&ctx->result, ctx->user_ctx);

    return DC_AUTOTUNE_DONE;
}

/* ------------------------------------------------------------------ */
/*  Main update (call every tick instead of DcMotor_Update)           */
/* ------------------------------------------------------------------ */

DcMotor_AutotuneState_t DcMotor_Autotune_Update(DcMotor_AutotuneCtx_t *ctx,
                                                 uint32_t               tick_ms,
                                                 float                  current_rpm)
{
    if (!ctx || !ctx->hdm) return DC_AUTOTUNE_ERR_PARAM;

    switch (ctx->state) {
        case DC_AUTOTUNE_WARMUP: return s_run_warmup(ctx, tick_ms, current_rpm);
        case DC_AUTOTUNE_RELAY:  return s_run_relay (ctx, tick_ms, current_rpm);
        case DC_AUTOTUNE_CALC:   return s_run_calc  (ctx);
        case DC_AUTOTUNE_DONE:   return DC_AUTOTUNE_DONE;
        default:                 return ctx->state;
    }
}
