#include "dc_motor_config.h"

#if DC_MOTOR_ENABLE_AUTOTUNE

#include "dc_motor_autotune.h"
#include "internal/dc_motor_types.h"
#include "internal/dc_motor_pid.h"
#include <string.h>
#include <math.h>

void DcMotor_Autotune_DefaultConfig(DcMotor_AutotuneCfg_t *cfg)
{
    if (!cfg) return;
    cfg->rpm_target       = 0.0f;
    cfg->rpm_max          = DC_MOTOR_DEFAULT_MAX_RPM;
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
    ctx->motor          = motor;
    ctx->cfg            = *cfg;
    ctx->done_cb        = done_cb;
    ctx->user           = user;
    ctx->state          = DC_AUTOTUNE_WARMUP;
    ctx->operating_duty = 0.5f; /* Will be refined during warmup */
    return DC_MOTOR_OK;
}

static DcMotor_AutotuneState_t s_finish_err(DcMotor_AutotuneCtx_t  *ctx,
                                             DcMotor_AutotuneState_t err)
{
    DcMotor_EmergencyStop(ctx->motor);
    ctx->state = err;
    return err;
}

static DcMotor_AutotuneState_t s_calc(DcMotor_AutotuneCtx_t *ctx, uint32_t tick_ms)
{
    if (ctx->cycle_count < DC_AUTOTUNE_MIN_CYCLES)
        return s_finish_err(ctx, DC_AUTOTUNE_ERR_NO_OSCILLATION);

    float a  = ctx->amplitude_acc / (float)ctx->cycle_count;
    float d  = ctx->cfg.relay_duty_step;
    float ku = (4.0f * d) / ((float)M_PI * a);

    /* FIX: tu_s must use the relay phase duration, not warmup duration.
     * relay_start_ms marks the beginning of the relay phase; tick_ms is
     * the current time at the end of it. Divide by cycle_count and multiply
     * by 2 because each relay cycle is a half-period (peak→valley or
     * valley→peak), so a full oscillation period is twice that. */
    float relay_elapsed_s = (float)(tick_ms - ctx->relay_start_ms) / 1000.0f;
    float tu_s = (relay_elapsed_s / (float)ctx->cycle_count) * 2.0f;

    ctx->result.ku   = ku;
    ctx->result.tu_s = tu_s;

    DcMotor_PidConfig_t *p = &ctx->tuned_pid;
    DcMotor_Pid_DefaultConfig(p);

    /* Ziegler-Nichols classic rules: faster response than Tyreus-Luyben.
     * Kp = 0.60 * Ku
     * Ki = Kp  / (0.50 * Tu)   =>  2 * Kp / Tu
     * Kd = Kp  * (0.125 * Tu)  =>  Kp * Tu / 8 */
    p->kp   = 0.60f * ku;
    p->ki   = 2.0f  * p->kp / tu_s;
    p->kd   = p->kp * tu_s / 8.0f;
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

    case DC_AUTOTUNE_WARMUP: {
        if (ctx->warmup_start_ms == 0U) ctx->warmup_start_ms = tick_ms;

        /* FIX: use a feedforward duty proportional to the target RPM instead
         * of a fixed 0.5f. This starts the relay phase much closer to the
         * real operating point, reducing settling time and relay asymmetry. */
        float max_rpm = (ctx->cfg.rpm_max > 0.0f) ? ctx->cfg.rpm_max : 3000.0f;
        float ff_duty = ctx->cfg.rpm_target / max_rpm;
        if (ff_duty < 0.1f)  ff_duty = 0.1f;
        if (ff_duty > 0.9f)  ff_duty = 0.9f;

        DcMotor_ApplyDuty(ctx->motor, ff_duty);

        /* Track the duty used once the motor is near target so the relay
         * phase is centred on the real operating point. */
        if (current_rpm > 0.0f &&
            fabsf(current_rpm - ctx->cfg.rpm_target) < ctx->cfg.relay_amp)
        {
            ctx->operating_duty = ff_duty;
        }

        if ((tick_ms - ctx->warmup_start_ms) >= DC_AUTOTUNE_WARMUP_MS) {
            ctx->state          = DC_AUTOTUNE_RELAY;
            ctx->relay_start_ms = tick_ms;
            ctx->peak_rpm       = current_rpm;
            ctx->valley_rpm     = current_rpm;
            ctx->relay_output   = ctx->cfg.relay_duty_step;
        }
        return DC_AUTOTUNE_WARMUP;
    }

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
            /* Positive→negative flank: motor exceeded upper threshold */
            ctx->relay_output = -ctx->cfg.relay_duty_step;
            float half_amp = (ctx->peak_rpm - ctx->valley_rpm) / 2.0f;
            if (half_amp > 0.0f) ctx->amplitude_acc += half_amp;
            ctx->cycle_count++;
            ctx->peak_rpm   = current_rpm;
            ctx->valley_rpm = current_rpm;
        } else if (ctx->relay_output < 0.0f && current_rpm < sp - amp) {
            /* FIX: also accumulate amplitude on the negative→positive flank.
             * The original code only counted cycles going down, discarding
             * half the data and increasing estimation variance. */
            ctx->relay_output = ctx->cfg.relay_duty_step;
            float half_amp = (ctx->peak_rpm - ctx->valley_rpm) / 2.0f;
            if (half_amp > 0.0f) ctx->amplitude_acc += half_amp;
            ctx->cycle_count++;
            ctx->peak_rpm   = current_rpm;
            ctx->valley_rpm = current_rpm;
        }

        /* FIX: centre the relay output on the real operating duty, not 0.5f.
         * Using a fixed 0.5f causes asymmetric excitation when the motor
         * operates far from that duty cycle, biasing the Ku estimate. */
        DcMotor_ApplyDuty(ctx->motor, ctx->operating_duty + ctx->relay_output);

        if (ctx->cycle_count >= 8U)
            return s_calc(ctx, tick_ms);

        return DC_AUTOTUNE_RELAY;
    }

    case DC_AUTOTUNE_CALC:
        return s_calc(ctx, tick_ms);

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

#endif
