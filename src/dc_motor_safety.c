#include "dc_motor_config.h"

#if DC_MOTOR_ENABLE_SAFETY

#include "internal/dc_motor_safety.h"

void DcMotor_Safety_DefaultConfig(DcMotor_SafetyConfig_t *cfg)
{
    if (!cfg) return;
    cfg->stall_timeout_ms    = DC_MOTOR_DEFAULT_STALL_TIMEOUT_MS;
    cfg->stall_min_rpm       = DC_MOTOR_DEFAULT_STALL_MIN_RPM;
    cfg->min_duty_for_stall  = DC_MOTOR_DEFAULT_MIN_DUTY_FOR_STALL;
}

void DcMotor_Safety_Reset(DcMotor_SafetyState_t *st)
{
    if (!st) return;
    st->stall_start_ms = 0U;
    st->stall_active   = false;
}

void DcMotor_Safety_Update(const DcMotor_SafetyConfig_t *cfg,
                            DcMotor_SafetyState_t        *st,
                            uint32_t                      tick_ms,
                            float                         duty,
                            float                         rpm,
                            void                        (*on_fault)(void *),
                            void                         *ctx)
{
    if (!cfg || !st || !on_fault) return;
    if (cfg->stall_timeout_ms == 0U) return;
    if (duty < cfg->min_duty_for_stall) { DcMotor_Safety_Reset(st); return; }

    if (rpm < cfg->stall_min_rpm) {
        if (!st->stall_active) {
            st->stall_active   = true;
            st->stall_start_ms = tick_ms;
        } else if ((tick_ms - st->stall_start_ms) >= cfg->stall_timeout_ms) {
            on_fault(ctx);
        }
    } else {
        DcMotor_Safety_Reset(st);
    }
}

#endif
