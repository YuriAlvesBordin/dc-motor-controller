#include "dc_motor_safety.h"
#include <math.h>

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void DcMotor_Safety_Reset(DcMotor_SafetyState_t *st)
{
    if (!st) return;
    st->stall_active    = false;
    st->stall_start_ms  = 0U;
}

void DcMotor_Safety_DefaultConfig(DcMotor_SafetyConfig_t *cfg)
{
    if (!cfg) return;
    *cfg = (DcMotor_SafetyConfig_t){
        .stall_timeout_ms   = DC_MOTOR_DEFAULT_STALL_TIMEOUT_MS,
        .stall_min_rpm      = DC_MOTOR_DEFAULT_STALL_MIN_RPM,
        .min_duty_for_stall = DC_MOTOR_DEFAULT_MIN_DUTY_FOR_STALL,
    };
}

void DcMotor_Safety_Update(const DcMotor_SafetyConfig_t *cfg,
                           DcMotor_SafetyState_t        *st,
                           uint32_t                      tick_ms,
                           float                         current_duty,
                           float                         current_rpm,
                           DcMotor_FaultCb_t             fault_cb,
                           void                         *fault_ctx)
{
    if (!cfg || !st) return;
    if (cfg->stall_timeout_ms == 0U) return;

    /* Don't check stall below the minimum duty threshold */
    if (current_duty < cfg->min_duty_for_stall) {
        st->stall_active = false;
        return;
    }

    if (fabsf(current_rpm) < cfg->stall_min_rpm) {
        if (!st->stall_active) {
            st->stall_start_ms = tick_ms;
            st->stall_active   = true;
        } else if ((tick_ms - st->stall_start_ms) >= cfg->stall_timeout_ms) {
            /* Stall confirmed — notify orchestrator via callback */
            st->stall_active = false;
            if (fault_cb) fault_cb(fault_ctx);
        }
    } else {
        st->stall_active = false;
    }
}
