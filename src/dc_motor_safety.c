#include "internal/dc_motor_safety.h"
#include "internal/dc_motor_types.h"

void DcMotor_Safety_Reset(DcMotor_SafetyState_t *st)
{
    if (!st) return;
    st->stall_start_ms = 0U;
    st->stall_active   = false;
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
    if (cfg->stall_timeout_ms == 0U) return;                    /* watchdog disabled */
    if (current_duty < cfg->min_duty_for_stall) {               /* motor not driven   */
        DcMotor_Safety_Reset(st);
        return;
    }

    if (current_rpm < cfg->stall_min_rpm) {
        if (!st->stall_active) {
            st->stall_active   = true;
            st->stall_start_ms = tick_ms;
        } else if ((tick_ms - st->stall_start_ms) >= cfg->stall_timeout_ms) {
            DcMotor_Safety_Reset(st);
            if (fault_cb) fault_cb(fault_ctx);
        }
    } else {
        DcMotor_Safety_Reset(st);
    }
}

void DcMotor_Safety_DefaultConfig(DcMotor_SafetyConfig_t *cfg)
{
    if (!cfg) return;
    cfg->stall_timeout_ms    = DC_MOTOR_DEFAULT_STALL_TIMEOUT_MS;
    cfg->stall_min_rpm       = DC_MOTOR_DEFAULT_STALL_MIN_RPM;
    cfg->min_duty_for_stall  = DC_MOTOR_DEFAULT_MIN_DUTY_FOR_STALL;
}
