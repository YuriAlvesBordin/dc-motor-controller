#ifndef DC_MOTOR_SAFETY_H
#define DC_MOTOR_SAFETY_H

#include "dc_motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Runtime state of the stall watchdog.
 */
typedef struct {
    uint32_t stall_start_ms;
    bool     stall_active;
} DcMotor_SafetyState_t;

/**
 * @brief  Callback invoked by the safety module when a stall fault is
 *         detected.  The orchestrator (dc_motor.c) provides this so the
 *         safety module stays decoupled from DcMotor_Handle_t.
 *
 * @param ctx   Opaque pointer forwarded from DcMotor_Safety_Update().
 */
typedef void (*DcMotor_FaultCb_t)(void *ctx);

/**
 * @brief  Run one stall-watchdog iteration.
 *
 * @param cfg         Safety configuration.
 * @param st          Mutable watchdog state (updated in-place).
 * @param tick_ms     Current system tick (milliseconds).
 * @param current_duty Present PWM duty used to decide whether to check.
 * @param current_rpm  Measured RPM.
 * @param fault_cb    Called exactly once when a stall is latched.
 * @param fault_ctx   Forwarded to fault_cb as-is.
 */
void DcMotor_Safety_Update(const DcMotor_SafetyConfig_t *cfg,
                           DcMotor_SafetyState_t        *st,
                           uint32_t                      tick_ms,
                           float                         current_duty,
                           float                         current_rpm,
                           DcMotor_FaultCb_t             fault_cb,
                           void                         *fault_ctx);

/**
 * @brief  Reset watchdog state (call after clearing a fault).
 */
void DcMotor_Safety_Reset(DcMotor_SafetyState_t *st);

/**
 * @brief  Populate a DcMotor_SafetyConfig_t with library defaults.
 */
void DcMotor_Safety_DefaultConfig(DcMotor_SafetyConfig_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* DC_MOTOR_SAFETY_H */
