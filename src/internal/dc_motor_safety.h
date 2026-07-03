#ifndef DC_MOTOR_SAFETY_H
#define DC_MOTOR_SAFETY_H

/**
 * @file  dc_motor_safety.h
 * @brief Internal stall-watchdog subsystem — do NOT include directly.
 */

#include "dc_motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void DcMotor_Safety_Update        (const DcMotor_SafetyConfig_t *cfg,
                                    DcMotor_SafetyState_t        *st,
                                    uint32_t tick_ms,
                                    float    current_duty,
                                    float    current_rpm,
                                    DcMotor_FaultCb_t fault_cb,
                                    void    *fault_ctx);
void DcMotor_Safety_Reset         (DcMotor_SafetyState_t *st);
void DcMotor_Safety_DefaultConfig (DcMotor_SafetyConfig_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* DC_MOTOR_SAFETY_H */
