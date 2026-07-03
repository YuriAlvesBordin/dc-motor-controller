#ifndef DC_MOTOR_RAMP_H
#define DC_MOTOR_RAMP_H

/**
 * @file  dc_motor_ramp.h
 * @brief Internal S-Curve ramp subsystem — do NOT include directly.
 *        Use <dc_motor.h> from your application code.
 */

#include "dc_motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

float DcMotor_Ramp_Step         (const DcMotor_RampConfig_t *cfg,
                                  float current_duty,
                                  float target_duty,
                                  float dt_s);
void  DcMotor_Ramp_DefaultConfig(DcMotor_RampConfig_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* DC_MOTOR_RAMP_H */
