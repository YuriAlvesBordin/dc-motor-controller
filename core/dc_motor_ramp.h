#ifndef DC_MOTOR_RAMP_H
#define DC_MOTOR_RAMP_H

#include "dc_motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Compute the next duty-cycle value following the S-Curve ramp.
 *
 * @param cfg           Ramp parameters (accel/decel rates, smooth_alpha).
 * @param current_duty  Present PWM duty (0..1).
 * @param target_duty   Commanded duty  (0..1).
 * @param dt_s          Elapsed time in seconds since last call.
 * @return              New duty value, clamped to [0, 1].
 */
float DcMotor_Ramp_Step(const DcMotor_RampConfig_t *cfg,
                        float                       current_duty,
                        float                       target_duty,
                        float                       dt_s);

/**
 * @brief  Populate a DcMotor_RampConfig_t with library defaults.
 */
void  DcMotor_Ramp_DefaultConfig(DcMotor_RampConfig_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* DC_MOTOR_RAMP_H */
