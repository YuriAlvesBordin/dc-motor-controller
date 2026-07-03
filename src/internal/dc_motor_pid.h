#ifndef DC_MOTOR_PID_H
#define DC_MOTOR_PID_H

/**
 * @file  dc_motor_pid.h
 * @brief Internal PID subsystem — do NOT include directly.
 *        Use <dc_motor.h> from your application code.
 */

#include "dc_motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float integral;
    float prev_error;
    float filtered_deriv;
} DcMotor_PidState_t;

void  DcMotor_Pid_Reset        (DcMotor_PidState_t *st);
float DcMotor_Pid_Compute      (const DcMotor_PidConfig_t *cfg,
                                 DcMotor_PidState_t        *st,
                                 float dt_s,
                                 float setpoint,
                                 float measured);
void  DcMotor_Pid_DefaultConfig(DcMotor_PidConfig_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* DC_MOTOR_PID_H */
