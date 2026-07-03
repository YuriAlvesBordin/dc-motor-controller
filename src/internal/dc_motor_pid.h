#ifndef DC_MOTOR_PID_H
#define DC_MOTOR_PID_H

#include "dc_motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

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

#endif
