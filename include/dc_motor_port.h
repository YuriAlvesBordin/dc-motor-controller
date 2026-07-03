#ifndef DC_MOTOR_PORT_H
#define DC_MOTOR_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*DcMotor_SetPwmDutyFn_t)(void *hw, float duty_norm);
typedef void (*DcMotor_StartPwmFn_t) (void *hw);
typedef void (*DcMotor_StopPwmFn_t)  (void *hw);

typedef struct {
    DcMotor_SetPwmDutyFn_t set_pwm_duty;
    DcMotor_StartPwmFn_t   start_pwm;
    DcMotor_StopPwmFn_t    stop_pwm;
} DcMotor_Port_t;

#ifdef __cplusplus
}
#endif

#endif
