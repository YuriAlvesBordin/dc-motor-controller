#ifndef DC_MOTOR_PORT_H
#define DC_MOTOR_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  HAL contract — three function pointers that the core calls.
 *         Implement these for your platform; the core never changes.
 *
 *  set_pwm_duty  — write normalised duty [0.0 .. 1.0] to the PWM peripheral.
 *  start_pwm     — enable PWM output  (may be NULL if not needed).
 *  stop_pwm      — disable PWM output (may be NULL if not needed).
 */
typedef void (*DcMotor_SetPwmDutyFn_t)(void *hw, float duty_norm);
typedef void (*DcMotor_StartPwmFn_t) (void *hw);
typedef void (*DcMotor_StopPwmFn_t)  (void *hw);

typedef struct {
    DcMotor_SetPwmDutyFn_t set_pwm_duty; /**< Required. */
    DcMotor_StartPwmFn_t   start_pwm;    /**< Optional — set NULL if unused. */
    DcMotor_StopPwmFn_t    stop_pwm;     /**< Optional — set NULL if unused. */
} DcMotor_Port_t;

#ifdef __cplusplus
}
#endif

#endif /* DC_MOTOR_PORT_H */
