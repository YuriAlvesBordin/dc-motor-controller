#ifndef DC_MOTOR_PORT_H
#define DC_MOTOR_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void (*set_pwm_duty)(void *hw, float duty_norm);
    void (*start_pwm)   (void *hw);
    void (*stop_pwm)    (void *hw);
} DcMotor_Port_t;

#ifdef __cplusplus
}
#endif

#endif /* DC_MOTOR_PORT_H */
