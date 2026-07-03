#ifndef DC_MOTOR_PID_H
#define DC_MOTOR_PID_H

#include "dc_motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Runtime state of the PID controller.
 *         Kept separate from DcMotor_PidConfig_t so the config can
 *         be swapped at runtime without losing the current state, or
 *         the state can be reset without touching the tuning params.
 */
typedef struct {
    float integral;
    float prev_error;
    float filtered_deriv;
} DcMotor_PidState_t;

/**
 * @brief  Reset integrator and derivative history.
 */
void  DcMotor_Pid_Reset(DcMotor_PidState_t *st);

/**
 * @brief  Run one PID iteration.
 *
 * @param cfg        Tuning parameters (kp/ki/kd, limits, dt_s).
 * @param st         Mutable PID state (updated in-place).
 * @param dt_s       Actual elapsed time since last call (seconds).
 * @param setpoint   Desired RPM.
 * @param measured   Measured RPM.
 * @return           Raw PID output (duty, before clamping to output limits).
 */
float DcMotor_Pid_Compute(const DcMotor_PidConfig_t *cfg,
                          DcMotor_PidState_t        *st,
                          float                      dt_s,
                          float                      setpoint,
                          float                      measured);

/**
 * @brief  Populate a DcMotor_PidConfig_t with library defaults.
 */
void  DcMotor_Pid_DefaultConfig(DcMotor_PidConfig_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* DC_MOTOR_PID_H */
