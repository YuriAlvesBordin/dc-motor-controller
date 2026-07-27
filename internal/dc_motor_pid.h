/**
 * @file    dc_motor_pid.h
 * @brief   HAL-agnostic PID controller used by the DC motor closed-loop mode.
 *
 * @details The controller implements:
 *            - Proportional, integral and derivative action.
 *            - Output clamping to a configurable [min, max] range.
 *            - Optional integrator anti-windup (clamping style).
 *            - Optional derivative-term low-pass filtering.
 *
 *          Every optional behaviour is selected at compile time through the
 *          macros defined in dc_motor_config.h. Disabling a macro removes
 *          the corresponding struct fields and code paths entirely.
 */

#ifndef DC_MOTOR_PID_H
#define DC_MOTOR_PID_H

#include "dc_motor_config.h"
#include "dc_motor_types.h"

#if DC_MOTOR_ENABLE_CLOSEDLOOP

/**
 * @brief PID controller state.
 *
 * @details All fields are private. The host must not access them directly;
 *          use the API functions declared below.
 */
typedef struct
{
    float kp;        /**< Proportional gain.                                   */
    float ki;        /**< Integral gain (per second).                          */
    float kd;        /**< Derivative gain (per second).                        */
    float out_min;   /**< Lower saturation limit of the output.                */
    float out_max;   /**< Upper saturation limit of the output.                */
    float integral;  /**< Accumulated integral term.                           */
    float prev_error;/**< Error sample from the previous update.               */
    float prev_setpoint;   /* tracks setpoint for step detection */
#if DC_MOTOR_PID_DERIV_ON_MEASUREMENT
    float prev_meas; /**< Measured value from the previous update.             */
#endif
#if DC_MOTOR_ENABLE_PID_D_FILTER
    float d_filter;  /**< Derivative low-pass coefficient (0..1).              */
    float deriv_filt;/**< Filtered derivative state.                           */
#endif
#if DC_MOTOR_ENABLE_PID_FEEDFORWARD
    float kff;         /**< Velocity feed-forward gain (output per unit setpoint). */
    float kff_static;  /**< Static feed-forward offset, applied when setpoint != 0
                             (helps overcome stiction/stall friction on startup). */
#endif
} dc_motor_pid_t;
/**
 * @brief Initialise a PID controller with explicit gains and limits.
 *
 * @param pid     Pointer to the controller state. Must not be NULL.
 * @param kp      Proportional gain.
 * @param ki      Integral gain (per second).
 * @param kd      Derivative gain (per second).
 * @param out_min Minimum output value (clamping lower bound).
 * @param out_max Maximum output value (clamping upper bound).
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if pid is NULL,
 *         or DC_MOTOR_ERR_RANGE if out_min >= out_max.
 */
dc_motor_status_t dc_motor_pid_init(dc_motor_pid_t *pid,
                                    float kp, float ki, float kd,
                                    float out_min, float out_max);

/**
 * @brief Initialise a PID controller using the default gains and limits
 *        defined in dc_motor_config.h.
 *
 * @param pid Pointer to the controller state. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if pid is NULL.
 */
dc_motor_status_t dc_motor_pid_init_default(dc_motor_pid_t *pid);

/**
 * @brief Reset the controller state (integral and derivative memories).
 *
 * @details Useful when switching setpoint stepwise or when re-enabling the
 *          controller after being idle. Gains and limits are preserved.
 *
 * @param pid Pointer to the controller state. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if pid is NULL.
 */
dc_motor_status_t dc_motor_pid_reset(dc_motor_pid_t *pid);

/**
 * @brief Compute one PID output sample.
 *
 * @param pid       Pointer to the controller state. Must not be NULL.
 * @param setpoint  Desired process value.
 * @param measured  Current process value (from the host's sensor).
 * @param dt        Time elapsed since the last update, in seconds. Must be > 0.
 * @return The clamped controller output in the range [out_min, out_max].
 *         If pid is NULL or dt <= 0 the function returns 0.0f.
 */
float dc_motor_pid_update(dc_motor_pid_t *pid,
                          float setpoint, float measured, float dt);

/**
 * @brief Update gains at runtime without resetting the state.
 *
 * @param pid Pointer to the controller state. Must not be NULL.
 * @param kp  New proportional gain.
 * @param ki  New integral gain.
 * @param kd  New derivative gain.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if pid is NULL.
 */
dc_motor_status_t dc_motor_pid_tune(dc_motor_pid_t *pid,
                                    float kp, float ki, float kd);

#if DC_MOTOR_ENABLE_PID_FEEDFORWARD
/**
 * @brief Update feed-forward gains at runtime.
 *
 * @param pid        Pointer to the controller state. Must not be NULL.
 * @param kff        Velocity feed-forward gain.
 * @param kff_static Static feed-forward offset (applied when setpoint != 0).
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if pid is NULL.
 */
dc_motor_status_t dc_motor_pid_set_feedforward(dc_motor_pid_t *pid,
                                               float kff, float kff_static);
#endif

#else

typedef struct dc_motor_pid_t_empty { uint8_t _unused; } dc_motor_pid_t;

#endif /* DC_MOTOR_ENABLE_CLOSEDLOOP */

#endif /* DC_MOTOR_PID_H */
