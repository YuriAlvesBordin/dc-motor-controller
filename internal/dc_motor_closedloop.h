/**
 * @file    dc_motor_closedloop.h
 * @brief   Closed-loop PID mode for the DC motor library.
 *
 * @details In closed-loop mode the host commands a target process value
 *          (e.g. angular speed in RPM, position in counts) and supplies the
 *          corresponding measured value through a feedback sensor. The
 *          library runs the PID controller at the configured control
 *          period and produces an output command in percent.
 *
 *          When DC_MOTOR_ENABLE_RAMP is defined the target setpoint is
 *          first passed through a slew-rate limiter so the PID never sees
 *          an instantaneous step. This yields smoother transients and
 *          avoids integrator saturation on large setpoint changes.
 *
 *          This module is enabled only when DC_MOTOR_ENABLE_CLOSEDLOOP is
 *          defined in dc_motor_config.h.
 */

#ifndef DC_MOTOR_CLOSEDLOOP_H
#define DC_MOTOR_CLOSEDLOOP_H

#include "dc_motor_config.h"
#include "dc_motor_types.h"
#include "dc_motor_pid.h"
#include "dc_motor_ramp.h"

#if DC_MOTOR_ENABLE_CLOSEDLOOP

/**
 * @brief Closed-loop controller state.
 */
typedef struct
{
    float setpoint_raw;   /**< Last commanded raw setpoint.                  */
    float setpoint_eff;   /**< Setpoint after the ramp (effective).          */
    float measured;       /**< Last measured process value.                  */
    float output;         /**< Most recent PID output, in percent.           */
    dc_motor_pid_t pid;   /**< Embedded PID controller.                      */
#if DC_MOTOR_ENABLE_RAMP
    dc_motor_ramp_t ramp; /**< Setpoint slew-rate generator.                 */
#endif
} dc_motor_closedloop_t;

/**
 * @brief Initialise a closed-loop controller with default PID gains and
 *        default ramp limits.
 *
 * @param cl Pointer to the closed-loop state. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if cl is NULL.
 */
dc_motor_status_t dc_motor_closedloop_init(dc_motor_closedloop_t *cl);

/**
 * @brief Command a new setpoint.
 *
 * @details The setpoint is stored as-is. When a ramp is enabled the
 *          effective setpoint fed to the PID slews towards this value on
 *          subsequent calls to dc_motor_closedloop_update().
 *
 * @param cl       Pointer to the closed-loop state. Must not be NULL.
 * @param setpoint Desired process value.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if cl is NULL.
 */
dc_motor_status_t dc_motor_closedloop_set_setpoint(dc_motor_closedloop_t *cl,
                                                   float setpoint);

/**
 * @brief Push the latest measured process value into the controller.
 *
 * @param cl       Pointer to the closed-loop state. Must not be NULL.
 * @param measured Latest reading from the host's sensor.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if cl is NULL.
 */
dc_motor_status_t dc_motor_closedloop_set_measured(dc_motor_closedloop_t *cl,
                                                   float measured);

/**
 * @brief Compute one closed-loop output sample.
 *
 * @details The ramp is advanced first, then the PID is evaluated against
 *          the (possibly slewed) effective setpoint and the most recently
 *          pushed measured value. The dt argument must match
 *          DC_MOTOR_CONTROL_PERIOD_SEC.
 *
 * @param cl Pointer to the closed-loop state. Must not be NULL.
 * @param dt Time elapsed since the last update, in seconds. Must be > 0.
 * @return The updated output level in percent. Returns 0.0f if cl is NULL
 *         or dt <= 0.
 */
float dc_motor_closedloop_update(dc_motor_closedloop_t *cl, float dt);

/**
 * @brief Reset the closed-loop state (PID + ramp).
 *
 * @details Gains and limits are preserved. The effective setpoint is
 *          forced to the current raw setpoint so no transient is triggered.
 *
 * @param cl Pointer to the closed-loop state. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if cl is NULL.
 */
dc_motor_status_t dc_motor_closedloop_reset(dc_motor_closedloop_t *cl);

/**
 * @brief Read-only accessor for the PID sub-state.
 *
 * @details Allows the host to retune the controller at runtime using
 *          dc_motor_pid_tune() without exposing the full struct.
 *
 * @param cl Pointer to the closed-loop state.
 * @return Pointer to the embedded PID, or NULL if cl is NULL.
 */
dc_motor_pid_t *dc_motor_closedloop_get_pid(dc_motor_closedloop_t *cl);

/**
 * @brief Return the effective setpoint (after the ramp).
 *
 * @param cl Pointer to the closed-loop state.
 * @return The effective setpoint, or 0.0f if cl is NULL.
 */
float dc_motor_closedloop_get_setpoint(const dc_motor_closedloop_t *cl);

/**
 * @brief Return the most recent PID output.
 *
 * @param cl Pointer to the closed-loop state.
 * @return The output level in percent, or 0.0f if cl is NULL.
 */
float dc_motor_closedloop_get_output(const dc_motor_closedloop_t *cl);

#else

typedef struct dc_motor_closedloop_t_empty { uint8_t _unused; } dc_motor_closedloop_t;

#endif /* DC_MOTOR_ENABLE_CLOSEDLOOP */

#endif /* DC_MOTOR_CLOSEDLOOP_H */
