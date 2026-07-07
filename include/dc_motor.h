/**
 * @file    dc_motor.h
 * @brief   Top-level API of the DC motor control library.
 *
 * @details This header is the only file the host application needs to
 *          include. It exposes a single opaque-by-convention handle type
 *          (dc_motor_t) and a small set of functions to:
 *            - initialise a motor instance,
 *            - select an operating mode (open-loop, closed-loop, idle),
 *            - push setpoints / measured values,
 *            - run one control iteration,
 *            - read back the produced command.
 *
 *          All optional features are selected at compile time via the
 *          macros in dc_motor_config.h. When a feature is disabled, the
 *          corresponding field, function, and code path disappear from
 *          the build entirely - the compiler emits no dead code and no
 *          dead data.
 *
 *          The library uses no global variables and no dynamic memory
 *          allocation. All state lives in host-declared dc_motor_t
 *          structures, so the library is reentrant and safe to use from
 *          multiple control tasks (one instance per task).
 */

#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include "dc_motor_config.h"
#include "dc_motor_types.h"
#include "dc_motor_pid.h"
#include "dc_motor_ramp.h"
#include "dc_motor_openloop.h"
#include "dc_motor_closedloop.h"

/**
 * @brief Top-level motor instance.
 *
 * @details The host declares one such structure per physical motor. The
 *          layout depends on which DC_MOTOR_ENABLE_* macros are defined:
 *          disabled features consume zero bytes because their sub-struct
 *          is replaced by a 1-byte placeholder (only used to keep C
 *          struct-tag compatibility).
 */
typedef struct
{
    dc_motor_mode_t mode;   /**< Current operating mode.                     */
    float output;           /**< Last computed command, in percent.          */
    float measured;         /**< Last reported measured process value.       */
    uint32_t elapsed_ms;    /**< Milliseconds since the last update tick.    */

#if DC_MOTOR_ENABLE_OPENLOOP
    dc_motor_openloop_t openloop;   /**< Open-loop sub-controller.          */
#endif

#if DC_MOTOR_ENABLE_CLOSEDLOOP
    dc_motor_closedloop_t closedloop; /**< Closed-loop sub-controller.      */
#endif

#if DC_MOTOR_ENABLE_WATCHDOG
    uint32_t watchdog_last_feed_ms;  /**< Timestamp of the last feed.       */
    uint8_t  watchdog_tripped;       /**< Non-zero once the watchdog fires. */
#endif
} dc_motor_t;

/**
 * @brief Initialise a motor instance.
 *
 * @details Sets the mode to IDLE, zeroes the output, and initialises
 *          every enabled sub-controller with the defaults from
 *          dc_motor_config.h.
 *
 * @param m Pointer to the motor instance. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if m is NULL.
 */
dc_motor_status_t dc_motor_init(dc_motor_t *m);

/**
 * @brief Switch the motor to IDLE mode and zero the output.
 *
 * @details Also resets the PID and ramp sub-controllers so the next
 *          command starts from a clean state.
 *
 * @param m Pointer to the motor instance. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if m is NULL.
 */
dc_motor_status_t dc_motor_stop(dc_motor_t *m);

/**
 * @brief Command the motor in open-loop mode.
 *
 * @details Switches the mode to OPENLOOP (if not already) sets the
 *          commanded level (e.g. 50.0 for half-throttle forward), and lets
 *          the ramp generator bring the output towards it. Available only
 *          when DC_MOTOR_ENABLE_OPENLOOP is defined.
 *
 * @param m      Pointer to the motor instance. Must not be NULL.
 * @param level  Desired output level in percent (signed for direction).
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if m is NULL,
 *         DC_MOTOR_ERR_DISABLED if open-loop is disabled at compile time.
 */
dc_motor_status_t dc_motor_set_openloop(dc_motor_t *m, float level);

/**
 * @brief Command the motor in closed-loop mode.
 *
 * @details Switches the mode to CLOSEDLOOP and sets the target process
 *          value the PID will track. The measured value must be pushed
 *          separately via dc_motor_set_measured(). Available only when
 *          DC_MOTOR_ENABLE_CLOSEDLOOP is defined.
 *
 * @param m         Pointer to the motor instance. Must not be NULL.
 * @param setpoint  Desired process value (units defined by the host).
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if m is NULL,
 *         DC_MOTOR_ERR_DISABLED if closed-loop is disabled at compile time.
 */
dc_motor_status_t dc_motor_set_closedloop(dc_motor_t *m, float setpoint);

/**
 * @brief Push the latest measured process value.
 *
 * @details Only used in closed-loop mode. In open-loop mode the call is
 *          accepted but has no effect on the output.
 *
 * @param m        Pointer to the motor instance. Must not be NULL.
 * @param measured Latest reading from the host's sensor.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if m is NULL.
 */
dc_motor_status_t dc_motor_set_measured(dc_motor_t *m, float measured);

/**
 * @brief Run one control iteration.
 *
 * @details Must be called by the host at the period defined by
 *          DC_MOTOR_CONTROL_PERIOD_SEC. The function:
 *            - checks the watchdog (if enabled),
 *            - dispatches to the open-loop or closed-loop sub-controller
 *              depending on the current mode,
 *            - copies the produced output into m->output.
 *
 * @param m  Pointer to the motor instance. Must not be NULL.
 * @param dt Control period in seconds. Must be > 0.
 * @return The updated output command in percent, or 0.0f on error.
 */
float dc_motor_update(dc_motor_t *m, float dt);

/**
 * @brief Refresh the watchdog without changing the operating mode.
 *
 * @details Only meaningful when DC_MOTOR_ENABLE_WATCHDOG is defined.
 *
 * @param m Pointer to the motor instance. Must not be NULL.
 * @return DC_MOTOR_OK on success, DC_MOTOR_ERR_NULL if m is NULL,
 *         DC_MOTOR_ERR_DISABLED if the watchdog is disabled.
 */
dc_motor_status_t dc_motor_feed_watchdog(dc_motor_t *m);

/**
 * @brief Notify the library of the elapsed real time so the watchdog can
 *        fire even when dc_motor_update() is not called periodically.
 *
 * @details The host typically calls this from a 1 ms tick with
 *          elapsed_ms = 1. When the elapsed time since the last feed
 *          exceeds DC_MOTOR_WATCHDOG_TIMEOUT_MS the output is forced to
 *          zero and the watchdog_tripped flag is set. The flag is cleared
 *          by calling dc_motor_stop() followed by dc_motor_init().
 *
 * @param m          Pointer to the motor instance. Must not be NULL.
 * @param elapsed_ms Milliseconds elapsed since the last tick call.
 * @return DC_MOTOR_OK on success,
 *         DC_MOTOR_ERR_NULL if m is NULL,
 *         DC_MOTOR_ERR_WATCHDOG if the watchdog has tripped (output forced
 *         to zero),
 *         DC_MOTOR_ERR_DISABLED if the watchdog is disabled.
 */
dc_motor_status_t dc_motor_tick(dc_motor_t *m, uint32_t elapsed_ms);

/**
 * @brief Return the current operating mode.
 *
 * @param m Pointer to the motor instance.
 * @return The mode, or DC_MOTOR_MODE_IDLE if m is NULL.
 */
dc_motor_mode_t dc_motor_get_mode(const dc_motor_t *m);

/**
 * @brief Return the most recent output command.
 *
 * @param m Pointer to the motor instance.
 * @return The output in percent, or 0.0f if m is NULL.
 */
float dc_motor_get_output(const dc_motor_t *m);

/**
 * @brief Return the current measured process value.
 *
 * @param m Pointer to the motor instance.
 * @return The measured value, or 0.0f if m is NULL.
 */
float dc_motor_get_measured(const dc_motor_t *m);

#endif /* DC_MOTOR_H */
