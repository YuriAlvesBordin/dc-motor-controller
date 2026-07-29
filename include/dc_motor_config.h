/**
 * @file    dc_motor_config.h
 * @brief   Compile-time configuration for the DC motor control library.
 *
 * @details This is the SINGLE source of truth for every tunable parameter
 *          and every optional feature in the library.
 *
 *          Feature macros follow the standard "default-on, override from
 *          outside" pattern:
 *
 *            #ifndef DC_MOTOR_ENABLE_<FEATURE>
 *            #define DC_MOTOR_ENABLE_<FEATURE> 1
 *            #endif
 *
 *          To enable a feature:  leave the default, OR pass
 *               -DDC_MOTOR_ENABLE_<FEATURE>=1
 *          To disable a feature: edit this file and set the value to 0, OR
 *               pass -DDC_MOTOR_ENABLE_<FEATURE>=0 on the compiler command
 *               line.
 *
 *          Every feature guarded by a DC_MOTOR_ENABLE_* macro is completely
 *          removed by the preprocessor when the macro evaluates to 0.
 */

#ifndef DC_MOTOR_CONFIG_H
#define DC_MOTOR_CONFIG_H

/**
 * @brief Enable the open-loop mode (acceleration-ramped direct command).
 */
#ifndef DC_MOTOR_ENABLE_OPENLOOP
#define DC_MOTOR_ENABLE_OPENLOOP          1
#endif

/**
 * @brief Enable the closed-loop PID mode.
 */
#ifndef DC_MOTOR_ENABLE_CLOSEDLOOP
#define DC_MOTOR_ENABLE_CLOSEDLOOP        1
#endif

/**
 * @brief Enable the acceleration / deceleration ramp generator.
 *
 * @details Shared by both the open-loop and the closed-loop modes.
 *          When set to 0, setpoints are applied instantaneously.
 */
#ifndef DC_MOTOR_ENABLE_RAMP
#define DC_MOTOR_ENABLE_RAMP              0
#endif

/**
 * @brief Enable anti-windup clamping on the PID integrator.
 *
 * @details Only meaningful when DC_MOTOR_ENABLE_CLOSEDLOOP is 1.
 *          The implementation uses saturation-freeze: the integral is only
 *          accumulated when the tentative output is within [out_min, out_max].
 */
#ifndef DC_MOTOR_ENABLE_PID_ANTI_WINDUP
#define DC_MOTOR_ENABLE_PID_ANTI_WINDUP   1
#endif

/**
 * @brief Enable feed-forward control in the PID controller.
 *
 * @details Feed-forward terms are used to improve the controller's response
 *          to setpoint changes by providing a direct path from the setpoint
 *          to the output.
 */
#ifndef DC_MOTOR_ENABLE_PID_FEEDFORWARD
#define DC_MOTOR_ENABLE_PID_FEEDFORWARD 1
#define DC_MOTOR_PID_DEFAULT_KFF          0.01f
#define DC_MOTOR_PID_DEFAULT_KFF_STATIC   0.1f
#endif

/**
 * @brief Enable derivative-term low-pass filtering.
 *
 * @details Reduces measurement-noise amplification of the D term.
 *          Only meaningful when DC_MOTOR_ENABLE_CLOSEDLOOP is 1.
 */
#ifndef DC_MOTOR_ENABLE_PID_D_FILTER
#define DC_MOTOR_ENABLE_PID_D_FILTER      1
#endif

/**
 * @brief Compute the derivative term on the measured value instead of
 *        on the error, preventing derivative kick on setpoint changes.
 *
 * @details Set to 0 to fall back to the classic derivative-on-error
 *          formulation.
 */
#ifndef DC_MOTOR_PID_DERIV_ON_MEASUREMENT
#define DC_MOTOR_PID_DERIV_ON_MEASUREMENT 1
#endif

/**
 * @brief Enable the watchdog: if dc_motor_update() is not called for
 *        more than DC_MOTOR_WATCHDOG_TIMEOUT_MS the motor is forcibly
 *        stopped.
 */
#ifndef DC_MOTOR_ENABLE_WATCHDOG
#define DC_MOTOR_ENABLE_WATCHDOG          0
#endif

/**
 * @brief Control-loop period in seconds.
 *
 * @details Must match the real period at which dc_motor_update() is
 *          called by the host application.
 */
#ifndef DC_MOTOR_CONTROL_PERIOD_SEC
#define DC_MOTOR_CONTROL_PERIOD_SEC        (0.1f)
#endif

/**
 * @brief Default proportional gain (Kp).
 */
#ifndef DC_MOTOR_PID_DEFAULT_KP
#define DC_MOTOR_PID_DEFAULT_KP            (0.5f)
#endif

/**
 * @brief Default integral gain (Ki, per second).
 */
#ifndef DC_MOTOR_PID_DEFAULT_KI
#define DC_MOTOR_PID_DEFAULT_KI            (1.0f)
#endif

/**
 * @brief Default derivative gain (Kd, per second).
 */
#ifndef DC_MOTOR_PID_DEFAULT_KD
#define DC_MOTOR_PID_DEFAULT_KD            (0.1f)
#endif

/**
 * @brief Default derivative low-pass filter coefficient (0..1).
 *
 * @details Lower values yield stronger smoothing. With dt=0.1s:
 *          - 0.01f → time constant ~9.9 s  (D term effectively dead)
 *          - 0.15f → time constant ~0.57 s (D term responsive)
 *          Only used when DC_MOTOR_ENABLE_PID_D_FILTER is 1.
 */
#ifndef DC_MOTOR_PID_DEFAULT_D_FILTER
#define DC_MOTOR_PID_DEFAULT_D_FILTER      (0.15f)
#endif

/**
 * @brief Default PID output lower bound (percent).
 *
 * @details Set to 0.0f so the PID can command zero output. The physical
 *          motor dead-band is handled separately via DC_MOTOR_PID_DEADBAND
 *          in the closed-loop output stage, not here.
 */
#ifndef DC_MOTOR_PID_DEFAULT_OUT_MIN
#define DC_MOTOR_PID_DEFAULT_OUT_MIN       (0.0f)
#endif

/**
 * @brief Default PID output upper bound (percent).
 */
#ifndef DC_MOTOR_PID_DEFAULT_OUT_MAX
#define DC_MOTOR_PID_DEFAULT_OUT_MAX       ( 100.0f)
#endif

/**
 * @brief Physical motor dead-band in percent PWM.
 *
 * @details The motor does not spin below this PWM duty cycle. Applied in
 *          dc_motor_closedloop_update() AFTER the PID computes its output,
 *          so the PID integrator is not affected by the dead-band.
 *          Set to 0.0f to disable.
 */
#ifndef DC_MOTOR_PID_DEADBAND
#define DC_MOTOR_PID_DEADBAND              (25.0f)
#endif

/**
 * @brief Default acceleration limit in percent-per-second.
 *
 * @details Applied when increasing |output| (speeding up).
 */
#ifndef DC_MOTOR_RAMP_DEFAULT_ACCEL
#define DC_MOTOR_RAMP_DEFAULT_ACCEL        (200.0f)
#endif

/**
 * @brief Default deceleration limit in percent-per-second.
 *
 * @details Applied when decreasing |output| (braking).
 */
#ifndef DC_MOTOR_RAMP_DEFAULT_DECEL
#define DC_MOTOR_RAMP_DEFAULT_DECEL        (400.0f)
#endif

/**
 * @brief Open-loop output lower bound (percent).
 */
#ifndef DC_MOTOR_OPENLOOP_OUT_MIN
#define DC_MOTOR_OPENLOOP_OUT_MIN          (-100.0f)
#endif

/**
 * @brief Open-loop output upper bound (percent).
 */
#ifndef DC_MOTOR_OPENLOOP_OUT_MAX
#define DC_MOTOR_OPENLOOP_OUT_MAX          ( 100.0f)
#endif

/**
 * @brief Dead-band around zero below which the open-loop output is
 *        forced to exactly zero.
 *
 * @details Set to 0 to disable. Suppresses PWM chatter near zero.
 */
#ifndef DC_MOTOR_OPENLOOP_DEADBAND
#define DC_MOTOR_OPENLOOP_DEADBAND         (0.1f)
#endif

/**
 * @brief Watchdog timeout in milliseconds.
 *
 * @details Only meaningful when DC_MOTOR_ENABLE_WATCHDOG is 1.
 */
#ifndef DC_MOTOR_WATCHDOG_TIMEOUT_MS
#define DC_MOTOR_WATCHDOG_TIMEOUT_MS       (5000u)
#endif

#endif
