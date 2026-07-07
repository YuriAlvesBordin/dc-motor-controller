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
 *               line (this works because the #ifndef only sets the default
 *               when the macro is NOT already defined).
 *
 *          Every feature guarded by a DC_MOTOR_ENABLE_* macro is completely
 *          removed by the preprocessor when the macro evaluates to 0: the
 *          corresponding .c files compile to empty translation units and
 *          the corresponding struct fields / API entry points are hidden.
 *          This guarantees zero dead code and zero dead RAM when a feature
 *          is not used.
 */

#ifndef DC_MOTOR_CONFIG_H
#define DC_MOTOR_CONFIG_H

/* ========================================================================== */
/* =====================  FEATURE ENABLE / DISABLE  ========================= */
/* ========================================================================== */

/**
 * @brief Enable the open-loop mode (acceleration-ramped direct command).
 *
 * @details When set to 1, dc_motor_set_openloop() is available and the
 *          dc_motor_t structure reserves storage for the open-loop ramp
 *          generator. When set to 0, every reference to the open-loop API
 *          is removed by the preprocessor.
 */
#ifndef DC_MOTOR_ENABLE_OPENLOOP
#define DC_MOTOR_ENABLE_OPENLOOP          0
#endif

/**
 * @brief Enable the closed-loop PID mode.
 *
 * @details When set to 1, dc_motor_set_closedloop() and the PID runtime
 *          (dc_motor_pid_t, dc_motor_pid_update, ...) are available.
 *          Setting it to 0 removes the entire PID module from the build.
 */
#ifndef DC_MOTOR_ENABLE_CLOSEDLOOP
#define DC_MOTOR_ENABLE_CLOSEDLOOP        1
#endif

/**
 * @brief Enable the acceleration / deceleration ramp generator.
 *
 * @details The ramp is shared by both the open-loop and the closed-loop
 *          modes. If set to 0, setpoints are applied instantaneously with
 *          no slew-rate limiting. Set it to 1 to obtain a smooth transition
 *          between setpoints and to protect the mechanical load.
 */
#ifndef DC_MOTOR_ENABLE_RAMP
#define DC_MOTOR_ENABLE_RAMP              0
#endif

/**
 * @brief Enable anti-windup clamping on the PID integrator.
 *
 * @details Only meaningful when DC_MOTOR_ENABLE_CLOSEDLOOP is 1. Setting
 *          it to 0 shrinks the PID structure by one branch and removes
 *          the conditional clamping logic.
 */
#ifndef DC_MOTOR_ENABLE_PID_ANTI_WINDUP
#define DC_MOTOR_ENABLE_PID_ANTI_WINDUP   1
#endif

/**
 * @brief Enable derivative-term low-pass filtering.
 *
 * @details Reduces measurement-noise amplification of the derivative term
 *          at the cost of one extra floating-point state per PID instance.
 *          Only meaningful when DC_MOTOR_ENABLE_CLOSEDLOOP is 1.
 */
#ifndef DC_MOTOR_ENABLE_PID_D_FILTER
#define DC_MOTOR_ENABLE_PID_D_FILTER      1
#endif

/**
 * @brief Compute the derivative term on the measured value instead of on
 *        the error.
 *
 * @details When set to 1, d/dt(error) is replaced by -d/dt(measured). This
 *          prevents the derivative term from reacting to setpoint changes
 *          (a phenomenon known as "derivative kick") and is the standard
 *          choice for motor control. Set to 0 to fall back to the classic
 *          derivative-on-error formulation.
 */
#ifndef DC_MOTOR_PID_DERIV_ON_MEASUREMENT
#define DC_MOTOR_PID_DERIV_ON_MEASUREMENT 1
#endif

/**
 * @brief Enable the watchdog: if dc_motor_update() is not called for more
 *        than DC_MOTOR_WATCHDOG_TIMEOUT_MS the motor is forcibly stopped.
 *
 * @details The host must call dc_motor_feed_watchdog() or dc_motor_update()
 *          within the timeout window. Setting it to 0 removes the watchdog
 *          fields and the timeout check.
 */
#ifndef DC_MOTOR_ENABLE_WATCHDOG
#define DC_MOTOR_ENABLE_WATCHDOG          1
#endif

/* ========================================================================== */
/* =========================  TIMING PARAMETERS  ============================ */
/* ========================================================================== */

/**
 * @brief Control-loop period in seconds.
 *
 * @details Must match the real period at which dc_motor_update() is called
 *          by the host application. Used as the integration / differentiation
 *          step by the PID and as the update tick by the ramp generator.
 */
#ifndef DC_MOTOR_CONTROL_PERIOD_SEC
#define DC_MOTOR_CONTROL_PERIOD_SEC        (0.001f)
#endif

/**
 * @brief Maximum number of motors the host will instantiate.
 *
 * @details Only used by the optional static-instance helper API. Set to 0
 *          to completely remove the helper table and force the host to
 *          declare dc_motor_t objects locally.
 */
#ifndef DC_MOTOR_MAX_INSTANCES
#define DC_MOTOR_MAX_INSTANCES             (4u)
#endif

/* ========================================================================== */
/* ===========================  PID DEFAULTS  =============================== */
/* ========================================================================== */

/**
 * @brief Default proportional gain (Kp) used by dc_motor_pid_init_default().
 */
#ifndef DC_MOTOR_PID_DEFAULT_KP
#define DC_MOTOR_PID_DEFAULT_KP            (1.0f)
#endif

/**
 * @brief Default integral gain (Ki) used by dc_motor_pid_init_default().
 */
#ifndef DC_MOTOR_PID_DEFAULT_KI
#define DC_MOTOR_PID_DEFAULT_KI            (0.5f)
#endif

/**
 * @brief Default derivative gain (Kd) used by dc_motor_pid_init_default().
 */
#ifndef DC_MOTOR_PID_DEFAULT_KD
#define DC_MOTOR_PID_DEFAULT_KD            (0.0f)
#endif

/**
 * @brief Default derivative low-pass filter coefficient (0..1).
 *
 * @details Lower values yield stronger smoothing. Only used when
 *          DC_MOTOR_ENABLE_PID_D_FILTER is 1.
 */
#ifndef DC_MOTOR_PID_DEFAULT_D_FILTER
#define DC_MOTOR_PID_DEFAULT_D_FILTER      (0.1f)
#endif

/**
 * @brief Default PID output lower bound.
 *
 * @details The PID output (and therefore the duty command) is clamped to
 *          the range [DC_MOTOR_PID_DEFAULT_OUT_MIN,
 *          DC_MOTOR_PID_DEFAULT_OUT_MAX]. Use signed values so the motor
 *          can reverse.
 */
#ifndef DC_MOTOR_PID_DEFAULT_OUT_MIN
#define DC_MOTOR_PID_DEFAULT_OUT_MIN       (-100.0f)
#endif

/**
 * @brief Default PID output upper bound.
 */
#ifndef DC_MOTOR_PID_DEFAULT_OUT_MAX
#define DC_MOTOR_PID_DEFAULT_OUT_MAX       ( 100.0f)
#endif

/* ========================================================================== */
/* ======================  RAMP / OPEN-LOOP DEFAULTS  ======================= */
/* ========================================================================== */

/**
 * @brief Default acceleration limit in percent-per-second.
 *
 * @details Applied when increasing |output| (i.e. speeding up in either
 *          direction). A value of 100.0 means the motor can go from 0 to
 *          full output in 1 second.
 */
#ifndef DC_MOTOR_RAMP_DEFAULT_ACCEL
#define DC_MOTOR_RAMP_DEFAULT_ACCEL        (200.0f)
#endif

/**
 * @brief Default deceleration limit in percent-per-second.
 *
 * @details Applied when decreasing |output| (i.e. braking). Typically set
 *          larger than the acceleration so the motor brakes harder than it
 *          accelerates.
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
 * @brief Dead-band around zero below which the open-loop output is forced
 *        to exactly zero.
 *
 * @details Set to 0 to disable the dead-band. Useful to suppress PWM
 *          chatter when the ramp crosses zero.
 */
#ifndef DC_MOTOR_OPENLOOP_DEADBAND
#define DC_MOTOR_OPENLOOP_DEADBAND         (0.1f)
#endif

/* ========================================================================== */
/* ============================  SAFETY LIMITS  ============================= */
/* ========================================================================== */

/**
 * @brief Watchdog timeout in milliseconds.
 *
 * @details Only meaningful when DC_MOTOR_ENABLE_WATCHDOG is 1.
 */
#ifndef DC_MOTOR_WATCHDOG_TIMEOUT_MS
#define DC_MOTOR_WATCHDOG_TIMEOUT_MS       (50u)
#endif

#endif /* DC_MOTOR_CONFIG_H */
