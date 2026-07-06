#ifndef DC_MOTOR_CONFIG_H
#define DC_MOTOR_CONFIG_H

#ifndef DC_MOTOR_ENABLE_PID
#define DC_MOTOR_ENABLE_PID        1
#endif

#ifndef DC_MOTOR_ENABLE_RAMP
#define DC_MOTOR_ENABLE_RAMP       1
#endif

#ifndef DC_MOTOR_ENABLE_SAFETY
#define DC_MOTOR_ENABLE_SAFETY     1
#endif

#ifndef DC_MOTOR_ENABLE_AUTOTUNE
#define DC_MOTOR_ENABLE_AUTOTUNE   1
#endif

#ifndef DC_MOTOR_DEFAULT_KP
#define DC_MOTOR_DEFAULT_KP              1.5f
#endif
#ifndef DC_MOTOR_DEFAULT_KI
#define DC_MOTOR_DEFAULT_KI              0.2f
#endif
#ifndef DC_MOTOR_DEFAULT_KD
#define DC_MOTOR_DEFAULT_KD              0.05f
#endif
#ifndef DC_MOTOR_DEFAULT_DT_S
#define DC_MOTOR_DEFAULT_DT_S            0.1f
#endif
#ifndef DC_DERIV_FILTER_ALPHA
#define DC_DERIV_FILTER_ALPHA            0.3f
#endif

#ifndef DC_MOTOR_DEFAULT_ACCEL_RATE
#define DC_MOTOR_DEFAULT_ACCEL_RATE      1.0f
#endif
#ifndef DC_MOTOR_DEFAULT_DECEL_RATE
#define DC_MOTOR_DEFAULT_DECEL_RATE      2.0f
#endif
#ifndef DC_MOTOR_RAMP_SMOOTH_ALPHA
#define DC_MOTOR_RAMP_SMOOTH_ALPHA       0.5f
#endif

#ifndef DC_MOTOR_DEFAULT_STALL_TIMEOUT_MS
#define DC_MOTOR_DEFAULT_STALL_TIMEOUT_MS  500U
#endif
/* Stall RPM threshold - must be set BELOW your minimum desired operating RPM.
 * Default changed to 10 RPM to avoid false stall detection at low setpoints.
 * Override this in your project config if needed. */
#ifndef DC_MOTOR_DEFAULT_STALL_MIN_RPM
#define DC_MOTOR_DEFAULT_STALL_MIN_RPM     10.0f
#endif
#ifndef DC_MOTOR_DEFAULT_MIN_DUTY_FOR_STALL
#define DC_MOTOR_DEFAULT_MIN_DUTY_FOR_STALL 0.1f
#endif

/* Minimum duty feedforward applied when running in closed-loop (PID).
 * This offsets the motor dead-zone so the PID output actually moves the motor.
 * Tune this to the lowest duty at which your motor reliably starts spinning.
 * Set to 0.0f to disable. */
#ifndef DC_MOTOR_DEFAULT_DEAD_ZONE_DUTY
#define DC_MOTOR_DEFAULT_DEAD_ZONE_DUTY    0.08f
#endif

/* Full-scale RPM used by the autotune warmup feedforward.
 * Override this with your motor's actual maximum RPM. */
#ifndef DC_MOTOR_DEFAULT_MAX_RPM
#define DC_MOTOR_DEFAULT_MAX_RPM         3000.0f
#endif

#ifndef DC_AUTOTUNE_DEFAULT_RELAY_AMP
#define DC_AUTOTUNE_DEFAULT_RELAY_AMP        20.0f
#endif
#ifndef DC_AUTOTUNE_DEFAULT_RELAY_DUTY_STEP
#define DC_AUTOTUNE_DEFAULT_RELAY_DUTY_STEP  0.30f
#endif
#ifndef DC_AUTOTUNE_DEFAULT_TIMEOUT_MS
#define DC_AUTOTUNE_DEFAULT_TIMEOUT_MS       60000U
#endif
#ifndef DC_AUTOTUNE_MIN_CYCLES
#define DC_AUTOTUNE_MIN_CYCLES               4U
#endif
#ifndef DC_AUTOTUNE_WARMUP_MS
#define DC_AUTOTUNE_WARMUP_MS                5000U
#endif

#endif
