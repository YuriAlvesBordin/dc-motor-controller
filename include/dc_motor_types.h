#ifndef DC_MOTOR_TYPES_H
#define DC_MOTOR_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DC_MOTOR_DEFAULT_KP
#define DC_MOTOR_DEFAULT_KP                 0.1f
#endif
#ifndef DC_MOTOR_DEFAULT_KI
#define DC_MOTOR_DEFAULT_KI                 0.01f
#endif
#ifndef DC_MOTOR_DEFAULT_KD
#define DC_MOTOR_DEFAULT_KD                 0.005f
#endif
#ifndef DC_MOTOR_DEFAULT_DT_S
#define DC_MOTOR_DEFAULT_DT_S               0.01f
#endif
#ifndef DC_MOTOR_PID_GAIN_SCALE
#define DC_MOTOR_PID_GAIN_SCALE             0.01f
#endif
#ifndef DC_MOTOR_DEFAULT_ACCEL_RATE
#define DC_MOTOR_DEFAULT_ACCEL_RATE         1.0f
#endif
#ifndef DC_MOTOR_DEFAULT_DECEL_RATE
#define DC_MOTOR_DEFAULT_DECEL_RATE         2.0f
#endif
#ifndef DC_MOTOR_RAMP_SMOOTH_ALPHA
#define DC_MOTOR_RAMP_SMOOTH_ALPHA          0.5f
#endif
#ifndef DC_MOTOR_DERIV_FILTER_ALPHA
#define DC_MOTOR_DERIV_FILTER_ALPHA         0.3f
#endif
#ifndef DC_MOTOR_DEFAULT_STALL_TIMEOUT_MS
#define DC_MOTOR_DEFAULT_STALL_TIMEOUT_MS   500U
#endif
#ifndef DC_MOTOR_DEFAULT_STALL_MIN_RPM
#define DC_MOTOR_DEFAULT_STALL_MIN_RPM      30.0f
#endif
#ifndef DC_MOTOR_DEFAULT_MIN_DUTY_FOR_STALL
#define DC_MOTOR_DEFAULT_MIN_DUTY_FOR_STALL 0.1f
#endif

typedef struct {
    float kp, ki, kd;
    float dt_s;
    float output_min, output_max;
    float integral_min, integral_max;
    float deriv_filter_alpha;
} DcMotor_PidConfig_t;

typedef struct {
    float accel_rate;
    float decel_rate;
    float smooth_alpha;
} DcMotor_RampConfig_t;

typedef struct {
    uint32_t stall_timeout_ms;
    float    stall_min_rpm;
    float    min_duty_for_stall;
} DcMotor_SafetyConfig_t;

typedef struct {
    float    integral;
    float    prev_error;
    float    filtered_deriv;
} DcMotor_PidState_t;

typedef struct {
    uint32_t stall_start_ms;
    bool     stall_active;
} DcMotor_SafetyState_t;

typedef void (*DcMotor_FaultCb_t)(void *ctx);

typedef enum {
    DC_MOTOR_IDLE        = 0,
    DC_MOTOR_RAMPING     = 1,
    DC_MOTOR_RUNNING     = 2,
    DC_MOTOR_FAULT_STALL = 3,
    DC_MOTOR_FAULT       = 4,
} DcMotor_State_t;

typedef enum {
    DC_MOTOR_OK           =  0,
    DC_MOTOR_ERR_NULL_PTR = -1,
    DC_MOTOR_ERR_PARAM    = -2,
    DC_MOTOR_ERR_NO_DATA  = -3,
    DC_MOTOR_ERR_STALL    = -4,
    DC_MOTOR_ERR_FAULT    = -5,
} DcMotor_Status_t;

#ifdef __cplusplus
}
#endif

#endif
