#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include "dc_motor_port.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DC_MOTOR_DEFAULT_KP
#define DC_MOTOR_DEFAULT_KP             0.1f
#endif
#ifndef DC_MOTOR_DEFAULT_KI
#define DC_MOTOR_DEFAULT_KI             0.01f
#endif
#ifndef DC_MOTOR_DEFAULT_KD
#define DC_MOTOR_DEFAULT_KD             0.005f
#endif
#ifndef DC_MOTOR_DEFAULT_DT_S
#define DC_MOTOR_DEFAULT_DT_S           0.01f   /* 10 ms */
#endif
#ifndef DC_MOTOR_PID_GAIN_SCALE
#define DC_MOTOR_PID_GAIN_SCALE         0.01f
#endif
#ifndef DC_MOTOR_DEFAULT_ACCEL_RATE
#define DC_MOTOR_DEFAULT_ACCEL_RATE     1.0f    /* duty/s */
#endif
#ifndef DC_MOTOR_DEFAULT_DECEL_RATE
#define DC_MOTOR_DEFAULT_DECEL_RATE     2.0f
#endif
#ifndef DC_MOTOR_RAMP_SMOOTH_ALPHA
#define DC_MOTOR_RAMP_SMOOTH_ALPHA      0.5f
#endif
#ifndef DC_MOTOR_DERIV_FILTER_ALPHA
#define DC_MOTOR_DERIV_FILTER_ALPHA     0.3f    /* lower = less noise, more lag */
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

/* ---------------------------------------------------------- */

typedef struct {
    float kp, ki, kd;
    float dt_s;
    float output_min, output_max;
    float integral_min, integral_max;
    float deriv_filter_alpha;
} DcMotor_PidConfig_t;

typedef struct {
    float accel_rate;       /* duty/s */
    float decel_rate;
    float smooth_alpha;     /* s-curve shape; 1.0 degenerates to linear */
} DcMotor_RampConfig_t;

typedef struct {
    uint32_t stall_timeout_ms;      /* 0 disables watchdog entirely */
    float    stall_min_rpm;
    float    min_duty_for_stall;    /* don't check stall below this duty */
} DcMotor_SafetyConfig_t;

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

typedef struct DcMotor_Handle_t DcMotor_Handle_t;

struct DcMotor_Handle_t {
    const DcMotor_Port_t *port;
    void                 *hw;

    DcMotor_PidConfig_t    pid_cfg;
    DcMotor_RampConfig_t   ramp_cfg;
    DcMotor_SafetyConfig_t safety_cfg;

    float target_duty;
    float current_duty;

    bool  closed_loop;
    float rpm_setpoint;
    float pid_integral;
    float pid_prev_error;
    float pid_filtered_deriv;

    DcMotor_State_t  state;
    DcMotor_Status_t last_status;

    uint32_t stall_start_ms;
    bool     stall_active;

    uint32_t last_tick_ms;
    bool     first_update;
};

/*
 * Pass NULL to pid/ramp/safety to get the defaults above.
 * hw is passed straight through to port callbacks — cast it
 * to whatever your port implementation expects.
 */
DcMotor_Status_t DcMotor_Init(DcMotor_Handle_t             *hdm,
                              const DcMotor_Port_t         *port,
                              void                         *hw,
                              const DcMotor_PidConfig_t    *pid,
                              const DcMotor_RampConfig_t   *ramp,
                              const DcMotor_SafetyConfig_t *safety);

DcMotor_Status_t DcMotor_SetDuty(DcMotor_Handle_t *hdm, float duty);
DcMotor_Status_t DcMotor_SetSpeed(DcMotor_Handle_t *hdm, float speed_pct);
DcMotor_Status_t DcMotor_Stop(DcMotor_Handle_t *hdm);
void             DcMotor_EmergencyStop(DcMotor_Handle_t *hdm);

/*
 * Call this from your timer / RTOS task.
 * current_rpm: whatever your encoder/hall driver last computed.
 *              Ignored in open-loop mode, but still used by the stall watchdog.
 */
void DcMotor_Update(DcMotor_Handle_t *hdm, uint32_t tick_ms, float current_rpm);

/* Enables closed-loop PID and sets the target. Pass 0 to stop instead. */
DcMotor_Status_t DcMotor_SetRpmSetpoint(DcMotor_Handle_t *hdm, float rpm_sp);
void             DcMotor_SetClosedLoop(DcMotor_Handle_t *hdm, bool enabled);

void DcMotor_SetPidConfig   (DcMotor_Handle_t *hdm, const DcMotor_PidConfig_t    *cfg);
void DcMotor_SetRampConfig  (DcMotor_Handle_t *hdm, const DcMotor_RampConfig_t   *cfg);
void DcMotor_SetSafetyConfig(DcMotor_Handle_t *hdm, const DcMotor_SafetyConfig_t *cfg);

/* After a FAULT_STALL the motor is locked until this is called. */
DcMotor_Status_t DcMotor_ClearFault(DcMotor_Handle_t *hdm);
void             DcMotor_ResetPid(DcMotor_Handle_t *hdm);

static inline bool DcMotor_IsRunning(const DcMotor_Handle_t *hdm) {
    return hdm && (hdm->state == DC_MOTOR_RUNNING || hdm->state == DC_MOTOR_RAMPING);
}
static inline bool DcMotor_IsFault(const DcMotor_Handle_t *hdm) {
    return hdm && (hdm->state == DC_MOTOR_FAULT_STALL || hdm->state == DC_MOTOR_FAULT);
}
static inline bool DcMotor_IsClosedLoop(const DcMotor_Handle_t *hdm) {
    return hdm && hdm->closed_loop;
}

#ifdef __cplusplus
}
#endif

#endif /* DC_MOTOR_H */
