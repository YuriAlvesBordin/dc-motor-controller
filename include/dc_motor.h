#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include "dc_motor_port.h"
#include "../src/internal/dc_motor_types.h"
#include "../src/internal/dc_motor_pid.h"
#include "../src/internal/dc_motor_ramp.h"
#include "../src/internal/dc_motor_safety.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Motor controller handle.
 *         Contains the HAL port, hardware reference, and the state of
 *         every subsystem (PID, ramp, safety).  Initialise with
 *         DcMotor_Init(); treat as opaque after that.
 */
typedef struct DcMotor_Handle_t DcMotor_Handle_t;

struct DcMotor_Handle_t {
    const DcMotor_Port_t *port;
    void                 *hw;

    /* Sub-module configurations */
    DcMotor_PidConfig_t    pid_cfg;
    DcMotor_RampConfig_t   ramp_cfg;
    DcMotor_SafetyConfig_t safety_cfg;

    /* Sub-module runtime states */
    DcMotor_PidState_t    pid_st;
    DcMotor_SafetyState_t safety_st;

    /* Orchestrator state */
    float target_duty;
    float current_duty;

    bool  closed_loop;
    float rpm_setpoint;

    DcMotor_State_t  state;
    DcMotor_Status_t last_status;

    uint32_t last_tick_ms;
    bool     first_update;
};

/* ------------------------------------------------------------------ */
/*  Lifecycle                                                          */
/* ------------------------------------------------------------------ */

DcMotor_Status_t DcMotor_Init(DcMotor_Handle_t             *hdm,
                              const DcMotor_Port_t         *port,
                              void                         *hw,
                              const DcMotor_PidConfig_t    *pid,
                              const DcMotor_RampConfig_t   *ramp,
                              const DcMotor_SafetyConfig_t *safety);

/* ------------------------------------------------------------------ */
/*  Control                                                            */
/* ------------------------------------------------------------------ */

DcMotor_Status_t DcMotor_SetDuty       (DcMotor_Handle_t *hdm, float duty);
DcMotor_Status_t DcMotor_SetSpeed      (DcMotor_Handle_t *hdm, float speed_pct);
DcMotor_Status_t DcMotor_SetRpmSetpoint(DcMotor_Handle_t *hdm, float rpm_sp);
void             DcMotor_SetClosedLoop  (DcMotor_Handle_t *hdm, bool enabled);
DcMotor_Status_t DcMotor_Stop          (DcMotor_Handle_t *hdm);
void             DcMotor_EmergencyStop  (DcMotor_Handle_t *hdm);

/* ------------------------------------------------------------------ */
/*  Periodic update  (call from timer / RTOS task)                    */
/* ------------------------------------------------------------------ */

void DcMotor_Update(DcMotor_Handle_t *hdm, uint32_t tick_ms, float current_rpm);

/* ------------------------------------------------------------------ */
/*  Runtime configuration setters                                     */
/* ------------------------------------------------------------------ */

void DcMotor_SetPidConfig   (DcMotor_Handle_t *hdm, const DcMotor_PidConfig_t    *cfg);
void DcMotor_SetRampConfig  (DcMotor_Handle_t *hdm, const DcMotor_RampConfig_t   *cfg);
void DcMotor_SetSafetyConfig(DcMotor_Handle_t *hdm, const DcMotor_SafetyConfig_t *cfg);

/* ------------------------------------------------------------------ */
/*  Fault handling                                                     */
/* ------------------------------------------------------------------ */

DcMotor_Status_t DcMotor_ClearFault(DcMotor_Handle_t *hdm);
void             DcMotor_ResetPid  (DcMotor_Handle_t *hdm);

/* ------------------------------------------------------------------ */
/*  Inline status queries                                              */
/* ------------------------------------------------------------------ */

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
