#ifndef DC_MOTOR_AUTOTUNE_H
#define DC_MOTOR_AUTOTUNE_H

/**
 * @file  dc_motor_autotune.h
 * @brief Autonomous PID auto-tuner — relay feedback (Åström-Hägglund)
 *        + Ziegler-Nichols classic gain rules.
 *
 * This is an OPTIONAL module. Include it only if you need auto-tuning.
 * It has no side-effects on normal DcMotor_Update() operation.
 *
 * Changelog vs. previous version:
 *  - Fixed tu_s calculation: now uses relay phase elapsed time, not warmup time.
 *  - Warmup duty is now a feedforward proportional to rpm_target/rpm_max
 *    instead of a fixed 0.5f, so the relay starts near the real operating
 *    point and converges faster.
 *  - Relay output is centred on operating_duty (measured during warmup)
 *    instead of 0.5f, avoiding asymmetric excitation.
 *  - Amplitude is now accumulated on both relay flanks (positive→negative
 *    AND negative→positive), halving estimation variance.
 *  - Switched from Tyreus-Luyben to Ziegler-Nichols classic rules for a
 *    faster rise-time response while maintaining adequate stability margins.
 *    Define DC_AUTOTUNE_USE_TYREUS_LUYBEN=1 to revert to the conservative
 *    tuning rules if needed.
 */

#include "dc_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  State machine                                                      */
/* ------------------------------------------------------------------ */

typedef enum {
    DC_AUTOTUNE_IDLE    = 0,
    DC_AUTOTUNE_WARMUP  = 1,
    DC_AUTOTUNE_RELAY   = 2,
    DC_AUTOTUNE_CALC    = 3,
    DC_AUTOTUNE_DONE    = 4,

    DC_AUTOTUNE_ERR_BASE             = 0x10,
    DC_AUTOTUNE_ERR_NO_OSCILLATION   = 0x10,
    DC_AUTOTUNE_ERR_STALL            = 0x11,
    DC_AUTOTUNE_ERR_TIMEOUT          = 0x12,
} DcMotor_AutotuneState_t;

/* ------------------------------------------------------------------ */
/*  Configuration                                                      */
/* ------------------------------------------------------------------ */

typedef struct {
    float    rpm_target;          /**< Representative operating RPM (required). */
    float    rpm_max;             /**< Motor full-scale RPM used to derive warmup
                                       feedforward duty. Set to 0 to use
                                       DC_MOTOR_DEFAULT_MAX_RPM fallback. */
    float    relay_amp;           /**< Relay switching amplitude around rpm_target (RPM). */
    float    relay_duty_step;     /**< Relay output step size (the 'd' in Ku = 4d/πa). */
    uint32_t relay_timeout_ms;    /**< Max relay phase duration before abort. */
    float    dt_s;                /**< Sample period — must match DcMotor_Update() rate. */
} DcMotor_AutotuneCfg_t;

/* ------------------------------------------------------------------ */
/*  Result                                                             */
/* ------------------------------------------------------------------ */

typedef struct {
    float ku;   /**< Ultimate gain   */
    float tu_s; /**< Ultimate period (seconds) */
} DcMotor_AutotuneResult_t;

/* ------------------------------------------------------------------ */
/*  Callback                                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief  Called by the auto-tuner when DONE.
 * @param  cfg   Ready-to-use PID config computed by the tuning rules.
 * @param  user  The pointer you passed to DcMotor_Autotune_Start().
 */
typedef void (*DcMotor_AutotuneDoneCb_t)(const DcMotor_PidConfig_t *cfg,
                                         void                       *user);

/* ------------------------------------------------------------------ */
/*  Context (opaque — do not access fields directly)                  */
/* ------------------------------------------------------------------ */

typedef struct {
    DcMotor_Handle_t        *motor;
    DcMotor_AutotuneCfg_t    cfg;
    DcMotor_AutotuneDoneCb_t done_cb;
    void                    *user;

    DcMotor_AutotuneState_t  state;

    /* relay phase */
    float    relay_output;
    float    operating_duty;  /**< Feedforward duty estimated during warmup. */
    float    peak_rpm;
    float    valley_rpm;
    uint32_t cycle_count;
    float    amplitude_acc;
    uint32_t relay_start_ms;

    /* warmup */
    uint32_t warmup_start_ms;

    /* result */
    DcMotor_AutotuneResult_t result;
    DcMotor_PidConfig_t      tuned_pid;
} DcMotor_AutotuneCtx_t;

/* ------------------------------------------------------------------ */
/*  API                                                                */
/* ------------------------------------------------------------------ */

/**
 * @brief  Populate cfg with safe defaults. Call before Start().
 */
void DcMotor_Autotune_DefaultConfig(DcMotor_AutotuneCfg_t *cfg);

/**
 * @brief  Begin auto-tuning.
 * @param  ctx      Caller-allocated context (static or stack).
 * @param  motor    Handle to the motor being tuned.
 * @param  cfg      Tuning parameters (call DefaultConfig first).
 *                  Set cfg->rpm_max to the motor's maximum RPM for best
 *                  feedforward accuracy.
 * @param  done_cb  Called with the computed PID config when finished.
 * @param  user     Forwarded to done_cb.
 */
DcMotor_Status_t DcMotor_Autotune_Start(DcMotor_AutotuneCtx_t    *ctx,
                                         DcMotor_Handle_t         *motor,
                                         const DcMotor_AutotuneCfg_t *cfg,
                                         DcMotor_AutotuneDoneCb_t  done_cb,
                                         void                     *user);

/**
 * @brief  Periodic update — call instead of DcMotor_Update() while tuning.
 * @return Current auto-tune state. Resume DcMotor_Update() when
 *         state == DC_AUTOTUNE_DONE or state >= DC_AUTOTUNE_ERR_BASE.
 */
DcMotor_AutotuneState_t DcMotor_Autotune_Update(DcMotor_AutotuneCtx_t *ctx,
                                                 uint32_t               tick_ms,
                                                 float                  current_rpm);

/**
 * @brief  Abort an ongoing tuning session (calls EmergencyStop).
 */
void DcMotor_Autotune_Abort(DcMotor_AutotuneCtx_t *ctx);

/**
 * @brief  Retrieve the raw Ku/Tu result after DONE.
 */
DcMotor_Status_t DcMotor_Autotune_GetResult(const DcMotor_AutotuneCtx_t *ctx,
                                             DcMotor_AutotuneResult_t    *out);

#ifdef __cplusplus
}
#endif

#endif /* DC_MOTOR_AUTOTUNE_H */
