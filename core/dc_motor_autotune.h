#ifndef DC_MOTOR_AUTOTUNE_H
#define DC_MOTOR_AUTOTUNE_H

/**
 * @file  dc_motor_autotune.h
 * @brief Autonomous PID auto-tuner using Relay Feedback (Åström-Hägglund)
 *        with Tyreus-Luyben gain rules.
 *
 * ## How it works
 *
 *  1. WARMUP  — ramps the motor to ~50 % of the target RPM so the plant
 *               is in a representative operating region before testing.
 *
 *  2. RELAY   — replaces the PID with a relay (bang-bang) that switches
 *               between (rpm_target + relay_amp) and (rpm_target - relay_amp).
 *               The motor enters a controlled oscillation.
 *
 *  3. COLLECT — detects peaks and valleys in the RPM signal.  At least
 *               DC_MOTOR_AUTOTUNE_MIN_CYCLES full cycles are averaged to
 *               produce reliable estimates of:
 *                 - a   : half-amplitude of oscillation [RPM]
 *                 - Tu  : ultimate period [s]
 *                 - Ku  : ultimate gain  = 4*d / (pi*a)
 *
 *  4. CALC    — applies Tyreus-Luyben rules (low overshoot, robust):
 *                 Kp = Ku / 3.2
 *                 Ki = Kp / (2.87 * Tu)
 *                 Kd = Kp * Tu / 11.4
 *
 *  5. DONE    — fills a DcMotor_PidConfig_t and calls the user callback.
 *               The motor is handed back to normal PID control.
 *
 * ## Usage
 *
 * @code
 *   static DcMotor_AutotuneCtx_t at_ctx;
 *
 *   void on_autotune_done(const DcMotor_PidConfig_t *cfg, void *user)
 *   {
 *       DcMotor_SetPidConfig(&motor, cfg);   // apply immediately
 *   }
 *
 *   // Start once:
 *   DcMotor_Autotune_Start(&at_ctx, &motor, &at_cfg, on_autotune_done, NULL);
 *
 *   // In your 10 ms timer ISR / RTOS task — INSTEAD of DcMotor_Update():
 *   DcMotor_AutotuneState_t s = DcMotor_Autotune_Update(&at_ctx, tick_ms, rpm);
 *   if (s == DC_AUTOTUNE_DONE || s >= DC_AUTOTUNE_ERR_BASE)
 *       DcMotor_Update(&motor, tick_ms, rpm);  // resume normal control
 * @endcode
 */

#include "dc_motor.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Compile-time tuneable constants                                    */
/* ------------------------------------------------------------------ */

/** Minimum complete oscillation cycles to average before calculating. */
#ifndef DC_MOTOR_AUTOTUNE_MIN_CYCLES
#define DC_MOTOR_AUTOTUNE_MIN_CYCLES    4U
#endif

/** Maximum cycles to wait before giving up (prevents infinite relay). */
#ifndef DC_MOTOR_AUTOTUNE_MAX_CYCLES
#define DC_MOTOR_AUTOTUNE_MAX_CYCLES    20U
#endif

/** Warmup ends when measured RPM reaches this fraction of the target. */
#ifndef DC_MOTOR_AUTOTUNE_WARMUP_FRAC
#define DC_MOTOR_AUTOTUNE_WARMUP_FRAC   0.50f
#endif

/** Maximum warmup time in ms before giving up. */
#ifndef DC_MOTOR_AUTOTUNE_WARMUP_TIMEOUT_MS
#define DC_MOTOR_AUTOTUNE_WARMUP_TIMEOUT_MS  5000U
#endif

/** Hysteresis band [RPM] — prevents chattering near zero-crossing. */
#ifndef DC_MOTOR_AUTOTUNE_HYSTERESIS
#define DC_MOTOR_AUTOTUNE_HYSTERESIS    5.0f
#endif

/* ------------------------------------------------------------------ */
/*  State machine                                                      */
/* ------------------------------------------------------------------ */

typedef enum {
    DC_AUTOTUNE_IDLE    = 0,  /**< Not running.                          */
    DC_AUTOTUNE_WARMUP  = 1,  /**< Motor ramping to operating region.    */
    DC_AUTOTUNE_RELAY   = 2,  /**< Relay active, collecting oscillations.*/
    DC_AUTOTUNE_CALC    = 3,  /**< Computing gains (single-tick).        */
    DC_AUTOTUNE_DONE    = 4,  /**< Finished successfully.                */

    /* Error codes (>= DC_AUTOTUNE_ERR_BASE) */
    DC_AUTOTUNE_ERR_BASE           = 10,
    DC_AUTOTUNE_ERR_NO_OSCILLATION = 10, /**< Motor didn't oscillate.    */
    DC_AUTOTUNE_ERR_STALL          = 11, /**< Stall detected mid-test.   */
    DC_AUTOTUNE_ERR_TIMEOUT        = 12, /**< Warmup or relay timed out. */
    DC_AUTOTUNE_ERR_PARAM          = 13, /**< Bad configuration.         */
} DcMotor_AutotuneState_t;

/* ------------------------------------------------------------------ */
/*  Configuration                                                      */
/* ------------------------------------------------------------------ */

typedef struct {
    /**
     * Target RPM around which the relay oscillates.
     * Should be a representative operating point (e.g. your normal setpoint).
     */
    float rpm_target;

    /**
     * Relay amplitude [RPM].
     * The relay switches when RPM crosses (rpm_target ± relay_amp).
     * Typical: 10–20 % of rpm_target.
     */
    float relay_amp;

    /**
     * Relay output step size in duty units [0..1].
     * This is the `d` in Ku = 4d/(pi*a).
     * Typical: 0.10 – 0.25.
     */
    float relay_duty_step;

    /**
     * Maximum time [ms] the relay phase may run before aborting.
     * 0 = use a sane default (30 s).
     */
    uint32_t relay_timeout_ms;

    /**
     * dt_s to carry into the resulting DcMotor_PidConfig_t.
     * Must match your Update() call period.
     */
    float dt_s;

    /** Output limits forwarded to the result PidConfig. */
    float output_min;
    float output_max;

    /** Integral limits forwarded to the result PidConfig. */
    float integral_min;
    float integral_max;

    /** Derivative filter alpha forwarded to the result PidConfig. */
    float deriv_filter_alpha;
} DcMotor_AutotuneCfg_t;

/* ------------------------------------------------------------------ */
/*  Result callback                                                    */
/* ------------------------------------------------------------------ */

/**
 * @brief  Called once when the autotune completes successfully.
 *
 * @param cfg   Freshly calculated PID configuration.
 * @param user  Opaque pointer supplied to DcMotor_Autotune_Start().
 */
typedef void (*DcMotor_AutotuneDoneCb_t)(const DcMotor_PidConfig_t *cfg,
                                         void                      *user);

/* ------------------------------------------------------------------ */
/*  Context (treat as opaque)                                         */
/* ------------------------------------------------------------------ */

#ifndef DC_MOTOR_AUTOTUNE_MAX_CYCLES
#define DC_MOTOR_AUTOTUNE_MAX_CYCLES 20U
#endif

typedef struct {
    /* Public config (set by Start) */
    DcMotor_AutotuneCfg_t   cfg;
    DcMotor_Handle_t       *hdm;
    DcMotor_AutotuneDoneCb_t done_cb;
    void                   *user_ctx;

    /* State machine */
    DcMotor_AutotuneState_t state;

    /* Relay state */
    bool     relay_high;        /**< Current relay output direction.     */
    float    last_rpm;

    /* Peak / valley tracking */
    float    peak_rpm;
    float    valley_rpm;
    uint32_t peak_tick_ms;
    uint32_t valley_tick_ms;
    bool     tracking_peak;     /**< true = looking for next peak.       */

    /* Cycle accumulation */
    uint32_t cycle_count;
    float    sum_amplitude;     /**< Sum of half-amplitudes [RPM].       */
    float    sum_period_s;      /**< Sum of full-period durations [s].   */
    uint32_t last_peak_tick_ms; /**< Tick of previous peak (period meas).*/
    uint32_t last_valley_tick_ms;

    /* Timing */
    uint32_t phase_start_ms;

    /* Result */
    DcMotor_PidConfig_t result;
} DcMotor_AutotuneCtx_t;

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief  Initialise and start the autotune procedure.
 *
 * After calling this, stop calling DcMotor_Update() and call
 * DcMotor_Autotune_Update() instead, until it returns DONE or an error.
 *
 * @param ctx      Caller-allocated context (may be static or stack).
 * @param hdm      Motor handle to tune (must be initialised).
 * @param cfg      Autotune configuration.
 * @param done_cb  Called when tuning completes (may be NULL).
 * @param user     Forwarded to done_cb.
 * @return DC_MOTOR_OK on success, error code otherwise.
 */
DcMotor_Status_t DcMotor_Autotune_Start(DcMotor_AutotuneCtx_t    *ctx,
                                         DcMotor_Handle_t         *hdm,
                                         const DcMotor_AutotuneCfg_t *cfg,
                                         DcMotor_AutotuneDoneCb_t  done_cb,
                                         void                     *user);

/**
 * @brief  Drive the autotune state machine.
 *
 * Call this at the same rate as DcMotor_Update() (e.g. every 10 ms).
 *
 * @param ctx         Autotune context.
 * @param tick_ms     Current system tick.
 * @param current_rpm Latest measured RPM from encoder/hall sensor.
 * @return Current autotune state.
 */
DcMotor_AutotuneState_t DcMotor_Autotune_Update(DcMotor_AutotuneCtx_t *ctx,
                                                 uint32_t               tick_ms,
                                                 float                  current_rpm);

/**
 * @brief  Retrieve the calculated PID configuration after DONE.
 *
 * @param ctx      Completed autotune context.
 * @param out_cfg  Destination for the result.
 * @return DC_MOTOR_OK, or DC_MOTOR_ERR_NO_DATA if not done yet.
 */
DcMotor_Status_t DcMotor_Autotune_GetResult(const DcMotor_AutotuneCtx_t *ctx,
                                             DcMotor_PidConfig_t         *out_cfg);

/**
 * @brief  Abort an ongoing autotune and return the motor to IDLE.
 */
void DcMotor_Autotune_Abort(DcMotor_AutotuneCtx_t *ctx);

/**
 * @brief  Convenience: fill a DcMotor_AutotuneCfg_t with sensible defaults.
 *
 * You still need to set rpm_target and dt_s for your application.
 */
void DcMotor_Autotune_DefaultConfig(DcMotor_AutotuneCfg_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* DC_MOTOR_AUTOTUNE_H */
