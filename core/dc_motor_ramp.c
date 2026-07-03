#include "dc_motor_ramp.h"

/* ------------------------------------------------------------------ */
/*  Internal helper                                                    */
/* ------------------------------------------------------------------ */

static float ramp_clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void DcMotor_Ramp_DefaultConfig(DcMotor_RampConfig_t *cfg)
{
    if (!cfg) return;
    *cfg = (DcMotor_RampConfig_t){
        .accel_rate  = DC_MOTOR_DEFAULT_ACCEL_RATE,
        .decel_rate  = DC_MOTOR_DEFAULT_DECEL_RATE,
        .smooth_alpha = DC_MOTOR_RAMP_SMOOTH_ALPHA,
    };
}

float DcMotor_Ramp_Step(const DcMotor_RampConfig_t *cfg,
                        float                       current_duty,
                        float                       target_duty,
                        float                       dt_s)
{
    if (!cfg || dt_s <= 0.0f) return current_duty;

    float err = target_duty - current_duty;
    if (err == 0.0f) return current_duty;

    float alpha    = ramp_clamp(cfg->smooth_alpha, 0.0f, 1.0f);
    float rate     = (err > 0.0f) ? cfg->accel_rate : cfg->decel_rate;
    float step     = err * alpha;
    float max_step = rate * dt_s;

    if (step >  max_step) step =  max_step;
    if (step < -max_step) step = -max_step;

    float next = current_duty + step;

    /* Clamp to target to avoid overshoot */
    if ((err > 0.0f && next > target_duty) ||
        (err < 0.0f && next < target_duty))
        next = target_duty;

    return ramp_clamp(next, 0.0f, 1.0f);
}
