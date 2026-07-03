#include "internal/dc_motor_ramp.h"
#include "internal/dc_motor_types.h"

static float s_clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float DcMotor_Ramp_Step(const DcMotor_RampConfig_t *cfg,
                        float                       current_duty,
                        float                       target_duty,
                        float                       dt_s)
{
    if (!cfg || dt_s <= 0.0f) return current_duty;

    float rate  = (target_duty > current_duty) ? cfg->accel_rate : cfg->decel_rate;
    float delta = rate * dt_s;
    float linear;

    if (target_duty > current_duty)
        linear = s_clamp(current_duty + delta, current_duty, target_duty);
    else
        linear = s_clamp(current_duty - delta, target_duty, current_duty);

    float next = cfg->smooth_alpha * linear
               + (1.0f - cfg->smooth_alpha) * current_duty;

    if ((target_duty > current_duty && next >= target_duty) ||
        (target_duty < current_duty && next <= target_duty))
        next = target_duty;

    return s_clamp(next, 0.0f, 1.0f);
}

void DcMotor_Ramp_DefaultConfig(DcMotor_RampConfig_t *cfg)
{
    if (!cfg) return;
    cfg->accel_rate   = DC_MOTOR_DEFAULT_ACCEL_RATE;
    cfg->decel_rate   = DC_MOTOR_DEFAULT_DECEL_RATE;
    cfg->smooth_alpha = DC_MOTOR_RAMP_SMOOTH_ALPHA;
}
