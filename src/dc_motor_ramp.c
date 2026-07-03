#include "dc_motor_config.h"

#if DC_MOTOR_ENABLE_RAMP

#include "internal/dc_motor_ramp.h"

void DcMotor_Ramp_DefaultConfig(DcMotor_RampConfig_t *cfg)
{
    if (!cfg) return;
    cfg->accel_rate  = DC_MOTOR_DEFAULT_ACCEL_RATE;
    cfg->decel_rate  = DC_MOTOR_DEFAULT_DECEL_RATE;
    cfg->smooth_alpha = DC_MOTOR_RAMP_SMOOTH_ALPHA;
}

float DcMotor_Ramp_Step(const DcMotor_RampConfig_t *cfg,
                         float                       current,
                         float                       target,
                         float                       dt_s)
{
    if (!cfg || dt_s <= 0.0f) return current;

    float rate  = (target > current) ? cfg->accel_rate : cfg->decel_rate;
    float delta = rate * dt_s;
    float linear;

    if (target > current) {
        linear = current + delta;
        if (linear > target) linear = target;
    } else {
        linear = current - delta;
        if (linear < target) linear = target;
    }

    float alpha = cfg->smooth_alpha;
    return alpha * linear + (1.0f - alpha) * current;
}

#endif
