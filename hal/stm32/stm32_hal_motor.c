#include "stm32_hal_motor.h"
#include "stm32_hal_motor_config.h"
#include <string.h>

#define MOTOR_COUNT   ((uint32_t)STM32_MOTOR_COUNT)

typedef struct
{
    TIM_HandleTypeDef *pwm_tim;
    uint32_t           pwm_channel;
    uint32_t           pwm_resolution;
    TIM_HandleTypeDef *enc_tim;
    uint32_t           enc_ppr;
    int16_t            last_encoder_count;
    dc_motor_t         motor;
} stm32_motor_runtime_t;

static stm32_motor_runtime_t s_motors[MOTOR_COUNT];

static void stm32_motor_apply_pwm(stm32_motor_runtime_t *rt, float cmd_pct)
{
    uint32_t duty;

    if (cmd_pct < 0.0f)
    {
        cmd_pct = 0.0f;
    }
    else if (cmd_pct > 100.0f)
    {
        cmd_pct = 100.0f;
    }

    duty = (uint32_t)((cmd_pct * (float)rt->pwm_resolution) / 100.0f);
    __HAL_TIM_SET_COMPARE(rt->pwm_tim, rt->pwm_channel, duty);
}

static float stm32_motor_read_rpm(stm32_motor_runtime_t *rt, float dt_sec)
{
    int16_t current_count;
    int16_t delta;
    float   revolutions;

    current_count = (int16_t)__HAL_TIM_GET_COUNTER(rt->enc_tim);
    delta = current_count - rt->last_encoder_count;
    rt->last_encoder_count = current_count;

    if (rt->enc_ppr == 0u)
    {
        return 0.0f;
    }

    revolutions = (float)delta / (float)rt->enc_ppr;
    return (revolutions / dt_sec) * 60.0f;
}

int stm32_hal_motor_init(void)
{
    uint32_t i;
    static const struct
    {
        TIM_HandleTypeDef *pwm_tim;
        uint32_t           pwm_channel;
        uint32_t           pwm_resolution;
        TIM_HandleTypeDef *enc_tim;
        uint32_t           enc_ppr;
    } cfg[MOTOR_COUNT] =
    {
        { STM32_MOTOR0_PWM_TIM, STM32_MOTOR0_PWM_CHANNEL, STM32_MOTOR0_PWM_RESOLUTION,
          STM32_MOTOR0_ENCODER_TIM, STM32_MOTOR0_ENCODER_PPR },
#if MOTOR_COUNT > 1
        { STM32_MOTOR1_PWM_TIM, STM32_MOTOR1_PWM_CHANNEL, STM32_MOTOR1_PWM_RESOLUTION,
          STM32_MOTOR1_ENCODER_TIM, STM32_MOTOR1_ENCODER_PPR },
#endif
    };

    memset(s_motors, 0, sizeof(s_motors));

    for (i = 0u; i < MOTOR_COUNT; ++i)
    {
        s_motors[i].pwm_tim        = cfg[i].pwm_tim;
        s_motors[i].pwm_channel    = cfg[i].pwm_channel;
        s_motors[i].pwm_resolution = cfg[i].pwm_resolution;
        s_motors[i].enc_tim        = cfg[i].enc_tim;
        s_motors[i].enc_ppr        = cfg[i].enc_ppr;
        s_motors[i].last_encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(cfg[i].enc_tim);

        dc_motor_init(&s_motors[i].motor);

        if (cfg[i].pwm_tim != NULL)
        {
            HAL_TIM_PWM_Start(cfg[i].pwm_tim, cfg[i].pwm_channel);
            __HAL_TIM_SET_COMPARE(cfg[i].pwm_tim, cfg[i].pwm_channel, 0u);
        }
    }

    return 0;
}

int stm32_hal_motor_set_target_rpm(stm32_motor_id_t id, float target_rpm)
{
    if ((uint32_t)id >= MOTOR_COUNT)
    {
        return -1;
    }
    dc_motor_set_closedloop(&s_motors[id].motor, target_rpm);
    return 0;
}

void stm32_hal_motor_stop(stm32_motor_id_t id)
{
    if ((uint32_t)id >= MOTOR_COUNT)
    {
        return;
    }
    dc_motor_stop(&s_motors[id].motor);
    stm32_motor_apply_pwm(&s_motors[id], 0.0f);
}

float stm32_hal_motor_get_rpm(stm32_motor_id_t id)
{
    if ((uint32_t)id >= MOTOR_COUNT)
    {
        return 0.0f;
    }
    return dc_motor_get_measured(&s_motors[id].motor);
}

void stm32_hal_motor_control_tick(stm32_motor_id_t id)
{
    stm32_motor_runtime_t *rt;
    float rpm;
    float cmd;
    const float dt = DC_MOTOR_CONTROL_PERIOD_SEC;

    if ((uint32_t)id >= MOTOR_COUNT)
    {
        return;
    }
    rt = &s_motors[id];

    rpm = stm32_motor_read_rpm(rt, dt);
    dc_motor_set_measured(&rt->motor, rpm);

    cmd = dc_motor_update(&rt->motor, dt);

    stm32_motor_apply_pwm(rt, cmd);
}

void stm32_hal_motor_1ms_tick(void)
{
    uint32_t i;
    for (i = 0u; i < MOTOR_COUNT; ++i)
    {
        dc_motor_tick(&s_motors[i].motor, 1u);
    }
}
