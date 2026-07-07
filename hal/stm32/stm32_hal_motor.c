#include "stm32_hal_motor.h"
#include "stm32_hal_motor_config.h"
#include <string.h>

static dc_motor_t        s_motor;
static TIM_HandleTypeDef *s_pwm_tim;
static uint32_t           s_pwm_channel;
static uint32_t           s_pwm_resolution;
static TIM_HandleTypeDef *s_enc_tim;
static uint32_t           s_enc_ppr;
static int16_t            s_last_encoder_count;

static void apply_pwm(float cmd_pct)
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

    duty = (uint32_t)((cmd_pct * (float)s_pwm_resolution) / 100.0f);
    __HAL_TIM_SET_COMPARE(s_pwm_tim, s_pwm_channel, duty);
}

static float read_rpm(float dt_sec)
{
    int16_t current_count;
    int16_t delta;
    float   revolutions;

    if (dt_sec <= 0.0f || s_enc_ppr == 0u)
    {
        return 0.0f;
    }

    current_count    = (int16_t)__HAL_TIM_GET_COUNTER(s_enc_tim);
    delta            = current_count - s_last_encoder_count;
    s_last_encoder_count = current_count;

    revolutions = (float)delta / (float)s_enc_ppr;
    return (revolutions / dt_sec) * 60.0f;
}

void stm32_hal_motor_init(void)
{
    memset(&s_motor, 0, sizeof(s_motor));

    s_pwm_tim        = STM32_MOTOR_PWM_TIM;
    s_pwm_channel    = STM32_MOTOR_PWM_CHANNEL;
    s_pwm_resolution = STM32_MOTOR_PWM_RESOLUTION;
    s_enc_tim        = STM32_MOTOR_ENCODER_TIM;
    s_enc_ppr        = STM32_MOTOR_ENCODER_PPR;
    s_last_encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(s_enc_tim);

    dc_motor_init(&s_motor);

    HAL_TIM_PWM_Start(s_pwm_tim, s_pwm_channel);
    __HAL_TIM_SET_COMPARE(s_pwm_tim, s_pwm_channel, 0u);
}

void stm32_hal_motor_set_target_rpm(float target_rpm)
{
    dc_motor_set_closedloop(&s_motor, target_rpm);
}

void stm32_hal_motor_stop(void)
{
    dc_motor_stop(&s_motor);
    apply_pwm(0.0f);
}

float stm32_hal_motor_get_rpm(void)
{
    return dc_motor_get_measured(&s_motor);
}

void stm32_hal_motor_control_tick(void)
{
    float rpm = read_rpm(DC_MOTOR_CONTROL_PERIOD_SEC);
    dc_motor_set_measured(&s_motor, rpm);
    apply_pwm(dc_motor_update(&s_motor, DC_MOTOR_CONTROL_PERIOD_SEC));
}

void stm32_hal_motor_1ms_tick(void)
{
    dc_motor_tick(&s_motor, 1u);
}
