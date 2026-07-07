#include "stm32_hal_motor.h"
#include "stm32_hal_motor_config.h"
#include <string.h>

static dc_motor_t        s_motor;
static TIM_HandleTypeDef *s_pwm_tim;
static uint32_t           s_pwm_channel;
static uint32_t           s_pwm_resolution;
static RpmCalc_Handle_t  *s_rpm_handle;

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
    double rpm_double = 0.0;
    RpmCalc_Status_t status;

    (void)dt_sec; // dt_sec unused; RpmCalc maintains its own timing

    if (s_rpm_handle == NULL)
    {
        return 0.0f;
    }

    status = RpmCalc_GetRPM(s_rpm_handle, &rpm_double);
    if (status != RPM_CALC_OK)
    {
        return 0.0f;
    }

    return (float)rpm_double;
}

void stm32_hal_motor_init(void)
{
    memset(&s_motor, 0, sizeof(s_motor));

    s_pwm_tim        = STM32_MOTOR_PWM_TIM;
    s_pwm_channel    = STM32_MOTOR_PWM_CHANNEL;
    s_pwm_resolution = STM32_MOTOR_PWM_RESOLUTION;
    s_rpm_handle     = STM32_MOTOR_RPM_HANDLE;

    dc_motor_init(&s_motor);

    HAL_TIM_PWM_Start(s_pwm_tim, s_pwm_channel);
    __HAL_TIM_SET_COMPARE(s_pwm_tim, s_pwm_channel, 0u);
}

void stm32_hal_motor_set_target_rpm(float target_rpm)
{
    dc_motor_set_closedloop(&s_motor, target_rpm);
}

void stm32_hal_motor_set_openloop(float duty_pct)
{
    if (duty_pct < 0.0f)   duty_pct = 0.0f;
    if (duty_pct > 100.0f) duty_pct = 100.0f;
    dc_motor_set_openloop(&s_motor, duty_pct);
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

dc_motor_pid_t *stm32_hal_motor_get_pid(void)
{
    return &s_motor.closedloop.pid;
}
