#include "dc_motor_stm32.h"

static void set_duty(void *hw, float duty_norm)
{
    DcMotor_Stm32Hw_t *h = (DcMotor_Stm32Hw_t *)hw;
    if (!h || !h->htim) return;
    uint32_t arr = h->htim->Instance->ARR;
    __HAL_TIM_SET_COMPARE(h->htim, h->channel, (uint32_t)(duty_norm * arr));
}

static void start_pwm(void *hw)
{
    DcMotor_Stm32Hw_t *h = (DcMotor_Stm32Hw_t *)hw;
    if (!h || !h->htim) return;
    HAL_TIM_PWM_Start(h->htim, h->channel);
}

static void stop_pwm(void *hw)
{
    DcMotor_Stm32Hw_t *h = (DcMotor_Stm32Hw_t *)hw;
    if (!h || !h->htim) return;
    HAL_TIM_PWM_Stop(h->htim, h->channel);
}

static const DcMotor_Port_t s_stm32_port = {
    .set_pwm_duty = set_duty,
    .start_pwm    = start_pwm,
    .stop_pwm     = stop_pwm,
};

const DcMotor_Port_t *DcMotor_GetStm32Port(void)
{
    return &s_stm32_port;
}
