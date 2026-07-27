/**
 * @file    stm32_hal_motor.c
 * @brief   STM32 HAL motor driver implementation.
 */

#include "stm32_hal_motor.h"
#include "stm32_hal_motor_config.h"
#include <string.h>

/**
 * @brief Apply PWM duty cycle to the motor timer.
 *
 * @param cmd_pct Commanded output in percent (0..100). Values outside this
 *                range are clamped.
 */
void apply_pwm(float cmd_pct)
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

    duty = (uint32_t)((cmd_pct * (float)STM32_MOTOR_PWM_RESOLUTION) / 100.0f);
    __HAL_TIM_SET_COMPARE(STM32_MOTOR_PWM_TIM, STM32_MOTOR_PWM_CHANNEL, duty);
}

/**
 * @brief Initialise the motor HAL.
 *
 * @details Zeroes the motor instance, initialises the control library,
 *          starts the PWM timer, and sets initial duty to zero.
 *
 * @param s_motor Motor instance to initialise (passed by value, updated
 *                in-place).
 */
void stm32_hal_motor_init(dc_motor_t s_motor)
{
    memset(&s_motor, 0, sizeof(s_motor));

    dc_motor_init(&s_motor);

    HAL_TIM_PWM_Start(STM32_MOTOR_PWM_TIM, STM32_MOTOR_PWM_CHANNEL);
    __HAL_TIM_SET_COMPARE(STM32_MOTOR_PWM_TIM, STM32_MOTOR_PWM_CHANNEL, 0u);
}