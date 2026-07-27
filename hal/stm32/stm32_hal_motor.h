/**
 * @file    stm32_hal_motor.h
 * @brief   Single-motor STM32 HAL public API.
 *
 * @details This HAL targets a single unidirectional DC motor driven by
 *          one PWM channel. RPM measurement is the caller's responsibility:
 *          the application measures RPM externally (e.g. via RpmSensor) and
 *          injects the value each control cycle with stm32_hal_motor_set_measured().
 *
 *          Typical integration:
 *          - Call stm32_hal_motor_init() once after HAL_Init() and
 *            all MX_TIMx_Init() calls.
 *          - Call stm32_hal_motor_1ms_tick() from the 1 ms SysTick handler.
 *          - Each control period: compute RPM, call stm32_hal_motor_set_measured(),
 *            then call stm32_hal_motor_control_tick().
 */

#ifndef STM32_HAL_MOTOR_H
#define STM32_HAL_MOTOR_H

#include "dc_motor.h"
#include "dc_motor_pid.h"
#include "stm32_hal_motor_config.h"

/**
 * @brief Initialise the motor HAL.
 *
 * @details Zeroes the motor instance, initialises the control library,
 *          starts the PWM timer, and sets initial duty to zero.
 *
 * @param s_motor Motor instance to initialise (passed by value, updated
 *                in-place).
 */
void stm32_hal_motor_init(dc_motor_t s_motor);

/**
 * @brief Apply PWM duty cycle to the motor timer.
 *
 * @param cmd_pct Commanded output in percent (0..100). Values outside this
 *                range are clamped.
 */
void apply_pwm(float cmd_pct);

#endif /* STM32_HAL_MOTOR_H */