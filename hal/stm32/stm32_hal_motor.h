/**
 * @file    stm32_hal_motor.h
 * @brief   Single-motor STM32 HAL public API.
 *
 * @details This HAL targets a single unidirectional DC motor driven by
 *          one PWM channel and measured by one quadrature encoder timer.
 *          All state is internal; the application only calls the functions
 *          declared here.
 *
 *          Typical integration:
 *          - Call stm32_hal_motor_init() once after HAL_Init() and
 *            all MX_TIMx_Init() calls.
 *          - Call stm32_hal_motor_1ms_tick() from the 1 ms SysTick handler.
 *          - Call stm32_hal_motor_control_tick() from the control-loop
 *            timer ISR at the period defined by DC_MOTOR_CONTROL_PERIOD_SEC.
 */

#ifndef STM32_HAL_MOTOR_H
#define STM32_HAL_MOTOR_H

#include "dc_motor.h"

/**
 * @brief Initialise the motor HAL.
 *
 * @details Resets internal state, copies peripheral handles from
 *          stm32_hal_motor_config.h, initialises the dc_motor_t instance,
 *          starts PWM output and sets duty to zero.
 *          Must be called once before any other function in this module.
 */
void stm32_hal_motor_init(void);

/**
 * @brief Set the closed-loop RPM target.
 *
 * @param target_rpm  Desired motor speed in RPM. Must be >= 0 (unidirectional).
 */
void stm32_hal_motor_set_target_rpm(float target_rpm);

/**
 * @brief Stop the motor immediately.
 *
 * @details Resets the dc_motor_t state and forces PWM duty to zero.
 */
void stm32_hal_motor_stop(void);

/**
 * @brief Return the last measured motor speed.
 *
 * @return Motor speed in RPM as reported by the encoder.
 */
float stm32_hal_motor_get_rpm(void);

/**
 * @brief Execute one control-loop iteration.
 *
 * @details Reads encoder RPM, feeds the measurement into dc_motor_t,
 *          calls dc_motor_update() and applies the resulting duty cycle
 *          to the PWM timer. Must be called at exactly
 *          DC_MOTOR_CONTROL_PERIOD_SEC intervals.
 */
void stm32_hal_motor_control_tick(void);

/**
 * @brief Feed the watchdog and advance internal timers.
 *
 * @details Must be called every 1 ms, typically from the SysTick handler.
 */
void stm32_hal_motor_1ms_tick(void);

#endif
