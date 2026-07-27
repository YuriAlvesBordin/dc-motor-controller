/**
 * @file    stm32_hal_motor_config.h
 * @brief   Peripheral configuration for the single-motor STM32 HAL.
 *
 * @details Edit the defines below to match the peripherals configured
 *          in CubeMX. Handle names (htim1, htim3, etc.) must match
 *          those generated in main.c.
 */

#ifndef STM32_HAL_MOTOR_CONFIG_H
#define STM32_HAL_MOTOR_CONFIG_H

#include "main.h"
#include "tim.h"
#include "app.h"

/**
 * @brief PWM timer handle (TIM1 CH2 on PB3).
 */
#define STM32_MOTOR_PWM_TIM           (&htim1)

/**
 * @brief PWM timer channel.
 */
#define STM32_MOTOR_PWM_CHANNEL       TIM_CHANNEL_2

/**
 * @brief PWM resolution: ARR value configured in CubeMX (TIM1 ARR = 3200).
 */
#define STM32_MOTOR_PWM_RESOLUTION    (3200u)

/**
 * @brief Pulses per revolution of the encoder (PPR).
 */
#define STM32_MOTOR_PPR                 (10u)

#endif
