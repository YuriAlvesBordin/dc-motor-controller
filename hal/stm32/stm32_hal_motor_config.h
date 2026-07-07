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

/**
 * @brief STM32 HAL header to include.
 *
 * @details Override via -DSTM32_HAL_INCLUDE='"stm32g0xx_hal.h"' to
 *          port to a different STM32 series without editing this file.
 */
#ifndef STM32_HAL_INCLUDE
#define STM32_HAL_INCLUDE "stm32f4xx_hal.h"
#endif

#include STM32_HAL_INCLUDE

/**
 * @brief PWM timer handle (e.g. TIM1 CH1).
 */
#define STM32_MOTOR_PWM_TIM           (&htim1)

/**
 * @brief PWM timer channel.
 */
#define STM32_MOTOR_PWM_CHANNEL       TIM_CHANNEL_1

/**
 * @brief PWM resolution: ARR value configured in CubeMX.
 */
#define STM32_MOTOR_PWM_RESOLUTION    (1000u)

/**
 * @brief Encoder timer handle configured in encoder mode (e.g. TIM3).
 */
#define STM32_MOTOR_ENCODER_TIM       (&htim3)

/**
 * @brief Encoder pulses per revolution (counts per revolution).
 */
#define STM32_MOTOR_ENCODER_PPR       (10u)

#endif
