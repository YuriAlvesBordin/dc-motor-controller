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
 * @brief PWM resolution: ARR value configured in CubeMX (TIM1 ARR = 999).
 */
#define STM32_MOTOR_PWM_RESOLUTION    (999u)

/**
 * @brief RPM calculator handle (uses TIM1 CH1 input capture via FreqCalc).
 *        Defined in app.c, initialized in main.c before stm32_hal_motor_init().
 */
#define STM32_MOTOR_RPM_HANDLE        (&app.s_rpm_calc)

#endif
