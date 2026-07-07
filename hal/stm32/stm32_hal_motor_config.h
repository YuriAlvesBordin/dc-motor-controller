/**
 * @file    stm32_hal_motor_config.h
 * @brief   Configuração de periféricos para a camada HAL STM32.
 *
 * @details Ajuste estes defines para casar com os periféricos configurados
 *          no CubeMX. Os nomes dos handles (htim1, htim3, etc.) devem bater
 *          com os gerados no main.c.
 */

#ifndef STM32_HAL_MOTOR_CONFIG_H
#define STM32_HAL_MOTOR_CONFIG_H

#include "stm32f4xx_hal.h"

/* ======================  MOTOR 0  ====================== */

/**
 * @brief Handle do timer PWM do motor 0 (ex.: TIM1 CH1).
 */
#define STM32_MOTOR0_PWM_TIM           (&htim1)
#define STM32_MOTOR0_PWM_CHANNEL       TIM_CHANNEL_1

/**
 * @brief Resolução do PWM em bits (ex.: ARR+1 = 2^N).
 */
#define STM32_MOTOR0_PWM_RESOLUTION    (1000u)

/**
 * @brief Handle do timer em modo encoder do motor 0 (ex.: TIM3).
 */
#define STM32_MOTOR0_ENCODER_TIM       (&htim3)

/**
 * @brief Número de pulsos por volta do encoder.
 *
 */
#define STM32_MOTOR0_ENCODER_PPR       (10u)

/* ======================  CONVERSÃO  ==================== */

/**
 * @brief Período do laço de controle em microssegundos.
 *
 * @details Deve bater com DC_MOTOR_CONTROL_PERIOD_SEC.
 */
#define STM32_CONTROL_LOOP_PERIOD_US   (1000u)

#endif /* STM32_HAL_MOTOR_CONFIG_H */
