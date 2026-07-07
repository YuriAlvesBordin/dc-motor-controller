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

/**
 * @brief Header HAL do fabricante a incluir.
 *
 * @details Sobrescreva via -DSTM32_HAL_INCLUDE='"stm32g0xx_hal.h"' para
 *          portar para outra série STM32 sem editar este arquivo.
 */
#ifndef STM32_HAL_INCLUDE
#define STM32_HAL_INCLUDE "stm32f4xx_hal.h"
#endif

#include STM32_HAL_INCLUDE

/**
 * @brief Número de instâncias de motor gerenciadas pela HAL.
 *
 * @details Deve ser igual ao número de motores físicos. Pode ser
 *          sobrescrito via -DSTM32_MOTOR_COUNT=N no sistema de build.
 */
#ifndef STM32_MOTOR_COUNT
#define STM32_MOTOR_COUNT    (1u)
#endif

/**
 * @brief Handle do timer PWM do motor 0 (ex.: TIM1 CH1).
 */
#define STM32_MOTOR0_PWM_TIM           (&htim1)
#define STM32_MOTOR0_PWM_CHANNEL       TIM_CHANNEL_1

/**
 * @brief Resolução do PWM: valor do ARR configurado no CubeMX.
 */
#define STM32_MOTOR0_PWM_RESOLUTION    (1000u)

/**
 * @brief Handle do timer em modo encoder do motor 0 (ex.: TIM3).
 */
#define STM32_MOTOR0_ENCODER_TIM       (&htim3)

/**
 * @brief Número de pulsos por volta do encoder (counts por revolução).
 */
#define STM32_MOTOR0_ENCODER_PPR       (10u)

/**
 * @brief Período do laço de controle em microssegundos.
 *
 * @details Deve bater com DC_MOTOR_CONTROL_PERIOD_SEC × 1 000 000.
 */
#define STM32_CONTROL_LOOP_PERIOD_US   (1000u)

#endif
