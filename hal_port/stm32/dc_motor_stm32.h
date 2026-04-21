#ifndef DC_MOTOR_STM32_H
#define DC_MOTOR_STM32_H

#include "dc_motor_port.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
} DcMotor_Stm32Hw_t;

const DcMotor_Port_t *DcMotor_GetStm32Port(void);

#ifdef __cplusplus
}
#endif

#endif /* DC_MOTOR_STM32_H */
