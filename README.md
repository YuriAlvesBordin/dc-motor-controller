# dc-motor-controller

Portable DC motor controller library in C — closed-loop PID, open-loop S-Curve ramp, stall watchdog, and a clean HAL abstraction. STM32 port included.

## Structure

```
Core/motor/
├── core/
│   ├── dc_motor.h
│   └── dc_motor.c
└── hal_port/
    ├── dc_motor_port.h
    └── stm32/
        ├── dc_motor_stm32.h
        └── dc_motor_stm32.c
```

`core/` has zero platform dependencies — only `<stdint.h>`, `<stdbool.h>` and `<math.h>`. The only file that touches STM32 HAL is `hal_port/stm32/dc_motor_stm32.c`.

## Usage (STM32)

```c
#include "core/dc_motor.h"
#include "hal_port/stm32/dc_motor_stm32.h"

DcMotor_Handle_t  motor;
DcMotor_Stm32Hw_t hw = { .htim = &htim2, .channel = TIM_CHANNEL_1 };

DcMotor_Init(&motor, DcMotor_GetStm32Port(), &hw, NULL, NULL, NULL);

// open-loop
DcMotor_SetSpeed(&motor, 60.0f);  /* 60% */

// closed-loop — call DcMotor_Update from a 10 ms task and pass the latest RPM
DcMotor_SetRpmSetpoint(&motor, 1200.0f);

void motor_task(void) {
    float rpm = encoder_get_rpm();
    DcMotor_Update(&motor, HAL_GetTick(), rpm);
}
```

## Porting

Implement the three callbacks in `dc_motor_port.h` for your hardware and pass the resulting `DcMotor_Port_t` to `DcMotor_Init`. The core logic never changes.

## License

MIT
