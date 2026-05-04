# dc-motor-controller

Portable DC motor controller library in C — closed-loop PID, open-loop S-Curve ramp, stall watchdog, and a clean HAL abstraction. STM32 port included.

Designed for real embedded systems: no dynamic allocation, no RTOS dependency, no global state. Drop the `core/` folder into any project and wire up three callbacks.

---

## Features

| Feature | Description |
|---|---|
| **Closed-loop PID** | Tracks an RPM setpoint using a configurable PID controller |
| **Open-loop S-Curve ramp** | Smooth acceleration/deceleration without encoder feedback |
| **Stall watchdog** | Detects motor stall and triggers a configurable callback |
| **HAL abstraction** | Core logic is 100% platform-agnostic via Port/Adapter pattern |
| **STM32 HAL port** | Ready-to-use port for STM32 PWM timers |

---

## Directory Structure

```
Core/motor/
├── core/
│   ├── dc_motor.h          # Public API + types
│   └── dc_motor.c          # State-machine + PID + ramp implementation
└── hal_port/
    ├── dc_motor_port.h     # Callback typedefs (HAL contract)
    └── stm32/
        ├── dc_motor_stm32.h
        └── dc_motor_stm32.c
```

`core/` has zero platform dependencies — only `<stdint.h>`, `<stdbool.h>` and `<math.h>`. The only file that touches STM32 HAL is `hal_port/stm32/dc_motor_stm32.c`.

---

## Quick Start (STM32)

```c
#include "core/dc_motor.h"
#include "hal_port/stm32/dc_motor_stm32.h"

DcMotor_Handle_t  motor;
DcMotor_Stm32Hw_t hw = { .htim = &htim2, .channel = TIM_CHANNEL_1 };

DcMotor_Init(&motor, DcMotor_GetStm32Port(), &hw, NULL, NULL, NULL);

// Open-loop: set duty cycle directly
DcMotor_SetSpeed(&motor, 60.0f);  /* 60% duty */

// Closed-loop: set RPM target and update from a periodic task
DcMotor_SetRpmSetpoint(&motor, 1200.0f);

void motor_task(void) {           /* call every 10 ms */
    float rpm = encoder_get_rpm();
    DcMotor_Update(&motor, HAL_GetTick(), rpm);
}
```

---

## Configuration

Pass a custom `DcMotor_Config_t` to `DcMotor_Init` (or `NULL` for defaults):

| Field | Default | Description |
|---|---|---|
| `pid_kp` | 0.8 | PID proportional gain |
| `pid_ki` | 0.2 | PID integral gain |
| `pid_kd` | 0.05 | PID derivative gain |
| `ramp_ms` | 500 | S-Curve ramp duration (ms) |
| `stall_rpm_threshold` | 50.0 | RPM below which stall is detected |
| `stall_timeout_ms` | 800 | Time at low RPM before stall fires |
| `pwm_max` | 100.0 | Maximum duty cycle (%) |

---

## Callbacks

The HAL contract (`dc_motor_port.h`) requires three function pointers:

| Callback | Signature | Purpose |
|---|---|---|
| `set_pwm_fn` | `void(void *hw, float duty)` | Write duty cycle to hardware |
| `get_tick_fn` | `uint32_t(void)` | Return current millisecond tick |
| `on_stall_fn` | `void(void *hw)` | Called when stall is detected (optional) |

---

## Porting to Other Platforms

1. Implement `set_pwm_fn` — write a `0.0–100.0` duty cycle to your PWM peripheral.
2. Implement `get_tick_fn` — return a millisecond counter (e.g. `millis()` on Arduino).
3. Optionally implement `on_stall_fn` — handle stall events (stop, alert, retry).
4. Pass the resulting `DcMotor_Port_t` to `DcMotor_Init`.

The core never changes. Only the HAL files are platform-specific.

---

## License

MIT
