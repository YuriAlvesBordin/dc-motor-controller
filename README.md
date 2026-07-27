# dc-motor-controller

A portable, HAL-agnostic C library for DC motor control, designed for resource-constrained embedded systems.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Language: C](https://img.shields.io/badge/language-C99-blue.svg)]()
[![Standard: MISRA-inspired](https://img.shields.io/badge/style-MISRA--inspired-lightgrey.svg)]()

---

## Features

- **Open-loop mode** — direct percent command with optional slew-rate ramp
- **Closed-loop mode** — PID controller with:
  - Anti-windup clamping
  - Derivative-on-measurement (eliminates derivative kick)
  - Configurable derivative low-pass filter
  - **Feed-forward control** — velocity feed-forward (kff) + static offset (kff_static) for improved setpoint tracking and stiction compensation
  - Runtime gain tuning via `dc_motor_pid_tune()` and `dc_motor_pid_set_feedforward()`
- **Acceleration / deceleration ramp** — shared by both modes, independent accel and decel rates, correct bidirectional behaviour
- **Hardware watchdog** — forces output to zero if the control loop stalls
- **Zero dynamic memory allocation** — all state lives in host-declared `dc_motor_t` structs
- **Zero global variables** — fully reentrant, safe for multi-task RTOS use
- **Dead-code elimination** — every optional feature is guarded by `DC_MOTOR_ENABLE_*` macros; disabled features produce no object code and no RAM usage
- **STM32 HAL example** — ready-to-use integration for STM32 timers (PWM + encoder), portable across all STM32 series

---

## Repository Layout

```
dc-motor-controller/
├── include/                   # Public API (add this path to your include dirs)
│   ├── dc_motor.h             # Top-level header — only file the application needs
│   ├── dc_motor_config.h      # Compile-time feature flags and default values
│   └── dc_motor_types.h       # Shared enumerations and status codes
├── internal/                  # Private module headers (do not include directly)
│   ├── dc_motor_pid.h
│   ├── dc_motor_ramp.h
│   ├── dc_motor_openloop.h
│   └── dc_motor_closedloop.h
├── src/                       # Implementation files (add all *.c to your build)
│   ├── dc_motor.c
│   ├── dc_motor_pid.c
│   ├── dc_motor_ramp.c
│   ├── dc_motor_openloop.c
│   └── dc_motor_closedloop.c
└── hal/
    └── stm32/                 # Optional STM32 HAL integration
        ├── stm32_hal_motor.c
        └── stm32_hal_motor_config.h
```

---

## Quick Start

### 1. Add to your build

Add `include/` and `internal/` to your compiler include paths, then compile every `.c` file under `src/`.

**Makefile example:**
```makefile
INC  += -Ipath/to/dc-motor-controller/include
INC  += -Ipath/to/dc-motor-controller/internal
SRC  += $(wildcard path/to/dc-motor-controller/src/*.c)
```

**CMake example:**
```cmake
target_include_directories(your_target PRIVATE
    dc-motor-controller/include
    dc-motor-controller/internal
)
file(GLOB DC_MOTOR_SRC dc-motor-controller/src/*.c)
target_sources(your_target PRIVATE ${DC_MOTOR_SRC})
```

### 2. Configure features

Edit `include/dc_motor_config.h` or pass flags on the command line:

```sh
# Enable open-loop mode, disable watchdog
-DDC_MOTOR_ENABLE_OPENLOOP=1 -DDC_MOTOR_ENABLE_WATCHDOG=0
```

### 3. Use the API

```c
#include "dc_motor.h"

dc_motor_t motor;

int main(void)
{
    dc_motor_init(&motor);

    /* Closed-loop: track 1500 RPM */
    dc_motor_set_closedloop(&motor, 1500.0f);

    /* Main control loop — call at DC_MOTOR_CONTROL_PERIOD_SEC intervals */
    while (1)
    {
        float measured_rpm = read_encoder_rpm();
        dc_motor_set_measured(&motor, measured_rpm);

        float duty_pct = dc_motor_update(&motor, DC_MOTOR_CONTROL_PERIOD_SEC);
        apply_pwm(duty_pct);

        /* Feed watchdog from 1 ms SysTick */
        dc_motor_tick(&motor, 1u);
    }
}
```

---

## Configuration Reference

All options are defined in `include/dc_motor_config.h` and can be overridden via compiler flags.

### Feature Flags

| Macro | Default | Description |
|---|---|---|
| `DC_MOTOR_ENABLE_OPENLOOP` | `0` | Enable open-loop (ramped direct command) mode |
| `DC_MOTOR_ENABLE_CLOSEDLOOP` | `1` | Enable closed-loop PID mode |
| `DC_MOTOR_ENABLE_RAMP` | `0` | Enable slew-rate limiter (shared by both modes) |
| `DC_MOTOR_ENABLE_PID_ANTI_WINDUP` | `1` | Enable integrator anti-windup clamping |
| `DC_MOTOR_ENABLE_PID_FEEDFORWARD` | `1` | Enable PID feed-forward control (kff, kff_static) |
| `DC_MOTOR_ENABLE_PID_D_FILTER` | `1` | Enable derivative low-pass filter |
| `DC_MOTOR_PID_DERIV_ON_MEASUREMENT` | `1` | Compute D term on measurement instead of error |
| `DC_MOTOR_ENABLE_WATCHDOG` | `1` | Enable control-loop watchdog |

### Timing

| Macro | Default | Description |
|---|---|---|
| `DC_MOTOR_CONTROL_PERIOD_SEC` | `0.001f` | Control loop period in seconds (1 ms) |

### PID Defaults

| Macro | Default | Description |
|---|---|---|
| `DC_MOTOR_PID_DEFAULT_KP` | `0.5` | Proportional gain |
| `DC_MOTOR_PID_DEFAULT_KI` | `1.0` | Integral gain (per second) |
| `DC_MOTOR_PID_DEFAULT_KD` | `0.1` | Derivative gain (per second) |
| `DC_MOTOR_PID_DEFAULT_D_FILTER` | `0.01` | D-term EMA filter coefficient (0 = max smoothing) |
| `DC_MOTOR_PID_DEFAULT_KFF` | `0.01` | Velocity feed-forward gain (output per unit setpoint) |
| `DC_MOTOR_PID_DEFAULT_KFF_STATIC` | `0.1` | Static feed-forward offset (overcomes stiction) |
| `DC_MOTOR_PID_DEFAULT_OUT_MIN` | `25.0` | Output lower bound (percent) |
| `DC_MOTOR_PID_DEFAULT_OUT_MAX` | `100.0` | Output upper bound (percent) |

### Ramp Defaults

| Macro | Default | Description |
|---|---|---|
| `DC_MOTOR_RAMP_DEFAULT_ACCEL` | `200.0` | Acceleration limit (percent/second) |
| `DC_MOTOR_RAMP_DEFAULT_DECEL` | `400.0` | Deceleration limit (percent/second) |

### Safety

| Macro | Default | Description |
|---|---|---|
| `DC_MOTOR_WATCHDOG_TIMEOUT_MS` | `50` | Watchdog timeout in milliseconds |

---

## API Reference

### Lifecycle

```c
dc_motor_status_t dc_motor_init(dc_motor_t *m);
dc_motor_status_t dc_motor_stop(dc_motor_t *m);
```

### Commands

```c
dc_motor_status_t dc_motor_set_openloop(dc_motor_t *m, float level);
dc_motor_status_t dc_motor_set_closedloop(dc_motor_t *m, float setpoint);
dc_motor_status_t dc_motor_set_measured(dc_motor_t *m, float measured);
```

### Control Loop

```c
float             dc_motor_update(dc_motor_t *m, float dt);
dc_motor_status_t dc_motor_tick(dc_motor_t *m, uint32_t elapsed_ms);
dc_motor_status_t dc_motor_feed_watchdog(dc_motor_t *m);

/* PID runtime tuning (closed-loop mode only) */
dc_motor_status_t dc_motor_pid_tune(dc_motor_t *m, float kp, float ki, float kd);
dc_motor_status_t dc_motor_pid_set_feedforward(dc_motor_t *m, float kff, float kff_static);
```

### Getters

```c
dc_motor_mode_t   dc_motor_get_mode(const dc_motor_t *m);
float             dc_motor_get_output(const dc_motor_t *m);
float             dc_motor_get_measured(const dc_motor_t *m);
```

### Status Codes

| Code | Meaning |
|---|---|
| `DC_MOTOR_OK` | Operation succeeded |
| `DC_MOTOR_ERR_NULL` | NULL pointer passed |
| `DC_MOTOR_ERR_MODE` | Operation invalid for current mode |
| `DC_MOTOR_ERR_DISABLED` | Feature disabled at compile time |
| `DC_MOTOR_ERR_RANGE` | Numerical argument out of range |
| `DC_MOTOR_ERR_WATCHDOG` | Watchdog tripped, output forced to zero |

---

## STM32 HAL Integration

The `hal/stm32/` directory provides a ready-to-use integration for STM32 microcontrollers using the STM32 HAL library (CubeMX-generated projects).

### Hardware model

- **Unidirectional** — single PWM channel, no direction pin
- **Encoder feedback** — quadrature encoder via timer in encoder mode (16-bit counter with automatic rollover handling)
- **Watchdog** — fed from `SysTick` via `stm32_hal_motor_1ms_tick()`

### Configuration (`stm32_hal_motor_config.h`)

```c
/* Number of motors */
#define STM32_MOTOR_COUNT          (1u)

/* HAL header — override for other STM32 series */
#define STM32_HAL_INCLUDE          "stm32f4xx_hal.h"

/* Motor 0 peripherals */
#define STM32_MOTOR0_PWM_TIM       (&htim1)
#define STM32_MOTOR0_PWM_CHANNEL   TIM_CHANNEL_1
#define STM32_MOTOR0_PWM_RESOLUTION (3200u)   /* ARR value from CubeMX */
#define STM32_MOTOR0_ENCODER_TIM   (&htim3)
#define STM32_MOTOR0_ENCODER_PPR   (10u)      /* Encoder pulses per revolution */
```

To port to a different STM32 series without editing the file:
```sh
-DSTM32_HAL_INCLUDE='"stm32g0xx_hal.h"'
```

### Usage

```c
#include "stm32_hal_motor.h"

/* Call once after HAL_Init() and MX_TIMx_Init() */
stm32_hal_motor_init();

/* Set target speed */
stm32_hal_motor_set_target_rpm(MOTOR_ID_0, 1500.0f);

/* Call from your 1 ms SysTick handler */
void HAL_SYSTICK_Callback(void)
{
    stm32_hal_motor_1ms_tick();
}

/* Call from your control loop timer ISR (period = DC_MOTOR_CONTROL_PERIOD_SEC) */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htimX)
        stm32_hal_motor_control_tick(MOTOR_ID_0);
}
```

---

## Design Principles

- **Single include** — the application only needs `#include "dc_motor.h"`; all internals are hidden
- **No heap, no globals** — the library is safe to use in any RTOS or bare-metal context
- **Compile-time feature selection** — unused modules produce zero object code and zero BSS; verified by inspecting `.map` files
- **Consistent error handling** — every function that can fail returns `dc_motor_status_t`; only `dc_motor_update()` and the ramp/PID compute functions return `float` (compute functions return `0.0f` on invalid input)
- **Doxygen-ready** — all public headers are fully documented with `@param`, `@return`, and `@details` tags

---

## Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feat/your-feature`
3. Keep all documentation in `.h` files only; no inline comments in `.c` files
4. Open a pull request with a clear description of the change and the issue it addresses

---

## License

MIT License. See [LICENSE](LICENSE) for details.
