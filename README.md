# dc-motor-controller

Portable DC motor controller library in C — closed-loop PID, open-loop S-Curve ramp, stall watchdog, autonomous PID auto-tuner, and a clean HAL abstraction. STM32 port included.

Designed for real embedded systems: no dynamic allocation, no RTOS dependency, no global state. Drop the `core/` folder into any project and wire up three callbacks.

---

## Features

| Feature | Description |
|---|---|
| **Closed-loop PID** | Tracks an RPM setpoint with configurable PID gains |
| **Open-loop S-Curve ramp** | Smooth acceleration/deceleration without encoder feedback |
| **Stall watchdog** | Detects motor stall and latches a fault state |
| **PID auto-tuner** | Relay feedback (Åström-Hägglund) + Tyreus-Luyben rules — finds Kp/Ki/Kd autonomously |
| **HAL abstraction** | Core logic is 100% platform-agnostic via Port/Adapter pattern |
| **STM32 HAL port** | Ready-to-use port for STM32 PWM timers |

---

## Directory Structure

```
dc-motor-controller/
├── core/
│   ├── dc_motor_types.h        # Enums, status codes, config structs
│   ├── dc_motor_pid.h/.c       # PID state + compute logic
│   ├── dc_motor_ramp.h/.c      # S-Curve ramp logic
│   ├── dc_motor_safety.h/.c    # Stall watchdog + fault latch
│   ├── dc_motor_autotune.h/.c  # Autonomous PID auto-tuner
│   ├── dc_motor.h              # Public API + handle definition
│   └── dc_motor.c              # Orchestrator — delegates to subsystems
└── hal_port/
    ├── dc_motor_port.h         # HAL contract (3 function pointers)
    └── stm32/
        ├── dc_motor_stm32.h
        └── dc_motor_stm32.c
```

`core/` has zero platform dependencies — only `<stdint.h>`, `<stdbool.h>` and `<math.h>`.
The only file that touches STM32 HAL is `hal_port/stm32/dc_motor_stm32.c`.

---

## Quick Start (STM32)

```c
#include "core/dc_motor.h"
#include "hal_port/stm32/dc_motor_stm32.h"

DcMotor_Handle_t  motor;
DcMotor_Stm32Hw_t hw = { .htim = &htim2, .channel = TIM_CHANNEL_1 };

// NULL = use built-in defaults for PID, ramp and safety
DcMotor_Init(&motor, DcMotor_GetStm32Port(), &hw, NULL, NULL, NULL);

// Open-loop: set duty cycle directly
DcMotor_SetSpeed(&motor, 60.0f);       /* 60 % duty */

// Closed-loop: set RPM target
DcMotor_SetRpmSetpoint(&motor, 1200.0f);

// Call every 10 ms from a timer ISR or RTOS task
void motor_task(void)
{
    float rpm = encoder_get_rpm();
    DcMotor_Update(&motor, HAL_GetTick(), rpm);
}
```

---

## Configuration

Pass custom structs to `DcMotor_Init()`, or `NULL` to use the defaults below.
All defaults can be overridden at compile time with `-D` flags.

### PID — `DcMotor_PidConfig_t`

| Field | Default | Description |
|---|---|---|
| `kp` | 0.1 | Proportional gain |
| `ki` | 0.01 | Integral gain |
| `kd` | 0.005 | Derivative gain |
| `dt_s` | 0.01 | Sample period (s) — must match your `Update()` call rate |
| `output_min / max` | 0.0 / 1.0 | Output clamp |
| `integral_min / max` | −0.5 / 0.5 | Anti-windup clamp |
| `deriv_filter_alpha` | 0.3 | Derivative low-pass (0 = max filtering, 1 = no filter) |

### Ramp — `DcMotor_RampConfig_t`

| Field | Default | Description |
|---|---|---|
| `accel_rate` | 1.0 duty/s | Maximum acceleration rate |
| `decel_rate` | 2.0 duty/s | Maximum deceleration rate |
| `smooth_alpha` | 0.5 | S-curve shape (1.0 = linear ramp) |

### Safety — `DcMotor_SafetyConfig_t`

| Field | Default | Description |
|---|---|---|
| `stall_timeout_ms` | 500 | Time at low RPM before stall fault (0 = disabled) |
| `stall_min_rpm` | 30.0 | RPM threshold for stall detection |
| `min_duty_for_stall` | 0.1 | Duty below this value skips stall check |

---

## PID Auto-Tuner

The auto-tuner finds optimal PID gains autonomously using **relay feedback**
(Åström-Hägglund method) and **Tyreus-Luyben** gain rules — which produce
low overshoot and robust closed-loop behaviour.

### How it works

```
WARMUP → RELAY → CALC → DONE
           └→ ERR_NO_OSCILLATION | ERR_STALL | ERR_TIMEOUT
```

1. **WARMUP** — ramps the motor to ~50 % of the target RPM
2. **RELAY** — replaces PID with a bang-bang relay; detects oscillation peaks/valleys over N cycles
3. **CALC** — computes `Ku` and `Tu`, then applies Tyreus-Luyben:

```
Ku = 4d / (π · a)       d = relay duty step, a = oscillation half-amplitude
Kp = Ku / 3.2
Ki = Kp / (2.87 · Tu)
Kd = Kp · Tu / 11.4
```

4. **DONE** — calls your callback with the ready-to-use `DcMotor_PidConfig_t`

### Usage

```c
#include "core/dc_motor_autotune.h"

static DcMotor_AutotuneCtx_t at_ctx;

void on_autotune_done(const DcMotor_PidConfig_t *cfg, void *user)
{
    DcMotor_SetPidConfig(&motor, cfg);  // apply immediately
    // optionally persist cfg to flash
}

void start_autotune(void)
{
    DcMotor_AutotuneCfg_t cfg;
    DcMotor_Autotune_DefaultConfig(&cfg);
    cfg.rpm_target = 1500.0f;   // representative operating point
    cfg.dt_s       = 0.01f;     // same as your Update() period

    DcMotor_Autotune_Start(&at_ctx, &motor, &cfg, on_autotune_done, NULL);
}

// In your 10 ms timer / RTOS task — REPLACES DcMotor_Update() while tuning:
void motor_task(void)
{
    float rpm = encoder_get_rpm();
    uint32_t t = HAL_GetTick();

    DcMotor_AutotuneState_t s = DcMotor_Autotune_Update(&at_ctx, t, rpm);

    if (s == DC_AUTOTUNE_DONE || s >= DC_AUTOTUNE_ERR_BASE)
        DcMotor_Update(&motor, t, rpm);  // resume normal PID control
}
```

### Auto-tune configuration — `DcMotor_AutotuneCfg_t`

| Field | Default | Description |
|---|---|---|
| `rpm_target` | — | **Required.** Representative operating RPM |
| `relay_amp` | 50 RPM | Relay switching amplitude around `rpm_target` |
| `relay_duty_step` | 0.15 | Relay output step size (the `d` in `Ku = 4d/πa`) |
| `relay_timeout_ms` | 30 000 | Maximum relay phase duration before abort |
| `dt_s` | 0.01 | Must match `Update()` call period |

---

## HAL Contract

The HAL port (`dc_motor_port.h`) requires three function pointers:

| Callback | Signature | Purpose |
|---|---|---|
| `set_pwm_duty` | `void(void *hw, float duty_norm)` | Write normalised duty [0..1] to PWM peripheral |
| `start_pwm` | `void(void *hw)` | Start PWM output (optional — may be NULL) |
| `stop_pwm` | `void(void *hw)` | Stop PWM output (optional — may be NULL) |

---

## Porting to Other Platforms

1. Implement `set_pwm_duty` — write a `0.0–1.0` normalised duty to your PWM peripheral.
2. Optionally implement `start_pwm` / `stop_pwm`.
3. Pass the resulting `DcMotor_Port_t` to `DcMotor_Init()`.

The core never changes. Only the HAL files are platform-specific.

---

## Subsystem Testing

Because each subsystem is a standalone module, you can unit-test them independently:

```c
// Test the PID in isolation — no motor, no HAL
DcMotor_PidConfig_t cfg;
DcMotor_Pid_DefaultConfig(&cfg);

DcMotor_PidState_t st = {0};
float out = DcMotor_Pid_Compute(&cfg, &st, 0.01f, 1500.0f, 1200.0f);

// Test the ramp in isolation
DcMotor_RampConfig_t rcfg;
DcMotor_Ramp_DefaultConfig(&rcfg);
float next = DcMotor_Ramp_Step(&rcfg, 0.0f, 0.8f, 0.01f);
```

---

## License

MIT
