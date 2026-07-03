# dc-motor-controller

Portable DC motor controller library written in C99.  
Provides closed-loop PID, open-loop S-curve ramping, stall watchdog, autonomous relay-feedback auto-tuner, and a clean HAL abstraction layer.

Designed for real embedded systems: no dynamic allocation, no RTOS dependency, no global state.

---

## Features

| Feature | Description |
|---|---|
| Closed-loop PID | Tracks an RPM setpoint with configurable gains and anti-windup |
| Open-loop S-curve ramp | Smooth acceleration / deceleration without encoder feedback |
| Stall watchdog | Detects motor stall and latches a fault that must be explicitly cleared |
| PID auto-tuner | Relay feedback (Åström–Hägglund) + Tyreus–Luyben rules |
| HAL abstraction | Core is 100 % platform-agnostic via three function pointers |
| STM32 HAL port | Ready-to-use port for STM32 PWM timers |
| Compile-time configuration | Each subsystem can be individually enabled or stripped at build time |

---

## Directory Layout

```
dc-motor-controller/
├── include/
│   ├── dc_motor_config.h     # Compile-time feature flags and default values
│   ├── dc_motor.h            # Public API and handle definition
│   ├── dc_motor_types.h      # Config structs, enums, status codes
│   ├── dc_motor_port.h       # HAL contract (3 function pointers)
│   └── dc_motor_autotune.h   # Optional auto-tuner API
├── src/
│   ├── dc_motor.c            # Orchestrator
│   ├── dc_motor_pid.c        # PID compute logic
│   ├── dc_motor_ramp.c       # S-curve ramp logic
│   ├── dc_motor_safety.c     # Stall watchdog
│   ├── dc_motor_autotune.c   # Auto-tuner
│   └── internal/             # Private headers — do not include directly
│       ├── dc_motor_types.h
│       ├── dc_motor_pid.h
│       ├── dc_motor_ramp.h
│       └── dc_motor_safety.h
└── hal_port/
    ├── dc_motor_port.h       # Redirect shim for legacy includes
    └── stm32/
        ├── dc_motor_stm32.h
        └── dc_motor_stm32.c
```

Application code must only include headers from `include/`.  
Headers under `src/internal/` are private implementation details.

**Compiler include paths required:**
```
-I path/to/dc-motor-controller/include
```

---

## Integration

Add all `.c` files from `src/` to your build. For STM32, also add `hal_port/stm32/dc_motor_stm32.c`.

**GCC example:**
```sh
gcc -Idc-motor-controller/include \
    dc-motor-controller/src/dc_motor.c \
    dc-motor-controller/src/dc_motor_pid.c \
    dc-motor-controller/src/dc_motor_ramp.c \
    dc-motor-controller/src/dc_motor_safety.c \
    your_app.c -o app
```

If using the auto-tuner, also add `src/dc_motor_autotune.c` and link with `-lm`.

---

## Compile-Time Configuration

All build-time knobs live in `include/dc_motor_config.h`. Every value can be overridden without editing the file by passing `-D` flags to the compiler.

### Feature Flags

Each subsystem can be independently enabled or stripped from the binary.

| Flag | Default | Effect when set to `0` |
|---|---|---|
| `DC_MOTOR_ENABLE_PID` | `1` | Removes PID compute, closed-loop path, and PID state from the handle. `SetRpmSetpoint` returns `DC_MOTOR_ERR_PARAM`. |
| `DC_MOTOR_ENABLE_RAMP` | `1` | Removes S-curve ramp. `SetDuty` writes the target directly to PWM without ramping. |
| `DC_MOTOR_ENABLE_SAFETY` | `1` | Removes stall watchdog. `ClearFault` and safety state are no-ops. |
| `DC_MOTOR_ENABLE_AUTOTUNE` | `1` | Removes the auto-tuner and its `<math.h>` dependency. |

**Example — bare-minimum open-loop build (no encoder, no watchdog):**
```sh
gcc -Idc-motor-controller/include \
    -DDC_MOTOR_ENABLE_PID=0 \
    -DDC_MOTOR_ENABLE_SAFETY=0 \
    -DDC_MOTOR_ENABLE_AUTOTUNE=0 \
    dc-motor-controller/src/dc_motor.c \
    dc-motor-controller/src/dc_motor_ramp.c \
    your_app.c -o app
```

When a subsystem is disabled, its `.c` file can still be included in the build — it compiles to an empty translation unit.

### Default Value Overrides

All numeric defaults are defined in `dc_motor_config.h` and follow the same `#ifndef` / `#define` pattern, so any of them can be overridden with a `-D` flag.

**PID defaults:**

| Flag | Default | Description |
|---|---|---|
| `DC_MOTOR_DEFAULT_KP` | `0.1` | Proportional gain |
| `DC_MOTOR_DEFAULT_KI` | `0.01` | Integral gain |
| `DC_MOTOR_DEFAULT_KD` | `0.005` | Derivative gain |
| `DC_MOTOR_DEFAULT_DT_S` | `0.01` | Nominal sample period (s) |
| `DC_MOTOR_DERIV_FILTER_ALPHA` | `0.3` | Derivative low-pass coefficient |

**Ramp defaults:**

| Flag | Default | Description |
|---|---|---|
| `DC_MOTOR_DEFAULT_ACCEL_RATE` | `1.0` | Acceleration rate (duty/s) |
| `DC_MOTOR_DEFAULT_DECEL_RATE` | `2.0` | Deceleration rate (duty/s) |
| `DC_MOTOR_RAMP_SMOOTH_ALPHA` | `0.5` | S-curve blending factor (1.0 = linear) |

**Safety defaults:**

| Flag | Default | Description |
|---|---|---|
| `DC_MOTOR_DEFAULT_STALL_TIMEOUT_MS` | `500` | Time at low RPM before fault is latched |
| `DC_MOTOR_DEFAULT_STALL_MIN_RPM` | `30.0` | RPM threshold for stall detection |
| `DC_MOTOR_DEFAULT_MIN_DUTY_FOR_STALL` | `0.1` | Duty below this skips the stall check |

**Auto-tuner defaults:**

| Flag | Default | Description |
|---|---|---|
| `DC_AUTOTUNE_DEFAULT_RELAY_AMP` | `50.0` | Relay switching amplitude (RPM) |
| `DC_AUTOTUNE_DEFAULT_RELAY_DUTY_STEP` | `0.15` | Relay output step size |
| `DC_AUTOTUNE_DEFAULT_TIMEOUT_MS` | `30000` | Maximum relay phase duration (ms) |
| `DC_AUTOTUNE_MIN_CYCLES` | `4` | Minimum oscillation cycles before `CALC` |
| `DC_AUTOTUNE_WARMUP_MS` | `2000` | Warmup phase duration (ms) |

---

## Quick Start

```c
#include "dc_motor.h"
#include "dc_motor_stm32.h"

DcMotor_Handle_t  motor;
DcMotor_Stm32Hw_t hw = { .htim = &htim2, .channel = TIM_CHANNEL_1 };

/* NULL = use built-in defaults for all subsystems */
DcMotor_Init(&motor, DcMotor_GetStm32Port(), &hw, NULL, NULL, NULL);

/* Open-loop: 60 % duty cycle */
DcMotor_SetSpeed(&motor, 60.0f);

/* Closed-loop: target 1200 RPM */
DcMotor_SetRpmSetpoint(&motor, 1200.0f);

/* Call every 10 ms from a timer ISR or RTOS task */
void motor_task(void)
{
    float rpm = encoder_get_rpm();
    DcMotor_Update(&motor, HAL_GetTick(), rpm);
}
```

---

## API Reference

### Lifecycle

```c
DcMotor_Status_t DcMotor_Init(
    DcMotor_Handle_t             *hdm,
    const DcMotor_Port_t         *port,
    void                         *hw,
    const DcMotor_PidConfig_t    *pid,     /* NULL = defaults */
    const DcMotor_RampConfig_t   *ramp,    /* NULL = defaults */
    const DcMotor_SafetyConfig_t *safety   /* NULL = defaults */
);
```

Initialises the handle and starts the PWM output at 0 % duty.  
Returns `DC_MOTOR_ERR_NULL_PTR` if `hdm` or `port` is NULL.  
Returns `DC_MOTOR_ERR_PARAM` if any config field is out of range.

---

### Control

```c
DcMotor_Status_t DcMotor_SetDuty       (DcMotor_Handle_t *hdm, float duty);        /* 0.0 – 1.0 */
DcMotor_Status_t DcMotor_SetSpeed      (DcMotor_Handle_t *hdm, float speed_pct);   /* 0 – 100 % */
DcMotor_Status_t DcMotor_SetRpmSetpoint(DcMotor_Handle_t *hdm, float rpm_sp);      /* enables closed-loop */
void             DcMotor_SetClosedLoop  (DcMotor_Handle_t *hdm, bool enabled);
DcMotor_Status_t DcMotor_Stop          (DcMotor_Handle_t *hdm);                    /* graceful ramp-down */
void             DcMotor_EmergencyStop  (DcMotor_Handle_t *hdm);                   /* immediate zero duty */
```

`SetDuty` and `SetSpeed` switch to open-loop mode and apply the target through the S-curve ramp on the next `Update()` call.  
`SetRpmSetpoint` enables closed-loop PID mode.  
`EmergencyStop` bypasses the ramp and writes 0 % duty immediately.

---

### Periodic Update

```c
void DcMotor_Update(DcMotor_Handle_t *hdm, uint32_t tick_ms, float current_rpm);
```

Must be called at a fixed rate (e.g., every 10 ms) from a timer ISR or RTOS task.  
Internally computes `dt` from the difference between `tick_ms` and the previous call timestamp — no fixed sample period assumption.  
Runs the active control loop (PID or ramp) and the stall watchdog.

---

### Fault Handling

```c
DcMotor_Status_t DcMotor_ClearFault(DcMotor_Handle_t *hdm);
void             DcMotor_ResetPid  (DcMotor_Handle_t *hdm);
```

After a stall fault, the motor is locked until `ClearFault()` is called explicitly.  
`ClearFault` transitions the state back to `DC_MOTOR_IDLE` and re-enables the PWM output.

---

### Runtime Configuration Setters

```c
void DcMotor_SetPidConfig   (DcMotor_Handle_t *hdm, const DcMotor_PidConfig_t    *cfg);
void DcMotor_SetRampConfig  (DcMotor_Handle_t *hdm, const DcMotor_RampConfig_t   *cfg);
void DcMotor_SetSafetyConfig(DcMotor_Handle_t *hdm, const DcMotor_SafetyConfig_t *cfg);
```

Allow hot-reconfiguration at runtime without re-initialising the handle.

---

### Status Queries

```c
bool DcMotor_IsRunning   (const DcMotor_Handle_t *hdm);
bool DcMotor_IsFault     (const DcMotor_Handle_t *hdm);
bool DcMotor_IsClosedLoop(const DcMotor_Handle_t *hdm);
```

---

### Status Codes — `DcMotor_Status_t`

| Value | Meaning |
|---|---|
| `DC_MOTOR_OK` | Success |
| `DC_MOTOR_ERR_NULL_PTR` | Required pointer is NULL |
| `DC_MOTOR_ERR_PARAM` | Invalid configuration value |
| `DC_MOTOR_ERR_NO_DATA` | Requested data not available yet |
| `DC_MOTOR_ERR_STALL` | Stall fault detected |
| `DC_MOTOR_ERR_FAULT` | Generic fault state |

---

### State Machine — `DcMotor_State_t`

```
         SetDuty / SetSpeed
IDLE ──────────────────────► RAMPING
  ▲                              │ ramp reaches target
  │                              ▼
  │                          RUNNING
  │                              │
  │       stall timeout          ▼
  │   ◄─────────────────── FAULT_STALL
  │                              │
  └──────── ClearFault() ────────┘
```

---

## PID Auto-Tuner

Finds optimal PID gains autonomously using **relay feedback** (Åström–Hägglund) and **Tyreus–Luyben** gain rules.

### State Machine

```
WARMUP → RELAY → CALC → DONE
               └→ ERR_NO_OSCILLATION | ERR_STALL | ERR_TIMEOUT
```

1. **WARMUP** — drives the motor to 50 % duty for a settling period.
2. **RELAY** — replaces the PID with a bang-bang relay and measures oscillation peaks and valleys over N cycles.
3. **CALC** — computes ultimate gain `Ku` and period `Tu`, then derives PID gains via Tyreus–Luyben:

```
Ku = (4 × d) / (π × a)      where d = relay duty step, a = half-amplitude
Kp = Ku / 3.2
Ki = Kp / (2.87 × Tu)
Kd = Kp × Tu / 11.4
```

4. **DONE** — calls the registered callback with a ready-to-use `DcMotor_PidConfig_t`.

### Usage

```c
#include "dc_motor_autotune.h"

static DcMotor_AutotuneCtx_t at_ctx;

void on_autotune_done(const DcMotor_PidConfig_t *cfg, void *user)
{
    DcMotor_SetPidConfig(&motor, cfg);
}

void start_autotune(void)
{
    DcMotor_AutotuneCfg_t cfg;
    DcMotor_Autotune_DefaultConfig(&cfg);
    cfg.rpm_target = 1500.0f;
    cfg.dt_s       = 0.01f;

    DcMotor_Autotune_Start(&at_ctx, &motor, &cfg, on_autotune_done, NULL);
}

void motor_task(void)
{
    float    rpm = encoder_get_rpm();
    uint32_t t   = HAL_GetTick();

    DcMotor_AutotuneState_t s = DcMotor_Autotune_Update(&at_ctx, t, rpm);

    if (s == DC_AUTOTUNE_DONE || s >= DC_AUTOTUNE_ERR_BASE)
        DcMotor_Update(&motor, t, rpm);
}
```

### Auto-Tuner API

```c
void                    DcMotor_Autotune_DefaultConfig(DcMotor_AutotuneCfg_t *cfg);
DcMotor_Status_t        DcMotor_Autotune_Start        (DcMotor_AutotuneCtx_t *, DcMotor_Handle_t *, const DcMotor_AutotuneCfg_t *, DcMotor_AutotuneDoneCb_t, void *user);
DcMotor_AutotuneState_t DcMotor_Autotune_Update       (DcMotor_AutotuneCtx_t *, uint32_t tick_ms, float current_rpm);
void                    DcMotor_Autotune_Abort         (DcMotor_AutotuneCtx_t *);
DcMotor_Status_t        DcMotor_Autotune_GetResult     (const DcMotor_AutotuneCtx_t *, DcMotor_AutotuneResult_t *out);
```

---

## HAL Contract

Defined in `include/dc_motor_port.h`:

```c
typedef struct {
    void (*set_pwm_duty)(void *hw, float duty_norm); /* required */
    void (*start_pwm)  (void *hw);                  /* optional — set NULL if unused */
    void (*stop_pwm)   (void *hw);                  /* optional — set NULL if unused */
} DcMotor_Port_t;
```

`duty_norm` is always in the range `[0.0, 1.0]`.

---

## Porting to a New Platform

1. Implement `set_pwm_duty` to write a `[0.0, 1.0]` duty to your PWM peripheral.
2. Optionally implement `start_pwm` / `stop_pwm`.
3. Fill a `DcMotor_Port_t` with your callbacks and pass it to `DcMotor_Init()`.

The `src/` files have zero platform-specific code. Only `hal_port/stm32/` touches STM32 HAL.

---

## Isolated Subsystem Testing

Each subsystem can be tested independently without instantiating a motor:

```c
/* PID in isolation */
DcMotor_PidConfig_t cfg;
DcMotor_Pid_DefaultConfig(&cfg);
DcMotor_PidState_t st = {0};
float out = DcMotor_Pid_Compute(&cfg, &st, 0.01f, 1500.0f, 1200.0f);

/* Ramp in isolation */
DcMotor_RampConfig_t rcfg;
DcMotor_Ramp_DefaultConfig(&rcfg);
float next = DcMotor_Ramp_Step(&rcfg, 0.0f, 0.8f, 0.01f);
```

---

## License

MIT
