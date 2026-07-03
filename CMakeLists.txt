cmake_minimum_required(VERSION 3.16)
project(dc-motor-controller C)

set(CMAKE_C_STANDARD 99)

# -----------------------------------------------------------------------
# Core library
# -----------------------------------------------------------------------
add_library(dc_motor STATIC
    src/dc_motor.c
    src/dc_motor_pid.c
    src/dc_motor_ramp.c
    src/dc_motor_safety.c
)

target_include_directories(dc_motor
    PUBLIC  include/          # dc_motor.h, dc_motor_autotune.h, dc_motor_port.h
    PRIVATE src/internal/     # dc_motor_types.h, dc_motor_pid.h, ...
)

# -----------------------------------------------------------------------
# Optional: autotune module (link separately if you need it)
# -----------------------------------------------------------------------
add_library(dc_motor_autotune STATIC
    src/dc_motor_autotune.c
)

target_include_directories(dc_motor_autotune
    PUBLIC  include/
    PRIVATE src/internal/
)

target_link_libraries(dc_motor_autotune PUBLIC dc_motor)

# -----------------------------------------------------------------------
# STM32 HAL port (optional — only add when targeting STM32)
# -----------------------------------------------------------------------
# add_library(dc_motor_stm32 STATIC
#     hal_port/stm32/dc_motor_stm32.c
# )
# target_include_directories(dc_motor_stm32
#     PUBLIC  include/ hal_port/stm32/
#     PRIVATE src/internal/
# )
# target_link_libraries(dc_motor_stm32 PUBLIC dc_motor)
