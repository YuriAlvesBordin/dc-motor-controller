cmake_minimum_required(VERSION 3.16)

project(dc_motor_controller
    VERSION     1.0.0
    DESCRIPTION "Portable HAL-agnostic DC motor control library"
    LANGUAGES   C
)

option(DC_MOTOR_ENABLE_OPENLOOP       "Enable open-loop mode"                  OFF)
option(DC_MOTOR_ENABLE_CLOSEDLOOP     "Enable closed-loop PID mode"            ON)
option(DC_MOTOR_ENABLE_RAMP           "Enable slew-rate ramp generator"        OFF)
option(DC_MOTOR_ENABLE_PID_ANTI_WINDUP "Enable PID integrator anti-windup"     ON)
option(DC_MOTOR_ENABLE_PID_D_FILTER   "Enable PID derivative low-pass filter"  ON)
option(DC_MOTOR_PID_DERIV_ON_MEASUREMENT "Compute D term on measurement"       ON)
option(DC_MOTOR_ENABLE_WATCHDOG       "Enable control-loop watchdog"           ON)

add_library(dc_motor_controller STATIC
    src/dc_motor.c
    src/dc_motor_pid.c
    src/dc_motor_ramp.c
    src/dc_motor_openloop.c
    src/dc_motor_closedloop.c
)

target_include_directories(dc_motor_controller
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/internal
)

target_compile_features(dc_motor_controller PUBLIC c_std_99)

foreach(_opt
    DC_MOTOR_ENABLE_OPENLOOP
    DC_MOTOR_ENABLE_CLOSEDLOOP
    DC_MOTOR_ENABLE_RAMP
    DC_MOTOR_ENABLE_PID_ANTI_WINDUP
    DC_MOTOR_ENABLE_PID_D_FILTER
    DC_MOTOR_PID_DERIV_ON_MEASUREMENT
    DC_MOTOR_ENABLE_WATCHDOG
)
    if(${_opt})
        target_compile_definitions(dc_motor_controller PUBLIC ${_opt}=1)
    else()
        target_compile_definitions(dc_motor_controller PUBLIC ${_opt}=0)
    endif()
endforeach()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(GNUInstallDirs)

    install(TARGETS dc_motor_controller
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
    install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/dc_motor_controller
    )
endif()
