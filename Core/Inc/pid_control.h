/**
 ******************************************************************************
 * @file    pid_control.h
 * @brief   PID temperature control loop plus its actuation stage (bistable
 *          relay direction switching, PWM duty cycle, startup short-circuit
 *          check, continuous overcurrent protection with duty-cycle
 *          recalibration).
 *
 * The actuation/driver logic is kept together with the PID loop rather than
 * split into its own file: it is the direct, tightly-coupled consumer of the
 * PID output (turns pidOut [A] into a relay direction + PWM duty cycle), so
 * this matches the requested split into current measurement / temperature
 * measurement / PID control.
 *
 * Ported from temperature_driver_sw's pidStateMachine()/driverStateMachine().
 * The Ziegler-Nichols autotuning routine (tuningStateMachine()) was dropped
 * entirely per request. All timing that used to run off dedicated hardware
 * timers (htim1 for the PID cycle, htim3 as a free-running 0.1ms driver
 * timebase) now uses HAL_GetTick() instead, since the PM board's only timer
 * (TIM3) is committed to the PWM output. The one-time startup calibration
 * sequence still runs as a short (~100-150 ms) blocking sequence, exactly
 * like the original - this is fine because I2C slave handling runs on
 * interrupts (see i2c_comm.c) and keeps responding even while this runs.
 *
 * Uses float (not double): the STM32C051 (Cortex-M0+) has no FPU
 * (-mfloat-abi=soft), so every floating-point op is software-emulated, and
 * the 64-bit routines for double are considerably larger than the 32-bit
 * ones for float.
 ******************************************************************************
 */
#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    PID_ERROR_NONE = 0,
    PID_ERROR_STARTUP_OVERCURRENT = 1, /* short-circuit-like current already at the initial test duty cycle */
    PID_ERROR_RUNTIME_OVERCURRENT = 2, /* current exceeded the configured limit during normal operation */
    PID_ERROR_SANITY = 3               /* temperature error grew implausibly (reversed polarity / instability) */
} PidControl_Error_t;

typedef enum
{
    PID_DIRECTION_HEATING = 0,
    PID_DIRECTION_COOLING = 1
} PidControl_Direction_t;

/* One-time setup. Call once from main() before the main loop. */
void PidControl_Init(void);

/* Cooperative, non-blocking step (aside from the one-time startup
 * calibration triggered by PidControl_Start(), see above). Call once per
 * main loop iteration; internally drives both the PID loop and the
 * actuation stage, same as the original's pidStateMachine() +
 * driverStateMachine() pair. */
void PidControl_Process(void);

/* Requests the controller to start / stop regulating. Start triggers the
 * short blocking startup calibration described above. */
void PidControl_Start(void);
void PidControl_Stop(void);
uint8_t PidControl_IsRunning(void);

/* Clears a latched error and returns to the idle (off) state - the I2C
 * equivalent of the original board's ERR_CONFIRM button, which does not
 * exist on the PM board. */
void PidControl_ClearError(void);
uint8_t PidControl_IsError(void);
PidControl_Error_t PidControl_GetErrorCode(void);

/* Setpoint and gains. */
void PidControl_SetSetpoint(float tSetDegC);
float PidControl_GetSetpoint(void);
void PidControl_SetGains(float kp, float ki, float kd);
void PidControl_GetGains(float *kp, float *ki, float *kd);

/* Symmetric current limit magnitude, in amperes (replaces the original's
 * separate +/- currentLimitP/N now that current is single-ended). */
void PidControl_SetCurrentLimit(float limitA);
float PidControl_GetCurrentLimit(void);

/* Live telemetry. */
float PidControl_GetControllerOutput(void);   /* pidOut, in amperes */
float PidControl_GetMeasuredCurrent(void);    /* most recent current reading, in amperes */
float PidControl_GetDutyCycle(void);          /* 0.0 .. 1.0 */
PidControl_Direction_t PidControl_GetDirection(void);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROL_H */
