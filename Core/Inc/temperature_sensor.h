/**
 ******************************************************************************
 * @file    temperature_sensor.h
 * @brief   Temperature acquisition for up to three PT1000 RTD channels,
 *          multiplexed via SEL1..SEL4 onto the single TEMP (PA1 / ADC1_IN1)
 *          analog input.
 *
 * Ported from temperature_driver_sw (STM32F401RET6) to the STM32C051K6T6
 * "PM" board. On the old board, resistance was derived from an RC
 * discharge-time measurement (CHARGE/MEASURE MOSFETs + external comparator
 * on the TRIG/EXTI pin). The new board has none of those signals and instead
 * exposes a single dedicated ADC channel (TEMP), so resistance is now
 * derived directly from an ADC reading of a resistor divider that the
 * SELx lines switch between four positions (one internal reference
 * resistor + three real RTD channels) - see the assumption called out in
 * temperature_sensor.c (RatioFromCounts / channel wiring).
 *
 * The piecewise-linear resistance->temperature table and the per-channel
 * calibration (offset/slope) concept are carried over unchanged from the
 * original driver.
 *
 * Uses float (not double): the STM32C051 (Cortex-M0+) has no FPU
 * (-mfloat-abi=soft), so every floating-point op is software-emulated, and
 * the 64-bit routines for double are considerably larger than the 32-bit
 * ones for float. float's ~7 significant digits are ample for 0.01 degC
 * resolution here.
 ******************************************************************************
 */
#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Three real RTD channels are exposed (SEL2..SEL4 in the original driver's
 * numbering). SEL1 is reserved internally as the fixed calibration resistor
 * position, exactly as in the original driver, and is not exposed here. */
typedef enum
{
    TEMP_CHANNEL_1 = 0,
    TEMP_CHANNEL_2,
    TEMP_CHANNEL_3,
    TEMP_CHANNEL_COUNT
} TempSensor_Channel_t;

/* Selects which temperature value is handed to the PID controller. */
typedef enum
{
    TEMP_SOURCE_EXTERNAL = 0,  /* Use value written via TempSensor_SetExternalTemperature() */
    TEMP_SOURCE_CH1      = 1,
    TEMP_SOURCE_CH2      = 2,
    TEMP_SOURCE_CH3      = 3
} TempSensor_Source_t;

/* One-time setup: zeroes filter memory, builds the PWL table, precalculates
 * derived constants, sets GPIO mux lines to a defined idle state. Call once
 * from main() before the main loop. */
void TempSensor_Init(void);

/* Cooperative, non-blocking state machine step. Call once per main loop
 * iteration. Internally cycles through: select channel -> let the analog
 * mux/filter settle -> read & convert -> advance to next channel. One full
 * pass over all four mux positions (reference + 3 channels) takes a few
 * milliseconds; this is comfortably fast compared to the thermal time
 * constants of a Peltier-regulated setup. */
void TempSensor_Process(void);

/* Calibrated temperature of a single channel, in degrees Celsius. Returns
 * the most recent measurement; updated once per completed scan of that
 * channel. */
float TempSensor_GetTemperature(TempSensor_Channel_t channel);

/* Raw (uncalibrated) resistance of a single channel in Ohms, mainly useful
 * for debugging/verifying the analog front-end assumption. */
float TempSensor_GetResistance(TempSensor_Channel_t channel);

/* Temperature currently selected to drive the PID controller (either one of
 * the three channels or the externally-supplied value). This is the
 * pointer-equivalent of the original T_CTRL. */
float TempSensor_GetControlTemperature(void);

/* Selects which source feeds the PID controller. */
void TempSensor_SetSource(TempSensor_Source_t source);
TempSensor_Source_t TempSensor_GetSource(void);

/* Supplies an externally-provided temperature value (e.g. received from the
 * I2C master) to be used when the source is TEMP_SOURCE_EXTERNAL. */
void TempSensor_SetExternalTemperature(float tExtDegC);

/* Per-channel linear calibration: T_calibrated = T_raw - offset - T_raw*slope
 * (same convention as the original driver). Defaults to 0/0 (uncalibrated)
 * since these are per-physical-unit values tied to the old board and must be
 * re-derived for each new PM board - see temperature_sensor.c. */
void TempSensor_SetCalibration(TempSensor_Channel_t channel, float offset, float slope);

#ifdef __cplusplus
}
#endif

#endif /* TEMPERATURE_SENSOR_H */
