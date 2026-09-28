/**
 ******************************************************************************
 * @file    current_sensor.h
 * @brief   Peltier current acquisition via the single-ended I_SENS_FILT
 *          (PA2 / ADC1_IN2) input.
 *
 * On the original board, current was derived differentially from two ADC
 * channels (I_SENS_REF / I_SENS_FILT) and could read either sign, with the
 * sign itself indicating flow direction. On the PM board the current sense
 * tap sits electrically after the direction relay, so only one polarity is
 * ever seen there: the measured value is always a magnitude (>= 0 A), and
 * heating vs. cooling is known from the relay state (RELAY_SET/RELAY_RESET),
 * not from the sign of the current reading. See current_sensor.c.
 *
 * Uses float (not double): the STM32C051 (Cortex-M0+) has no FPU
 * (-mfloat-abi=soft), so every floating-point op is software-emulated, and
 * the 64-bit routines for double are considerably larger than the 32-bit
 * ones for float. float's ~7 significant digits are ample for amperes here.
 ******************************************************************************
 */
#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One-time setup. Call once from main() before the main loop. */
void CurrentSensor_Init(void);

/* Samples I_SENS_FILT (8x oversampled with min/max trim, same denoising
 * technique as the original driver), converts it to amperes using the
 * zero-current reference captured by CurrentSensor_CalibrateZero(), and
 * returns the result. Always >= 0 A - see the file header. Blocking, takes
 * roughly the time of 8 ADC conversions (microseconds). */
float CurrentSensor_Read(void);

/* Captures the current zero-current ADC baseline. Call this while the PWM
 * output is confirmed at 0% duty cycle / the driver is off, so that any
 * small residual offset of the current-sense amplifier is calibrated out
 * rather than guessed at. Safe to call again later (e.g. periodically while
 * the driver is off) to track thermal drift of that offset. */
void CurrentSensor_CalibrateZero(void);

/* 1 if the most recent CurrentSensor_Read() was at the top of the ADC range:
 * the real current is then above the measuring range (~6.6 A) and unknown.
 * The driver treats this as overcurrent. */
uint8_t CurrentSensor_IsSaturated(void);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_SENSOR_H */
