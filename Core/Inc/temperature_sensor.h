/**
 ******************************************************************************
 * @file    temperature_sensor.h
 * @brief   PT1000 temperature measurement of the PM board.
 *
 * Hardware (PM board):
 *   A constant-current source drives 100 uA through ONE of three resistors,
 *   selected by the SEL lines. An analog front-end turns the small voltage
 *   drop into 0 V (PT1000 at 0 degC = 1000 Ohm) ... 3.3 V (70 degC =
 *   ~1270.75 Ohm), read by ADC1 channel 1 (TEMP, PA1):
 *
 *      CH1 / SEL1 : PT1000 sensor      - the only channel used in normal operation
 *      CH2 / SEL2 : 1.27 kOhm resistor - calibration only (top of the range)
 *      CH3 / SEL3 : 1.00 kOhm resistor - calibration only (bottom of the range)
 *      SEL4       : unused on this board revision (kept low)
 *
 * Measurement: one reading every TEMP_SAMPLE_INTERVAL_MS (100 ms, fixed).
 * Unlike the original RC-charging method there is nothing to wait for - no
 * settling time, no filter time, no temporal filter. A reading is a short burst
 * of ADC conversions on the already-selected CH1, averaged against ADC noise.
 *
 * Calibration: the two precision resistors sit exactly at the two ends of the
 * measuring range, so measuring them gives a 2-point calibration of the whole
 * analog chain (current source, amplifier gain/offset, ADC offset/gain):
 *
 *      R = 1000 Ohm + (counts - counts@1k) * (1270 - 1000) Ohm / (counts@1.27k - counts@1k)
 *
 * The PT1000 resistance is then converted to degC with a piecewise-linear
 * table in 10 K steps (values from the PT1000 sensor data sheet, see
 * temperature_sensor.c). The calibration runs on the
 * same fixed 100 ms clock, right after TempSensor_Init() and on request
 * (TempSensor_Recalibrate()):
 *
 *      tick 0: connect the 1.00 kOhm reference (done in Init / on request)
 *      tick 1: measure it,            connect the 1.27 kOhm reference
 *      tick 2: measure it, calibrate, connect the PT1000 again
 *      tick 3: first PT1000 reading -> TempSensor_IsReady() = 1
 *
 * So a calibration takes ~300 ms. (The one interval between connecting a
 * reference and measuring it is just a consequence of the fixed clock, not a
 * separate settling timer.)
 *
 * Measuring range: 0 ... ~70 degC. Beyond that the front-end output saturates
 * at 0 V / 3.3 V and the reading is no longer valid (see the note in
 * temperature_sensor.c).
 *
 * Cooperative: call TempSensor_Process() once per main loop iteration. A tick
 * blocks for the ADC burst only (~1-2 ms; ~5-8 ms for a calibration reading).
 ******************************************************************************
 */
#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Where the temperature that the PID controller regulates on comes from. */
typedef enum
{
    TEMP_SOURCE_EXTERNAL = 0,  /* value supplied by the I2C master (TempSensor_SetExternalTemperature) */
    TEMP_SOURCE_INTERNAL = 1   /* the PT1000 on CH1 */
} TempSensor_Source_t;

/* Problems with the temperature the PID controller regulates on. */
typedef enum
{
    TEMP_FAULT_NONE = 0,
    TEMP_FAULT_PT1000_RANGE,   /* internal source: PT1000 reading at an ADC rail (outside 0..70 degC, broken/shorted lead) */
    TEMP_FAULT_EXT_TIMEOUT     /* external source: T_EXT never written or not refreshed within EXT_TEMP_TIMEOUT_MS */
} TempSensor_Fault_t;

/* One-time setup; starts the calibration sequence. Call from main() before the main loop. */
void TempSensor_Init(void);

/* Fixed-interval measurement / calibration state machine. Call every main loop iteration. */
void TempSensor_Process(void);

/* Requests a new 2-point calibration (starts at the next 100 ms tick, takes ~300 ms,
 * during which the last temperature value is held). Only do this while the
 * controller is not regulating - i2c_comm.c enforces that. */
void TempSensor_Recalibrate(void);

/* 1 once the temperature the controller uses is valid: with the internal
 * source that means "calibration succeeded, a first PT1000 value exists and it
 * is plausible"; with the external source it means "T_EXT was written within
 * EXT_TEMP_TIMEOUT_MS". */
uint8_t TempSensor_IsReady(void);

/* Current fault of the temperature the controller uses (see TempSensor_Fault_t).
 * Checked by the PID controller on every cycle while regulating. */
TempSensor_Fault_t TempSensor_GetFault(void);

/* Returns 1 (once) for every new PT1000 reading, i.e. on the fixed 100 ms
 * measurement clock. The PID controller runs one calculation per new sample,
 * so its cycle is locked to the measurement (also with the external source,
 * which then simply supplies the most recent T_EXT). */
uint8_t TempSensor_ConsumeNewSample(void);

/* PT1000 (CH1) results. */
float TempSensor_GetTemperature(void);   /* degC, including the optional trim */
float TempSensor_GetResistance(void);    /* Ohm */

/* Temperature the PID controller regulates on (internal PT1000 or external value, see above). */
float TempSensor_GetControlTemperature(void);
void  TempSensor_SetSource(TempSensor_Source_t source);
TempSensor_Source_t TempSensor_GetSource(void);
/* Call on EVERY master write of T_EXT (even with an unchanged value): it also
 * refreshes the timeout of the external source. */
void  TempSensor_SetExternalTemperature(float tempDegC);

/* Optional final trim against a reference thermometer (e.g. to remove the
 * PT1000's own tolerance): T = Traw - offsetDegC - Traw * slope. Default 0 / 0.
 * (Not to be confused with the automatic resistor calibration above.) */
void TempSensor_SetTrim(float offsetDegC, float slope);

/* Raw ADC counts (0..4095) measured on the two calibration resistors during
 * the last calibration - for bring-up diagnostics. Expected with the nominal
 * front-end (0 V at 1000 Ohm, 3.3 V at ~1270.75 Ohm): about 0 and about 4084. */
uint16_t TempSensor_GetCalCountsLow(void);   /* 1.00 kOhm reference (CH3) */
uint16_t TempSensor_GetCalCountsHigh(void);  /* 1.27 kOhm reference (CH2) */

#ifdef __cplusplus
}
#endif

#endif /* TEMPERATURE_SENSOR_H */
