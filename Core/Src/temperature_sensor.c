/**
 ******************************************************************************
 * @file    temperature_sensor.c
 * @brief   See temperature_sensor.h.
 *
 * Note on the measuring range: the front-end maps 0 degC -> 0 V and 70 degC ->
 * 3.3 V. Below 0 degC / above ~70 degC the output saturates (and a broken
 * PT1000 lead makes the constant-current source drive the output to the top
 * rail, i.e. it looks like "70 degC"). Such readings are flagged by a
 * plausibility check on the raw counts (TEMP_FAULT_LOW/HIGH_COUNTS), reported
 * through TempSensor_GetFault(), and make the PID controller stop with an error.
 ******************************************************************************
 */
#include "temperature_sensor.h"
#include "adc_reader.h"

/* ---- Front-end description (PM board) ---------------------------------- */
#define ADC_MAX_COUNTS            4095.0f
#define R_CAL_LOW_OHM             1000.0f   /* CH3 / SEL3: bottom-of-range reference resistor */
#define R_CAL_HIGH_OHM            1270.0f   /* CH2 / SEL2: top-of-range reference resistor    */

/* Nominal front-end, used until the first calibration has succeeded:
 * 0 V (0 counts) <-> PT1000 at 0 degC, 3.3 V (full scale) <-> PT1000 at 70 degC (IEC 60751). */
#define R_NOMINAL_AT_0_COUNTS     1000.0f
#define R_NOMINAL_AT_FULLSCALE    1270.75f
#define R_PT1000_AT_0C            1000.0f

/* ---- Tunables ----------------------------------------------------------- */
#define TEMP_SAMPLE_INTERVAL_MS   100u     /* fixed measurement interval (same as the PID cycle) */
#define TEMP_OVERSAMPLE            16u     /* ADC conversions averaged per reading (one short burst) */
#define CAL_OVERSAMPLE             64u     /* ADC conversions averaged per calibration resistor      */
#define CAL_MIN_SPAN_COUNTS      1000.0f   /* plausibility: counts(1.27k) - counts(1k) must be at least this (nominal ~4084) */

/* PT1000 plausibility: a reading this close to either ADC rail means the
 * front-end is saturated (temperature outside 0..70 degC) or the sensor lead is
 * broken / shorted. Must be seen on TEMP_FAULT_DEBOUNCE consecutive readings
 * (= 300 ms) before it counts as a fault, so a single noisy reading does not stop the controller. */
#define TEMP_FAULT_LOW_COUNTS       5.0f
#define TEMP_FAULT_HIGH_COUNTS   4090.0f
#define TEMP_FAULT_DEBOUNCE         3u

/* External temperature (T_EXT via I2C): the value is only trusted if the master
 * has written it within this time. Adjust to the master's actual update rate
 * (with some margin). */
#define EXT_TEMP_TIMEOUT_MS      2000u

/* ReadAveragedCounts() drops the highest and the lowest sample, so it needs at least 3.
 * (Preprocessor check instead of C11 _Static_assert: the CubeIDE editor's parser flags
 * _Static_assert as a syntax error, while gcc accepts both.) */
#if (TEMP_OVERSAMPLE < 3u) || (CAL_OVERSAMPLE < 3u)
#error "TEMP_OVERSAMPLE and CAL_OVERSAMPLE must be at least 3 (ReadAveragedCounts drops min and max)"
#endif

/* Piecewise-linear resistance/temperature table, carried over verbatim from
 * temperature_driver_sw (values calculated from the datasheet formula for
 * NB-PTCO-164 PTFC102A1A0; these are samples of the standard PT1000 curve,
 * IEC 60751). Against that formula the table stays within ~0.007 K over
 * 0..70 degC (checked) - far below one ADC step (0.017 K). */
#define PWL_SIZE 10u
static const float pwlResistanceOhm[PWL_SIZE] = { 900.0f, 950.0f, 1000.0f, 1050.0f, 1100.0f, 1150.0f, 1200.0f, 1250.0f, 1300.0f, 1350.0f };
static const float pwlTemperatureC[PWL_SIZE]  = { -25.488f, -12.7685f, 0.0f, 12.818f, 25.6845f, 38.6005f, 51.5665f, 64.583f, 77.651f, 90.7705f };
static float pwlSlope[PWL_SIZE - 1u];   /* degC per Ohm of each table segment */
static const float linpolSlopeFallback = 0.2593035671f; /* used only outside the table (-25..91 degC), where the front-end cannot measure anyway */

/* ---- SEL multiplexer ---------------------------------------------------- */
typedef enum
{
    MUX_PT1000 = 0,   /* CH1 / SEL1 */
    MUX_CAL_HIGH,     /* CH2 / SEL2, 1.27 kOhm */
    MUX_CAL_LOW       /* CH3 / SEL3, 1.00 kOhm */
} MuxChannel_t;

static void SelectChannel(MuxChannel_t channel)
{
    /* Break-before-make: switch everything off first, so two resistors are
     * never connected to the current source at the same time. */
    HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEL2_GPIO_Port, SEL2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEL3_GPIO_Port, SEL3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEL4_GPIO_Port, SEL4_Pin, GPIO_PIN_RESET); /* unused on this board revision */

    switch (channel)
    {
    case MUX_PT1000:   HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_SET); break;
    case MUX_CAL_HIGH: HAL_GPIO_WritePin(SEL2_GPIO_Port, SEL2_Pin, GPIO_PIN_SET); break;
    case MUX_CAL_LOW:  HAL_GPIO_WritePin(SEL3_GPIO_Port, SEL3_Pin, GPIO_PIN_SET); break;
    default: break;
    }
}

/* Mean of n consecutive ADC conversions of the TEMP channel. The single
 * highest and lowest value are dropped as outliers (same idea as in
 * current_sensor.c). */
static float ReadAveragedCounts(uint32_t n)
{
    uint32_t sum  = 0u;
    uint32_t minV = 0xFFFFFFFFu;
    uint32_t maxV = 0u;
    uint32_t i;

    for (i = 0u; i < n; i++)
    {
        const uint32_t v = AdcReader_ReadBlocking(ADC_READER_CH_TEMP);
        sum += v;
        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
    }

    return (float)(sum - minV - maxV) / (float)(n - 2u);
}

/* ---- State ---------------------------------------------------------------- */
typedef enum
{
    TS_CAL_MEASURE_LOW = 0, /* 1.00 kOhm reference is connected: measure it, connect the 1.27 kOhm one */
    TS_CAL_MEASURE_HIGH,    /* 1.27 kOhm reference is connected: measure it, calibrate, connect the PT1000 */
    TS_RUN                  /* normal operation: PT1000 only */
} TempState_t;

static TempState_t state    = TS_CAL_MEASURE_LOW;
static uint32_t    lastTick = 0u;

static uint8_t recalRequested   = 0u;
static uint8_t calibrationOk    = 0u;
static uint8_t measurementValid = 0u;
static uint8_t newSample        = 0u;   /* set on every PT1000 reading (100 ms clock), consumed by the PID */

static uint8_t pt1000FaultCount = 0u;
static uint8_t pt1000Fault      = 0u;

/* Raw counts measured on the two calibration resistors (diagnostics). */
static float calCountsLow  = 0.0f;
static float calCountsHigh = 0.0f;

/* Active counts -> resistance mapping: two known points on a straight line. */
static float mapCountsLow   = 0.0f;
static float mapResLow      = R_NOMINAL_AT_0_COUNTS;
static float mapOhmPerCount = (R_NOMINAL_AT_FULLSCALE - R_NOMINAL_AT_0_COUNTS) / ADC_MAX_COUNTS;

static float resistanceOhm    = R_PT1000_AT_0C;
static float temperatureRawC  = 0.0f;
static float temperatureC     = 0.0f;
static float trimOffsetC      = 0.0f;
static float trimSlope        = 0.0f;

static TempSensor_Source_t controlSource = TEMP_SOURCE_INTERNAL;
static float externalTemperatureC = 0.0f;
static uint8_t  externalValid     = 0u;   /* 1 once the master has written T_EXT at least once */
static uint32_t externalLastTick  = 0u;   /* HAL_GetTick() of the last T_EXT write */

static void SetMapping(float countsLow, float countsHigh, float resLowOhm, float resHighOhm)
{
    mapCountsLow   = countsLow;
    mapResLow      = resLowOhm;
    mapOhmPerCount = (resHighOhm - resLowOhm) / (countsHigh - countsLow);
}

static float ResistanceToTemperature(float resistanceOhmValue)
{
    uint8_t i = 0;

    while (i < PWL_SIZE && resistanceOhmValue > pwlResistanceOhm[i])
        i++;

    if (i > 0 && i < PWL_SIZE)
    {
        i--;
        /* PWL interpolation: T(R) = Tlower + (R - Rlower) * slope, see AN0970 (Analog Devices) */
        return pwlTemperatureC[i] + (resistanceOhmValue - pwlResistanceOhm[i]) * pwlSlope[i];
    }

    /* Outside the characterised range: fall back to a simple linear approximation around 0 degC */
    return (resistanceOhmValue - R_PT1000_AT_0C) * linpolSlopeFallback;
}

/* Connects the 1.00 kOhm reference; the state machine takes it from there. */
static void BeginCalibration(void)
{
    calibrationOk    = 0u;
    measurementValid = 0u;
    pt1000FaultCount = 0u;
    pt1000Fault      = 0u;
    SelectChannel(MUX_CAL_LOW);
    state = TS_CAL_MEASURE_LOW;
}

static void FinishCalibration(void)
{
    if ((calCountsHigh - calCountsLow) >= CAL_MIN_SPAN_COUNTS)
    {
        SetMapping(calCountsLow, calCountsHigh, R_CAL_LOW_OHM, R_CAL_HIGH_OHM);
        calibrationOk = 1u;
    }
    else
    {
        /* Reference readings implausible (resistors not connected / SEL lines not
         * wired / front-end dead): keep the previous mapping, but do not report ready. */
        calibrationOk = 0u;
    }
}

/* One PT1000 reading: counts -> resistance -> temperature (+ trim). */
static void MeasurePt1000(void)
{
    const float counts = ReadAveragedCounts(TEMP_OVERSAMPLE);

    resistanceOhm   = mapResLow + (counts - mapCountsLow) * mapOhmPerCount;
    temperatureRawC = ResistanceToTemperature(resistanceOhm);
    temperatureC    = temperatureRawC - trimOffsetC - (temperatureRawC * trimSlope);
    measurementValid = 1u;
    newSample        = 1u;

    /* Plausibility check on the raw counts (see file header) */
    if ((counts < TEMP_FAULT_LOW_COUNTS) || (counts > TEMP_FAULT_HIGH_COUNTS))
    {
        if (pt1000FaultCount < TEMP_FAULT_DEBOUNCE)
            pt1000FaultCount++;
        if (pt1000FaultCount >= TEMP_FAULT_DEBOUNCE)
            pt1000Fault = 1u;
    }
    else
    {
        pt1000FaultCount = 0u;
        pt1000Fault      = 0u;
    }
}

/* ---- Public API ---------------------------------------------------------- */
void TempSensor_Init(void)
{
    uint8_t i;

    /* Precalculate the PWL segment slopes */
    for (i = 0; i < PWL_SIZE - 1u; i++)
        pwlSlope[i] = (pwlTemperatureC[i + 1] - pwlTemperatureC[i]) / (pwlResistanceOhm[i + 1] - pwlResistanceOhm[i]);

    SetMapping(0.0f, ADC_MAX_COUNTS, R_NOMINAL_AT_0_COUNTS, R_NOMINAL_AT_FULLSCALE);
    recalRequested = 0u;

    BeginCalibration();          /* calibrate first, then measure */
    lastTick = HAL_GetTick();
}

void TempSensor_Process(void)
{
    const uint32_t now = HAL_GetTick();

    if ((now - lastTick) < TEMP_SAMPLE_INTERVAL_MS)
        return;
    lastTick = now;

    switch (state)
    {
    case TS_CAL_MEASURE_LOW:
        calCountsLow = ReadAveragedCounts(CAL_OVERSAMPLE);
        SelectChannel(MUX_CAL_HIGH);
        state = TS_CAL_MEASURE_HIGH;
        break;

    case TS_CAL_MEASURE_HIGH:
        calCountsHigh = ReadAveragedCounts(CAL_OVERSAMPLE);
        FinishCalibration();
        SelectChannel(MUX_PT1000);
        state = TS_RUN;
        break;

    case TS_RUN:
        if (recalRequested)
        {
            recalRequested = 0u;
            BeginCalibration();
        }
        else
        {
            MeasurePt1000();
        }
        break;

    default:
        BeginCalibration();
        break;
    }
}

void TempSensor_Recalibrate(void)
{
    recalRequested = 1u;
}

uint8_t TempSensor_IsReady(void)
{
    if (controlSource == TEMP_SOURCE_EXTERNAL)
        return (TempSensor_GetFault() == TEMP_FAULT_NONE) ? 1u : 0u;

    return (calibrationOk && measurementValid && !pt1000Fault) ? 1u : 0u;
}

TempSensor_Fault_t TempSensor_GetFault(void)
{
    if (controlSource == TEMP_SOURCE_EXTERNAL)
    {
        if (!externalValid || ((HAL_GetTick() - externalLastTick) > EXT_TEMP_TIMEOUT_MS))
            return TEMP_FAULT_EXT_TIMEOUT;
        return TEMP_FAULT_NONE;
    }

    return pt1000Fault ? TEMP_FAULT_PT1000_RANGE : TEMP_FAULT_NONE;
}

uint8_t TempSensor_ConsumeNewSample(void)
{
    const uint8_t flag = newSample;
    newSample = 0u;
    return flag;
}

float TempSensor_GetTemperature(void)
{
    return temperatureC;
}

float TempSensor_GetResistance(void)
{
    return resistanceOhm;
}

float TempSensor_GetControlTemperature(void)
{
    return (controlSource == TEMP_SOURCE_EXTERNAL) ? externalTemperatureC : temperatureC;
}

void TempSensor_SetSource(TempSensor_Source_t source)
{
    controlSource = source;
}

TempSensor_Source_t TempSensor_GetSource(void)
{
    return controlSource;
}

void TempSensor_SetExternalTemperature(float tempDegC)
{
    externalTemperatureC = tempDegC;
    externalLastTick     = HAL_GetTick();
    externalValid        = 1u;
}

void TempSensor_SetTrim(float offsetDegC, float slope)
{
    trimOffsetC = offsetDegC;
    trimSlope   = slope;
}

uint16_t TempSensor_GetCalCountsLow(void)
{
    return (uint16_t)(calCountsLow + 0.5f);
}

uint16_t TempSensor_GetCalCountsHigh(void)
{
    return (uint16_t)(calCountsHigh + 0.5f);
}
