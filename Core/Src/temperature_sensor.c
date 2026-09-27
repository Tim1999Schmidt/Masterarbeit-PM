/**
 ******************************************************************************
 * @file    temperature_sensor.c
 * @brief   See temperature_sensor.h.
 *
 * ASSUMPTION TO VERIFY AGAINST THE PM SCHEMATIC
 * -----------------------------------------------
 * The original board measured resistance via an RC discharge time (CHARGE /
 * MEASURE MOSFETs + external comparator on a TRIG/EXTI pin). None of those
 * signals exist on the PM board's IOC - instead there is a single ADC input
 * (TEMP, PA1). This file assumes SEL1..SEL4 now switch a resistor divider of
 * the form
 *
 *      VDDA --[[ R_fixed (shared, same for all 4 mux positions) ]]-- TEMP(ADC) --[[ R_selected ]]-- GND
 *
 * i.e. the selected resistor (calibration resistor on SEL1, RTD sensors on
 * SEL2..SEL4) sits on the GND side. Because R_fixed is identical for all four
 * mux positions, it cancels out when a channel's reading is taken *relative*
 * to the SEL1 reference reading - exactly like fclk/c_Meas cancelled out in
 * the original RC-timing formula - so its absolute value does not need to be
 * known. What DOES need to be verified is the side of the divider: if the
 * selected resistor is actually on the VDDA side instead, invert the ratio
 * in RatioFromCounts() (swap `counts` and `ADC_FULL_SCALE - counts`).
 *
 * R_CAL_OHM (the SEL1 reference resistor) is carried over unchanged from the
 * original driver (1000 Ohm nominal).
 ******************************************************************************
 */
#include "temperature_sensor.h"
#include <string.h>

extern ADC_HandleTypeDef hadc1; /* initialised by MX_ADC1_Init() in main.c, shared with current_sensor.c */

/* ---- Tunables --------------------------------------------------------- */
#define ADC_FULL_SCALE          4096.0f  /* 12-bit ADC1, matches old board's resolution */
#define R_CAL_OHM                1000.0f  /* SEL1 reference resistor, same nominal value as original driver */
#define TEMP_MUX_SETTLE_MS          2u   /* wait after switching SELx before reading TEMP; original RC design settled within ~100us, 2ms leaves generous margin for the new analog mux + any anti-alias filter */

#define MEDIAN_MEMSIZE               7u   /* odd number of scan rounds kept per channel, same as original driver */
#define MEDIAN_AVG_SIDES             2u   /* neighbours averaged on each side of the median, same as original driver */

/* Piecewise-linear resistance/temperature table, carried over verbatim from
 * temperature_driver_sw (datasheet values for NB-PTCO-164 PTFC102A1A0). */
#define PWL_SIZE 10u
static const float pwlResistanceOhm[PWL_SIZE] = { 900.0f, 950.0f, 1000.0f, 1050.0f, 1100.0f, 1150.0f, 1200.0f, 1250.0f, 1300.0f, 1350.0f };
static const float pwlTemperatureC[PWL_SIZE]  = { -25.488f, -12.7685f, 0.0f, 12.818f, 25.6845f, 38.6005f, 51.5665f, 64.583f, 77.651f, 90.7705f };
static float pwlSlope[PWL_SIZE];
static const float linpolSlopeFallback = 0.2593035671f; /* used only outside the PWL-covered range */

/* ---- Median/average filter, one instance per mux position (0=reference) */
typedef struct
{
    uint32_t history[MEDIAN_MEMSIZE];
    uint32_t sorted[MEDIAN_MEMSIZE];
    float    filtered;
} MedianFilter_t;

static MedianFilter_t filters[4];
static uint8_t scanRoundIndex = 0;

static void MedianFilter_Push(MedianFilter_t *f, uint32_t value, uint8_t writeIndex)
{
    int i, j;
    uint32_t tmp;
    const uint8_t mid = (MEDIAN_MEMSIZE - 1u) / 2u;
    float sum;

    f->history[writeIndex] = value;
    memcpy(f->sorted, f->history, sizeof(f->sorted));

    /* insertion sort - fine for MEDIAN_MEMSIZE this small */
    for (i = 1; i < (int)MEDIAN_MEMSIZE; i++)
    {
        tmp = f->sorted[i];
        j = i - 1;
        while (j >= 0 && f->sorted[j] > tmp)
        {
            f->sorted[j + 1] = f->sorted[j];
            j--;
        }
        f->sorted[j + 1] = tmp;
    }

    sum = (float)f->sorted[mid];
    for (i = 1; i <= (int)MEDIAN_AVG_SIDES; i++)
    {
        sum += (float)f->sorted[mid + i];
        sum += (float)f->sorted[mid - i];
    }
    f->filtered = sum / ((MEDIAN_AVG_SIDES * 2.0f) + 1.0f);
}

/* ---- Module state ------------------------------------------------------ */
typedef enum
{
    TEMP_STATE_SETUP_CHANNEL = 0,
    TEMP_STATE_SETTLE,
    TEMP_STATE_READ
} TempMeasState_t;

static TempMeasState_t measState = TEMP_STATE_SETUP_CHANNEL;
static uint8_t activePosition = 0; /* 0 = SEL1 reference, 1..3 = TEMP_CHANNEL_1..3 */
static uint32_t settleStartTick = 0;

static float resistanceOhm[TEMP_CHANNEL_COUNT];
static float temperatureRawC[TEMP_CHANNEL_COUNT];
static float temperatureCalC[TEMP_CHANNEL_COUNT];
static float calOffset[TEMP_CHANNEL_COUNT];
static float calSlope[TEMP_CHANNEL_COUNT];

static TempSensor_Source_t controlSource = TEMP_SOURCE_CH1;
static float externalTemperatureC = 0.0f;

/* ---- Helpers ------------------------------------------------------------ */
static void SelectMuxPosition(uint8_t position)
{
    /* Exactly one SELx line active at a time, same convention as the original driver */
    HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, (position == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEL2_GPIO_Port, SEL2_Pin, (position == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEL3_GPIO_Port, SEL3_Pin, (position == 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEL4_GPIO_Port, SEL4_Pin, (position == 3) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint32_t ReadTempAdcBlocking(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = ADC_CHANNEL_1; /* TEMP / PA1 */
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig); /* explicit re-select before every conversion: robust regardless of the sequencer's NbrOfConversion setting */

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint32_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return value;
}

/* See the file header for the assumption this formula encodes; flip the
 * ratio (swap counts and ADC_FULL_SCALE-counts) if the sensor turns out to
 * sit on the VDDA side instead of the GND side of the divider. */
static float RatioFromCounts(float counts)
{
    if (counts < 1.0f)
        counts = 1.0f;
    if (counts > (ADC_FULL_SCALE - 1.0f))
        counts = ADC_FULL_SCALE - 1.0f;

    return counts / (ADC_FULL_SCALE - counts);
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

    /* Outside the characterised range: fall back to a simple linear approximation around R_CAL_OHM */
    return (resistanceOhmValue - R_CAL_OHM) * linpolSlopeFallback;
}

/* ---- Public API ---------------------------------------------------------- */
void TempSensor_Init(void)
{
    uint8_t i;

    memset(filters, 0, sizeof(filters));
    scanRoundIndex = 0;
    activePosition = 0;
    measState = TEMP_STATE_SETUP_CHANNEL;

    for (i = 0; i < TEMP_CHANNEL_COUNT; i++)
    {
        resistanceOhm[i] = R_CAL_OHM;
        temperatureRawC[i] = 0.0f;
        temperatureCalC[i] = 0.0f;
        /* Per-unit calibration (offset/slope) is tied to one specific physical
         * board's component tolerances. The original driver carried a
         * hand-picked set ("System 003") for the OLD board; since PM is a
         * different physical unit it needs its own calibration run, so this
         * starts uncalibrated (0/0) rather than reusing stale values. */
        calOffset[i] = 0.0f;
        calSlope[i] = 0.0f;
    }

    /* Precalculate PWL segment slopes (verbatim from the original driver) */
    for (i = 0; i < PWL_SIZE - 1u; i++)
        pwlSlope[i] = (pwlTemperatureC[i + 1] - pwlTemperatureC[i]) / (pwlResistanceOhm[i + 1] - pwlResistanceOhm[i]);
    /* last slope is extrapolated, uppermost reference point has no upper neighbour */
    pwlSlope[PWL_SIZE - 1u] = 1.00398f * pwlSlope[PWL_SIZE - 2u];

    SelectMuxPosition(0);
}

void TempSensor_Process(void)
{
    switch (measState)
    {
    case TEMP_STATE_SETUP_CHANNEL:
        SelectMuxPosition(activePosition);
        settleStartTick = HAL_GetTick();
        measState = TEMP_STATE_SETTLE;
        break;

    case TEMP_STATE_SETTLE:
        if ((HAL_GetTick() - settleStartTick) >= TEMP_MUX_SETTLE_MS)
            measState = TEMP_STATE_READ;
        break;

    case TEMP_STATE_READ:
    {
        uint32_t counts = ReadTempAdcBlocking();
        MedianFilter_Push(&filters[activePosition], counts, scanRoundIndex);

        if (activePosition > 0)
        {
            uint8_t ch = activePosition - 1u; /* 0..2 */
            float ratioRef = RatioFromCounts(filters[0].filtered);
            float ratioCh = RatioFromCounts(filters[activePosition].filtered);

            resistanceOhm[ch] = R_CAL_OHM * (ratioCh / ratioRef);
            temperatureRawC[ch] = ResistanceToTemperature(resistanceOhm[ch]);
            temperatureCalC[ch] = temperatureRawC[ch] - calOffset[ch] - temperatureRawC[ch] * calSlope[ch];
        }

        activePosition = (uint8_t)((activePosition + 1u) % 4u);
        if (activePosition == 0)
            scanRoundIndex = (uint8_t)((scanRoundIndex + 1u) % MEDIAN_MEMSIZE);

        measState = TEMP_STATE_SETUP_CHANNEL;
        break;
    }
    }
}

float TempSensor_GetTemperature(TempSensor_Channel_t channel)
{
    if (channel >= TEMP_CHANNEL_COUNT)
        return 0.0f;
    return temperatureCalC[channel];
}

float TempSensor_GetResistance(TempSensor_Channel_t channel)
{
    if (channel >= TEMP_CHANNEL_COUNT)
        return 0.0f;
    return resistanceOhm[channel];
}

float TempSensor_GetControlTemperature(void)
{
    switch (controlSource)
    {
    case TEMP_SOURCE_CH1: return temperatureCalC[TEMP_CHANNEL_1];
    case TEMP_SOURCE_CH2: return temperatureCalC[TEMP_CHANNEL_2];
    case TEMP_SOURCE_CH3: return temperatureCalC[TEMP_CHANNEL_3];
    case TEMP_SOURCE_EXTERNAL:
    default:
        return externalTemperatureC;
    }
}

void TempSensor_SetSource(TempSensor_Source_t source)
{
    controlSource = source;
}

TempSensor_Source_t TempSensor_GetSource(void)
{
    return controlSource;
}

void TempSensor_SetExternalTemperature(float tExtDegC)
{
    externalTemperatureC = tExtDegC;
}

void TempSensor_SetCalibration(TempSensor_Channel_t channel, float offset, float slope)
{
    if (channel >= TEMP_CHANNEL_COUNT)
        return;
    calOffset[channel] = offset;
    calSlope[channel] = slope;
}
