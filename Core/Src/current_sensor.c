/**
 ******************************************************************************
 * @file    current_sensor.c
 * @brief   See current_sensor.h.
 ******************************************************************************
 */
#include "current_sensor.h"

extern ADC_HandleTypeDef hadc1; /* initialised by MX_ADC1_Init() in main.c, shared with temperature_sensor.c */

/* ---- Tunables, carried over from the original driver ------------------- */
#define ADC_FULL_SCALE       4096.0f  /* 12-bit ADC1 */
#define VDDA_MV               3300.0f  /* nominal supply, same constant as original driver (V_DDA_INITIAL) */
#define SHUNT_OHM              0.001f  /* shunt resistor - unchanged unless the analog current-sense circuit itself was redesigned */
#define INA_GAIN              200.0f  /* current-sense amplifier gain - unchanged unless the analog current-sense circuit itself was redesigned */
#define OVERSAMPLE_COUNT           8u   /* same oversampling factor as the original driver's readADC() */

static uint32_t zeroCounts = 0;

static uint32_t ReadCurrentAdcBlocking(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = ADC_CHANNEL_2; /* I_SENS_FILT / PA2 */
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig); /* explicit re-select before every conversion: robust regardless of the sequencer's NbrOfConversion setting */

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint32_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return value;
}

/* Takes OVERSAMPLE_COUNT raw samples, insertion-sorts them and averages the
 * centre 6 of 8 (discarding the highest and lowest as outliers) - the same
 * denoising technique the original driver applied to each of its two
 * channels, applied here to the single remaining channel. */
static float ReadFilteredCounts(void)
{
    uint32_t samples[OVERSAMPLE_COUNT];
    uint32_t tmp;
    int i, j;
    float sum = 0.0f;

    for (i = 0; i < (int)OVERSAMPLE_COUNT; i++)
        samples[i] = ReadCurrentAdcBlocking();

    for (i = 1; i < (int)OVERSAMPLE_COUNT; i++)
    {
        tmp = samples[i];
        j = i - 1;
        while (j >= 0 && samples[j] > tmp)
        {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = tmp;
    }

    for (i = 1; i < (int)OVERSAMPLE_COUNT - 1; i++)
        sum += (float)samples[i];

    return sum / (float)(OVERSAMPLE_COUNT - 2u);
}

void CurrentSensor_Init(void)
{
    zeroCounts = 0;
}

void CurrentSensor_CalibrateZero(void)
{
    /* Call only while confirmed at 0 A (PWM duty cycle 0 / driver off) - see header. */
    zeroCounts = (uint32_t)ReadFilteredCounts();
}

float CurrentSensor_Read(void)
{
    float counts = ReadFilteredCounts();
    float current = (counts - (float)zeroCounts) * VDDA_MV / (ADC_FULL_SCALE * 1000.0f * SHUNT_OHM * INA_GAIN);

    /* The sense tap sits after the direction relay: only one polarity is
     * physically possible here (see current_sensor.h). Clamp away any small
     * negative reading caused by noise around the zero-current baseline. */
    if (current < 0.0f)
        current = 0.0f;

    return current;
}
