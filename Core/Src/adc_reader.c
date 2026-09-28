/**
 ******************************************************************************
 * @file    adc_reader.c
 * @brief   See adc_reader.h.
 ******************************************************************************
 */
#include "adc_reader.h"

extern ADC_HandleTypeDef hadc1; /* initialised by MX_ADC1_Init() in main.c */

#define ADC_CHANNEL_NONE_SELECTED   0xFFFFFFFFu

static uint32_t selectedChannel = ADC_CHANNEL_NONE_SELECTED;

/* Leaves exactly `channel` in the ADC's (fixed) channel sequencer. Must be
 * called while no conversion is running - true here, because every read below
 * ends with HAL_ADC_Stop(). */
static void SelectOnly(uint32_t channel)
{
    static const uint32_t usedChannels[] = { ADC_READER_CH_TEMP, ADC_READER_CH_CURRENT };
    ADC_ChannelConfTypeDef cfg = {0};
    uint32_t i;

    /* ADC_RANK_NONE removes a channel from the sequencer ... */
    cfg.Rank = ADC_RANK_NONE;
    for (i = 0; i < (sizeof(usedChannels) / sizeof(usedChannels[0])); i++)
    {
        if (usedChannels[i] != channel)
        {
            cfg.Channel = usedChannels[i];
            (void)HAL_ADC_ConfigChannel(&hadc1, &cfg);
        }
    }

    /* ... ADC_RANK_CHANNEL_NUMBER adds one. */
    cfg.Channel = channel;
    cfg.Rank = ADC_RANK_CHANNEL_NUMBER;
    (void)HAL_ADC_ConfigChannel(&hadc1, &cfg);

    selectedChannel = channel;
}

uint32_t AdcReader_ReadBlocking(uint32_t adcChannel)
{
    uint32_t value;

    if (adcChannel != selectedChannel)
        SelectOnly(adcChannel);

    (void)HAL_ADC_Start(&hadc1);
    (void)HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    value = HAL_ADC_GetValue(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);

    return value;
}
