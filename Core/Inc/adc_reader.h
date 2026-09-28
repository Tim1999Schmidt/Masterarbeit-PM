/**
 ******************************************************************************
 * @file    adc_reader.h
 * @brief   Blocking single-conversion read of exactly ONE ADC1 channel.
 *
 * Why this exists: MX_ADC1_Init() (CubeMX) puts BOTH ADC_CHANNEL_1 (TEMP) and
 * ADC_CHANNEL_2 (I_SENS_FILT) into the ADC's fixed channel sequencer, and
 * HAL_ADC_ConfigChannel(..., ADC_RANK_CHANNEL_NUMBER) only ever ADDS a channel
 * to that sequence - it never removes the other one (see HAL_ADC_ConfigChannel
 * in stm32c0xx_hal_adc.c). A plain HAL_ADC_Start()/PollForConversion() would
 * therefore always deliver the lowest-numbered channel first, i.e. TEMP, no
 * matter which channel a caller "selected". This helper removes the other
 * channel(s) from the sequencer first, so each caller really gets its own
 * channel. Shared by temperature_sensor.c and current_sensor.c so both use
 * exactly the same, single implementation.
 ******************************************************************************
 */
#ifndef ADC_READER_H
#define ADC_READER_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_READER_CH_TEMP      ADC_CHANNEL_1   /* TEMP        (PA1) */
#define ADC_READER_CH_CURRENT   ADC_CHANNEL_2   /* I_SENS_FILT (PA2) */

/* Returns the raw 12-bit result (0..4095) of one conversion of adcChannel.
 * Call from the main loop only (not from interrupts). */
uint32_t AdcReader_ReadBlocking(uint32_t adcChannel);

#ifdef __cplusplus
}
#endif

#endif /* ADC_READER_H */
