#ifndef ADCSERVICE_H
#define ADCSERVICE_H

#include <stdint.h>
#include <stdbool.h>

// ADC service for sampling an analog input using ADC + DMA.
// The ADC continuously fills a 4096-sample double buffer in circular mode.
// Each half-buffer represents one 2048-sample frame, which can be read by the caller.

void ADCService_Init(uint32_t adc_index);

// Returns the number of completed half-buffer transfers.
uint32_t ADCService_GetCompletedCount(uint32_t adc_index);

// Returns error counters for ADC overrun and DMA issues.
uint32_t ADCService_GetOverrunCount(uint32_t adc_index);
uint32_t ADCService_GetDmaErrorCount(uint32_t adc_index);
uint32_t ADCService_GetRestartCount(uint32_t adc_index);

// Last min/max values recorded in the last frame. Not yet implemented.
uint32_t ADCService_GetLastMinimum(uint32_t adc_index);
uint32_t ADCService_GetLastMaximum(uint32_t adc_index);

bool ADCService_ReadFrame(uint32_t adc_index, int16_t *dest);

#ifdef SHOW_SAMPLING_LOG
uint32_t ADCService_GetFramePeriodUs(uint32_t adc_index);
#endif

#endif /* ADCSERVICE_H */
