#ifndef ADCSERVICE_H
#define ADCSERVICE_H

#include <stdint.h>
#include <stdbool.h>

// ADC service for sampling an analog input using ADC + DMA.
// The ADC continuously fills a 4096-sample double buffer in circular mode.
// Each half-buffer represents one 2048-sample frame, which can be read by the caller.

void ADCService_Init(void);

// Returns the number of completed half-buffer transfers.
uint32_t ADCService_GetCompletedCount(void);

// Returns error counters for ADC overrun and DMA issues.
uint32_t ADCService_GetOverrunCount(void);
uint32_t ADCService_GetDmaErrorCount(void);
uint32_t ADCService_GetRestartCount(void);

// Last min/max values recorded in the last frame. Not yet implemented.
uint32_t ADCService_GetLastMinimum(void);
uint32_t ADCService_GetLastMaximum(void);

// Copy the newest completed 2048-sample frame into dest.
// Returns true when a new frame was available and copied.
bool ADCService_ReadFrame(int16_t *dest);
#ifdef SHOW_SAMPLING_LOG
uint32_t ADCService_GetFramePeriodUs(void);
#endif

#endif /* ADCSERVICE_H */
