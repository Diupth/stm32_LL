#ifndef DACSERVICE_H
#define DACSERVICE_H

#include <stdint.h>

// DAC service for outputting a precomputed waveform through DAC1 Channel 1.
// The waveform is stored in a 2048-sample buffer and replayed continuously by DMA.

void DACService_Init(const uint16_t *samples, uint32_t sample_count);

// Number of full buffer cycles processed by the DAC DMA transfer.
uint32_t DACService_GetCompletedCount(void);
#ifdef SHOW_SAMPLING_LOG
uint32_t DACService_GetFramePeriodUs(void);
#endif

#endif /* DACSERVICE_H */
