#ifndef ADCSERVICE_H
#define ADCSERVICE_H

#include <stdint.h>
#include <stdbool.h>

void ADCService_Init(void);

uint32_t ADCService_GetCompletedCount(void);
uint32_t ADCService_GetOverrunCount(void);
uint32_t ADCService_GetDmaErrorCount(void);
uint32_t ADCService_GetRestartCount(void);
uint32_t ADCService_GetLastMinimum(void);
uint32_t ADCService_GetLastMaximum(void);

// Read the latest converted 2048-sample frame.
// Returns true if a new frame was copied to dest.
bool ADCService_ReadFrame(int16_t *dest);

#endif /* ADCSERVICE_H */
