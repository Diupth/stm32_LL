#ifndef SYNCSIGNAL_H
#define SYNCSIGNAL_H

#include <stdbool.h>
#include <stdint.h>

// Common timing source used to synchronize ADC and DAC conversions.
// Timer 6 emits a periodic TRGO event at 160 kHz to drive the sample clock.

void SyncSignal_Init(void);

// Returns the current countdown value of Timer 6.
uint32_t SyncSignal_GetTimerCounter(void);

// Returns true if the sync timer is currently enabled and running.
bool SyncSignal_IsTimerEnabled(void);

#ifdef SHOW_ADC_DAC_DEBUG
// Prepares and sends debug telemetry frame
void SyncSignal_SendADCDACDebug(uint32_t counter);
#endif

#ifdef SHOW_SAMPLING_LOG
// Prepares and sends sampling log telemetry frame
void SyncSignal_SendSamplingLog(uint32_t counter);
#endif

#endif /* SYNCSIGNAL_H */
