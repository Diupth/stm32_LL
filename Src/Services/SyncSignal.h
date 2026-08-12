#ifndef SYNCSIGNAL_H
#define SYNCSIGNAL_H

#include <stdint.h>
#include <stdbool.h>

void SyncSignal_Init(void);
uint32_t SyncSignal_GetTimerCounter(void);
bool SyncSignal_IsTimerEnabled(void);

#endif /* SYNCSIGNAL_H */
