#include "SyncSignalApp.h"

#include "SyncSignal.h"
#include "stm32h5xx.h"

#ifdef SHOW_SAMPLING_LOG
static uint32_t log_sequence;
static uint32_t last_log_tick;
#endif

void SyncSignalApp_Init(void) {
  SyncSignal_Init();
#ifdef SHOW_SAMPLING_LOG
  last_log_tick = HAL_GetTick();
  log_sequence = 0U;
#endif
}

void SyncSignalApp_Process(void) {
#ifdef SHOW_SAMPLING_LOG
  uint32_t now = HAL_GetTick();
  if (now - last_log_tick >= 1000U) {
    last_log_tick = now;
    SyncSignal_SendSamplingLog(log_sequence);
    log_sequence++;
  }
#endif

#ifdef SHOW_ADC_DAC_DEBUG
  SyncSignal_SendADCDACDebug(log_sequence);
#endif
}