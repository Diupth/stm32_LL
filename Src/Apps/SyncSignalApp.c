#include "SyncSignalApp.h"

#include "ComMgr.h"
#ifdef SHOW_SAMPLING_LOG
#include "ADCService.h"
#include "DACService.h"
#endif
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
  if (now - last_log_tick < 1000U) {
    return;
  }

  last_log_tick = now;

  uint8_t log_frame[28] = {'L', 'O', 'G', '1', 0, 0, 0, 0};
  log_frame[8] = (uint8_t)(log_sequence & 0xFFU);
  log_frame[9] = (uint8_t)((log_sequence >> 8) & 0xFFU);
  log_frame[10] = (uint8_t)((log_sequence >> 16) & 0xFFU);
  log_frame[11] = (uint8_t)((log_sequence >> 24) & 0xFFU);

  uint32_t adc_pri_us = ADCService_GetFramePeriodUs();
  uint32_t dac_pri_us = DACService_GetFramePeriodUs();
  uint32_t adc_fs_hz = adc_pri_us == 0U ? 0U : (2048U * 1000000U) / adc_pri_us;
  uint32_t dac_fs_hz = dac_pri_us == 0U ? 0U : (2048U * 1000000U) / dac_pri_us;

  uint32_t values[] = {adc_fs_hz, adc_pri_us, dac_fs_hz, dac_pri_us};
  for (uint32_t i = 0U; i < 4U; i++) {
    for (uint32_t byte = 0U; byte < 4U; byte++) {
      log_frame[12U + i * 4U + byte] = (uint8_t)(values[i] >> (byte * 8U));
    }
  }

  ComMgr_SendData(log_frame, sizeof(log_frame));
  log_sequence++;
#endif

#ifdef SHOW_ADC_DAC_DEBUG
  SyncSignal_SendADCDACDebug(log_sequence);
#endif
}