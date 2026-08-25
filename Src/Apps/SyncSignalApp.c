#include "SyncSignalApp.h"

#include "SyncSignal.h"
#include "ADCService.h"
#include "ComMgr.h"
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

bool SyncSignalApp_HasFrames(void) {
  return ADCService_HasFrame(1U) && ADCService_HasFrame(2U);
}

bool SyncSignalApp_WaitForFrames(void) {
  /* Nếu cả 2 kênh đều chưa có dữ liệu thì không làm gì */
  if (!ADCService_HasFrame(1U) && !ADCService_HasFrame(2U)) {
    return false;
  }

  /* Khi có ít nhất 1 kênh sẵn sàng, đợi kênh còn lại để đảm bảo đồng bộ 2 kênh */
  while (!ADCService_HasFrame(1U) || !ADCService_HasFrame(2U)) {
    ComMgr_Process();
  }

  return true;
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