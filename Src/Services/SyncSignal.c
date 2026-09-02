#include "SyncSignal.h"
#include "stm32h5xx.h"

// Timer 6 is the master sampling clock for the signal chain.
// It runs at 160 kHz and emits a TRGO pulse every time the counter overflows.
// The ADC and DAC are both configured to use this trigger, so they stay
// synchronized.

void SyncSignal_Init(void) {
  // 1. Enable the Timer 6 clock.
  RCC->APB1LENR |= RCC_APB1LENR_TIM6EN;

  // Read-back ensures the peripheral clock is active before register writes.
  (void)RCC->APB1LENR;

  // 2. Timer configuration:
  // SYSCLK = 240 MHz, APB1 prescaler = 1, so Timer 6 runs from 240 MHz.
  // Desired sample rate = 96 kHz => period = 2500 timer ticks.
  // PSC = 0 -> no prescale; ARR = 2500 - 1 = 2499.
  TIM6->PSC = 0U;
  TIM6->ARR = 2499U;

  // 3. Configure TRGO output on update event.
  // MMS=2 means: on Update Event, the timer sends an update trigger to slave
  // peripherals.
  TIM6->CR2 &= ~TIM_CR2_MMS_Msk;
  TIM6->CR2 |= (2U << TIM_CR2_MMS_Pos);

  // 4. Force an Update generation so PSC/ARR are loaded immediately.
  TIM6->EGR |= TIM_EGR_UG;

  // 5. Clear the update interrupt flag produced by the forced update.
  TIM6->SR &= ~TIM_SR_UIF;

  // 6. Start the timer.
  TIM6->CR1 |= TIM_CR1_CEN;
}

// Return the current value of Timer 6's counter.
uint32_t SyncSignal_GetTimerCounter(void) { return TIM6->CNT; }

// Return true while the timer is running, false otherwise.
bool SyncSignal_IsTimerEnabled(void) { return (TIM6->CR1 & TIM_CR1_CEN) != 0U; }

#ifdef SHOW_ADC_DAC_DEBUG
#include "ADCService.h"
#include "ComMgr.h"
#include "DACService.h"
#include "LogService.h"

void SyncSignal_SendADCDACDebug(uint32_t counter) {
  uint32_t tick = HAL_GetTick();
  uint32_t active_rx = ComMgr_GetRxSelect();
  if (active_rx == 0U) {
    active_rx = 1U;
  }
  uint32_t adc_count = ADCService_GetCompletedCount(active_rx);
  uint32_t dac_count = DACService_GetCompletedCount();
  uint32_t timer_counter = SyncSignal_GetTimerCounter();
  bool timer_enabled = SyncSignal_IsTimerEnabled();

  uint32_t overrun_cnt = ADCService_GetOverrunCount(active_rx);
  uint32_t dma_err_cnt = ADCService_GetDmaErrorCount(active_rx);
  uint32_t restart_cnt = ADCService_GetRestartCount(active_rx);

  LOGD("ADC/DAC Debug [#%lu]: \ntick=%lums, \nrx=%lu, \nadc_cnt=%lu, \ndac_cnt=%lu, \ntim_cnt=%lu, \ntim_en=%d, \novr=%lu, \ndma_err=%lu, \nrst=%lu",
       counter, tick, active_rx, adc_count, dac_count, timer_counter, (int)timer_enabled, overrun_cnt, dma_err_cnt, restart_cnt);
}
#endif

#ifdef SHOW_SAMPLING_LOG
#include "ADCService.h"
#include "DACService.h"
#include "LogService.h"

void SyncSignal_SendSamplingLog(uint32_t counter) {
  uint32_t adc1_pri_us = ADCService_GetFramePeriodUs(1U);
  uint32_t adc2_pri_us = ADCService_GetFramePeriodUs(2U);
  uint32_t dac_pri_us = DACService_GetFramePeriodUs();

  uint32_t adc1_fs_hz = adc1_pri_us == 0U ? 0U : (ADC_FRAME_SAMPLE_COUNT * 1000000U) / adc1_pri_us;
  uint32_t adc2_fs_hz = adc2_pri_us == 0U ? 0U : (ADC_FRAME_SAMPLE_COUNT * 1000000U) / adc2_pri_us;
  uint32_t dac_fs_hz = dac_pri_us == 0U ? 0U : (DAC_SAMPLE_COUNT * 1000000U) / dac_pri_us;

  LOGD("Sampling Log [#%lu]: \nADC1=%luHz (%luus), \nADC2=%luHz (%luus), \nDAC=%luHz (%luus)",
       counter, adc1_fs_hz, adc1_pri_us, adc2_fs_hz, adc2_pri_us, dac_fs_hz, dac_pri_us);
}
#endif

