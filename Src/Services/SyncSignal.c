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
  // Desired sample rate = 160 kHz => period = 1500 timer ticks.
  // PSC = 0 -> no prescale; ARR = 1500 - 1 = 1499.
  TIM6->PSC = 0U;
  TIM6->ARR = 1499U;

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

void SyncSignal_SendADCDACDebug(uint32_t counter) {
  /* DBG1 deliberately does not touch ADC, DAC, DMA, or SyncSignal. */
  uint8_t frame[164] = {'D', 'B', 'G', '1', 1U, 0U, 164U, 0U};
  uint32_t tick = HAL_GetTick();
  frame[8] = (uint8_t)(counter & 0xffU);
  frame[9] = (uint8_t)((counter >> 8U) & 0xffU);
  frame[10] = (uint8_t)((counter >> 16U) & 0xffU);
  frame[11] = (uint8_t)(counter >> 24U);
  frame[12] = (uint8_t)(tick & 0xffU);
  frame[13] = (uint8_t)((tick >> 8U) & 0xffU);
  frame[14] = (uint8_t)((tick >> 16U) & 0xffU);
  frame[15] = (uint8_t)(tick >> 24U);
  uint32_t adc_count = ADCService_GetCompletedCount();
  uint32_t dac_count = DACService_GetCompletedCount();
  for (uint32_t index = 0U; index < 4U; index++) {
    frame[16U + index] = (uint8_t)(adc_count >> (index * 8U));
    frame[20U + index] = (uint8_t)(dac_count >> (index * 8U));
  }
  uint32_t timer_counter = SyncSignal_GetTimerCounter();
  for (uint32_t index = 0U; index < 4U; index++) {
    frame[24U + index] = (uint8_t)(timer_counter >> (index * 8U));
  }
  frame[28] = SyncSignal_IsTimerEnabled() ? 1U : 0U;
  uint32_t registers[] = {ADC1->CR,
                          ADC1->CFGR,
                          ADC1->ISR,
                          DAC1->CR,
                          GPDMA1_Channel0->CCR,
                          GPDMA1_Channel0->CSR,
                          GPDMA1_Channel1->CCR,
                          GPDMA1_Channel1->CSR,
                          GPDMA1_Channel0->CLBAR,
                          GPDMA1_Channel0->CLLR,
                          GPDMA1_Channel0->CBR1,
                          GPDMA1_Channel1->CLBAR,
                          GPDMA1_Channel1->CLLR,
                          GPDMA1_Channel1->CBR1,
                          ADC1->SQR1,
                          ADC1->SMPR1,
                          ADC1->DR,
                          GPDMA1_Channel0->CTR1,
                          GPDMA1_Channel0->CTR2,
                          GPDMA1_Channel0->CTR3,
                          GPDMA1_Channel1->CTR1,
                          DAC1->DHR12R1,
                          TIM6->CR1,
                          TIM6->CR2,
                          DAC1->DOR1,
                          TIM6->ARR};
  for (uint32_t register_index = 0U; register_index < 25U; register_index++) {
    for (uint32_t byte_index = 0U; byte_index < 4U; byte_index++) {
      frame[32U + register_index * 4U + byte_index] =
          (uint8_t)(registers[register_index] >> (byte_index * 8U));
    }
  }
  uint32_t diagnostics[] = {ADCService_GetOverrunCount(),
                            ADCService_GetDmaErrorCount(),
                            ADCService_GetRestartCount(),
                            GPDMA1_Channel0->CTR3,
                            ADC1->IER,
                            ADCService_GetLastMinimum(),
                            ADCService_GetLastMaximum(),
                            GPDMA1_Channel0->CDAR,
                            GPDMA1_Channel0->CLLR,
                            ADCService_GetLastMinimum(),
                            ADCService_GetLastMaximum()};
  for (uint32_t diagnostic_index = 0U; diagnostic_index < 8U;
       diagnostic_index++) {
    for (uint32_t byte_index = 0U; byte_index < 4U; byte_index++) {
      frame[132U + diagnostic_index * 4U + byte_index] =
          (uint8_t)(diagnostics[diagnostic_index] >> (byte_index * 8U));
    }
  }
  ComMgr_SendData(frame, sizeof(frame));
}
#endif

#ifdef SHOW_SAMPLING_LOG
#include "ADCService.h"
#include "DACService.h"
#include "ComMgr.h"

void SyncSignal_SendSamplingLog(uint32_t counter) {
  uint8_t log_frame[28] = {'L', 'O', 'G', '1', 0, 0, 0, 0};
  log_frame[8] = (uint8_t)(counter & 0xFFU);
  log_frame[9] = (uint8_t)((counter >> 8) & 0xFFU);
  log_frame[10] = (uint8_t)((counter >> 16) & 0xFFU);
  log_frame[11] = (uint8_t)((counter >> 24) & 0xFFU);

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
}
#endif

