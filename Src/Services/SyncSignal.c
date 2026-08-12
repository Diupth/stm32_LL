#include "SyncSignal.h"
#include "stm32h5xx.h"

// Timer 6 is the master sampling clock for the signal chain.
// It runs at 160 kHz and emits a TRGO pulse every time the counter overflows.
// The ADC and DAC are both configured to use this trigger, so they stay synchronized.

void SyncSignal_Init(void)
{
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
    // MMS=2 means: on Update Event, the timer sends an update trigger to slave peripherals.
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
uint32_t SyncSignal_GetTimerCounter(void)
{
    return TIM6->CNT;
}

// Return true while the timer is running, false otherwise.
bool SyncSignal_IsTimerEnabled(void)
{
    return (TIM6->CR1 & TIM_CR1_CEN) != 0U;
}
