#include "SyncSignal.h"
#include "stm32h5xx.h"

void SyncSignal_Init(void)
{
    // 1. Enable TIM6 clock
    RCC->APB1LENR |= RCC_APB1LENR_TIM6EN;
    
    // Read back to ensure clock is enabled
    (void)RCC->APB1LENR;

    // 2. Configure Prescaler and Auto-Reload Register
    // SYSCLK = 240 MHz, HCLK = 240 MHz. APB1 Prescaler = 1, so TIM6 clock is 240 MHz.
    // 240,000,000 / 160,000 Hz = 1500 ticks.
    // ARR = 1500 - 1 = 1499.
    TIM6->PSC = 0U;
    TIM6->ARR = 1499U;

    // 3. Configure Trigger Output (TRGO) on Update Event
    // MMS bits (bits 4:6) in CR2 should be set to 010 (0x2) for Update Event
    TIM6->CR2 &= ~TIM_CR2_MMS_Msk;
    TIM6->CR2 |= (2U << TIM_CR2_MMS_Pos);

    // 4. Force Update Event to load PSC and ARR values
    TIM6->EGR |= TIM_EGR_UG;

    // 5. Clear update flag
    TIM6->SR &= ~TIM_SR_UIF;

    // 6. Enable TIM6 Counter
    TIM6->CR1 |= TIM_CR1_CEN;
}

uint32_t SyncSignal_GetTimerCounter(void)
{
    return TIM6->CNT;
}

bool SyncSignal_IsTimerEnabled(void)
{
    return (TIM6->CR1 & TIM_CR1_CEN) != 0U;
}
