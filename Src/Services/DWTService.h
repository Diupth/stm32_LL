#ifndef DWTSERVICE_H
#define DWTSERVICE_H

#include <stdint.h>
#include "stm32h5xx.h"

#ifdef SHOW_TIMING_LOG

static inline void DWTService_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t DWTService_GetCycles(void)
{
    return DWT->CYCCNT;
}

static inline uint32_t DWTService_CyclesToUs(uint32_t cycles)
{
    return cycles / (SystemCoreClock / 1000000U);
}

#endif /* SHOW_TIMING_LOG */

#endif /* DWTSERVICE_H */
