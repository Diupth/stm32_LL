#include "stm32h5xx.h"
#include "ComMgr.h"
#include "SystemClock.h"
#include "Led.h"
#include "FakedReceiver.h"

void Error_Handler(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    Led_Init();
    ComMgr_Init();
    FakedReceiver_Init();

    while (1)
    {
        ComMgr_Process();
        FakedReceiver_Process();
        Led_Process();
    }
}

// Ngắt SysTick dùng để tăng biến đếm thời gian hệ thống của HAL (uwTick)
void SysTick_Handler(void)
{
    HAL_IncTick();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
