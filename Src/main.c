#include "stm32h5xx.h"
#include "CrashHandler.h"
#include "ComMgr.h"
#include "SystemClock.h"
#include "Receiver.h"
#include "Transmitter.h"
#include "SyncSignalApp.h"
#include <string.h>

void Error_Handler(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    HAL_ICACHE_Enable();
    CrashHandler_Init();

    // Khởi tạo ComMgr (Sử dụng UART4 DMA 6 Mbps)
    ComMgr_Init();
    
    // Initialize applications
    Transmitter_Init();
    Receiver_Init();
    SyncSignalApp_Init(); // Initialize timer last to start conversions

    while (1)
    {
        // 1. Process communication tasks
        ComMgr_Process();

        // 2. Receive a physical frame and forward it over UART
        Receiver_Process();

        // 3. Send periodic synchronization telemetry and heartbeat log (1s)
        SyncSignalApp_Process();

        // 4. Cập nhật nhiễu simulation mỗi frame DAC (no-op ngoài SIMULATION_MODE)
        Transmitter_Process();
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

