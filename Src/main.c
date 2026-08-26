#include "stm32h5xx.h"
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

    // Khởi tạo ComMgr (đã bao gồm USB và UART với baudrate COMMGR_UART_BAUDRATE = 6 Mbps)
    ComMgr_Init();
    
    // Initialize applications
    Transmitter_Init();
    Receiver_Init();
    SyncSignalApp_Init(); // Initialize timer last to start conversions

    uint32_t last_uart_tick = HAL_GetTick();
    const char *msg = "hello world\r\n";

    while (1)
    {
        // 1. Gửi bản tin qua UART sau mỗi 1s bằng ComMgr (RingBuffer DMA 100% Non-blocking)
        if (HAL_GetTick() - last_uart_tick >= 1000)
        {
            last_uart_tick = HAL_GetTick();
            ComMgr_SendUartString(msg);
        }

        // 2. Process USB tasks (cdc flush, tud task)
        ComMgr_Process();

        // 3. Receive a physical frame and forward it over USB
        Receiver_Process();

        // 4. Send periodic synchronization telemetry
        SyncSignalApp_Process();
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

