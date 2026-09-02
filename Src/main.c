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
    ComMgr_SendString("\r\n"
                      " _____                        \r\n"
                      "/  ___|                       \r\n"
                      "\\ `--.  ___  _ __   __ _ _ __ \r\n"
                      " `--. \\/ _ \\| '_ \\ / _` | '__|\r\n"
                      "/\\__/ / (_) | | | | (_| | |   \r\n"
                      "\\____/ \\___/|_| |_|\\__,_|_|   \r\n"
                      "                              \r\n"
                      "STM32 Sonar DSP System Initializing...\r\n\r\n");
    
    // Initialize applications
    Transmitter_Init();
    ComMgr_SendString("[INIT] Transmitter OK\r\n");

    Receiver_Init();
    ComMgr_SendString("[INIT] Receiver OK\r\n");

    SyncSignalApp_Init(); // Initialize timer last to start conversions
    ComMgr_SendString("[INIT] SyncSignalApp OK\r\n");
    ComMgr_SendString("[SYS] System Ready - Pipeline Running\r\n\r\n");

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

