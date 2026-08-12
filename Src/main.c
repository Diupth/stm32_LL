#include "stm32h5xx.h"
#include "ComMgr.h"
#include "SystemClock.h"
#include "SyncSignal.h"
#include "ADCService.h"
#include "DACService.h"

void Error_Handler(void);

static uint8_t usb_frame[4112];

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    ComMgr_Init();
    
    // Initialize services
    DACService_Init();
    ADCService_Init();
    SyncSignal_Init(); // Initialize timer last to start conversions

    // Initialize FRX1 frame header (16 bytes)
    usb_frame[0] = 'F';
    usb_frame[1] = 'R';
    usb_frame[2] = 'X';
    usb_frame[3] = '1';
    usb_frame[4] = (uint8_t)(2048U & 0xFFU);
    usb_frame[5] = (uint8_t)((2048U >> 8) & 0xFFU);
    // Bytes 6-15 set to 0 (padding for 16-byte header)
    for (int i = 6; i < 16; i++)
    {
        usb_frame[i] = 0U;
    }

    uint32_t last_log_tick = HAL_GetTick();
    uint32_t log_sequence = 0U;

    while (1)
    {
        // 1. Process USB tasks (cdc flush, tud task)
        ComMgr_Process();

        // 2. Check for new ADC frame
        if (ADCService_ReadFrame((int16_t *)&usb_frame[16]))
        {
            // Send binary signal frame
            ComMgr_SendData(usb_frame, sizeof(usb_frame));
        }

        // 3. Send periodic telemetry and diagnostics
        uint32_t now = HAL_GetTick();
        if (now - last_log_tick >= 1000U)
        {
            last_log_tick = now;

            // Send LOG1 telemetry frame (20 bytes)
            uint8_t log_frame[20] = { 'L', 'O', 'G', '1', 0, 0, 0, 0 };
            log_frame[8] = (uint8_t)(log_sequence & 0xFFU);
            log_frame[9] = (uint8_t)((log_sequence >> 8) & 0xFFU);
            log_frame[10] = (uint8_t)((log_sequence >> 16) & 0xFFU);
            log_frame[11] = (uint8_t)((log_sequence >> 24) & 0xFFU);

            uint32_t fs = 160000U;
            log_frame[12] = (uint8_t)(fs & 0xFFU);
            log_frame[13] = (uint8_t)((fs >> 8) & 0xFFU);
            log_frame[14] = (uint8_t)((fs >> 16) & 0xFFU);
            log_frame[15] = (uint8_t)((fs >> 24) & 0xFFU);

            uint32_t period = 6250U; // 6.25 us = 6250 ns
            log_frame[16] = (uint8_t)(period & 0xFFU);
            log_frame[17] = (uint8_t)((period >> 8) & 0xFFU);
            log_frame[18] = (uint8_t)((period >> 16) & 0xFFU);
            log_frame[19] = (uint8_t)((period >> 24) & 0xFFU);

            ComMgr_SendData(log_frame, sizeof(log_frame));

            // Send DBG1 debug frame (164 bytes) - Commented to remove unnecessary logs
            // ComMgr_SendDebug(log_sequence);

            log_sequence++;
        }
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
