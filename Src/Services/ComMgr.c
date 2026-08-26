#include "ComMgr.h"
#include "UARTDriver.h"
#include "ADCService.h"
#include "DACService.h"
#include "SyncSignal.h"
#include "Transmitter.h"
#include <string.h>

static char rx_command[32];
static uint32_t rx_command_length = 0U;
static uint32_t rx_select = 1U; // Mặc định truyền tín hiệu Rx1 ngay khi khởi động
static ComMgr_StreamMode stream_mode = COMMGR_STREAM_RAW;

// Xử lý từng byte nhận được từ UART (chạy trong ngắt UART4)
static void ComMgr_OnUartByteReceived(uint8_t character)
{
    if (character == '\n' || character == '\r')
    {
        if (rx_command_length > 0U)
        {
            rx_command[rx_command_length] = '\0';
            if (strcmp(rx_command, "cfg:lfm") == 0 || strcmp(rx_command, "cfg:barker13") == 0)
            {
                Transmitter_SetPulseType(TRANSMITTER_PULSE_LFM);
            }
            else if (strcmp(rx_command, "cfg:single") == 0)
            {
                Transmitter_SetPulseType(TRANSMITTER_PULSE_SINGLE);
            }
            else if (strncmp(rx_command, "rx_select:", 10) == 0)
            {
                rx_select = (uint32_t)(rx_command[10] - '0');
            }
            else if (strcmp(rx_command, "mode:raw") == 0)
            {
                stream_mode = COMMGR_STREAM_RAW;
            }
            else if (strcmp(rx_command, "mode:bpf") == 0)
            {
                stream_mode = COMMGR_STREAM_BPF;
            }
            else if (strcmp(rx_command, "mode:compressed") == 0)
            {
                stream_mode = COMMGR_STREAM_COMPRESSED;
            }
            rx_command_length = 0U;
        }
    }
    else if (rx_command_length < sizeof(rx_command) - 1U)
    {
        rx_command[rx_command_length++] = (char)character;
    }
    else
    {
        rx_command_length = 0U;
    }
}

void ComMgr_Init(void)
{
    // 1. Khởi tạo UARTDriver với tốc độ baud 6 Mbps (kèm RingBuffer TX DMA + Ngắt RX)
    UARTDriver_Init(COMMGR_UART_BAUDRATE);

    // 2. Đăng ký callback nhận lệnh từ SonarViewer qua UART
    UARTDriver_SetRxCallback(ComMgr_OnUartByteReceived);
}

void ComMgr_Process(void)
{
    // Không cần xử lý vòng lặp nặng, DMA và ISR đã làm ngầm 100%
}

uint32_t ComMgr_GetRxSelect(void)
{
    return rx_select;
}

ComMgr_StreamMode ComMgr_GetStreamMode(void)
{
    return stream_mode;
}

void ComMgr_SendData(void const *data, uint32_t length)
{
    // Gửi toàn bộ khung truyền (Signal, Telemetry, DSP, Debug) qua UART RingBuffer DMA
    UARTDriver_SendData(data, length);
}

void ComMgr_SendString(const char *str)
{
    UARTDriver_SendString(str);
}
