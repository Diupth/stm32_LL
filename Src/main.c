#include "stm32h5xx.h"
#include "ComMgr.h"
#include "SystemClock.h"
#include "Receiver.h"
#include "Transmitter.h"
#include "SyncSignalApp.h"
#include <string.h>

void UART4_Init(uint32_t baudrate);
void UART4_SendString(const char *str);
void Error_Handler(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    HAL_ICACHE_Enable();

    // Khởi tạo UART4 (PB9: TX, PB8: RX) tốc độ tối đa 6 Mbps (6000000 bps)
    UART4_Init(6000000);

    ComMgr_Init();
    
    // Initialize applications
    Transmitter_Init();
    Receiver_Init();
    SyncSignalApp_Init(); // Initialize timer last to start conversions

    uint32_t last_uart_tick = HAL_GetTick();
    const char *msg = "hello world\r\n";

    while (1)
    {
        // 1. Gửi bản tin qua UART4 sau mỗi 1s
        if (HAL_GetTick() - last_uart_tick >= 1000)
        {
            last_uart_tick = HAL_GetTick();
            UART4_SendString(msg);
        }

        // 2. Process USB tasks (cdc flush, tud task)
        ComMgr_Process();

        // 3. Receive a physical frame and forward it over USB
        Receiver_Process();

        // 4. Send periodic synchronization telemetry
        SyncSignalApp_Process();
    }
}

// Cấu hình Low-Layer (Register-level) cho UART4 trên PB9 (TX) và PB8 (RX)
void UART4_Init(uint32_t baudrate)
{
    // 1. Cấp clock cho GPIOB và UART4 (UART4 nằm trên APB1L)
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    // Chọn nguồn clock cho UART4 là PCLK1 (000 trong CCIPR1.UART4SEL)
    RCC->CCIPR1 &= ~RCC_CCIPR1_UART4SEL_Msk;

    RCC->APB1LENR |= RCC_APB1LENR_UART4EN;
    (void)RCC->APB1LENR; // Đọc lại để đảm bảo clock đã tích cực

    // 2. Cấu hình chân PB8 (RX) và PB9 (TX) sang Alternate Function AF8 (UART4)
    // MODER: 10 = Alternate function mode
    GPIOB->MODER &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
    GPIOB->MODER |= ((2U << (8 * 2)) | (2U << (9 * 2)));

    // OSPEEDR: Very High Speed (11)
    GPIOB->OSPEEDR |= ((3U << (8 * 2)) | (3U << (9 * 2)));

    // PUPDR: Pull-up (01)
    GPIOB->PUPDR &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
    GPIOB->PUPDR |= ((1U << (8 * 2)) | (1U << (9 * 2)));

    // AFR: AF8 (0x8) cho PB8 và PB9 trong AFR[1] (AFRH)
    GPIOB->AFR[1] &= ~((0xFU << ((8 - 8) * 4)) | (0xFU << ((9 - 8) * 4)));
    GPIOB->AFR[1] |= ((8U << ((8 - 8) * 4)) | (8U << ((9 - 8) * 4)));

    // 3. Cấu hình UART4
    UART4->CR1 = 0U; // Tắt UART4 trước khi cấu hình
    
    // Tần số xung clock cấp cho UART4 (PCLK1 = 240 MHz theo clock config)
    // Với fCK = 240,000,000 Hz, OVER8 = 0 (oversampling 16): BRR = fCK / baudrate
    uint32_t fck = 240000000U;
    UART4->BRR = (fck + (baudrate / 2U)) / baudrate;

    // Kích hoạt Bộ phát (TE), Bộ nhận (RE) và Bật UART (UE)
    UART4->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void UART4_SendChar(char c)
{
    // Đợi thanh ghi truyền rỗng (TXFNF / TXE)
    while (!(UART4->ISR & USART_ISR_TXE_TXFNF))
    {
    }
    UART4->TDR = (uint8_t)c;
}

void UART4_SendString(const char *str)
{
    while (*str)
    {
        UART4_SendChar(*str++);
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
