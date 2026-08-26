#include "UARTDriver.h"
#include <string.h>

// Request ID cho UART4_TX trên GPDMA1 (định nghĩa chuẩn STM32H5)
#ifndef GPDMA1_REQUEST_UART4_TX
#define GPDMA1_REQUEST_UART4_TX 28U
#endif

// Bộ đệm vòng (Ring Buffer) 16KB dành riêng cho UART TX
#define UART_RING_BUFFER_SIZE 16384U
__attribute__((aligned(32))) static uint8_t uart_ring_buf[UART_RING_BUFFER_SIZE];

static volatile uint32_t uart_head = 0U; // Vị trí ghi vào của CPU
static volatile uint32_t uart_tail = 0U; // Vị trí đã gửi xong của DMA
static volatile uint32_t uart_dma_len = 0U; // Số byte DMA đang truyền
static volatile bool uart_dma_busy = false;

static void UARTDriver_StartDmaTransfer_Locked(void);

void UARTDriver_Init(uint32_t baudrate)
{
    // 1. Cấp clock cho GPIOB, UART4 (APB1L) và GPDMA1 (AHB1)
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPDMA1_CLK_ENABLE();

    // Chọn nguồn clock cho UART4 là PCLK1 (000 trong CCIPR1.UART4SEL)
    RCC->CCIPR1 &= ~RCC_CCIPR1_UART4SEL_Msk;

    __HAL_RCC_UART4_CLK_ENABLE();
    (void)RCC->APB1LENR; // Đọc lại để đảm bảo clock đã tích cực

    // 2. Cấu hình chân PB8 (RX) và PB9 (TX) sang Alternate Function AF8 (UART4) - Register level
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

    // 3. Cấu hình GPDMA1 Channel 3 cho UART4 TX
    // Channel 3 có độ ưu tiên thấp nhất (PRIO = 0: Low priority) để không cản trở ADC/DAC.
    GPDMA1_Channel3->CCR = 0U; // Tắt và reset kênh DMA
    GPDMA1_Channel3->CLLR = 0U; // Chế độ truyền đơn (không dùng Linked-List)
    GPDMA1_Channel3->CLBAR = 0U;

    // Xóa cờ ngắt cũ nếu có
    GPDMA1_Channel3->CFCR = DMA_CFCR_TCF | DMA_CFCR_HTF | DMA_CFCR_DTEF | DMA_CFCR_USEF | DMA_CFCR_ULEF | DMA_CFCR_TOF;

    // CTR1: 
    // - SDW_LOG2 = 0 (8-bit source data width)
    // - DDW_LOG2 = 0 (8-bit dest data width)
    // - SINC = 1 (Tăng địa chỉ nguồn trong RAM)
    // - DINC = 0 (Không tăng địa chỉ đích TDR)
    // - SAP = 1 (Đọc dữ liệu từ SRAM qua Port 1)
    // - DAP = 0 (Ghi dữ liệu ra ngoại vi APB qua Port 0)
    GPDMA1_Channel3->CTR1 = (0U << DMA_CTR1_SDW_LOG2_Pos) | 
                            (0U << DMA_CTR1_DDW_LOG2_Pos) | 
                            DMA_CTR1_SINC | 
                            DMA_CTR1_SAP;

    // CTR2: 
    // - REQSEL = 28 (GPDMA1_REQUEST_UART4_TX)
    // - DREQ = 1 (Ngoại vi UART4_TX đóng vai trò Flow Controller điều khiển nhịp truyền)
    GPDMA1_Channel3->CTR2 = (GPDMA1_REQUEST_UART4_TX << DMA_CTR2_REQSEL_Pos) | DMA_CTR2_DREQ;

    // Cấu hình địa chỉ đích cố định là thanh ghi UART4->TDR
    GPDMA1_Channel3->CDAR = (uint32_t)&(UART4->TDR);

    // Bật ngắt hoàn thành truyền (TCIE) và ngắt lỗi (USEIE, DTEIE)
    GPDMA1_Channel3->CCR = DMA_CCR_TCIE | DMA_CCR_USEIE | DMA_CCR_DTEIE;

    // Độ ưu tiên ngắt NVIC = 12 (thấp hơn nhiều so với ADC/DAC có Priority = 0)
    HAL_NVIC_SetPriority(GPDMA1_Channel3_IRQn, 12, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel3_IRQn);

    // 4. Cấu hình UART4
    UART4->CR1 = 0U; // Tắt UART4 trước khi cấu hình
    
    // Tần số xung clock cấp cho UART4 (PCLK1 = 240 MHz theo clock config)
    uint32_t fck = 240000000U;
    UART4->BRR = (fck + (baudrate / 2U)) / baudrate;

    // Bật chế độ DMA Transmitter trong CR3 (DMAT)
    UART4->CR3 |= USART_CR3_DMAT;

    // Kích hoạt Bộ phát (TE), Bộ nhận (RE), Ngắt nhận byte (RXNEIE) và Bật UART (UE)
    UART4->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_UE;

    // Cấu hình ngắt UART4 trong NVIC (độ ưu tiên thấp 12 để không chặn ADC/DAC)
    HAL_NVIC_SetPriority(UART4_IRQn, 12, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);

    uart_head = 0U;
    uart_tail = 0U;
    uart_dma_len = 0U;
    uart_dma_busy = false;
}

static UARTDriver_RxCallback uart_rx_cb = NULL;

void UARTDriver_SetRxCallback(UARTDriver_RxCallback cb)
{
    uart_rx_cb = cb;
}

uint32_t UARTDriver_GetTxFree(void)
{
    uint32_t head = uart_head;
    uint32_t tail = uart_tail;
    if (head >= tail)
    {
        return (UART_RING_BUFFER_SIZE - 1U) - (head - tail);
    }
    return (tail - head) - 1U;
}

// Bắt đầu một transaction DMA truyền từ uart_tail đến uart_head (chạy trong critical section)
static void UARTDriver_StartDmaTransfer_Locked(void)
{
    if (uart_dma_busy || (uart_head == uart_tail))
    {
        return;
    }

    uint32_t len;
    if (uart_head > uart_tail)
    {
        len = uart_head - uart_tail;
    }
    else
    {
        // Chạm biên vòng tròn, truyền đoạn từ tail đến cuối buffer trước
        len = UART_RING_BUFFER_SIZE - uart_tail;
    }

    uart_dma_len = len;
    uart_dma_busy = true;

    // Cấu hình thanh ghi DMA Channel 3
    GPDMA1_Channel3->CCR = 0U;
    GPDMA1_Channel3->CFCR = DMA_CFCR_TCF | DMA_CFCR_HTF | DMA_CFCR_DTEF | DMA_CFCR_USEF | DMA_CFCR_ULEF | DMA_CFCR_TOF;
    GPDMA1_Channel3->CSAR = (uint32_t)&uart_ring_buf[uart_tail];
    GPDMA1_Channel3->CDAR = (uint32_t)&(UART4->TDR);
    GPDMA1_Channel3->CBR1 = (len & DMA_CBR1_BNDT);
    GPDMA1_Channel3->CCR = DMA_CCR_TCIE | DMA_CCR_USEIE | DMA_CCR_DTEIE | DMA_CCR_EN;
}

void UARTDriver_SendData(const void *data, uint32_t length)
{
    if (data == NULL || length == 0U) return;

    const uint8_t *src = (const uint8_t *)data;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // Kiểm tra dung lượng trống
    uint32_t free_space = UARTDriver_GetTxFree();
    if (length > free_space)
    {
        length = free_space;
    }

    if (length > 0U)
    {
        // Đẩy dữ liệu vào Ring Buffer
        for (uint32_t i = 0U; i < length; i++)
        {
            uart_ring_buf[uart_head] = src[i];
            uart_head = (uart_head + 1U) % UART_RING_BUFFER_SIZE;
        }

        // Kích hoạt DMA nếu đang rảnh
        if (!uart_dma_busy)
        {
            UARTDriver_StartDmaTransfer_Locked();
        }
    }

    if (!primask)
    {
        __enable_irq();
    }
}

void UARTDriver_SendString(const char *str)
{
    if (str == NULL) return;
    UARTDriver_SendData(str, (uint32_t)strlen(str));
}

void UARTDriver_SendChar(char c)
{
    UARTDriver_SendData(&c, 1U);
}

// Ngắt DMA Channel 3 khi hoàn tất truyền dữ liệu UART
void GPDMA1_Channel3_IRQHandler(void)
{
    uint32_t csr = GPDMA1_Channel3->CSR;

    if (csr & DMA_CSR_TCF)
    {
        GPDMA1_Channel3->CFCR |= DMA_CFCR_TCF;

        // Cập nhật vị trí tail đã truyền xong
        uart_tail = (uart_tail + uart_dma_len) % UART_RING_BUFFER_SIZE;
        uart_dma_busy = false;

        // Nếu còn dữ liệu chưa truyền hết trong Ring Buffer, kích hoạt DMA lượt tiếp theo ngay lập tức
        if (uart_head != uart_tail)
        {
            UARTDriver_StartDmaTransfer_Locked();
        }
    }

    if (csr & (DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF))
    {
        GPDMA1_Channel3->CFCR |= (DMA_CFCR_DTEF | DMA_CFCR_ULEF | DMA_CFCR_USEF);
        uart_dma_busy = false;
    }
}

// Ngắt UART4 khi nhận dữ liệu (RXNE) hoặc lỗi đường truyền
void UART4_IRQHandler(void)
{
    uint32_t isr = UART4->ISR;

    // Đọc byte nhận được
    if (isr & USART_ISR_RXNE_RXFNE)
    {
        uint8_t byte = (uint8_t)(UART4->RDR);
        if (uart_rx_cb != NULL)
        {
            uart_rx_cb(byte);
        }
    }

    // Xóa cờ lỗi nếu có (Overrun, Framing, Noise)
    if (isr & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE))
    {
        UART4->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;
    }
}
