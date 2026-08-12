#include "DACService.h"
#include "stm32h5xx.h"
#include <math.h>

// DAC waveform generator service.
// It builds a 2048-sample sine table and replays it continuously via DMA.
// The timer provides the sample clock, and the DAC updates the analog output at each trigger.

#define DAC_BUFFER_SIZE 2048U
#define PI 3.14159265358979323846f

// Sine wave buffer used as the DMA source for DAC1 Channel 1.
// 32-byte alignment is used to keep the buffer safe for hardware DMA transfers.
__attribute__((aligned(32))) static uint16_t dac_buffer[DAC_BUFFER_SIZE];
static volatile uint32_t dac_completed_count = 0;

// Minimal GPDMA linked-list node: only the fields required for circular reload are kept.
typedef struct {
    uint32_t CBR1; // Transfer size register
    uint32_t CSAR; // Source address register
    uint32_t CDAR; // Destination address register
    uint32_t CLLR; // Link-list register
} GPDMA_NodeTypeDef;

__attribute__((aligned(32))) static GPDMA_NodeTypeDef dac_dma_node;

void DACService_Init(void)
{
    // 1. Generate a 300 Hz sine wave table.
    // Sample rate = 160 kHz, buffer length = 2048 samples.
    for (uint32_t i = 0U; i < DAC_BUFFER_SIZE; i++)
    {
        // A 12-bit DAC uses values from 0 to 4095.
        // Mid-scale 2048 corresponds to 1.65V (reference center), with ±2000 amplitude.
        float val = 2048.0f + 2000.0f * sinf(2.0f * PI * 300.0f * (float)i / 160000.0f);
        if (val < 0.0f) val = 0.0f;
        if (val > 4095.0f) val = 4095.0f;
        dac_buffer[i] = (uint16_t)val;
    }

    // 2. Bật xung nhịp (clock) cho GPIOA, DAC1 và GPDMA1
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA1EN;
    (void)RCC->AHB1ENR; // Đọc lại để đảm bảo clock đã sẵn sàng hoạt động

    // 3. Cấu hình chân PA4 ở chế độ Analog Mode để xuất ngõ ra DAC
    GPIOA->MODER |= (3U << (4 * 2));   // Chế độ Analog (11)
    GPIOA->PUPDR &= ~(3U << (4 * 2));  // Không sử dụng điện trở kéo lên/kéo xuống (No Pull)

    // 4. Cấu hình GPDMA1 Channel 1 để truyền dữ liệu cho DAC1 Channel 1
    GPDMA1_Channel1->CCR = 0U; // Xóa thanh ghi điều khiển kênh trước khi cài đặt

    // CTR1:
    // Source Data Width = 16-bit (SDW_LOG2 = 1)
    // Destination Data Width = 32-bit (DDW_LOG2 = 2, do DAC register yêu cầu ghi 32-bit trên APB)
    // Tự động tăng địa chỉ nguồn SRAM (SINC)
    // Địa chỉ nguồn nằm ở SRAM nên phân bổ kênh đọc qua Port 1 (SAP)
    GPDMA1_Channel1->CTR1 = (1U << DMA_CTR1_SDW_LOG2_Pos) | 
                            (2U << DMA_CTR1_DDW_LOG2_Pos) |
                            DMA_CTR1_SINC |
                            DMA_CTR1_SAP;

    // CTR2:
    // REQSEL = 2 (Yêu cầu DMA từ phần cứng DAC1 CH1)
    // DREQ = 1 (Tốc độ truyền được kiểm soát bởi ngoại vi đích - Destination paced)
    GPDMA1_Channel1->CTR2 = (2U << DMA_CTR2_REQSEL_Pos) | DMA_CTR2_DREQ;

    // Gán địa chỉ nguồn (SRAM) và đích (Thanh ghi DHR12R1 của DAC1)
    GPDMA1_Channel1->CSAR = (uint32_t)dac_buffer;
    GPDMA1_Channel1->CDAR = (uint32_t)&(DAC1->DHR12R1);

    // CBR1: Số byte cần truyền trong 1 chu kỳ = 2048 mẫu * 2 bytes/mẫu = 4096 bytes
    GPDMA1_Channel1->CBR1 = DAC_BUFFER_SIZE * 2U;

    // 5. Cài đặt nút LLI để tự động lặp lại (Circular Mode) trỏ ngược về chính nó
    dac_dma_node.CBR1 = GPDMA1_Channel1->CBR1;
    dac_dma_node.CSAR = GPDMA1_Channel1->CSAR;
    dac_dma_node.CDAR = GPDMA1_Channel1->CDAR;
    dac_dma_node.CLLR = ((uint32_t)&dac_dma_node & DMA_CLLR_LA_Msk) | 
                        DMA_CLLR_ULL | DMA_CLLR_USA | DMA_CLLR_UDA | DMA_CLLR_UB1;

    // Nạp cấu hình LLI vào các thanh ghi của kênh GPDMA1_Channel1
    GPDMA1_Channel1->CLBAR = (uint32_t)&dac_dma_node & 0xFFFF0000U;
    GPDMA1_Channel1->CLLR = dac_dma_node.CLLR;

    // Bật ngắt khi hoàn thành truyền tải toàn bộ block (TCIE = Transfer Complete Interrupt Enable)
    GPDMA1_Channel1->CCR |= DMA_CCR_TCIE;

    // Cấu hình độ ưu tiên ngắt trong NVIC và cho phép ngắt GPDMA1 Channel 1 chạy
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

    // Bật kênh GPDMA1 Channel 1 bắt đầu chạy
    GPDMA1_Channel1->CCR |= DMA_CCR_EN;

    // 6. Cấu hình khối DAC1 Channel 1
    // Kết nối DAC ra chân ngoài PA4 và bật bộ đệm ngõ ra Analog (MODE1 = 000)
    DAC1->MCR &= ~DAC_MCR_MODE1_Msk;

    // Cài đặt Trigger ngoại vi từ Timer 6 TRGO (TSEL = 5 trên dòng H5, tức set bit TSEL1_2 và TSEL1_0)
    // Bật tính năng Trigger (TEN1) và cho phép yêu cầu DMA (DMAEN1)
    DAC1->CR &= ~DAC_CR_TSEL1_Msk;
    DAC1->CR |= DAC_CR_TSEL1_2 | DAC_CR_TSEL1_0;
    DAC1->CR |= DAC_CR_TEN1 | DAC_CR_DMAEN1;

    // Nạp trước mẫu đầu tiên (trung vị 2048) vào thanh ghi đệm để tránh bị tụt áp 0V khi trigger đầu tiên kích hoạt
    DAC1->DHR12R1 = dac_buffer[0];

    // Cho phép DAC1 Channel 1 hoạt động (EN1)
    DAC1->CR |= DAC_CR_EN1;
}

// Return how many times the DAC finished one full buffer cycle.
uint32_t DACService_GetCompletedCount(void)
{
    return dac_completed_count;
}

// GPDMA1 Channel 1 IRQ handler.
// Every time the DMA finishes one circular pass, the buffer has been replayed once.
void GPDMA1_Channel1_IRQHandler(void)
{
    // TCF is set when the full DMA transfer completes.
    if (GPDMA1_Channel1->CSR & DMA_CSR_TCF)
    {
        // Clear transfer complete flag.
        GPDMA1_Channel1->CFCR |= DMA_CFCR_TCF;
        dac_completed_count++;
    }
    
    // Clear DMA error flags to prevent the channel from getting stuck.
    if (GPDMA1_Channel1->CSR & (DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF))
    {
        GPDMA1_Channel1->CFCR |= (DMA_CFCR_DTEF | DMA_CFCR_ULEF | DMA_CFCR_USEF);
    }
}
