#include "DACService.h"
#include "stm32h5xx.h"
#include <math.h>

#define DAC_BUFFER_SIZE 2048U
#define PI 3.14159265358979323846f

__attribute__((aligned(32))) static uint16_t dac_buffer[DAC_BUFFER_SIZE];
static volatile uint32_t dac_completed_count = 0;

// GPDMA Node structure (4 words for fast reload)
typedef struct {
    uint32_t CBR1;
    uint32_t CSAR;
    uint32_t CDAR;
    uint32_t CLLR;
} GPDMA_NodeTypeDef;

__attribute__((aligned(32))) static GPDMA_NodeTypeDef dac_dma_node;

void DACService_Init(void)
{
    // 1. Generate 300 Hz Sine Wave
    // fs = 160 kHz, 2048 samples
    for (uint32_t i = 0U; i < DAC_BUFFER_SIZE; i++)
    {
        float val = 2048.0f + 2000.0f * sinf(2.0f * PI * 300.0f * (float)i / 160000.0f);
        if (val < 0.0f) val = 0.0f;
        if (val > 4095.0f) val = 4095.0f;
        dac_buffer[i] = (uint16_t)val;
    }

    // 2. Enable GPIOA, DAC1, and GPDMA1 Clocks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA1EN;
    (void)RCC->AHB1ENR; // Read back to ensure clock is active

    // 3. Configure PA4 in Analog Mode
    GPIOA->MODER |= (3U << (4 * 2));   // Analog mode for PA4
    GPIOA->PUPDR &= ~(3U << (4 * 2));  // No pull-up/pull-down

    // 4. Configure GPDMA1 Channel 1 for DAC1 CH1 hardware-paced transfer
    // Clear channel control register
    GPDMA1_Channel1->CCR = 0U;

    // CTR1: 16-bit src, 32-bit dest data width, src increment, dest no increment, src port allocated to Port 1 (SRAM)
    // SDW_LOG2 = 1 (16-bit), DDW_LOG2 = 2 (32-bit)
    GPDMA1_Channel1->CTR1 = (1U << DMA_CTR1_SDW_LOG2_Pos) | 
                            (2U << DMA_CTR1_DDW_LOG2_Pos) |
                            DMA_CTR1_SINC |
                            DMA_CTR1_SAP;

    // CTR2: REQSEL = 2 (GPDMA1_REQUEST_DAC1_CH1), DREQ = 1 (pacing by destination)
    GPDMA1_Channel1->CTR2 = (2U << DMA_CTR2_REQSEL_Pos) | DMA_CTR2_DREQ;

    // CSAR & CDAR
    GPDMA1_Channel1->CSAR = (uint32_t)dac_buffer;
    GPDMA1_Channel1->CDAR = (uint32_t)&(DAC1->DHR12R1);

    // CBR1: 2048 samples * 2 bytes = 4096 bytes
    GPDMA1_Channel1->CBR1 = DAC_BUFFER_SIZE * 2U;

    // 5. Setup GPDMA Node for Circular Mode looping back to itself (4 registers only)
    dac_dma_node.CBR1 = GPDMA1_Channel1->CBR1;
    dac_dma_node.CSAR = GPDMA1_Channel1->CSAR;
    dac_dma_node.CDAR = GPDMA1_Channel1->CDAR;
    dac_dma_node.CLLR = ((uint32_t)&dac_dma_node & DMA_CLLR_LA_Msk) | 
                        DMA_CLLR_ULL | DMA_CLLR_USA | DMA_CLLR_UDA | DMA_CLLR_UB1;

    // Load Link List settings into GPDMA Channel 1 registers
    GPDMA1_Channel1->CLBAR = (uint32_t)&dac_dma_node & 0xFFFF0000U;
    GPDMA1_Channel1->CLLR = dac_dma_node.CLLR;

    // Enable GPDMA Transfer Complete Interrupt (TCIE) to track complete count
    GPDMA1_Channel1->CCR |= DMA_CCR_TCIE;

    // Enable GPDMA1 Channel 1 Interrupt in NVIC
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

    // Enable GPDMA1 Channel 1
    GPDMA1_Channel1->CCR |= DMA_CCR_EN;

    // 6. Configure DAC1 Channel 1
    // Connect to external pin, buffer enabled
    DAC1->MCR &= ~DAC_MCR_MODE1_Msk;

    // Enable trigger, select TIM6 TRGO (trigger 5: TSEL = 5, which is DAC_CR_TSEL1_2 | DAC_CR_TSEL1_0)
    DAC1->CR &= ~DAC_CR_TSEL1_Msk;
    DAC1->CR |= DAC_CR_TSEL1_2 | DAC_CR_TSEL1_0;
    DAC1->CR |= DAC_CR_TEN1 | DAC_CR_DMAEN1;

    // Preload the first sample (midpoint) to avoid transient 0V on first trigger
    DAC1->DHR12R1 = dac_buffer[0];

    // Enable DAC1 Channel 1
    DAC1->CR |= DAC_CR_EN1;
}

uint32_t DACService_GetCompletedCount(void)
{
    return dac_completed_count;
}

void GPDMA1_Channel1_IRQHandler(void)
{
    if (GPDMA1_Channel1->CSR & DMA_CSR_TCF)
    {
        // Clear Transfer Complete flag
        GPDMA1_Channel1->CFCR |= DMA_CFCR_TCF;
        dac_completed_count++;
    }
    
    // Clear other error flags if present to avoid stuck channel
    if (GPDMA1_Channel1->CSR & (DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF))
    {
        GPDMA1_Channel1->CFCR |= (DMA_CFCR_DTEF | DMA_CFCR_ULEF | DMA_CFCR_USEF);
    }
}
