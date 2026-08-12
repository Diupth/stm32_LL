#include "ADCService.h"
#include "stm32h5xx.h"

// ADC acquisition service.
// - PA0 is configured as analog input.
// - ADC1 converts samples at a fixed trigger rate from TIM6 TRGO.
// - GPDMA transfers converted values continuously into a circular double buffer.
// - Each half of the buffer represents one complete frame of 2048 samples.

#define ADC_HALF_BUFFER_SIZE 2048U
#define ADC_DOUBLE_BUFFER_SIZE (ADC_HALF_BUFFER_SIZE * 2U)

// Double buffer: 2 x 2048 samples, used as a ping-pong circular DMA target.
__attribute__((aligned(32))) static uint16_t adc_double_buffer[ADC_DOUBLE_BUFFER_SIZE];

static volatile uint32_t adc_completed_count = 0;
static volatile uint32_t adc_overrun_count = 0;
static volatile uint32_t adc_dma_error_count = 0;
static volatile uint32_t adc_restart_count = 0;

// Indicates if a half-buffer just finished and can be consumed by the application.
static volatile bool new_frame_ready = false;
static volatile uint32_t active_buffer_offset = 0;
#ifdef SHOW_SAMPLING_LOG
static volatile uint32_t adc_last_timestamp = 0;
static volatile uint32_t adc_frame_period_us = 0;

static void ADCService_EnableCycleCounter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void ADCService_RecordFrameTimestamp(void)
{
    uint32_t timestamp = DWT->CYCCNT;
    if (adc_last_timestamp != 0U)
    {
        adc_frame_period_us = (timestamp - adc_last_timestamp) / (SystemCoreClock / 1000000U);
    }
    adc_last_timestamp = timestamp;
}
#endif

// Minimal LLI node used by GPDMA to reload the channel in circular mode.
typedef struct {
    uint32_t CBR1;
    uint32_t CSAR;
    uint32_t CDAR;
    uint32_t CLLR;
} GPDMA_NodeTypeDef;

__attribute__((aligned(32))) static GPDMA_NodeTypeDef adc_dma_node;

void ADCService_Init(void)
{
#ifdef SHOW_SAMPLING_LOG
    ADCService_EnableCycleCounter();
#endif
    // 1. Enable GPIOA, ADC, and GPDMA1 clocks.
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA1EN;
    (void)RCC->AHB1ENR; // Read-back ensures the peripheral clock is active.

    // Set ADC common clock mode to Synchronous HCLK / 4 (60 MHz)
    ADC12_COMMON->CCR &= ~ADC_CCR_CKMODE_Msk;
    ADC12_COMMON->CCR |= (3U << ADC_CCR_CKMODE_Pos);

    // 2. Configure PA0 in Analog Mode
    GPIOA->MODER |= (3U << (0 * 2));   // Analog mode for PA0
    GPIOA->PUPDR &= ~(3U << (0 * 2));  // No pull-up/pull-down

    // 3. Configure GPDMA1 Channel 0 for ADC1 hardware-paced transfer
    GPDMA1_Channel0->CCR = 0U;

    // CTR1: 16-bit src/dest data width, src no increment, dest increment, dest port allocated to Port 1 (SRAM)
    // SDW_LOG2 = 1 (16-bit), DDW_LOG2 = 1 (16-bit)
    GPDMA1_Channel0->CTR1 = (1U << DMA_CTR1_SDW_LOG2_Pos) | 
                            (1U << DMA_CTR1_DDW_LOG2_Pos) |
                            DMA_CTR1_DINC |
                            DMA_CTR1_DAP;

    // CTR2: REQSEL = 0 (GPDMA1_REQUEST_ADC1), DREQ = 0 (pacing by source)
    GPDMA1_Channel0->CTR2 = (0U << DMA_CTR2_REQSEL_Pos);

    // CSAR & CDAR
    GPDMA1_Channel0->CSAR = (uint32_t)&(ADC1->DR);
    GPDMA1_Channel0->CDAR = (uint32_t)adc_double_buffer;

    // CBR1: 4096 samples * 2 bytes = 8192 bytes
    GPDMA1_Channel0->CBR1 = ADC_DOUBLE_BUFFER_SIZE * 2U;

    // Setup Node for Circular Mode looping back to itself (4 registers only)
    adc_dma_node.CBR1 = GPDMA1_Channel0->CBR1;
    adc_dma_node.CSAR = GPDMA1_Channel0->CSAR;
    adc_dma_node.CDAR = GPDMA1_Channel0->CDAR;
    adc_dma_node.CLLR = ((uint32_t)&adc_dma_node & DMA_CLLR_LA_Msk) | 
                        DMA_CLLR_ULL | DMA_CLLR_USA | DMA_CLLR_UDA | DMA_CLLR_UB1;

    // Load Link List settings into GPDMA Channel 0 registers
    GPDMA1_Channel0->CLBAR = (uint32_t)&adc_dma_node & 0xFFFF0000U;
    GPDMA1_Channel0->CLLR = adc_dma_node.CLLR;

    // Enable GPDMA Half Transfer (HTIE) and Transfer Complete (TCIE) interrupts
    GPDMA1_Channel0->CCR |= DMA_CCR_HTIE | DMA_CCR_TCIE;

    // Enable GPDMA1 Channel 0 Interrupt in NVIC
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);

    // Enable GPDMA1 Channel 0
    GPDMA1_Channel0->CCR |= DMA_CCR_EN;

    // 4. Configure ADC1
    // Disable Deep Power Down and enable ADC1 voltage regulator
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;
    HAL_Delay(1);
    
    // Single-ended calibration
    ADC1->CR &= ~ADC_CR_ADCALDIF;
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL) {}

    // Enable ADC1
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) {}

    // Configure Rank 1 for Channel 0, L=0 (1 conversion)
    ADC1->SQR1 = (0U << ADC_SQR1_SQ1_Pos) | (0U << ADC_SQR1_L_Pos);

    // Set sampling time for Channel 0 to 81.5 ADC clock cycles (value 5)
    ADC1->SMPR1 &= ~ADC_SMPR1_SMP0_Msk;
    ADC1->SMPR1 |= (5U << ADC_SMPR1_SMP0_Pos);

    // Connect physical pin PA0 to ADC1 Channel 0
    ADC1->OR |= ADC_OR_OP0;

    // Configure External Trigger from TIM6 TRGO (value 13) on Rising Edge (value 1)
    // and enable DMA circular mode (DMAEN, DMACFG)
    ADC1->CFGR = (13U << ADC_CFGR_EXTSEL_Pos) | 
                 (1U << ADC_CFGR_EXTEN_Pos) | 
                 ADC_CFGR_DMAEN | 
                 ADC_CFGR_DMACFG;

    // Enable ADC overrun interrupt
    ADC1->IER |= ADC_IER_OVRIE;
    HAL_NVIC_SetPriority(ADC1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_IRQn);

    // Start ADC conversion
    ADC1->CR |= ADC_CR_ADSTART;
}

uint32_t ADCService_GetCompletedCount(void)
{
    return adc_completed_count;
}

uint32_t ADCService_GetOverrunCount(void)
{
    return adc_overrun_count;
}

uint32_t ADCService_GetDmaErrorCount(void)
{
    return adc_dma_error_count;
}

uint32_t ADCService_GetRestartCount(void)
{
    return adc_restart_count;
}

#ifdef SHOW_SAMPLING_LOG
uint32_t ADCService_GetFramePeriodUs(void)
{
    return adc_frame_period_us;
}
#endif

uint32_t ADCService_GetLastMinimum(void)
{
    return 0U;
}

uint32_t ADCService_GetLastMaximum(void)
{
    return 0U;
}

bool ADCService_ReadFrame(int16_t *dest)
{
    if (!new_frame_ready)
    {
        return false;
    }

    uint32_t offset = active_buffer_offset;

    for (uint32_t i = 0U; i < ADC_HALF_BUFFER_SIZE; i++)
    {
        dest[i] = (int16_t)adc_double_buffer[offset + i];
    }
    new_frame_ready = false;

    return true;
}

void GPDMA1_Channel0_IRQHandler(void)
{
    uint32_t csr = GPDMA1_Channel0->CSR;

    // DMA raises HTF when the first half-buffer is filled and TCF when the second half is filled.
    // This allows the application to read a stable 2048-sample frame without stopping the ADC.
    if (csr & DMA_CSR_HTF)
    {
        // Clear half-transfer flag.
        GPDMA1_Channel0->CFCR |= DMA_CFCR_HTF;

        active_buffer_offset = 0U;
    #ifdef SHOW_SAMPLING_LOG
        ADCService_RecordFrameTimestamp();
    #endif
        new_frame_ready = true;
        adc_completed_count++;
    }
    
    if (csr & DMA_CSR_TCF)
    {
        // Clear full-transfer flag.
        GPDMA1_Channel0->CFCR |= DMA_CFCR_TCF;

        active_buffer_offset = ADC_HALF_BUFFER_SIZE;
    #ifdef SHOW_SAMPLING_LOG
        ADCService_RecordFrameTimestamp();
    #endif
        new_frame_ready = true;
        adc_completed_count++;
    }

    // DMA error flags: data transfer error, unaligned/unsupported access, etc.
    if (csr & (DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF))
    {
        GPDMA1_Channel0->CFCR |= (DMA_CFCR_DTEF | DMA_CFCR_ULEF | DMA_CFCR_USEF);
        adc_dma_error_count++;
    }
}

void ADC1_IRQHandler(void)
{
    // Overrun occurs when a new ADC conversion finishes before the previous data is read.
    // In circular DMA mode, this usually means the software is not draining the buffer quickly enough.
    if (ADC1->ISR & ADC_ISR_OVR)
    {
        // Clear overrun flag.
        ADC1->ISR |= ADC_ISR_OVR;
        adc_overrun_count++;
    }
}
