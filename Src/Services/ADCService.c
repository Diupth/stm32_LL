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
__attribute__((aligned(32))) static uint16_t adc_double_buffers[2][ADC_DOUBLE_BUFFER_SIZE];

static volatile uint32_t adc_completed_counts[2] = {0U, 0U};
static volatile uint32_t adc_overrun_counts[2] = {0U, 0U};
static volatile uint32_t adc_dma_error_counts[2] = {0U, 0U};
static volatile uint32_t adc_restart_counts[2] = {0U, 0U};

// One bit per half-buffer. The DMA ISR publishes completed frames and the
// application consumes them independently, allowing acquisition to continue.
static volatile uint32_t ready_buffer_masks[2] = {0U, 0U};
#ifdef SHOW_SAMPLING_LOG
static volatile uint32_t adc_last_timestamps[2] = {0U, 0U};
static volatile uint32_t adc_frame_period_us[2] = {0U, 0U};

static void ADCService_EnableCycleCounter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void ADCService_RecordFrameTimestamp(uint32_t adc_index)
{
    uint32_t timestamp = DWT->CYCCNT;
    uint32_t idx = adc_index - 1U;
    if (adc_last_timestamps[idx] != 0U)
    {
        uint32_t delta = timestamp - adc_last_timestamps[idx];
        uint32_t divisor = SystemCoreClock / 1000000U;
        if (divisor > 0U)
        {
            adc_frame_period_us[idx] = (delta + (divisor / 2U)) / divisor;
        }
    }
    adc_last_timestamps[idx] = timestamp;
}
#endif

// Minimal LLI node used by GPDMA to reload the channel in circular mode.
typedef struct {
    uint32_t CBR1;
    uint32_t CSAR;
    uint32_t CDAR;
    uint32_t CLLR;
    uint32_t reserved[4]; // Pad to 32 bytes for alignment
} GPDMA_NodeTypeDef;

__attribute__((aligned(32))) static GPDMA_NodeTypeDef adc_dma_nodes[2];

static ADC_TypeDef *const adcs[2] = {ADC1, ADC2};
static DMA_Channel_TypeDef *const dma_channels[2] = {GPDMA1_Channel0, GPDMA1_Channel2};
static const IRQn_Type dma_irqs[2] = {GPDMA1_Channel0_IRQn, GPDMA1_Channel2_IRQn};
static const uint32_t option_masks[2] = {ADC_OR_OP0, ADC_OR_OP1};
static const IRQn_Type adc_irqs[2] = {ADC1_IRQn, ADC2_IRQn};

static void ADCService_CommonInit(
    ADC_TypeDef *adc,
    DMA_Channel_TypeDef *dma_channel,
    uint32_t reqsel,
    uint16_t *buffer,
    GPDMA_NodeTypeDef *dma_node,
    IRQn_Type dma_irq,
    uint32_t channel,
    uint32_t pin,
    uint32_t option_mask,
    IRQn_Type adc_irq
) {
    // 1. Enable GPIOA, ADC, and GPDMA1 clocks.
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA1EN;
    (void)RCC->AHB1ENR; // Read-back ensures the peripheral clock is active.

    // Set ADC common clock mode to Synchronous HCLK / 4 (60 MHz)
    ADC12_COMMON->CCR &= ~ADC_CCR_CKMODE_Msk;
    ADC12_COMMON->CCR |= (3U << ADC_CCR_CKMODE_Pos);

    // 2. Configure Pin in Analog Mode
    GPIOA->MODER |= (3U << (pin * 2));   // Analog mode for pin
    GPIOA->PUPDR &= ~(3U << (pin * 2));  // No pull-up/pull-down

    // 3. Configure GPDMA1 Channel for hardware-paced transfer
    dma_channel->CCR = 0U;

    // CTR1: 16-bit src/dest data width, src no increment, dest increment, dest port allocated to Port 1 (SRAM)
    dma_channel->CTR1 = (1U << DMA_CTR1_SDW_LOG2_Pos) | 
                        (1U << DMA_CTR1_DDW_LOG2_Pos) |
                        DMA_CTR1_DINC |
                        DMA_CTR1_DAP;

    // CTR2: REQSEL, DREQ = 0 (pacing by source)
    dma_channel->CTR2 = (reqsel << DMA_CTR2_REQSEL_Pos);

    // CSAR & CDAR
    dma_channel->CSAR = (uint32_t)&(adc->DR);
    dma_channel->CDAR = (uint32_t)buffer;

    // CBR1: 4096 samples * 2 bytes = 8192 bytes
    dma_channel->CBR1 = ADC_DOUBLE_BUFFER_SIZE * 2U;

    // Setup Node for Circular Mode looping back to itself (4 registers only)
    dma_node->CBR1 = dma_channel->CBR1;
    dma_node->CSAR = dma_channel->CSAR;
    dma_node->CDAR = dma_channel->CDAR;
    dma_node->CLLR = ((uint32_t)dma_node & DMA_CLLR_LA_Msk) | 
                     DMA_CLLR_ULL | DMA_CLLR_USA | DMA_CLLR_UDA | DMA_CLLR_UB1;

    // Load Link List settings into GPDMA Channel registers
    dma_channel->CLBAR = (uint32_t)dma_node & 0xFFFF0000U;
    dma_channel->CLLR = dma_node->CLLR;

    // Enable GPDMA Half Transfer (HTIE) and Transfer Complete (TCIE) interrupts
    dma_channel->CCR |= DMA_CCR_HTIE | DMA_CCR_TCIE;

    // Enable GPDMA Interrupt in NVIC
    HAL_NVIC_SetPriority(dma_irq, 0, 0);
    HAL_NVIC_EnableIRQ(dma_irq);

    // Enable GPDMA Channel
    dma_channel->CCR |= DMA_CCR_EN;

    // 4. Configure ADC
    // Disable Deep Power Down and enable ADC voltage regulator
    adc->CR &= ~ADC_CR_DEEPPWD;
    adc->CR |= ADC_CR_ADVREGEN;
    HAL_Delay(1);
    
    // Single-ended calibration
    adc->CR &= ~ADC_CR_ADCALDIF;
    adc->CR |= ADC_CR_ADCAL;
    while (adc->CR & ADC_CR_ADCAL) {}

    // Enable ADC
    adc->CR |= ADC_CR_ADEN;
    while (!(adc->ISR & ADC_ISR_ADRDY)) {}

    // Configure Rank 1 for channel, L=0 (1 conversion)
    adc->SQR1 = (channel << ADC_SQR1_SQ1_Pos) | (0U << ADC_SQR1_L_Pos);

    // Set sampling time for Channel to 81.5 ADC clock cycles (value 5)
    if (channel < 10U)
    {
        adc->SMPR1 &= ~(7U << (channel * 3U));
        adc->SMPR1 |= (5U << (channel * 3U));
    }
    else
    {
        adc->SMPR2 &= ~(7U << ((channel - 10U) * 3U));
        adc->SMPR2 |= (5U << ((channel - 10U) * 3U));
    }

    // Connect physical pin to ADC Channel
    adc->OR |= option_mask;

    // Configure External Trigger from TIM6 TRGO (value 13) on Rising Edge (value 1)
    // and enable DMA circular mode (DMAEN, DMACFG)
    adc->CFGR = (13U << ADC_CFGR_EXTSEL_Pos) | 
                (1U << ADC_CFGR_EXTEN_Pos) | 
                ADC_CFGR_DMAEN | 
                ADC_CFGR_DMACFG;

    // Enable ADC overrun interrupt
    adc->IER |= ADC_IER_OVRIE;
    HAL_NVIC_SetPriority(adc_irq, 0, 0);
    HAL_NVIC_EnableIRQ(adc_irq);

    // Start ADC conversion
    adc->CR |= ADC_CR_ADSTART;
}

void ADCService_Init(uint32_t adc_index)
{
    if (adc_index != 1U && adc_index != 2U)
    {
        return;
    }

    uint32_t idx = adc_index - 1U;

#ifdef SHOW_SAMPLING_LOG
    if (adc_index == 1U)
    {
        ADCService_EnableCycleCounter();
    }
#endif

    ADCService_CommonInit(
        adcs[idx],
        dma_channels[idx],
        idx, // reqsel (0U or 1U)
        adc_double_buffers[idx],
        &adc_dma_nodes[idx],
        dma_irqs[idx],
        idx, // channel (0U or 1U)
        idx, // pin (0U or 1U)
        option_masks[idx],
        adc_irqs[idx]
    );
}

uint32_t ADCService_GetCompletedCount(uint32_t adc_index)
{
    return (adc_index == 1U || adc_index == 2U) ? adc_completed_counts[adc_index - 1U] : 0U;
}

uint32_t ADCService_GetOverrunCount(uint32_t adc_index)
{
    return (adc_index == 1U || adc_index == 2U) ? adc_overrun_counts[adc_index - 1U] : 0U;
}

uint32_t ADCService_GetDmaErrorCount(uint32_t adc_index)
{
    return (adc_index == 1U || adc_index == 2U) ? adc_dma_error_counts[adc_index - 1U] : 0U;
}

uint32_t ADCService_GetRestartCount(uint32_t adc_index)
{
    return (adc_index == 1U || adc_index == 2U) ? adc_restart_counts[adc_index - 1U] : 0U;
}

#ifdef SHOW_SAMPLING_LOG
uint32_t ADCService_GetFramePeriodUs(uint32_t adc_index)
{
    return (adc_index == 1U || adc_index == 2U) ? adc_frame_period_us[adc_index - 1U] : 0U;
}
#endif

uint32_t ADCService_GetLastMinimum(uint32_t adc_index) { (void)adc_index; return 0U; }
uint32_t ADCService_GetLastMaximum(uint32_t adc_index) { (void)adc_index; return 0U; }

static bool ADCService_CommonRead(
    volatile uint32_t *ready_mask,
    uint16_t *double_buffer,
    int16_t *dest
) {
    uint32_t ready = *ready_mask;
    if (ready == 0U)
    {
        return false;
    }

    uint32_t buffer_bit = ready & (~ready + 1U);
    uint32_t offset = (buffer_bit == 1U) ? 0U : ADC_HALF_BUFFER_SIZE;

    // Claim the completed half atomically. Copying happens after re-enabling
    // interrupts, so the next half-buffer can be acquired without waiting.
    __disable_irq();
    if ((*ready_mask & buffer_bit) == 0U)
    {
        __enable_irq();
        return false;
    }
    *ready_mask &= ~buffer_bit;
    __enable_irq();

    for (uint32_t i = 0U; i < ADC_HALF_BUFFER_SIZE; i++)
    {
        dest[i] = (int16_t)double_buffer[offset + i];
    }

    return true;
}

bool ADCService_ReadFrame(uint32_t adc_index, int16_t *dest)
{
    if (adc_index == 1U || adc_index == 2U)
    {
        uint32_t idx = adc_index - 1U;
        return ADCService_CommonRead(&ready_buffer_masks[idx], adc_double_buffers[idx], dest);
    }
    return false;
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

    #ifdef SHOW_SAMPLING_LOG
        ADCService_RecordFrameTimestamp(1U);
    #endif
        ready_buffer_masks[0] |= 1U;
        adc_completed_counts[0]++;
    }
    
    if (csr & DMA_CSR_TCF)
    {
        // Clear full-transfer flag.
        GPDMA1_Channel0->CFCR |= DMA_CFCR_TCF;

    #ifdef SHOW_SAMPLING_LOG
        ADCService_RecordFrameTimestamp(1U);
    #endif
        ready_buffer_masks[0] |= 2U;
        adc_completed_counts[0]++;
    }

    // DMA error flags: data transfer error, unaligned/unsupported access, etc.
    if (csr & (DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF))
    {
        GPDMA1_Channel0->CFCR |= (DMA_CFCR_DTEF | DMA_CFCR_ULEF | DMA_CFCR_USEF);
        adc_dma_error_counts[0]++;
    }
}

void GPDMA1_Channel2_IRQHandler(void)
{
    uint32_t csr = GPDMA1_Channel2->CSR;

    if (csr & DMA_CSR_HTF)
    {
        GPDMA1_Channel2->CFCR |= DMA_CFCR_HTF;

    #ifdef SHOW_SAMPLING_LOG
        ADCService_RecordFrameTimestamp(2U);
    #endif
        ready_buffer_masks[1] |= 1U;
        adc_completed_counts[1]++;
    }
    
    if (csr & DMA_CSR_TCF)
    {
        GPDMA1_Channel2->CFCR |= DMA_CFCR_TCF;

    #ifdef SHOW_SAMPLING_LOG
        ADCService_RecordFrameTimestamp(2U);
    #endif
        ready_buffer_masks[1] |= 2U;
        adc_completed_counts[1]++;
    }

    if (csr & (DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF))
    {
        GPDMA1_Channel2->CFCR |= (DMA_CFCR_DTEF | DMA_CFCR_ULEF | DMA_CFCR_USEF);
        adc_dma_error_counts[1]++;
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
        adc_overrun_counts[0]++;
    }
}

void ADC2_IRQHandler(void)
{
    if (ADC2->ISR & ADC_ISR_OVR)
    {
        ADC2->ISR |= ADC_ISR_OVR;
        adc_overrun_counts[1]++;
    }
}
