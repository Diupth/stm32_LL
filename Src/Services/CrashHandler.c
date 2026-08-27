#include "CrashHandler.h"
#include <stdio.h>
#include <stdbool.h>

// Định nghĩa vùng nhớ Flash và SRAM của STM32H5 để kiểm tra địa chỉ hợp lệ trong Backtrace
#define FLASH_START_ADDR        0x08000000U
#define FLASH_SIZE_BYTES        (2U * 1024U * 1024U) // 2MB Flash max cho H5 series
#define FLASH_END_ADDR          (FLASH_START_ADDR + FLASH_SIZE_BYTES)

#define SRAM_START_ADDR         0x20000000U
#define SRAM_SIZE_BYTES         (640U * 1024U)       // 640KB SRAM tổng cho STM32H562
#define SRAM_END_ADDR           (SRAM_START_ADDR + SRAM_SIZE_BYTES)

// Cấu hình Clock và UART cho CrashHandler
#define CRASH_UART_PCLK_HZ      240000000U
#define CRASH_UART_BAUDRATE     6000000U
#define CRASH_UART_BRR_VAL      ((CRASH_UART_PCLK_HZ + (CRASH_UART_BAUDRATE / 2U)) / CRASH_UART_BAUDRATE)

#define CRASH_UART_TX_PIN       9U
#define CRASH_UART_TX_AF        8U // AF8 = UART4

#define STACK_FRAME_REG_COUNT   8U   // R0, R1, R2, R3, R12, LR, PC, xPSR
#define STACK_WALK_MAX_WORDS    128U // Quét tối đa 128 words (512 bytes) trên stack
#define UART_SETTLE_LOOP_COUNT  50000U

// Cấu trúc Exception Stack Frame được CPU tự động đẩy vào Stack
typedef struct {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
} ExceptionStackFrame_t;

void CrashHandler_Init(void)
{
    // Bật các bẫy lỗi cụ thể trong System Handler Control and State Register (SHCSR)
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk | 
                  SCB_SHCSR_BUSFAULTENA_Msk | 
                  SCB_SHCSR_USGFAULTENA_Msk;

    // Bật bẫy chia cho 0 (DIV_0_TRP) trong Configuration and Control Register (CCR)
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
}

void CrashHandler_Emergency_PutChar(char c)
{
    // Đảm bảo clock UART4 và GPIOB luôn bật
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();

    // Tự động thêm '\r' trước '\n' nếu cần
    if (c == '\n') {
        while (!(UART4->ISR & USART_ISR_TXE_TXFNF));
        UART4->TDR = (uint8_t)'\r';
    }
    while (!(UART4->ISR & USART_ISR_TXE_TXFNF));
    UART4->TDR = (uint8_t)c;
}

void CrashHandler_Emergency_Print(const char *str)
{
    if (str == NULL) return;
    while (*str) {
        CrashHandler_Emergency_PutChar(*str++);
    }
}

static void PrintHex32(uint32_t val)
{
    const char hex_chars[] = "0123456789ABCDEF";
    CrashHandler_Emergency_Print("0x");
    for (int i = 7; i >= 0; i--) {
        uint8_t nibble = (val >> (i * 4)) & 0x0FU;
        CrashHandler_Emergency_PutChar(hex_chars[nibble]);
    }
}

static bool IsValidCodeAddress(uint32_t addr)
{
    // Nằm trong Flash
    return (addr >= FLASH_START_ADDR && addr < FLASH_END_ADDR);
}

// Hàm C xử lý Crash Handler chính
void CrashHandler_Process(ExceptionStackFrame_t *frame, uint32_t lr_exc)
{
    // Tắt toàn bộ ngắt để tránh lồng ngắt khi crash
    __disable_irq();

    // Dừng kênh DMA UART4 TX nếu đang chạy
    GPDMA1_Channel3->CCR = 0U;

    // Bật clock GPIOB và UART4
    __HAL_RCC_GPIOB_CLK_ENABLE();
    RCC->CCIPR1 &= ~RCC_CCIPR1_UART4SEL_Msk;
    __HAL_RCC_UART4_CLK_ENABLE();
    (void)RCC->APB1LENR;

    // Cấu hình chân PB9 (TX) sang Alternate Function AF8 (UART4)
    GPIOB->MODER &= ~(3U << (CRASH_UART_TX_PIN * 2U));
    GPIOB->MODER |= (2U << (CRASH_UART_TX_PIN * 2U));
    GPIOB->OSPEEDR |= (3U << (CRASH_UART_TX_PIN * 2U));
    GPIOB->AFR[1] &= ~(0xFU << ((CRASH_UART_TX_PIN - 8U) * 4U));
    GPIOB->AFR[1] |= (CRASH_UART_TX_AF << ((CRASH_UART_TX_PIN - 8U) * 4U));

    // Cấu hình lại Baudrate 6 Mbps cho UART4
    UART4->CR1 = 0U;
    UART4->BRR = CRASH_UART_BRR_VAL;
    UART4->CR3 &= ~USART_CR3_DMAT;
    UART4->CR1 = USART_CR1_TE | USART_CR1_UE;

    // Đợi UART ổn định
    for (volatile uint32_t i = 0; i < UART_SETTLE_LOOP_COUNT; i++);

    CrashHandler_Emergency_Print("\r\n\r\n==================== [SYSTEM PANIC / CRASH] ====================\r\n");
    
    // In thanh ghi Exception Type & EXC_RETURN
    CrashHandler_Emergency_Print("EXC_RETURN : ");
    PrintHex32(lr_exc);
    CrashHandler_Emergency_Print((lr_exc & 0x4U) ? " (Using PSP)\r\n" : " (Using MSP)\r\n");

    // In thông tin lỗi từ SCB (CFSR, HFSR, BFAR, MMAR)
    uint32_t cfsr = SCB->CFSR;
    uint32_t hfsr = SCB->HFSR;

    CrashHandler_Emergency_Print("CFSR       : "); PrintHex32(cfsr); CrashHandler_Emergency_Print("\r\n");
    CrashHandler_Emergency_Print("HFSR       : "); PrintHex32(hfsr); CrashHandler_Emergency_Print("\r\n");

    // Chi tiết lỗi Configurable Fault Status
    if (cfsr & SCB_CFSR_USGFAULTSR_Msk) {
        CrashHandler_Emergency_Print("Fault Type : UsageFault (");
        if (cfsr & SCB_CFSR_DIVBYZERO_Msk)   CrashHandler_Emergency_Print("DIVBYZERO ");
        if (cfsr & SCB_CFSR_UNALIGNED_Msk)   CrashHandler_Emergency_Print("UNALIGNED ");
        if (cfsr & SCB_CFSR_NOCP_Msk)        CrashHandler_Emergency_Print("NOCP ");
        if (cfsr & SCB_CFSR_INVPC_Msk)       CrashHandler_Emergency_Print("INVPC ");
        if (cfsr & SCB_CFSR_INVSTATE_Msk)    CrashHandler_Emergency_Print("INVSTATE ");
        if (cfsr & SCB_CFSR_UNDEFINSTR_Msk)  CrashHandler_Emergency_Print("UNDEFINSTR ");
        CrashHandler_Emergency_Print(")\r\n");
    }
    if (cfsr & SCB_CFSR_BUSFAULTSR_Msk) {
        CrashHandler_Emergency_Print("Fault Type : BusFault (");
        if (cfsr & SCB_CFSR_BFARVALID_Msk) {
            CrashHandler_Emergency_Print("BFARVALID ");
            CrashHandler_Emergency_Print("Addr="); PrintHex32(SCB->BFAR); CrashHandler_Emergency_Print(" ");
        }
        if (cfsr & SCB_CFSR_PRECISERR_Msk)   CrashHandler_Emergency_Print("PRECISERR ");
        if (cfsr & SCB_CFSR_IMPRECISERR_Msk) CrashHandler_Emergency_Print("IMPRECISERR ");
        CrashHandler_Emergency_Print(")\r\n");
    }
    if (cfsr & SCB_CFSR_MEMFAULTSR_Msk) {
        CrashHandler_Emergency_Print("Fault Type : MemManageFault (");
        if (cfsr & SCB_CFSR_MMARVALID_Msk) {
            CrashHandler_Emergency_Print("MMARVALID ");
            CrashHandler_Emergency_Print("Addr="); PrintHex32(SCB->MMFAR); CrashHandler_Emergency_Print(" ");
        }
        CrashHandler_Emergency_Print(")\r\n");
    }

    // In Stack Frame Registers
    CrashHandler_Emergency_Print("\r\nCore Registers:\r\n");
    CrashHandler_Emergency_Print("  R0  : "); PrintHex32(frame->r0);
    CrashHandler_Emergency_Print("  R1  : "); PrintHex32(frame->r1);
    CrashHandler_Emergency_Print("  R2  : "); PrintHex32(frame->r2);
    CrashHandler_Emergency_Print("  R3  : "); PrintHex32(frame->r3);
    CrashHandler_Emergency_Print("\r\n");

    CrashHandler_Emergency_Print("  R12 : "); PrintHex32(frame->r12);
    CrashHandler_Emergency_Print("  SP  : "); PrintHex32((uint32_t)frame);
    CrashHandler_Emergency_Print("  LR  : "); PrintHex32(frame->lr);
    CrashHandler_Emergency_Print("  PC  : "); PrintHex32(frame->pc);
    CrashHandler_Emergency_Print("\r\n");

    CrashHandler_Emergency_Print("  xPSR: "); PrintHex32(frame->psr);
    CrashHandler_Emergency_Print("\r\n\r\n");

    // In Backtrace theo phong cách ESP32
    CrashHandler_Emergency_Print("Backtrace: ");
    
    // In vị trí PC và LR đầu tiên
    PrintHex32(frame->pc);
    CrashHandler_Emergency_Print(" ");
    
    if (IsValidCodeAddress(frame->lr)) {
        PrintHex32(frame->lr);
        CrashHandler_Emergency_Print(" ");
    }

    // Quét stack để tìm các Return Address (Stack Walk / Unwind)
    uint32_t *stack_ptr = (uint32_t *)frame + STACK_FRAME_REG_COUNT; // Bỏ qua registers của Exception Frame
    uint32_t stack_depth_limit = STACK_WALK_MAX_WORDS;

    while (stack_depth_limit-- > 0) {
        // Kiểm tra xem con trỏ stack có còn nằm trong SRAM hợp lệ không
        if ((uint32_t)stack_ptr < SRAM_START_ADDR || (uint32_t)stack_ptr >= SRAM_END_ADDR) {
            break;
        }

        uint32_t candidate_addr = *stack_ptr;
        if (IsValidCodeAddress(candidate_addr)) {
            PrintHex32(candidate_addr);
            CrashHandler_Emergency_Print(" ");
        }
        stack_ptr++;
    }

    CrashHandler_Emergency_Print("\r\n=================================================================\r\n");

    // Treo hệ thống hoặc khởi động lại (WDT / NVIC_SystemReset)
    while (1) {
        __NOP();
    }
}

// Các hàm Handler cấp thấp (Assembly Trampoline để lấy đúng SP và EXC_RETURN)
__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile (
        "tst lr, #4                        \n"
        "ite eq                            \n"
        "mrseq r0, msp                     \n"
        "mrsne r0, psp                     \n"
        "mov r1, lr                        \n"
        "b CrashHandler_Process            \n"
    );
}

__attribute__((naked)) void MemManage_Handler(void)
{
    __asm volatile (
        "tst lr, #4                        \n"
        "ite eq                            \n"
        "mrseq r0, msp                     \n"
        "mrsne r0, psp                     \n"
        "mov r1, lr                        \n"
        "b CrashHandler_Process            \n"
    );
}

__attribute__((naked)) void BusFault_Handler(void)
{
    __asm volatile (
        "tst lr, #4                        \n"
        "ite eq                            \n"
        "mrseq r0, msp                     \n"
        "mrsne r0, psp                     \n"
        "mov r1, lr                        \n"
        "b CrashHandler_Process            \n"
    );
}

__attribute__((naked)) void UsageFault_Handler(void)
{
    __asm volatile (
        "tst lr, #4                        \n"
        "ite eq                            \n"
        "mrseq r0, msp                     \n"
        "mrsne r0, psp                     \n"
        "mov r1, lr                        \n"
        "b CrashHandler_Process            \n"
    );
}
