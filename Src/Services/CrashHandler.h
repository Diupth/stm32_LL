#ifndef CRASH_HANDLER_H
#define CRASH_HANDLER_H

#include "stm32h5xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Khởi tạo Crash Handler (Kích hoạt UsageFault, BusFault, MemManage Fault và bẫy chia cho 0/unaligned access nếu cần)
 */
void CrashHandler_Init(void);

/**
 * @brief Gửi 1 ký tự trực tiếp qua UART4 bằng chế độ Polling (dành riêng cho tình huống khẩn cấp / crash)
 */
void CrashHandler_Emergency_PutChar(char c);

/**
 * @brief Gửi chuỗi string trực tiếp qua UART4 bằng chế độ Polling
 */
void CrashHandler_Emergency_Print(const char *str);

#ifdef __cplusplus
}
#endif

#endif // CRASH_HANDLER_H
