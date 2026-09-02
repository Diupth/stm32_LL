#ifndef __LOGSERVICE_H
#define __LOGSERVICE_H

#include "stm32h5xx.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LOG_LEVEL_PRODUCTION = 0,   // LOGP - Production log
    LOG_LEVEL_DEV,              // LOGD - Development log
    LOG_LEVEL_BUG               // LOGB - Bug/Error log
} LogLevel_t;

/**
 * @brief Khởi tạo LogService
 */
void LogService_Init(void);

/**
 * @brief Lấy timestamp hệ thống tính bằng microsecond (us)
 */
uint64_t LogService_GetTimestampUs(void);

/**
 * @brief Ghi log qua ComMgr với định dạng timestamp (us), file, line và message.
 * 
 * @param level Mức log (LOG_LEVEL_PRODUCTION, LOG_LEVEL_DEV, hoặc LOG_LEVEL_BUG)
 * @param file Tên file gọi log
 * @param line Dòng code gọi log
 * @param fmt Định dạng chuỗi (printf style)
 * @param ... Các đối số
 */
void LogService_Log(LogLevel_t level, const char *file, uint32_t line, const char *fmt, ...);

/**
 * @brief Macro LOGP - Log Production (luôn hoạt động)
 * Định dạng output: [stamp][LOGP][file:line] message\r\n
 */
#define LOGP(...) LogService_Log(LOG_LEVEL_PRODUCTION, __FILE__, __LINE__, __VA_ARGS__)

/**
 * @brief Macro LOGD - Log Development (chỉ hoạt động khi bật LOG_DEV)
 * Định dạng output: [stamp][LOGD][file:line] message\r\n
 */
#ifdef LOG_DEV
#define LOGD(...) LogService_Log(LOG_LEVEL_DEV, __FILE__, __LINE__, __VA_ARGS__)
#else
#define LOGD(...) ((void)0)
#endif

/**
 * @brief Macro LOGB - Log Bug/Debug (chỉ hoạt động khi bật LOG_BUG)
 * Định dạng output: [stamp][LOGB][file:line] message\r\n
 */
#ifdef LOG_BUG
#define LOGB(...) LogService_Log(LOG_LEVEL_BUG, __FILE__, __LINE__, __VA_ARGS__)
#else
#define LOGB(...) ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __LOGSERVICE_H */
