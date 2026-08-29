#ifndef __COMMGR_H
#define __COMMGR_H

#include "stm32h5xx.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef enum
{
  COMMGR_STREAM_RAW = 0U,
  COMMGR_STREAM_BPF,
  COMMGR_STREAM_COMPRESSED,
  COMMGR_STREAM_DEMODULATED,
  COMMGR_STREAM_DOWNSAMPLING
} ComMgr_StreamMode;

// Cấu hình Baud Rate cho UART truyền nhận dữ liệu
#define COMMGR_UART_BAUDRATE 6000000U

// Khởi tạo ComMgr (UART Driver và các thiết lập truyền thông)
void ComMgr_Init(void);

// Xử lý các tác vụ nền nếu có (gọi trong main loop)
void ComMgr_Process(void);

// Gửi khung dữ liệu (frame/log/debug) qua UART (Non-blocking RingBuffer DMA)
void ComMgr_SendData(void const *data, uint32_t length);

// Gửi chuỗi ký tự qua UART
void ComMgr_SendString(const char *str);

// Lấy kênh Rx được chọn từ SonarViewer
uint32_t ComMgr_GetRxSelect(void);

// Lấy chế độ stream tín hiệu (Raw, BPF, Compressed) từ SonarViewer
ComMgr_StreamMode ComMgr_GetStreamMode(void);

#endif /* __COMMGR_H */
