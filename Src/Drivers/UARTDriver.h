#ifndef __UART_DRIVER_H
#define __UART_DRIVER_H

#include "stm32h5xx.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * @brief Khởi tạo UART4 và GPDMA1 Channel 3 với RingBuffer TX (PB9: TX, PB8: RX)
 * @param baudrate Tốc độ baud (vd: 6000000)
 */
void UARTDriver_Init(uint32_t baudrate);

/**
 * @brief Kiểm tra số byte còn trống trong RingBuffer TX của UART
 * @return Số byte trống khả dụng
 */
uint32_t UARTDriver_GetTxFree(void);

/**
 * @brief Đẩy chuỗi ký tự vào RingBuffer và tự động kích hoạt DMA truyền ngầm (Non-blocking)
 * @param str Chuỗi null-terminated
 */
void UARTDriver_SendString(const char *str);

/**
 * @brief Đẩy mảng dữ liệu vào RingBuffer và tự động kích hoạt DMA truyền ngầm (Non-blocking)
 * @param data Con trỏ dữ liệu
 * @param length Chiều dài dữ liệu (bytes)
 */
void UARTDriver_SendData(const void *data, uint32_t length);

/**
 * @brief Gửi 1 byte qua UART
 * @param c Ký tự / byte cần gửi
 */
void UARTDriver_SendChar(char c);

#endif /* __UART_DRIVER_H */
