#ifndef __COMMGR_H
#define __COMMGR_H

#include "stm32h5xx.h"
#include "tusb.h"

// Định nghĩa USB VID và PID cho Virtual COM Port (CDC)
#define USBD_VID     0x0483
#define USBD_PID     0x5740 // USB CDC PID

#define TUSB_DESC_LEN_CDC (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

// Định nghĩa số lượng Interface của USB CDC
enum
{
  ITF_NUM_CDC = 0,
  ITF_NUM_CDC_DATA,
  ITF_NUM_TOTAL
};

// Khai báo extern các cấu trúc Descriptor (định nghĩa thực tế ở ComMgr.c)
extern tusb_desc_device_t const desc_device;
extern uint8_t const desc_configuration[];
extern char const* string_desc_arr[];

// Khởi tạo phần cứng USB và TinyUSB Stack
void ComMgr_Init(void);

// Xử lý các tác vụ nền của TinyUSB (gọi trong main loop)
void ComMgr_Process(void);

// Gửi dữ liệu qua USB Virtual COM
void ComMgr_SendData(void const *data, uint32_t length);

// Lấy kênh Rx được chọn từ SonarViewer
uint32_t ComMgr_GetRxSelect(void);

#endif /* __COMMGR_H */
