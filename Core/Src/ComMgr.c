#include "ComMgr.h"
#include "tusb.h"

// Khai báo ngoài để dùng hàm Error_Handler từ main.c
extern void Error_Handler(void);

// ====================================================================
// Cấu hình Low-Level USB cho STM32H5
// ====================================================================

void ComMgr_Init(void)
{
    // 1. Chọn nguồn xung nhịp HSI48 cho USB
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    // 2. Kích hoạt clock cổng GPIOA
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // 3. Cấu hình chân PA11 và PA12 cho USB (AF10)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_USB;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 4. Bật nguồn VDDUSB
    HAL_PWREx_EnableVddUSB();

    // 5. Bật clock ngoại vi USB
    __HAL_RCC_USB_CLK_ENABLE();

    // 6. Cấu hình độ ưu tiên ngắt và kích hoạt ngắt USB
    HAL_NVIC_SetPriority(USB_DRD_FS_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);

    // 7. Khởi tạo TinyUSB stack
    tusb_init();
}

void ComMgr_Process(void)
{
    tud_task();
}

void ComMgr_SendLEDState(uint8_t led_state)
{
    if (tud_cdc_connected())
    {
        if (led_state)
        {
            tud_cdc_write_str("LED State: ON\r\n");
        }
        else
        {
            tud_cdc_write_str("LED State: OFF\r\n");
        }
        tud_cdc_write_flush();
    }
}

// Hàm xử lý ngắt USB
void USB_DRD_FS_IRQHandler(void)
{
    tud_int_handler(0);
}


// ====================================================================
// USB Descriptors (Định nghĩa thiết bị USB CDC Virtual COM)
// ====================================================================

// Device Descriptor
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USBD_VID,
    .idProduct          = USBD_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Configuration Descriptor

#define EPNUM_CDC_NOTO    0x81
#define EPNUM_CDC_DATA    0x02

uint8_t const desc_configuration[] =
{
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_LEN_CDC, 0x00, 100),
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTO, 8, EPNUM_CDC_DATA, 0x80 + EPNUM_CDC_DATA, 64)
};

// String Descriptors
char const* string_desc_arr [] =
{
  (const char[]) { 0x09, 0x04 }, // 0: Supported language is English (0x0409)
  "WeAct Studio",                 // 1: Manufacturer
  "STM32H5 CDC LED",              // 2: Product
  "123456",                       // 3: Serials
  "WeAct CDC",                    // 4: CDC Interface
};

static uint16_t _desc_str[32];

// Các hàm Callbacks của TinyUSB để lấy thông tin Descriptor
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;
  uint8_t chr_count;

  if (index == 0)
  {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  }
  else
  {
    if (!(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0]))) return NULL;

    const char* str = string_desc_arr[index];

    chr_count = strlen(str);
    if (chr_count > 31) chr_count = 31;

    for (uint8_t i=0; i<chr_count; i++)
    {
      _desc_str[1+i] = str[i];
    }
  }

  _desc_str[0] = (TUSB_DESC_STRING << 8) | (2*chr_count + 2);
  return _desc_str;
}

uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}

uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index;
  return desc_configuration;
}
