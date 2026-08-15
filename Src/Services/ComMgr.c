#include "ComMgr.h"
#include "tusb.h"
#include "ADCService.h"
#include "DACService.h"
#include "SyncSignal.h"
#include "Transmitter.h"
#include <string.h>

// Khai báo ngoài để dùng hàm Error_Handler từ main.c
extern void Error_Handler(void);

#define COMMGR_TX_QUEUE_SIZE 16384U // Signal frame + telemetry, non-blocking.

static uint8_t tx_queue[COMMGR_TX_QUEUE_SIZE]; // Lưu tạm dữ liệu chờ USB truyền.
static uint32_t tx_queue_head;                 // Vị trí ghi dữ liệu mới vào queue.
static uint32_t tx_queue_tail;                 // Vị trí đọc dữ liệu cũ để gửi đi.
static char rx_command[32];
static uint32_t rx_command_length;
static uint32_t rx_select = 0; // Mặc định không gửi (Rx 0)
static volatile StreamMode_t stream_mode = STREAM_MODE_BPF; // Mặc định bật BPF (volatile để tránh tối ưu hóa ngắt)

// Tính số byte hiện đang chờ trong queue.
static uint32_t ComMgr_TxQueued(void)
{
  return (tx_queue_head - tx_queue_tail) % COMMGR_TX_QUEUE_SIZE;
}

// Tính số byte còn có thể ghi mà không làm mất dữ liệu đang chờ.
// Chừa lại một ô trống để phân biệt queue đầy và queue rỗng.
static uint32_t ComMgr_TxFree(void)
{
  return (COMMGR_TX_QUEUE_SIZE - 1U) - ComMgr_TxQueued();
}

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
    HAL_NVIC_SetPriority(USB_DRD_FS_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);

    // 7. Khởi tạo TinyUSB stack
    tusb_init();
}

void ComMgr_Process(void)
{
  /* TinyUSB's STM32 FS driver has no USB peripheral DMA path. This task only
   * drains the queue into the USB FIFO; ADC/DAC DMA never waits for it. */
  // Xử lý sự kiện USB trước, ví dụ: nhận packet, hoàn tất truyền packet.
    tud_task();

  // Chưa kết nối hoặc không có dữ liệu thì thoát ngay, không block main loop.
    /* CDC DTR is optional for a data-only stream. macOS may open the port
     * without asserting it, so do not suppress TX solely on connected(). */
    if (ComMgr_TxQueued() == 0U)
    {
        return;
    }

  // Chỉ lấy lượng dữ liệu mà FIFO của TinyUSB còn nhận được.
    uint32_t available = tud_cdc_write_available();
    uint32_t queued = ComMgr_TxQueued();
    uint32_t length = queued < available ? queued : available;

    if (length == 0U)
    {
        return;
    }

    // Queue vòng có thể bị chia thành hai đoạn ở cuối và đầu mảng.
    // Lần này chỉ gửi đoạn liên tục từ vị trí tail đến cuối mảng.
    uint32_t contiguous = COMMGR_TX_QUEUE_SIZE - tx_queue_tail;
    if (length > contiguous)
    {
        length = contiguous;
    }

    // Gửi một phần nhỏ để main loop luôn có cơ hội chạy các tác vụ khác.
    uint32_t written = tud_cdc_write(&tx_queue[tx_queue_tail], length);
    tx_queue_tail = (tx_queue_tail + written) % COMMGR_TX_QUEUE_SIZE;

    // Đưa dữ liệu từ FIFO TinyUSB ra USB ngay trong lần xử lý này.
    tud_cdc_write_flush();
}

void tud_cdc_rx_cb(uint8_t itf)
{
  (void)itf;
  char input[32];
  uint32_t length = tud_cdc_read(input, sizeof(input));

  for (uint32_t index = 0U; index < length; index++)
  {
    char character = input[index];
    if (character == '\n' || character == '\r')
    {
      rx_command[rx_command_length] = '\0';
      if (strcmp(rx_command, "cfg:barker13") == 0)
      {
        Transmitter_SetPulseType(TRANSMITTER_PULSE_BARKER13);
      }
      else if (strcmp(rx_command, "cfg:single") == 0)
      {
        Transmitter_SetPulseType(TRANSMITTER_PULSE_SINGLE);
      }
      else if (strncmp(rx_command, "rx_select:", 10) == 0)
      {
        rx_select = (uint32_t)(rx_command[10] - '0');
      }
      else if (strcmp(rx_command, "mode:raw") == 0)
      {
        stream_mode = STREAM_MODE_RAW;
      }
      else if (strcmp(rx_command, "mode:bpf") == 0)
      {
        stream_mode = STREAM_MODE_BPF;
      }
      else if (strcmp(rx_command, "mode:demod") == 0 || strcmp(rx_command, "mode:demodulated") == 0)
      {
        stream_mode = STREAM_MODE_DEMOD;
      }
      rx_command_length = 0U;
    }
    else if (rx_command_length < sizeof(rx_command) - 1U)
    {
      rx_command[rx_command_length++] = character;
    }
    else
    {
      rx_command_length = 0U;
    }
  }
}

uint32_t ComMgr_GetRxSelect(void)
{
    return rx_select;
}

uint8_t ComMgr_IsBpfEnabled(void)
{
    return (stream_mode == STREAM_MODE_BPF);
}

StreamMode_t ComMgr_GetStreamMode(void)
{
    return stream_mode;
}

void ComMgr_SendData(void const *data, uint32_t length)
{
    // Chỉ gửi dữ liệu khi Host thực sự đang kết nối để tránh kẹt bộ đệm USB
    if (!tud_cdc_connected())
    {
        return;
    }

    // Hàm này không chờ USB; nếu dữ liệu chưa gửi được thì queue giữ lại.
    if (data == NULL || length == 0U)
    {
        return;
    }

    // Never enqueue a partial frame: it would desynchronize the USB parser.
    if (length > ComMgr_TxFree())
    {
      return;
    }
    uint32_t accepted = length;
    uint8_t const *buffer = (uint8_t const *)data;

    // Chép dữ liệu vào queue và tăng head theo kiểu vòng tròn.
    for (uint32_t index = 0U; index < accepted; index++)
    {
        tx_queue[tx_queue_head] = buffer[index];
        tx_queue_head = (tx_queue_head + 1U) % COMMGR_TX_QUEUE_SIZE;
    }
}

// Callback của TinyUSB khi trạng thái kết nối cổng COM thay đổi (DTR/RTS)
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)itf;
    (void)rts;
    if (!dtr)
    {
        // Host đã ngắt kết nối, xóa sạch hàng đợi để giải phóng bộ đệm USB
        tx_queue_head = 0U;
        tx_queue_tail = 0U;
        tud_cdc_write_clear();
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
