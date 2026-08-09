#include "stm32h5xx.h"
#include "tusb.h"

// Biến toàn cục để theo dõi thời gian và trạng thái LED
static uint32_t last_toggle_tick = 0;
static uint8_t led_state = 0;

void SystemClock_Config(void);
void Error_Handler(void);

// Định nghĩa hàm trễ sử dụng SysTick
void delay_ms(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms)
    {
        tud_task(); // Chạy nền TinyUSB trong lúc đợi trễ
    }
}

int main(void)
{
    // 1. Reset toàn bộ ngoại vi và khởi tạo Flash, SysTick
    HAL_Init();

    // 2. Cấu hình xung nhịp hệ thống (Bật HSI48 cho USB và đồng bộ bằng CRS)
    SystemClock_Config();

    // 3. Khởi tạo GPIO cho chân LED (PB2)
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Tắt LED ban đầu
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

    // 4. Khởi tạo thủ công phần cứng mức thấp cho USB (Do TinyUSB chạy trực tiếp bằng thanh ghi, không gọi MspInit)
    
    // a. Chọn nguồn xung nhịp HSI48 cho USB
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    // b. Kích hoạt clock cổng GPIOA (PA11 và PA12 là hai chân DM và DP của USB)
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // c. Cấu hình chân PA11 và PA12 cho USB (AF10)
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_USB;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // d. Bật nguồn VDDUSB (Cần thiết cho cổng USB của dòng H5 hoạt động)
    HAL_PWREx_EnableVddUSB();

    // e. Bật clock ngoại vi USB
    __HAL_RCC_USB_CLK_ENABLE();

    // f. Cấu hình độ ưu tiên ngắt và kích hoạt ngắt USB
    HAL_NVIC_SetPriority(USB_DRD_FS_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);

    // 5. Khởi tạo USB Stack (TinyUSB)
    tusb_init();

    last_toggle_tick = HAL_GetTick();

    while (1)
    {
        // Chạy tác vụ nền của TinyUSB
        tud_task();

        // Kiểm tra xem đã qua 500ms chưa để chớp tắt LED và gửi trạng thái
        if (HAL_GetTick() - last_toggle_tick >= 500)
        {
            last_toggle_tick = HAL_GetTick();
            led_state = !led_state;

            // Toggle LED vật lý
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);

            // Gửi dữ liệu trạng thái LED qua cổng USB Virtual COM (CDC)
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
                tud_cdc_write_flush(); // Đẩy dữ liệu đi ngay lập tức
            }
        }
    }
}

// Cấu hình xung nhịp hệ thống 250MHz (từ nguồn CSI 4MHz qua PLL1)
// Đồng thời kích hoạt nguồn xung nhịp HSI48 cấp cho USB và CRS tự động hiệu chuẩn.
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

    // Cấu hình điện áp Core
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    // Bật bộ dao động nội CSI (4MHz) và HSI48 (48MHz cấp cho USB)
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_CSI;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.CSIState = RCC_CSI_ON;
    RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
    
    // Cấu hình PLL1 từ nguồn CSI để có SYSCLK = 250MHz
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 125;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    // Cấu hình Bus clock dividers
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_PCLK3;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }

    // Bật CRS và tự động hiệu chỉnh xung nhịp HSI48 bằng cách đồng bộ tín hiệu từ USB
    __HAL_RCC_CRS_CLK_ENABLE();
    RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
    RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
    RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
    RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
    RCC_CRSInitStruct.ErrorLimitValue = 34;
    RCC_CRSInitStruct.HSI48CalibrationValue = 32;
    HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

// Hàm xử lý ngắt USB (Gọi thư viện TinyUSB xử lý các sự kiện USB)
void USB_DRD_FS_IRQHandler(void)
{
    tud_int_handler(0);
}

// Ngắt SysTick dùng để tăng biến đếm thời gian hệ thống của HAL (uwTick)
void SysTick_Handler(void)
{
    HAL_IncTick();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
