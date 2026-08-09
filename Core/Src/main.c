#include "stm32h5xx.h"
#include "ComMgr.h"

// Biến toàn cục để theo dõi thời gian và trạng thái LED
static uint32_t last_toggle_tick = 0;
static uint8_t led_state = 0;

void SystemClock_Config(void);
void Error_Handler(void);

int main(void)
{
    // 1. Reset toàn bộ ngoại vi và khởi tạo Flash, SysTick
    HAL_Init();

    // 2. Cấu hình xung nhịp hệ thống (SYSCLK 250MHz, kích hoạt HSI48 & CRS cho USB)
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

    // 4. Khởi tạo bộ quản lý giao tiếp (USB & TinyUSB)
    ComMgr_Init();

    last_toggle_tick = HAL_GetTick();

    while (1)
    {
        // Xử lý các tác vụ nền của cổng COM
        ComMgr_Process();

        // Kiểm tra xem đã qua 500ms chưa để chớp tắt LED và gửi trạng thái
        if (HAL_GetTick() - last_toggle_tick >= 500)
        {
            last_toggle_tick = HAL_GetTick();
            led_state = !led_state;

            // Toggle LED vật lý
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);

            // Gửi trạng thái LED qua cổng USB Virtual COM
            ComMgr_SendLEDState(led_state);
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
