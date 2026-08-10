#include "stm32h5xx.h"
#include "SystemClock.h"

void Error_Handler(void);

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc_config = {0};
    RCC_ClkInitTypeDef clock_config = {0};
    RCC_CRSInitTypeDef crs_config = {0};

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    osc_config.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_CSI;
    osc_config.HSI48State = RCC_HSI48_ON;
    osc_config.CSIState = RCC_CSI_ON;
    osc_config.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
    osc_config.PLL.PLLState = RCC_PLL_ON;
    osc_config.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
    osc_config.PLL.PLLM = 1;
    osc_config.PLL.PLLN = 125;
    osc_config.PLL.PLLP = 2;
    osc_config.PLL.PLLQ = 2;
    osc_config.PLL.PLLR = 2;
    osc_config.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
    osc_config.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
    osc_config.PLL.PLLFRACN = 0;

    if (HAL_RCC_OscConfig(&osc_config) != HAL_OK)
    {
        Error_Handler();
    }

    clock_config.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                           | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                           | RCC_CLOCKTYPE_PCLK3;
    clock_config.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clock_config.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clock_config.APB1CLKDivider = RCC_HCLK_DIV1;
    clock_config.APB2CLKDivider = RCC_HCLK_DIV1;
    clock_config.APB3CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clock_config, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_RCC_CRS_CLK_ENABLE();
    crs_config.Prescaler = RCC_CRS_SYNC_DIV1;
    crs_config.Source = RCC_CRS_SYNC_SOURCE_USB;
    crs_config.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
    crs_config.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
    crs_config.ErrorLimitValue = 34;
    crs_config.HSI48CalibrationValue = 32;
    HAL_RCCEx_CRSConfig(&crs_config);
}
