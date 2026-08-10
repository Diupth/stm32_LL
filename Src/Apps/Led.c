#include "Led.h"
#include "ComMgr.h"
#include <string.h>

static uint32_t last_toggle_tick;
static uint8_t led_state;

void Led_Init(void)
{
    GPIO_InitTypeDef gpio_config = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio_config.Pin = GPIO_PIN_2;
    gpio_config.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_config.Pull = GPIO_NOPULL;
    gpio_config.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio_config);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

    last_toggle_tick = HAL_GetTick();
}

void Led_Process(void)
{
    if (HAL_GetTick() - last_toggle_tick < 500)
    {
        return;
    }

    last_toggle_tick = HAL_GetTick();
    led_state = !led_state;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);

    char const *message = led_state ? "LED State: ON\r\n" : "LED State: OFF\r\n";
    ComMgr_SendData(message, strlen(message));
}
