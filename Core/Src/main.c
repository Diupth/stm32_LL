#include "stm32h5xx.h"

// Hàm delay đơn giản bằng vòng lặp busy-wait
void delay(volatile uint32_t count)
{
    while (count--)
    {
        __NOP(); // Ngăn compiler tối ưu hóa mất vòng lặp
    }
}

int main(void)
{
    // 1. Bật clock cho GPIO Port B (LED kết nối với PB2 trên mạch của bạn)
    // GPIOB nằm trên AHB2 bus
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    // Đợi clock ổn định
    (void)RCC->AHB2ENR;

    // 2. Cấu hình chân PB2 làm Output
    // Trong thanh ghi MODER: 00 = Input, 01 = Output, 10 = Alternate, 11 = Analog
    // Mỗi chân chiếm 2 bit. PB2 chiếm bits [5:4]
    GPIOB->MODER &= ~(3U << (2 * 2)); // Xóa cấu hình cũ của PB2
    GPIOB->MODER |= (1U << (2 * 2));  // Cấu hình PB2 là General purpose output mode

    // 3. Cấu hình kiểu Output là Push-Pull
    GPIOB->OTYPER &= ~(1U << 2);

    // 4. Cấu hình tốc độ Output là Low Speed
    GPIOB->OSPEEDR &= ~(3U << (2 * 2));

    // 5. Cấu hình Pull-up/Pull-down là No pull-up, no pull-down
    GPIOB->PUPDR &= ~(3U << (2 * 2));

    while (1)
    {
        // Toggle PB2 (chớp tắt LED)
        GPIOB->ODR ^= (1U << 2);

        // Trễ khoảng 500ms ở xung nhịp nội HSI 64MHz mặc định
        delay(3000000);
    }
}
