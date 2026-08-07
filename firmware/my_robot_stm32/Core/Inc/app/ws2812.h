#ifndef APP_WS2812_H
#define APP_WS2812_H

#include <stdint.h>
#include <main.h>

void FAN_on()
{
	// 风扇开启
	HAL_GPIO_WritePin(FAN_SIG_GPIO_Port, FAN_SIG_Pin, GPIO_PIN_SET);
}

void FAN_off()
{
	// 风扇关闭
	HAL_GPIO_WritePin(FAN_SIG_GPIO_Port, FAN_SIG_Pin, GPIO_PIN_RESET);
}

/*
 * This bit-banged implementation is tuned for STM32F303 at 72 MHz.
 * If you change the MCU clock or optimization level significantly,
 * verify the waveform on a scope and retune the NOP counts if needed.
 */

/* 强制关闭 PA10 上的 WS2812 灯珠 */
void WS2812_Force_Off_PA10(void) {
    // 1. 发送复位信号 (Reset): 拉低引脚 > 300us
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(1); // 延迟 1ms 确保复位充分

    // 2. 发送 24 个“0”比特 (对应一颗灯的 GRB = 0,0,0)
    for (int i = 0; i < 24; i++) {
        // --- 发送逻辑“0” ---
        // 高电平需持续约 350ns (72MHz 下约 25 个指令周期)
        GPIOA->BSRR = GPIO_PIN_10;
        __asm("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;");
        __asm("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;");

        // 低电平需持续约 800ns (72MHz 下约 57 个指令周期)
        GPIOA->BRR = GPIO_PIN_10;
        __asm("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;");
        __asm("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;");
        __asm("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;");
        __asm("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;");
    }

    // 3. 再次拉低引脚锁定数据
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(1);
}

#endif /* APP_WS2812_H */
