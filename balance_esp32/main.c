//
// Created by 15902 on 2026/11/21.
//

#include "main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "st7789.h"
#include "balance_link.h"

void app_main(void)
{
    ST7789_Init();

    ST7789_Status waiting = {0};
    waiting.state = 255;
    ST7789_SetStatus(&waiting);

    BalanceLink_Init();

    uint32_t loop_cnt = 0;
    while (1) {
        BalanceLink_Poll();
        ST7789_Poll();

        // 每跑 16 次主动让出 1 个系统节拍：
        // 既不会触发看门狗复位，也不会把 30FPS 刷屏拖慢
        if (++loop_cnt >= 16) {
            loop_cnt = 0;
            vTaskDelay(1);
        }
    }
}