#ifndef DISPLAY_LINK_H
#define DISPLAY_LINK_H
#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct {
    float angle, gyro, encoder_a, encoder_b;
    uint16_t pwm_a, pwm_b;
    uint8_t state, fault, stby, key;
} DisplayStatus;

// 在 main 函数中调用一次，放在 MX_USART1_UART_Init() 之后。
HAL_StatusTypeDef DisplayLink_Init(UART_HandleTypeDef *uart);

// 在 main 函数中每 200 毫秒调用一次；如果仍在发送，则返回 HAL_BUSY 并丢弃当前帧。
HAL_StatusTypeDef DisplayLink_SendStatus(const DisplayStatus *s);

// 读取从 ESP32 接收的原始字节；只能在 main 函数中调用。返回 1 表示读到字节，返回 0 表示没有数据。
uint8_t DisplayLink_ReadByte(uint8_t *out);

// 获取并清除接收错误或溢出标志。返回非零时，应丢弃当前输入帧。
uint8_t DisplayLink_TakeRxError(void);

// 处理接收到的控制帧。只有收到完整且有效的一组数据时才返回 1。
uint8_t DisplayLink_PollCommand(float *speed, float *turn);

// 以下是供已有 HAL 回调函数调用的处理函数，本身不是 HAL 回调函数。
void DisplayLink_OnTxComplete(UART_HandleTypeDef *uart);
void DisplayLink_OnRxComplete(UART_HandleTypeDef *uart);
void DisplayLink_OnError(UART_HandleTypeDef *uart);
#endif
