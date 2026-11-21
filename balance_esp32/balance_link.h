#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "st7789.h"

#ifdef __cplusplus
extern "C" {
#endif

// ESP32串口配置
#define BALANCE_UART_RX   17
#define BALANCE_UART_TX   18
#define BALANCE_UART_BAUD 115200

void BalanceLink_Init(void);
void BalanceLink_Poll(void);
bool BalanceLink_Send(const uint8_t *data, size_t length);
bool BalanceLink_SendTargets(float speed, float turn);

#ifdef __cplusplus
}
#endif