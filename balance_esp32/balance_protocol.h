#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "st7789.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
  uint8_t frame[32];
  uint8_t used;
} BalanceFrameParser;

void BalanceFrameParser_Reset(BalanceFrameParser *parser);
bool BalanceFrameParser_Push(BalanceFrameParser *parser, uint8_t byte, ST7789_Status *status);

#ifdef __cplusplus
}
#endif