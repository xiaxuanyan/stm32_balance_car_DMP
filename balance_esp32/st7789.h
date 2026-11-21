#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 引脚定义，按你的 ESP32-S3 实际接线核对
#define ST7789_PIN_SCK   4
#define ST7789_PIN_MOSI  5
#define ST7789_PIN_CS    15
#define ST7789_PIN_DC    7
#define ST7789_PIN_RST   6
#define ST7789_PIN_BL    (-1) // -1 表示不由 GPIO 控制背光

#define ST7789_WIDTH     240U
#define ST7789_HEIGHT    240U
#define ST7789_X_OFFSET  0U
#define ST7789_Y_OFFSET  0U
#define ST7789_MADCTL    0x00U

typedef struct {
  float angle, gyro, encoder_a, encoder_b;
  uint16_t pwm_a, pwm_b;
  uint8_t state, fault, stby, key;
} ST7789_Status;

void ST7789_Init(void);
void ST7789_SetLinkOnline(bool online);
void ST7789_SetStatus(const ST7789_Status *status);
void ST7789_Poll(void);
uint8_t ST7789_GetStatus(void);

#ifdef __cplusplus
}
#endif