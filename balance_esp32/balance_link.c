#include "balance_link.h"
#include "balance_protocol.h"
#include <math.h>
#include "driver/uart.h"
#include "esp_timer.h"

#define BALANCE_UART_PORT    UART_NUM_1
#define BALANCE_UART_BUF_LEN 512


static BalanceFrameParser parser;         // 帧解析器实例
static bool online = false;               // STM32在线状态标记
static uint32_t last_frame = 0;           // 最后一帧收到的时间戳

static void my_uart_init(void)
{
  // 配置通信参数（波特率、8N1 等）
  uart_config_t uart_cfg = {
    .baud_rate  = BALANCE_UART_BAUD,
    .data_bits  = UART_DATA_8_BITS,        // 8 位数据
    .parity     = UART_PARITY_DISABLE,     // 无校验
    .stop_bits  = UART_STOP_BITS_1,        // 1 位停止位
    .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,// 关闭硬件流控
    .source_clk = UART_SCLK_DEFAULT,       // 默认时钟源（IDF v4.x 用 UART_SCLK_APB）
  };
  ESP_ERROR_CHECK(uart_param_config(BALANCE_UART_PORT, &uart_cfg));

  //绑定 GPIO 引脚（顺序：串口号, TX, RX, RTS, CTS）
  ESP_ERROR_CHECK(uart_set_pin(BALANCE_UART_PORT,
                               BALANCE_UART_TX,
                               BALANCE_UART_RX,
                               UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));
  // 安装驱动（申请收发缓冲区并自动开启底层中断）
  ESP_ERROR_CHECK(uart_driver_install(BALANCE_UART_PORT,
                                      1024,   // RX 缓冲区大小（必须 > 128）
                                      512,    // TX 缓冲区大小（0 表示阻塞发送）
                                      0,      // 事件队列长度（不用事件就填 0）
                                      NULL,   // 事件队列句柄（不用就填 NULL）
                                      0));    // 中断分配标志（默认 0）
}
void BalanceLink_Init(void) {
  my_uart_init();                                                                       // 启动串口
  ST7789_SetLinkOnline(false);                                                              //调用屏幕
}

static uint32_t now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

void BalanceLink_Poll(void) {
  ST7789_Status status = {0};
  uint8_t buf[128];
  //限制本次调用最多处理 128 个字节。
  int len = uart_read_bytes(BALANCE_UART_PORT, buf, sizeof(buf), 0);
  for (int i = 0; i < len; ++i) {
    // 调用验证拼接函数，成功则刷新屏幕，标记在线，更新时间戳
    if (BalanceFrameParser_Push(&parser, buf[i], &status)) {
      ST7789_SetStatus(&status);
      ST7789_SetLinkOnline(true);
      online = true;
      last_frame = now_ms();
    }
  }
  //超时或者没连上STM，则重置数据接收长度，刷新屏幕为出错屏，返回通讯状态
  if (online && (uint32_t)(now_ms() - last_frame) >= 1000U) {
    online = false;
    BalanceFrameParser_Reset(&parser);
    ST7789_Status blank = {0};
    blank.state = 255;
    ST7789_SetStatus(&blank);
    ST7789_SetLinkOnline(false);
  }
}
// 数组和长度非空，长度不超 128；写入串口发送缓冲区，返回是否全部写入成功
bool BalanceLink_Send(const uint8_t *data, size_t length) {
  if (!data || !length || length > 128) return false;
  return uart_write_bytes(BALANCE_UART_PORT, data, length) == (int)length;
}

bool BalanceLink_SendTargets(float speed, float turn) {
  //条件限制范围
  if (!isfinite(speed) || !isfinite(turn) || speed < -327.68f || speed > 327.67f ||
      turn < -327.68f || turn > 327.67f) return false;
  int32_t s=(int32_t)lroundf(speed*100.0f), t=(int32_t)lroundf(turn*100.0f);
  if (s < -32768 || s > 32767 || t < -32768 || t > 32767) return false;
  uint16_t a=(uint16_t)s, b=(uint16_t)t;
  //拆分打包发送
  uint8_t frame[10]={0x5A,0xA5,1,2,(uint8_t)a,(uint8_t)(a>>8),(uint8_t)b,(uint8_t)(b>>8),0,0};
  //CRC16校验
  uint16_t crc=0xffffU;
  for (unsigned i=2; i<8; ++i) {
    crc ^= (uint16_t)frame[i]<<8;
    for (unsigned bit=0; bit<8; ++bit)
      crc=(crc&0x8000U)?(uint16_t)((crc<<1)^0x1021U):(uint16_t)(crc<<1);
  }
  //放校验位，打包返回情况
  frame[8]=(uint8_t)crc; frame[9]=(uint8_t)(crc>>8);
  return BalanceLink_Send(frame,sizeof(frame));
}









