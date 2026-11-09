#include "balance_link.h"
#include "balance_protocol.h"
#include <math.h>
static HardwareSerial BalanceSerial(1);  // ESP32的UART1串口对象
static BalanceFrameParser parser;         // 帧解析器实例
static bool online = false;               // STM32在线状态标记
static uint32_t last_frame = 0;           // 最后一帧收到的时间戳
void BalanceLink_Init() {
  BalanceSerial.begin(BALANCE_UART_BAUD, SERIAL_8N1, BALANCE_UART_RX, BALANCE_UART_TX);     // 启动串口
  ST7789_SetLinkOnline(false);                                                              //调用屏幕
}
void BalanceLink_Poll() {
  ST7789_Status status{};
  //限制本次调用最多处理 128 个字节。
  unsigned budget=128;
  while (budget-- && BalanceSerial.available()) {
    //调用验证拼接函数,成功则刷新屏幕，返回与STM连接成功，更新last_frame
    if (parser.push((uint8_t)BalanceSerial.read(), status)) {
      ST7789_SetStatus(&status); ST7789_SetLinkOnline(true); online=true; last_frame=millis();
    }
  }
  //超时或者没连上STM，则重置数据接收长度，刷新屏幕为出错屏，返回通讯状态
  if (online && (uint32_t)(millis()-last_frame)>=1000U) {
    online=false; parser.reset(); ST7789_Status blank{}; blank.state=255;
    ST7789_SetStatus(&blank); ST7789_SetLinkOnline(false);
  }
}
//数组和长度非空，长度不超缓冲区，缓冲区还有字节可读；返回写入的值和输入的长度是否相同的bool值
bool BalanceLink_Send(const uint8_t *data, size_t length) {
  if (!data || !length || length > 128 || BalanceSerial.availableForWrite() < (int)length) return false;
  return BalanceSerial.write(data,length)==length;
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









