#include "balance_protocol.h"
#include <string.h>
#include <stdint.h>

//开始CRC16检验数据字节
static uint16_t crc16(const uint8_t *p, unsigned len) {
  uint16_t crc = 0xffff;
  while (len--) {
    crc ^= (uint16_t)*p++ << 8;
    for (unsigned i=0; i<8; ++i) crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
  }
  return crc;
}
//数据从数组中取出拼接成数据
static uint16_t u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static int32_t i32(const uint8_t *p) {
  uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  return u <= INT32_MAX ? (int32_t)u : (int32_t)((int64_t)u - INT64_C(4294967296));
}

//used 是用来标志存放了几个有效字节，放入读数组的个数
void BalanceFrameParser_Reset(BalanceFrameParser *parser) {
  if (parser) {
    parser->used = 0;
  }
}

bool BalanceFrameParser_Push(BalanceFrameParser *parser, uint8_t byte, struct ST7789_Status *status) {
  if (!parser || !status) return false;
  //判断帧头
  if (parser->used == 0 && byte != 0xA5) return false;
  if (parser->used == 1 && byte != 0x5A) { parser->used = byte == 0xA5 ? 1 : 0; return false; }
  parser->frame[parser->used++] = byte;
  if (parser->used < 32) return false;
  //验证标志字节
  const bool valid = parser->frame[2] == 1 && parser->frame[3] == 26 &&
      crc16(parser->frame + 2, 28) == u16(parser->frame + 30) &&
      parser->frame[26] <= 3 && parser->frame[28] <= 1 && parser->frame[29] <= 1;
  //验证成功，进行字节拼接成数据放到frame数组
  if (valid) {
    ST7789_Status next = {0};
    next.angle = i32(parser->frame+6) / 100.0f;
    next.gyro = i32(parser->frame+10) / 100.0f;
    next.encoder_a = i32(parser->frame+14) / 100.0f;
    next.encoder_b = i32(parser->frame+18) / 100.0f;
    next.pwm_a = u16(parser->frame+22);
    next.pwm_b = u16(parser->frame+24);
    next.state=parser->frame[26];
    next.fault=parser->frame[27];
    next.stby=parser->frame[28];
    next.key=parser->frame[29];
    status=next; parser->used=0; return true;
  }
  // 判断条件（valid）不符，遍历数组，可能已经混入了下一帧的一部分，看是否会出现帧头，重新重帧头组合
  for (uint8_t i=1; i<31; ++i) {
    if (parser->frame[i]==0xA5 && parser->frame[i+1]==0x5A) {
      parser->used=32-i;
      memmove(parser->frame,parser->frame+i,parser->used); return false;
    }
  }
  parser->used = parser->frame[31]==0xA5 ? 1 : 0;
  if (parser->used) parser->frame[0]=0xA5;
  return false;
}




