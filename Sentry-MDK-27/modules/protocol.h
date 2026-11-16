/******************************************************************************
 * @file    protocol.h
 * @brief   通用通信协议
 * @note    可适配裁判系统、与视觉通信
 ******************************************************************************/
#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "main.h"
#include "crc.h"
#include "string.h"

/*******************************宏********************************/

#define PROTOCOL_MAX_DATA_LEN   30

/*******************************结构体********************************/
typedef struct
{
  uint8_t SOF;
  uint16_t data_len;
  uint8_t seq;
  uint8_t CRC8;
}protocol_frameHeader_s;

typedef struct
{
  protocol_frameHeader_s header;
  uint16_t cmd_id;
  uint8_t data[PROTOCOL_MAX_DATA_LEN];
  uint16_t CRC16;
}protocol_frame_s;

typedef enum
{
  protocol_ok,
  protocol_errer,
}protocol_status_e;

/*******************************外部调用********************************/

/**
 * @brief  打包：结构体 -> 字节数组，并计算 CRC8 / CRC16
 * @param  raw    待打包的数据包结构体
 * @param  array  输出缓冲区，需能容纳 data_len + 9 字节
 * @param  len    输出实际打包长度 = 5 + 2 + data_len + 2
 * @note   纯内存操作，不校验 data_len 是否超过 PROTOCOL_MAX_DATA_LEN
 */
void protocol_struct_to_array(protocol_frame_s *raw, uint8_t *array,uint16_t *len);

/**
 * @brief  解包：字节数组 -> 结构体，并校验 CRC8 / CRC16
 * @param  array  收到的原始数据帧
 * @param  str    输出解析结果
 * @return protocol_ok = 校验通过并解析成功；protocol_errer = array 为空或校验失败
 */
protocol_status_e protocol_array_to_struct(uint8_t *array,protocol_frame_s *str);



#endif
