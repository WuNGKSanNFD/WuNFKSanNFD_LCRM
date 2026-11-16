#include "protocol.h"

/**
 * @brief  数据包结构体（protocol_frame_s）--> 数组，并计算 CRC8 / CRC16
 * @param  raw    待发送的数据包结构体
 * @param  array  输出缓冲区，需能容纳 data_len + 9 字节
 * @param  len    输出实际打包长度 = 5 + 2 + data_len + 2
 * @note   帧头 CRC8 取前 4 字节，帧尾 CRC16 取整帧除末 2 字节外全部内容
 */
void protocol_struct_to_array(protocol_frame_s *raw, uint8_t *array,uint16_t *len)
{
  //数据长度
  *len = 5+2+raw->header.data_len+2;
  
  //帧头
  array[0] = raw->header.SOF;
  array[1] = (uint8_t)raw->header.data_len;
  array[2] = (uint8_t)(raw->header.data_len >> 8);
  array[3] = raw->header.seq;
  array[4] = crc8_getSum(&array[0],4);
  
  //命令码
  array[5] = (uint8_t)raw->cmd_id;
  array[6] = (uint8_t)(raw->cmd_id >> 8);
  
  //数据
  memcpy(&array[7],raw->data,raw->header.data_len);
  
  //帧尾CRC16校验位
  uint16_t crc16 = crc16_getSum(array,*len-2);
  array[*len-2] = (uint8_t)crc16;
  array[*len-1] = (uint8_t)(crc16 >> 8);
}

/**
 * @brief  数组 --> 数据包结构体（protocol_frame_s），并校验 CRC8 / CRC16
 * @param  array  收到的原始数据帧
 * @param  str    输出解析结果
 * @return protocol_ok = 校验通过并解析成功；protocol_errer = array 为空或校验失败 
 */
protocol_status_e protocol_array_to_struct(uint8_t *array,protocol_frame_s *str)
{
  if(array == NULL)
    return protocol_errer;
  
  //检查CRC8校验
  if(crc8_checkData(&array[0],5) != 1)
    return protocol_errer;
  
  //帧头
  str->header.SOF       = array[0];
  str->header.data_len  = (uint16_t)array[1] | ((uint16_t)array[2] << 8);
  str->header.seq       = array[3];
  str->header.CRC8      = array[4];
  
  uint16_t len = 5+2+str->header.data_len+2;
  //检查CRC16校验
  if(crc16_checkData(array,len) != 1)
    return protocol_errer;
  
  //命令码
  str->cmd_id = (uint16_t)array[5] | ((uint16_t)array[6] << 8);
  
  //数据段
  memcpy(str->data,array+7,str->header.data_len);
  
  //帧尾
  str->CRC16 = (uint16_t)array[len-2] | ((uint16_t)array[len-1] << 8);
  
  return protocol_ok;
}
