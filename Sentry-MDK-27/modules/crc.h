/******************************************************************************
 * @file        crc.h
 * @brief       CRC8 / CRC16 查表校验.
 ******************************************************************************/

#ifndef __CRC_H
#define __CRC_H

#include "main.h"


/**
 * @brief  计算 CRC8 校验值（查表法，支持分段累计）
 * @param  data    待校验数据首地址，内部不判空，调用方保证非 NULL
 * @param  len     参与计算的字节数；为 0 时不读数据，直接返回 ucCRC8
 * @return 计算后的 CRC8 校验值 uint8_t
 * @note   纯计算函数，不修改 data、无静态变量，可重入，可在中断上下文调用
 */
uint8_t crc8_getSum(uint8_t *data, uint16_t len);

/**
 * @brief  校验带 CRC8 帧尾的数据：用前 len-1 字节重算 CRC8 并与最后 1 字节比较
 * @param  data  整帧首地址（最后 1 字节为发送端 CRC8）
 * @param  len   整帧字节数（含末尾校验字节）
 * @return 1=校验通过；0=校验失败或 data == NULL 或 len <= 2
 * @note   host.c 中以 len = 4 校验帧头
 */
uint8_t crc8_checkData(uint8_t *data, uint16_t len);

/**
 * @brief  计算 CRC16 校验值（查表法，右移/LSB 优先，支持分段累计）
 * @param  data  待校验数据首地址
 * @param  len   参与计算的字节数；为 0 时不读数据，直接返回 wCRC
 * @return 计算后的 CRC16 校验值 uint16_t；data == NULL 时返回 0xFFFF
 */
uint16_t crc16_getSum(uint8_t *data, uint16_t len);

/**
 * @brief  校验带 CRC16 帧尾的数据：用前 len-2 字节重算 CRC16 并与最后 2 字节比较
 * @param  data  整帧首地址（最后 2 字节为发送端 CRC16，低字节在前）
 * @param  len   整帧字节数（含末尾 2 字节校验）
 * @return 1=校验通过；0=校验失败或 data == NULL 或 len <= 2
 */
uint8_t crc16_checkData(uint8_t *data, uint16_t len);

#endif
