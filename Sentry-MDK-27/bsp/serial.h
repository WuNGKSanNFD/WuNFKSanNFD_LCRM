/******************************************************************************
 * @file        serial.h
 * @brief       串口（UART）驱动：实例注册、DMA 空闲中断接收、数据取用与格式化打印。
 * @note        模块回调在 UART/DMA 中断上下文执行，必须短小且不可阻塞。
 ******************************************************************************/

#ifndef __SERIAL_H
#define __SERIAL_H

#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/***************结构体******************/
//定义模块回调函数

/** @brief 模块接收回调函数指针类型（数据须在回调内用 Serial_getRxdata() 取走） */
typedef void (*serial_moduleCallback)();

/** @brief 串口模式（只有 receive 与 T_R 参与 DMA 接收，两者处理相同） */
typedef enum
{
  serial_mode_send,//只发，隐式取值 0
  serial_mode_receive,//只收，隐式取值 1：Serial_receive() 会启动 DMA 接收
  serial_mode_T_R,//收发，隐式取值 2：既可用发送接口，也参与 DMA 接收
}serial_mode_e;


/** @brief 串口通信注册实例（内容会被 Serial_init() 拷贝到内部实例） */
typedef struct
{
  serial_mode_e mode;                 ///< 工作模式，决定是否参与接收
  UART_HandleTypeDef *huart;          ///< UART 句柄（如 &huart3/&huart6）
  uint8_t rx_DLC;                     ///< 接收最大长度，单位：字节；0 表示不使用 DMA 接收
  serial_moduleCallback moduleCallback;///< 模块回调函数，接收/T_R 模式下必须非 NULL
}serial_config_s;


/*****************外部调用*********************/

/**
  * @brief  串口初始化（只做注册与赋值，不启动接收）
  * @param  config 串口注册实例（模式/句柄/接收长度/回调），不可为 NULL
  * @note   需要接收时还须调用 Serial_receive(huart)；只在初始化阶段调用
  */
void Serial_init(serial_config_s *config);

/**
  * @brief  启用串口接收（初始化时调用一次，驱动随后自动续接）
  * @param  huart 串口号（须已注册为 receive/T_R 模式且 rx_DLC 非 0）
  * @note   未注册、模式为 send 或 rx_DLC 为 0 时直接返回（空操作）
  */
void Serial_receive(UART_HandleTypeDef *huart);

/**
  * @brief  串口打印函数（阻塞发送，禁止在中断中调用）
  * @param  huart  串口号（只需已由 HAL 初始化，不必注册到本驱动）
  * @param  format 打印内容（用法同 printf）
  * @param  ...    与 format 对应的可变参数
  * @note   单条输出超过 159 字节会被截断；多个任务不要并发调用同一串口
  */
void Serial_Printf(UART_HandleTypeDef *huart,char *format, ...);

/**
  * @brief  接收数据调用接口（典型调用位置是模块回调内部）
  * @param  huart  串口号（需已注册为 receive/T_R 模式）
  * @param  rxdata 接收数据包，缓冲区由调用者提供，长度须 ≥ 该实例注册的 rx_DLC
  * @note   一次拷贝 rx_DLC 字节，无长度检查与实例判空
  * @warning 传入未注册的 huart 会退化为读取第 0 个实例的数据，一个实例都没注册时会解引用空指针
  */
void Serial_getRxdata(UART_HandleTypeDef *huart,uint8_t *rxdata);



#endif
