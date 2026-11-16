/******************************************************************************
 * @file        bsp_usb.h
 * @brief       USB CDC（虚拟串口）驱动：注册接收回调、软开关数据通道并发送数据。
 * @note        接收回调在 USB 中断上下文执行，必须尽快返回；调用任何接口前须先 usb_init()。
 ******************************************************************************/

#ifndef __BSP_USB_H
#define __BSP_USB_H

#include "usbd_cdc_if.h"
#include "string.h"
#include "stdlib.h"

/*************************结构体****************************/

/** @brief USB CDC 接收回调函数指针类型（Buf 指向的缓冲区仅在本次回调内有效） */
typedef void (*usb_moduleCallback_f)(uint8_t* Buf, uint32_t Len);

//注：usb_data_s 为早期“驱动内部缓存接收数据”的预留定义，现改为直接把缓冲区透传给回调，故未启用。

//typedef struct
//{
//  uint8_t buff[APP_RX_DATA_SIZE];
//}usb_data_s;

/** @brief USB 接口返回状态（usb_ok=0、usb_error=1） */
typedef enum
{
  usb_ok,     ///< 成功，隐式取值 0：数据已提交给 CDC 发送接口
  usb_error,  ///< 失败，隐式取值 1：BSP 层通道已关闭
}usb_status_e;

/*************************外部调用****************************/

/**
  * @brief  USB 通信初始化：创建实例并注册接收回调（只封装 BSP 层，初始化阶段调用一次）
  * @param  moduleCallback 接收回调，必须提供有效函数
  * @warning 传入 NULL 不会报错，但收到 USB 数据时会在中断中调用空指针
  */
void usb_init(usb_moduleCallback_f moduleCallback);

/**
  * @brief  关闭 USB 数据通道（软开关，不关闭 USB 外设与端点）
  * @note   与 usb_start() 成对使用；实例未创建时不可调用
  */
void usb_stop(void);

/**
  * @brief  重新启用 USB 数据通道（软开关）
  * @note   用于 usb_stop() 后的恢复；实例未创建时不可调用
  */
void usb_start(void);

/**
  * @brief  通过 USB CDC 发送数据（非阻塞提交，不等待发送完成）
  * @param  tx_data 待发送数据首地址，须在数据真正发出前保持有效
  * @param  len     发送字节数，单位：字节，上限为 APP_TX_DATA_SIZE（512）
  * @return usb_ok=已提交给 CDC；usb_error=通道已关闭，本次不发送
  * @note   CDC 忙时本帧被丢弃且仍返回 usb_ok；不要在接收回调中大量调用
  */
usb_status_e usb_sendData(uint8_t *tx_data,uint16_t len);


#endif
