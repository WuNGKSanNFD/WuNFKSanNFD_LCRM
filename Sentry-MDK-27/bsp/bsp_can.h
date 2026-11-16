/******************************************************************************
 * @file        bsp_can.h
 * @brief       CAN 通信驱动：CAN 口服务初始化、接收实例注册与标准数据帧发送。
 * @note        接收回调在 CAN 中断上下文执行，不可阻塞；注册只应在初始化阶段完成。
 ******************************************************************************/

#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "can.h"
#include "string.h"
#include "stdlib.h"

/*************************************宏*******************************************/

/** @brief 接收注册表容量：can_rxInstance[] 的数组长度，实现无越界判断 */
#define CAN_CNT   12    ///< 可注册的接收实例上限，单位：个


/*************************************结构体*************************************/

/** @brief CAN 接收数据包（每个实例内部各持一份，下次同 ID 报文时覆盖） */
typedef struct
{
  CAN_HandleTypeDef* hcan;//CAN 口（&hcan1/&hcan2）
  uint16_t  rx_id;        //接收标准帧 ID（11 位）
  uint8_t   rx_data[8];   //最近一次收到的数据域（定长 8 字节）
  uint8_t   len;          //最近一次接收的实际 DLC（0~8）
}can_Rxdata_s;

/** @brief CAN 接收回调函数指针类型（形参为该实例的常驻数据包） */
typedef void (*can_moduleCallback_f)(can_Rxdata_s *);


/** @brief CAN 发送实例（调用者持有并复用，驱动只读） */
typedef struct 
{
  CAN_HandleTypeDef* hcan;//CAN 口（&hcan1/&hcan2）
  uint16_t  tx_id;        //发送标准帧 ID（11 位）
  uint8_t   len;          //发送帧长度 1~8（即 DLC）
  uint8_t   tx_data[8];   //发送数据，有效字节数由 len 决定
}can_txInstance_s;

//注：can_txConfig_s 为早期“发送也集中注册”的废弃定义，已停用（发送改由调用者直接填 can_txInstance_s）。
//typedef struct
//{
//  CAN_HandleTypeDef* hcan;//CAN 口
//  uint16_t  tx_id;        //发送 ID
//  uint8_t   len;          //发送帧长度 1-8
//}can_txConfig_s;

/** @brief CAN 接收注册信息（仅作注册入参，内容会被拷贝到内部实例） */
typedef struct
{
  CAN_HandleTypeDef* hcan;//要监听的 CAN 口（&hcan1/&hcan2）
  uint16_t  rx_id;        //要监听的标准帧 ID（11 位）
  can_moduleCallback_f  callback;//匹配到该 ID 时在中断中调用，不可为 NULL
}can_rxConfig_s;

/** @brief CAN 接口返回状态（can_ok=0、can_error=1） */
typedef enum
{
  can_ok,     ///< 成功，隐式取值 0：报文已写入空闲发送邮箱
  can_error,  ///< 失败，隐式取值 1：发送邮箱全忙，本帧被丢弃
}can_status_e;

/*************************************外部调用*************************************/

//函数链接

/**
  * @brief  启用指定 CAN 口的通信服务（初始化阶段调用）
  * @param  hcan 已由 HAL 初始化的 CAN 句柄（&hcan1 或 &hcan2）
  * @note   含 HAL 调用且非线程安全；只注册接收实例而不调用本函数收不到报文
  */
void can_Serviceinit(CAN_HandleTypeDef* hcan);

/**
  * @brief  注册一个 CAN 接收实例（初始化阶段调用）
  * @param  rxConfig 接收注册信息（句柄/ID/回调），不可为 NULL，内容会被拷贝
  * @note   表容量为 CAN_CNT，实现无容量检查，超出会越界写内存
  */
void can_rxConfigInit(can_rxConfig_s *rxConfig);

/**
  * @brief  发送一帧标准数据帧（非阻塞，DLC = len）
  * @param  txInstance 发送实例，hcan/tx_id/len/tx_data 需预先填好，不可为 NULL
  * @return can_ok=已入发送邮箱；can_error=邮箱全忙，本帧丢弃
  * @note   多任务并发调用同一 CAN 口不安全
  */
can_status_e can_SendData(can_txInstance_s *txInstance);

#endif
