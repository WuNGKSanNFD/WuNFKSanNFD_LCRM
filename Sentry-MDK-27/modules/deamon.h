/******************************************************************************
 * @file        deamon.h
 * @brief       模块在线检测（软件看门狗）：登记模块实例，周期性检查数据是否超时未更新，超时回调通知上层。
 * @note        计时单位 = 一次 deamonStart() 调用，本工程为 10ms。
 ******************************************************************************/

#ifndef __DEAMON_H
#define __DEAMON_H

#include "main.h"
#include <stdlib.h>
#include <string.h>

/************************结构体*************************/
//回调函数
typedef void (*deamon_callback_f)(void *);

/**
 * @brief 模块类型基址（注册 id 的高 4 位）
 * @note  id = 模块类型基址 | 实例号(低 4 位)
 */
typedef enum
{
  deamon_module_DJImotor = 0x10,  //DJI 电机（CAN）：id 0x10 ~ 0x1F
  deamon_module_remote   = 0x20,  //遥控器接收机（DT7/DBUS，UART3）：id 0x20 ~ 0x2F
  deamon_module_BMI      = 0x30,  //BMI088 惯性传感器（SPI）：id 0x30 ~ 0x3F
  deamon_module_VISION   = 0x40,  //上位机/视觉（USB）：id 0x40 ~ 0x4F
}deamon_moduleType_e;


/**
  * @brief 模块监测的注册配置，由 deamon_init() 整体拷入内部实例
  */
typedef struct
{
  uint16_t reload_count;//离线判定阈值（计数值），单位 = 一次 deamonStart()（本工程 10ms）；取值 0 ~ 65534，不要填 0xFFFF
  uint8_t id;  //模块注册 id（deamon_moduleType_e | 实例号），全工程唯一
  
  void *owner_id;//调用回调时原样透传的参数，可为 NULL
  deamon_callback_f deamon_callback; //掉线回调；掉线期间每个检测周期都会重复调用，需自行做幂等
}deamon_config_s;


/**********************外部调用**********************/

/**
 * @brief 注册一个被监测模块：把 config 拷贝到内部实例表末尾
 * @param config 注册配置指针，不可为 NULL
 * @return 无
 * @note  须在模块首次通信之前调用（如 remote_init()、host_init()）
 */
void deamon_init(deamon_config_s *config);

/**
 * @brief 修改已注册模块的掉线超时阈值
 * @param id           模块注册 id，且必须已被 deamon_init() 注册
 * @param reload_count 超时计数值，单位 = 一次 deamonStart()(10ms)，取值 0 ~ 65534
 * @return 无
 * @note  id 未匹配时直接返回，不清零当前计数
 */
void deamon_Reload(uint8_t id,uint16_t reload_count);

/**
 * @brief 喂狗：清零该 id 的离线计数值，表示模块本周期仍在线
 * @param id 模块注册 id，且必须已被 deamon_init() 注册
 * @return 无
 * @note  每收到一帧有效数据调用一次；可在中断/回调上下文调用
 * @warning 未注册的 id 会让 deamon.c 以 -1 为下标访问实例表（越界写），调用前必须确保已注册。
 */
void deamon_WDOG(uint8_t id);


/**
 * @brief 模块检测主体：遍历已注册实例，未超时的计数 +1，超时的调用其掉线回调
 * @return 无
 * @note  必须周期性调用，本工程在 deamonTask 中每 10ms 一次；不会自动喂狗
 */
void deamonStart(void);

#endif
