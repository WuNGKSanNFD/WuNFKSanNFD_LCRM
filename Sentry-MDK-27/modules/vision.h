/******************************************************************************
 * @file        vision.h
 * @brief       上位机（视觉）通信：USB CDC 收发协议帧、断线守护。
 * @note        SOF 固定 0x11；帧格式见 protocol.h；接收回调在 USB 中断上下文。
 ******************************************************************************/

#ifndef __VISION_H
#define __VISION_H

#include "main.h"
#include "bsp_usb.h"
#include "protocol.h"
#include "deamon.h"

/**********************结构体************************/
/**
 * @brief 上位机下发的命令位（cmd_id 按位解析）
 * @note  0x0000 同时代表"底盘关"与"云台关"，二者靠不同掩码位区分
 */
typedef enum
{
  vision_cmd_off              = 0x0000,  ///< 关闭
                                     
  vision_chassis_no_following = 0x0001,  ///< 底盘不跟随（bit0-3）
  vision_chassis_following    = 0x0002,  ///< 底盘跟随

  vision_gimbal_enable        = 0x0010,  ///< 云台使能（bit4-7）
  vision_gimbal_small_top     = 0x0020,  ///< 小陀螺

  vision_shoot_friction_on    = 0x0100,  ///< 摩擦轮开（bit8-15）
  vision_shoot_fire           = 0x0200,  ///< 开火
}vision_cmd_e;

/** @brief 本板 -> 上位机：固定的 3 个 float */
typedef struct
{
  uint16_t cmd_id;     ///< 原样回传上位机发来的 cmd_id
  float yaw;           ///< 云台 yaw 编码器角度，单位：度
  float pitch;         ///< 云台 pitch 编码器角度，单位：度
  float free_data;     ///< 预留，当前恒发 0
}vision_tx_s;

/** @brief 上位机 -> 本板：固定的 5 个 float + 状态 */
typedef struct
{
  uint16_t cmd_id;     ///< 命令位，按 vision_cmd_e 解析
  float forward;       ///< 底盘前后速度
  float turn;          ///< 底盘左右速度
  float rotate;        ///< 底盘自转速度（不跟随模式下生效）
  float yaw;           ///< 云台 yaw 目标角，单位：度
  float pitch;         ///< 云台 pitch 目标角，单位：度
  uint8_t state;       ///< 1 = 通信正常；0 = 已停止/离线（由 vision_getData() 填写）
}vision_rx_s;

/** @brief 视觉接口返回状态（vision_ok=0、vision_error=1） */
typedef enum
{
  vision_ok,     ///< 成功
  vision_error,  ///< 失败：通道已关闭
}vision_status_e;

/**************************外部调用****************************/
/** @brief 初始化：创建实例、注册 USB 接收回调与断线守护，启动前须调用一次 */
void vision_init(void);

/** @brief 开启视觉通信（软开关，不关闭 USB 外设） */
void vision_start(void);

/** @brief 停止视觉通信（软开关） */
void vision_stop(void);

/**
 * @brief  向视觉端发送一帧（cmd_id + yaw + pitch + free_data）
 * @param  tx_struct 待发送内容，按值传入
 * @return vision_ok=已提交给 USB；vision_error=通道已关闭
 */
vision_status_e vision_sendData(vision_tx_s tx_struct);

/** @brief 取最近一帧解析结果（按值返回，可安全在任务中调用） */
vision_rx_s vision_getData(void);


#endif
