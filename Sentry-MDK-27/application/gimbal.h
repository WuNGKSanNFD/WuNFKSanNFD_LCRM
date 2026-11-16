/**
  * @file    gimbal.h
  * @brief   云台三轴（yaw / big pitch / small pitch）使能与闭环接口
  * @note    目标与反馈统一用 rad；gimbal_work() 由 gimbalTask 每 2ms 调用
  */

#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "cmd.h"
#include "IMU.h"
#include "DJI_motor.h"
#include "DM_motor.h"

/***************************宏*****************************/
#define GIMBAL_DISABLE_RANGE    0.5f                    //可失能角度判定范围 rad
#define GIMBAL_DEG_TO_RAD       (3.1415926f / 180.0f)   //度转弧度


/***************************结构体*****************************/
/**
 * @brief 云台目标值（gimbal_work 内部维护，单位 rad）
 */
typedef struct
{
  float yaw_angle;        //yaw 目标角 rad
  float smallPitch_angle; //small pitch 目标角 rad
  float bigPitch_angle;   //big pitch 目标角 rad（当前固定 π/2）
}gimbal_data_s;


/**
 * @brief 云台状态
 */
typedef enum
{
  gimbal_ok,//成功
  gimbal_error,//失败（遥控关闭 CMD_OFF）
}gimbal_status_e;

/***************************外部调用*****************************/

/** @brief 初始化三轴 DM 电机并注册，完成后为失能态（robot_init 中调用） */
void gimbal_init(void);

/** @brief 使能三轴 DM 电机 */
void gimbal_enable(void);

/** @brief 失能三轴 DM 电机 */
void gimbal_disable(void);

/** @brief 云台周期任务：读指令与 IMU，维护使能状态并刷新三轴目标/反馈（2ms） */
gimbal_status_e gimbal_work(void);

/** @brief 读云台使能状态：1=使能，0=失能 */
uint8_t gimbal_getStatus(void);
#endif
