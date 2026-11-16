/**
 * @file    cmd.h
 * @brief   指令解析：把遥控器/视觉数据换算成底盘、云台、发射机构控制量
 * @note    模式位域(0xF000)与各机构功能位域(低12位)叠加，比较前须先掩码
 */

#ifndef __CMD_H
#define __CMD_H

#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"

#include "DJI_motor.h"
#include "DM_motor.h"
#include "pid.h"

#include "IMU.h"
#include "remote.h"
#include "vision.h"

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"


/*********************************宏**************************************/
//#define CMD_CHASSIS_FOLLOWING


/***************************结构体********************************/

/** @brief 区间参数 */
typedef struct
{
  float max;
  float min;
}cmd_ParaRange_s;

/** @brief 底盘控制量（workFlag=0 时整块清零） */
typedef struct
{
  uint8_t workFlag;          ///< 0=失能，1=使能,2小陀螺
  float angle;               ///< 移动方向，deg（遥控恒为0）
  float forward;             ///< 前进速度，m/s
  float turn;                ///< 横移速度，m/s
  float chassis_rotate;      ///< 底盘自转速度，rad/s
  float top_speed;           ///< 小陀螺速度，rad/s
}cmd_chassisCMD_s;

/** @brief 云台控制量（遥控为增量控制，松手保持角度） */
typedef struct
{
  uint8_t workFlag;          ///< 0=失能，其余=使能
  float small_yaw;           ///< 小yaw目标角，deg（相对small_yaw_init）
  float pitch;               ///< pitch目标角，deg（限幅，yaw当前无限幅）
}cmd_gimbalCMD_s;

/** @brief 发射机构控制量 */
typedef struct
{
  uint8_t workFlag;          ///< 0=失能 1=仅使能 2=仅摩擦轮 3=连发 4=单发
  float fric_speed;          ///< 摩擦轮转速，rpm
  float dial_speed;          ///< 拨盘转速，rpm
}cmd_shootCMD_s;

/**
 * @brief 控制模式位掩码：0xF000 模式 | 0x0F00 发射 | 0x00F0 云台 | 0x000F 底盘
 * @note  先掩码再比较，如 (mode & 0x0F00) == CMD_MODE_FIRE_AUTO
 */
typedef enum
{
  CMD_MODE_OFF = 0x0000,        ///< 关闭：全部失能

  CMD_MODE_REMOTE = 0x1000,     ///< 遥控
  CMD_MODE_KEYBOARD = 0x2000,   ///< 键鼠（未实现）
  CMD_MODE_VISION = 0x3000,     ///< 视觉

  CMD_MODE_CHASSIS_FOLLOW = 0x0001,    ///< 底盘跟随
  CMD_MODE_CHASSIS_NO_FOLLOW = 0x0002, ///< 底盘不跟随

  CMD_MODE_GIMBAL_ON = 0x0010,  ///< 使能云台

  CMD_MODE_TOP = 0x0020,        ///< 小陀螺

  CMD_MODE_SHOOT_ON = 0x0100,       ///< 启用发射机构
  CMD_MODE_FRICTION_ON = 0x0200,    ///< 打开摩擦轮
  CMD_MODE_FIRE_AUTO = 0x0300,      ///< 连发
  CMD_MODE_FIRE_SINGLE = 0x0400,    ///< 单发
}cmd_mode_e;

/***************************外部调用********************************/

/** @brief 初始化：装载默认参数与PID，最后全车失能（调度器启动前调用一次） */
void cmd_init(void);

/** @brief 周期任务：刷新输入、更新控制模式与各机构控制量（cmdTask 4ms，遥控离线即失能） */
void cmd_work(void);

/** @brief 底盘失能：chassisCMD 整块清零 */
void cmd_chassis_disable(void);

/** @brief 云台失能：gimbalCMD 整块清零 */
void cmd_gimbal_disable(void);

/** @brief 发射机构失能：shootCMD 整块清零 */
void cmd_shoot_disable(void);

/** @brief 全车失能：依次失能三机构并清 workStatus */
void cmd_all_disable(void);

/** @brief 读取底盘命令（按值返回副本；cmd 未初始化时返回全 0，即 workFlag=0 失能） */
cmd_chassisCMD_s cmd_chassisGetCMD(void);

/** @brief 读取云台命令（按值返回副本；cmd 未初始化时返回全 0，即 workFlag=0 失能） */
cmd_gimbalCMD_s cmd_gimbalGetCMD(void);

/** @brief 读取发射机构命令（按值返回副本；cmd 未初始化时返回全 0，即 workFlag=0 失能） */
cmd_shootCMD_s cmd_shootGetCMD(void);

#endif
