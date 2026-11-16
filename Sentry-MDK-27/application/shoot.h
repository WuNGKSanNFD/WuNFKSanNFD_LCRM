/**
  * @file    shoot.h
  * @brief   发射机构：双摩擦轮（M3508×2）+ 拨盘（M2006）速度环接口
  */

#ifndef __SHOOT_H
#define __SHOOT_H

#include "DJI_motor.h"
#include "cmd.h"

#define SHOOT_DAIL_CNT 9.0f //拨盘孔位数量
#define SHOOT_DIAL_HZ_TO_RPM  60.0f / SHOOT_DAIL_CNT
#define SHOOT_FRIC_DISABLE_SPEED  1.0f //摩擦轮失能速度阈值，降到该值以下才允许失能

/*********************************结构体************************************/
/** @brief 机构使能位：bit0=摩擦轮，bit4=拨盘 */
typedef enum
{
  shoot_off = 0x00,//失能
  shoot_fric_on = 0x01,//摩擦轮使能
  shoot_dial_on = 0x10,//拨盘使能
}shoot_workStatus_e;

typedef enum
{
  shoot_ok = 0x01,
  shoot_error = 0x00,
}shoot_status_e;
/*********************************外部调用************************************/
/** @brief 初始化摩擦轮×2、拨盘电机（CAN2），结束后全部失能 */
void shoot_init(void);

/** @brief 摩擦轮使能（两轮转速在 shoot_work 中按方向下发） */
void shoot_FricEnable(void);

/** @brief 摩擦轮失能 */
void shoot_FricDisable(void);

/** @brief 拨盘使能 */
void shoot_dialEnable(void);

/** @brief 拨盘失能 */
void shoot_dialDisable(void);

/** @brief 发射周期任务：按 cmd.workFlag（0失能/1使能/2摩擦轮/3连发/4单发）下发目标 */
shoot_status_e shoot_work(void);

/** @brief 读机构状态：任一机构使能返回 1，否则 0 */
uint8_t shoot_getStatus(void);
#endif
