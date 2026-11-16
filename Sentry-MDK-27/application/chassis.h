#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "math.h"

#include "DJI_motor.h"
#include "DM_motor.h"

#include "cmd.h"

/*******************宏********************/
//底盘电机位置编号（motor_ID[] 与 motor_vector[] 的下标）
#define CHASSIS_UL    0//左上
#define CHASSIS_DL    1//左下
#define CHASSIS_DR    2//右下
#define CHASSIS_UR    3//右上

/****************结构体*****************/
/**
 * @brief 底盘类型（仅全向轮实现完整，麦轮只有普通解算，舵轮未实现）
 */
typedef enum
{
  chassis_type_steering,//舵轮（未实现）
  chassis_type_omni,//全向轮
  chassis_type_mecanum,//麦轮
}chassis_type_e;

typedef struct
{
  float chassis_speed[4];
}chassis_data_s;


/**
 * @brief 底盘状态
 */
typedef enum
{
  chassis_ok,//成功
  chassis_error,//失败（电机未注册/失能，或底盘类型不支持）
}chassis_status_e;

/*********************外部调用函数**********************/

/**
 * @brief  初始化底盘：分配实例并写入底盘类型、电机 ID、几何参数与小陀螺参数
 * @note   须在 robot_init() 中调用；重复调用会重新分配实例
 */
void chassis_init(void);

/**
 * @brief  使能底盘：四个 M3508 与 yaw 轴 DM 电机依次上电
 * @return chassis_ok 成功；chassis_error 任一电机使能失败
 */
chassis_status_e chassis_enable(void);

/**
 * @brief  失能底盘：清零指令与解算结果后关闭全部电机
 * @return chassis_ok 成功；chassis_error 任一电机关闭失败
 */
chassis_status_e chassis_disable(void);

/**
 * @brief  底盘工作：读 cmd 指令，按 workFlag 切换行进/小陀螺并解算下发各轮转速
 * @note   失能指令或解算失败时自动调用 chassis_disable()
 * @return chassis_ok 正常；chassis_error 已失能或解算失败
 */
chassis_status_e chassis_work(void);


uint8_t chassis_getStatus(void);

chassis_data_s chassis_getData(void);

#endif
