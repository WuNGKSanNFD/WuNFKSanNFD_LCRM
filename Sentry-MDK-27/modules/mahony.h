/** @file   mahony.h
 *  @brief  6 轴 Mahony 姿态融合：用加速度计重力方向修正陀螺仪，四元数积分后输出 roll/pitch/yaw。
 *  @note   调用流程 mahony_init() 取 id -> 每周期 mahony_update() -> mahony_getData()；陀螺仪单位 °/s、加速度单位 g。
 */

#ifndef __MAHONY_H
#define __MAHONY_H

#include "main.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"

/*****************************宏********************************/
#define MAHONY_CNT    1   ///< 可同时注册的滤波器实例上限（mahony[] 数组长度），本工程只用 1 个


/*****************************结构体********************************/
/** @brief Mahony 滤波器配置（由调用方填写，mahony_init() 按值拷贝） */
typedef struct
{
  float kp;                       ///< 重力修正比例增益：越大收敛越快，线性加速度/振动造成的误差也越大
  float ki;                       ///< 陀螺仪零偏积分增益，=0 关闭积分修正（6 轴无磁力计，yaw 零偏不可观测）
  float dt;                       ///< 更新周期，单位 s（本工程 0.001f，对应 imuTask 的 1ms）
}mahony_config_s;

/** @brief Mahony 融合输出数据（mahony_getData() 按值返回） */
typedef struct
{
  float q0;                       ///< 四元数 w
  float q1;                       ///< 四元数 x
  float q2;                       ///< 四元数 y
  float q3;                       ///< 四元数 z
  float roll;                     ///< 横滚角，单位 deg（ZYX 标准定义，与工程 atan2(acc_y,acc_z) 同向）
  float pitch;                    ///< 俯仰角，单位 deg（ZYX 标准定义，与工程 atan2(acc_x,acc_z) 反向）
  float yaw;                      ///< 偏航角，单位 deg（欧拉角 -180~180，陀螺仪 z 轴积分得到，上电从 0 开始且会缓慢漂移）
}mahony_data_s;

/** @brief Mahony 执行状态 */
typedef enum
{
  mahony_ok,                      ///< 0：更新完成
  mahony_error,                   ///< 1：实例 id 非法或未初始化，本次未计算
}mahony_status_e;


/*****************************外部调用********************************/
/**
 * @brief  初始化一个 Mahony 滤波器实例（四元数置为单位四元数，积分项清零）
 * @param  config 配置指针，不能为 NULL；kp/ki/dt 被拷贝
 * @return 实例 id，范围 0 ~ MAHONY_CNT-1，按注册顺序自增（首个为 0）；无失败返回值
 * @note   必须在周期性任务开始前调用，不要放在循环或重复初始化路径中（最多注册 MAHONY_CNT 个实例）
 */
uint8_t mahony_init(mahony_config_s *config);

/**
 * @brief  执行一次 6 轴 Mahony 融合（重力修正 + 四元数积分 + 欧拉角换算）
 * @param  id       mahony_init() 返回的实例 id
 * @param  gx/gy/gz 陀螺仪角速度，单位 °/s（内部换算为 rad/s）
 * @param  ax/ay/az 加速度计加速度，单位 g（内部归一化；模长 < 0.1g 时本次不修正，仅积分陀螺仪）
 * @return mahony_ok=完成；mahony_error=id 非法或实例未初始化
 * @note   须按 config.dt 周期调用；结果用 mahony_getData() 读取
 */
mahony_status_e mahony_update(uint8_t id,float gx,float gy,float gz,float ax,float ay,float az);

/**
 * @brief  读取最近一次融合结果（纯读取，不触发计算）
 * @param  id mahony_init() 返回的实例 id
 * @return 四元数与欧拉角副本；id 非法或从未更新时返回单位四元数/零角度
 * @note   须在 mahony_update() 返回 mahony_ok 之后再读取
 */
mahony_data_s mahony_getData(uint8_t id);

#endif
