/******************************************************************************
 * @file        kalman_filter.h
 * @brief       线性卡尔曼滤波器（状态 2 维）：kalman_init1 观测 1 维、kalman_init2 观测 2 维，用于 IMU 角度/角速度融合。
 * @note        调用流程：kalman_initX() 取 id -> 每周期 kalman_setMeasure() -> kalman_work() -> kalman_getData()。
 ******************************************************************************/

#ifndef  __KALMAN_FILTER_H
#define  __KALMAN_FILTER_H

#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "matrix.h"
/*******************************宏*******************************/
/**
 * @brief 可同时注册的滤波器实例上限（kal[] 数组长度），取值 >= 1
 */
#define KALMAN_CNT    3   //滤波器实例上限：kal[] 数组长度，取值 >=1，当前 3 个（roll/pitch/yaw 各一）

/*******************************结构体*******************************/
/**
  * @brief 双观测量滤波器配置（状态 2 维、观测 2 维），配合 kalman_init2() 使用
  * @note  数组按行优先展开传入，如 A = {1, dt, 0, 1} 即 [[1, dt], [0, 1]]
  */
typedef struct
{
  //有两个观测值使用
  float A[2][2];//状态转移矩阵(2x2)：X_pre = A * X_out_l + B * u
  float B[2][1];//控制矩阵(2x1)：把 1 维控制量 u 映射到 2 维状态；无控制量填 0
  float Q[2][2];//过程噪声协方差(2x2，对称)：越大越信任观测
  float R[2][2];//测量噪声协方差(2x2，对称)：越大越信任预测
  float H[2][2];//观测矩阵(2x2)：Z = H * X
  
}kalman_config2_s;

/**
  * @brief 单观测量滤波器配置（状态 2 维、观测 1 维），配合 kalman_init1() 使用
  * @note  H = {0, 1} 时只观测 2 维状态中的第 2 个量
  */
typedef struct
{
  //有一个观测值使用
  float A[2][2];//状态转移矩阵(2x2)：X_pre = A * X_out_l + B * u
  float B[2][1];//控制矩阵(2x1)：把 1 维控制量 u 映射到 2 维状态；无控制量填 0
  float Q[2][2];//过程噪声协方差(2x2，对称)：越大越信任观测
  float R[1][1];//测量噪声协方差(1x1 方差)：越大越信任预测
  float H[1][2];//观测矩阵(1x2)：Z = H * X，2 维状态映射为 1 维观测
  
}kalman_config1_s;


/**
  * @brief 卡尔曼滤波输出数据（kalman_getData() 按值返回的副本）
  */
typedef struct
{
  float X_out[2][1];//最优（后验）估计 [x0, x1]^T
  float X_pre[2][1];//先验估计（预测值）X_pre = A * X_out_l + B * u
  float test[2][2];//调试预留：当前恒被填 0，无代码写入
}kalman_data_s;

/**
  * @brief 卡尔曼滤波执行状态（kalman_work() 的返回值）
  */
typedef enum
{
  kalman_ok,     //0：执行成功，已完成预测 + 更新，可读取新的 X_out
  kalman_error,  //1：无新的测量数据，本次未做任何计算
}kalman_status_e;
/********************************外部调用********************************/
/**
 * @brief  初始化一个单观测量滤波器实例（状态 2 维、观测 1 维）
 * @param  config  配置指针，不可为 NULL；A/B/Q/R/H 被拷贝，其余内部量清零
 * @return 实例 id，范围 0 ~ KALMAN_CNT-1，按注册顺序自增（首个为 0）；无失败返回值
 * @note   必须在周期性控制任务开始前调用，不要放在循环或重复初始化路径中
 */
uint8_t kalman_init1(kalman_config1_s *config);

/**
 * @brief  初始化一个双观测量滤波器实例（状态 2 维、观测 2 维）
 * @param  config  配置指针，不可为 NULL；A/B/Q/R/H 被拷贝，其余内部量清零
 * @return 实例 id，范围 0 ~ KALMAN_CNT-1，与 kalman_init1() 共用同一自增计数
 * @note   与 init1 的唯一区别是观测维数为 2：H、R、卡尔曼增益 K_k 均为 2x2
 */
uint8_t kalman_init2(kalman_config2_s *config);

/**
 * @brief  写入本次观测量与控制量，并置位“数据已更新”标志
 * @param  id       kalman_init1()/kalman_init2() 返回的实例 id
 * @param  measure  观测向量 Z：init1 的实例必须 1x1，init2 的实例必须 2x1，维数须与初始化一致
 * @param  u        控制向量 1x1（对应 B 为 2x1）；无控制量传 1x1 的 0
 * @return 无
 * @note   每周期先调用本函数，紧接着调用 kalman_work()
 */
void kalman_setMeasure(uint8_t id,matrix_instance_s measure,matrix_instance_s u);

/**
 * @brief  执行一次卡尔曼迭代（先验预测 + 观测更新）
 * @param  id  kalman_init1()/kalman_init2() 返回的实例 id
 * @return kalman_ok=已完成；kalman_error=无新测量数据，本次未计算
 * @note   迭代：X_pre=A*X_out+B*u；K=P_pre*H^T*(H*P_pre*H^T+R)^-1；X_out=X_pre+K*(Z-H*X_pre)
 * @warning id >= KALMAN_CNT 时会越界；同时 kalman_filter.c 未检查 malloc 返回值。
 */
kalman_status_e kalman_work(uint8_t id);

/**
 * @brief  读取滤波器上一次迭代的结果（纯读取，不触发计算）
 * @param  id  kalman_init1()/kalman_init2() 返回的实例 id
 * @return X_out 为最优（后验）估计、X_pre 为先验估计，test 恒为 0；从未成功 work 过则返回全 0
 * @note   须在 kalman_work() 返回 kalman_ok 之后再读取
 */
kalman_data_s kalman_getData(uint8_t id);

#endif
