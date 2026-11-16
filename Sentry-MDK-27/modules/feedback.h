/**
  * @file        feedback.h
  * @brief       通用前馈与重力补偿：单实例结构体 + 模式位可裁剪的速度/加速度前馈、摩擦补偿与重力补偿
  * @note        纯计算函数无全局状态，可在任意任务/中断调用，但同一 feedback_instance_s 不可并发调用
  * @note        各增益量纲隐含采样周期（未做 dt 归一化），改变调用周期需重新整定
  * @note        输出可赋给 pid_instance_s::f_out 参与 PID 求和；也可单独用作电流/力矩前馈
  */

#ifndef  __FEEDBACK_H
#define  __FEEDBACK_H

#include "main.h"
#include <math.h>

/***************结构体声明*******************/

/**
 * @brief 前馈模式选择（位掩码，可按位或组合后赋给 feedback_instance_s::mode）
 * @note  每个模式位独立控制一个补偿项，未置位的项输出恒为 0，对应参数可填 0
 * @note  feedback_mode_error(0x0F) 覆盖 bit0~bit3，feedback_calculate() 用它判断是否至少置位一个模式
 */
typedef enum
{
  //Keil(AC5)不支持 0b 二进制字面量写法，故用 (1 << n) 表示模式位
  feedback_mode_velocity     =  (1 << 0),//bit0 速度前馈：k_v × 参考速度
  feedback_mode_acceleration =  (1 << 1),//bit1 加速度前馈：k_a × 参考加速度
  feedback_mode_friction     =  (1 << 2),//bit2 摩擦补偿：按参考速度方向输出 ±k_s，零速附近线性过渡
  feedback_mode_gravity      =  (1 << 3),//bit3 重力补偿：gravity_gain × cos(角度 - 相位)
  
  feedback_mode_error        =  (1 << 4) - 1 //模式错误 0x0F，bit0~bit3 全为 0 时非法
  
}feedback_mode_e;

/**
 * @brief 前馈/重力补偿实例结构体
 * @note  "初始化调用"分区由调用者填写；"初始化不需调用"分区是 feedback_calculate() 维护的状态量，初始化时整体清零即可
 * @note  输出量纲由调用者统一（电流环 count、速度环 rpm、位置环 count 等），所有增益按该量纲整定
 * @note  重力补偿按单连杆模型 T = gravity_gain × cos(angle - gravity_phase)；多连杆机构需按实际模型叠加
 */
typedef struct
{
  /******初始化调用*********/
  
  float k_v;//速度前馈增益（输出/参考速度）；=0 时该模式位不起作用
  float k_a;//加速度前馈增益（输出/参考加速度）；=0 时该模式位不起作用
  float k_s;//摩擦补偿幅值（输出单位）；|v| > v_eps 时按速度方向输出 ±k_s，符号按实际摩擦方向给定
  float v_eps;//摩擦补偿速度阈值（同参考速度单位）；|v| <= v_eps 时线性过渡，避免零速附近符号抖振；必须 > 0
  float gravity_gain;//重力补偿幅值（输出单位）；无重力时填 0；符号按重力方向给定
  float gravity_phase;//重力补偿相位（rad）；angle = gravity_phase 时补偿最大；角度从水平面起算填 0，从竖直起算填 π/2
  float maxout;//输出限幅 [-maxout, +maxout]，单位同输出量，必填；=0 则输出恒为 0
  feedback_mode_e mode;//前馈模式，必填；写 0 会在 feedback_calculate() 中直接返回 feedback_error
  
  /**********初始化不需调用**************/
  
  //输入量
  float target_vel;//参考速度，单位随对象（rad/s、rpm、count/周期 等）；每次调用由入参覆盖
  float target_acc;//参考加速度，单位随对象；每次调用由入参覆盖
  float angle;//重力补偿用角度（rad），建议传实际角度使补偿随姿态变化；无重力补偿时传 0
  
  //输出环节
  float v_out;//速度前馈输出 = k_v * target_vel
  float a_out;//加速度前馈输出 = k_a * target_acc
  float s_out;//摩擦补偿输出，已按 v_eps 做零速过渡
  float g_out;//重力补偿输出 = gravity_gain * cos(angle - gravity_phase)
  float output;//本轮总输出 = v_out + a_out + s_out + g_out（已按 maxout 限幅）
  
}feedback_instance_s;


/**
  * @brief 前馈/重力补偿状态枚举
  */
typedef enum
{
  feedback_ok,//计算成功，feedback_instance_s::output 已更新
  feedback_error//计算失败：mode 非法（bit0~bit3 全为 0），本次不更新任何输出
}feedback_status_e;



/****************外部调用函数******************/


/**
 * @brief 前馈 + 重力补偿计算（结果就地写入实例的 output 及各分项）
 * @param target_vel 参考速度，单位随被控对象；无速度前馈时传 0
 * @param target_acc 参考加速度，单位随被控对象；无加速度前馈时传 0
 * @param angle      重力补偿角度 rad，建议用实际角度；无重力补偿时传 0
 * @param feedback_instance 参数指针，指向已初始化的实例，不可为 NULL；增益与模式须提前配好
 * @return feedback_ok=计算成功；feedback_error=mode 非法，本次不更新任何输出
 * @note  周期调用，每次会就地更新 v_out/a_out/s_out/g_out/output 与输入状态量
 * @note  输出可赋给 pid_instance_s::f_out；参考速度/加速度可由上层轨迹给定，也可由目标差分得到
 */
feedback_status_e feedback_calculate(float target_vel,float target_acc,float angle,feedback_instance_s *feedback_instance);

/**
 * @brief 单独计算重力补偿（不检查 mode，不修改 v_out/a_out/s_out）
 * @param angle 重力补偿角度 rad，建议用实际角度
 * @param feedback_instance 参数指针，不可为 NULL；gravity_gain/gravity_phase 须提前配好
 * @return 重力补偿量 = gravity_gain * cos(angle - gravity_phase)，同时写入 g_out 与 angle
 * @note  仅需重力补偿时可直接把返回值赋给 pid_instance_s::f_out；与 feedback_calculate() 二选一，避免重复补偿
 */
float feedback_gravity_calculate(float angle,feedback_instance_s *feedback_instance);

#endif
