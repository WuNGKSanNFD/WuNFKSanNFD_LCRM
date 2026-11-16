/**
  * @file        pid.h
  * @brief       通用位置式 PID 控制器：单实例结构体 + 模式位可裁剪的积分/微分优化算法
  * @note        纯计算函数无全局状态，可在任意任务/中断调用，但同一 pid_instance_s 不可并发调用
  * @note        kp/ki/kd 量纲隐含采样周期（未做 dt 归一化），改变调用周期需重新整定
  * @note        前馈不再由本模块计算：f_out 由外部前馈/重力补偿模块（feedback）写入，pid_calculate() 只参与求和
  */

#ifndef  __PID_H
#define  __PID_H

#include "main.h"
#include <math.h>
#include <string.h>

/***************结构体声明*******************/

/**
 * @brief PID模式选择（位掩码，可按位或组合后赋给 pid_instance_s::pid_mode）
 * @note  pid_mode_normal 置位会屏蔽 bit1~bit5；变速积分优先于积分分离；bit6 输出滤波可叠加
 * @note  pid_mode_error(0x7F) 覆盖 bit0~bit6，pid_calculate() 用它判断模式是否合法
 */
typedef enum
{
  //Keil(AC5)不支持 0b 二进制字面量写法，故用 (1 << n) 表示模式位
  pid_mode_normal           =  (1 << 0),//bit0 普通模式：基础积分+误差微分
  pid_mode_i_separation     =  (1 << 1),//bit1 积分分离
  pid_mode_i_variableSpeed  =  (1 << 2),//bit2 变速积分
  pid_mode_i_trapezoidal    =  (1 << 3),//bit3 梯形积分
  pid_mode_d_incomplete     =  (1 << 4),//bit4 不完全微分（微分低通）
  pid_mode_d_onMeasurement  =  (1 << 5),//bit5 微分先行（对测量值微分）
  
  //滤波
  pid_mode_output_filter    =  (1 << 6),//bit6 输出滤波（输出低通）
  
  pid_mode_error            =  (1 << 7) - 1 //模式错误 0x7F，bit7 仅用于合法性判断
  
}pid_mode_e;

/**
 * @brief PID实例结构体
 * @note  "初始化调用"分区由调用者填写（kp/ki/kd 也可由 setPidConfig 在线改写）；
 *        "初始化不需调用"分区是 pid_calculate() 维护的状态量，初始化时整体清零即可
 * @note  各增益未做单位归一：输出量纲 = kp×误差 + ki×误差积分 + kd×误差差分
 */
typedef struct
{
  /******初始化调用*********/
  
  float ki;//积分系数；=0 表示不使用积分作用（i_out 恒为 0）
  float kp;//比例系数；位置环与速度环的 kp 需分别整定
  float kd;//微分系数；=0 时微分环节不参与输出
  float max_i;//积分限幅 [-max_i, +max_i]，单位同输出量，必须 > 0；只限 i_out，不限制 i_term 累积
  float maxout;//输出限幅 [-maxout, +maxout]，单位同输出量，必填；=0 则输出恒为 0
  float dead;//死区阈值（判 |error|），单位同误差；|error| > dead 才刷新 output，否则保持上轮值
  
  pid_mode_e pid_mode;//PID模式，必填；写 0 会在 pid_calculate() 中直接返回 pid_error
  
  //积分分离参数
  float epsilon;//积分分离阈值，单位同误差（位置环 rad/速度环 rpm）；|error| > epsilon 时清零 i_term，=0 等价纯 P
  
  //变速积分参数
  float A;//变速积分外阈值，单位同误差；|error| >= A 时清零 i_term；须满足 A > B
  float B;//变速积分内阈值，单位同误差；|error| <= B 时 i_term 不变，B < |error| < A 时按比例衰减
  
  //不完全微分参数
  float d_f_alpha;//微分低通系数 [0, 1)，越大滤波越强；仅 pid_mode_d_incomplete 置位时生效
  
  //输入滤波参数
  float input_f_alpha;//输入滤波系数：pid.c 中未使用，仅为预留占位，填 0 即可
  
  //输出滤波参数
  float output_f_alpha;//输出低通系数 [0, 1)，越大滤波越强；仅在 pid_mode_output_filter 置位且限幅之后执行
  
//  float t_f;//滤波时间常数
  
  
  /**********初始化不需调用**************/
  
  
  float measure;//测量值（反馈），单位随对象（rad、rpm、count 等）；每次调用由入参覆盖
  float target;//期望值，单位与 measure 一致；每次调用由入参覆盖
  float dt;//控制周期 s：pid.c 中未使用，仅为预留占位，不需要填写
  
  //输出环节
  float error;//本次误差 = target - measure，单位同 measure
  float p_out;//比例环节输出 = kp * error，未限幅
  float i_out;//积分环节输出 = ki * i_term，已按 max_i 限幅
  float d_out;//微分环节输出 = kd * d_term，未限幅
  float f_out;//前馈输出：由外部前馈/重力补偿模块（feedback）写入，pid_calculate() 不修改，仅参与求和
  float output;//本轮最终输出（限幅与滤波后），死区内保持上轮值；送给执行机构的量
  
  //过程变量
  float last_measure;//上一次测量值；用于微分先行
  float last_e;//上一次误差；普通模式与非普通模式的默认微分共用
  float i_term;//误差积分累加项（未乘 ki）；积分分离时被清零
  float d_term;//误差微分项（未乘 kd）；普通模式与其余模式符号相反，调试注意区分
  float last_d_term;//上一次误差微分；仅不完全微分作低通历史项
  float last_output;//上一次输出；用于输出滤波历史项与死区保持

}pid_instance_s;


/**
  * @brief PID状态枚举
  */
typedef enum
{
  pid_ok,//计算成功，pid_instance_s::output 已更新
  pid_error,//计算失败：pid_mode 非法（bit0~bit6 全为 0）
  pid_false//保留/兼容用返回值，pid_calculate() 不会返回该值
}pid_status_e;



/****************外部调用函数******************/


/**
 * @brief PID 计算（结果就地写入实例的 output 及各分项）
 * @param target    期望值，单位随被控对象（位置环 rad、速度环 rpm 等）
 * @param measure   实际值（反馈），单位与 target 一致
 * @param pid_instance 参数指针，指向已初始化的 PID 实例，不可为 NULL；增益与模式须提前配好
 * @return pid_ok=计算成功；pid_error=pid_mode 非法，本次不更新任何输出
 * @note  周期调用，每次会就地更新 error/p_out/i_out/d_out/output 与 last_e/last_measure/last_output 等状态量
 * @note  f_out 由调用者在调用前写入（如 feedback_calculate() 的 output），本函数不清零、不修改
 */
pid_status_e pid_calculate(float target,float measure,pid_instance_s *pid_instance);

#endif
