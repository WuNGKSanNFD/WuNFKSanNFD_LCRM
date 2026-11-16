/**
  * @file    DJI_motor.h
  * @brief   DJI 电机（M2006/M3508/GM6020）CAN 通信与双环 PID 控制
  * @note    DJI_Motor_work() 内部完成计算与发帧，控制周期调用一次即可
  */

#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H

#include "pid.h"
#include "bsp_can.h"
#include <stdlib.h>

#define DJI_MOTOR_CNT 10
#define M2508_REDUCTION_RATIO   (268.0f/17.0f)
#define M2006_REDUCTION_RATIO   36.0f

/** @brief 电机反馈数据（CAN 中断更新，读取按快照使用） */
typedef struct
{
  float angle;             //输出轴角度 deg
  float speed;             //输出轴转速 rpm
  float temperature;       //电机温度 ℃，仅 GM6020 有效
  float rotor_angle;       //转子机械角度 deg [0,360)
  float rotor_speed;       //转子转速 rpm
  int16_t count;           //转子圈数
  float last_rotor_angle;  //上次转子角度 deg
  float pidSpeed_output;   //速度环上一轮输出
  float pidAngle_output;   //位置环上一轮输出
}DJI_motor_rxData_s;

/** @brief 电机型号 */
typedef enum
{
  DJI_M2006 = 0x00,//M2006 + C610
  DJI_M3508 = 0x01,//M3508 + C620
  DJI_G6020 = 0x02 //GM6020
}DJI_motor_type_e;

/** @brief 电机身份（型号 + 电调 ID + CAN 口） */
typedef struct
{
  DJI_motor_type_e motor_type;
  uint8_t ID;              //1~8
  CAN_HandleTypeDef* hcan; //&hcan1 或 &hcan2
}DJI_motor_motorID_s;

/** @brief 使能状态 */
typedef enum
{
  motor_enable,
  motor_disable
}DJI_motor_workstatus_e;

/** @brief 闭环模式 */
typedef enum
{
  DJI_motor_pidSpeed,   //速度单环
  DJI_motor_pidPosition //位置环 + 速度环
}DJI_motor_pidMode_e;

/** @brief 电机注册参数 */
typedef struct
{
  DJI_motor_motorID_s motorID;
  DJI_motor_pidMode_e pidMode;
  pid_instance_s pid_speed;
  pid_instance_s pid_position;
}DJI_Motor_config_s;

/** @brief 函数状态 */
typedef enum
{
  DJI_motor_ok,
  DJI_motor_error,
  DJI_motor_false
}DJI_motor_status_e;

/**
  * @brief 注册电机并启动对应 CAN 服务
  * @param config 注册参数
  * @return ok=成功；error=参数非法/重复注册/实例已满
  */
DJI_motor_status_e DJI_motor_init(DJI_Motor_config_s *config);

/**
  * @brief 使能电机
  * @param motorID 电机身份
  */
DJI_motor_status_e DJI_motor_enable(DJI_motor_motorID_s motorID);

/**
  * @brief 失能电机并把该电机输出清 0
  * @param motorID 电机身份
  */
DJI_motor_status_e DJI_motor_disable(DJI_motor_motorID_s motorID);

/**
  * @brief 设置目标位置/速度
  * @param motorID 电机身份
  * @param targetPosition 目标位置 deg
  * @param targetSpeed 目标速度 rpm
  * @note 位置模式下 targetSpeed 会被位置环输出覆盖
  */
DJI_motor_status_e DJI_motor_setTarget(DJI_motor_motorID_s motorID,float targetPosition,float targetSpeed);

/**
  * @brief 设置外部反馈，调用后不再使用 CAN 反馈作为测量值
  * @param motorID 电机身份
  * @param Position 测量位置 deg
  * @param Speed 测量速度 rpm
  */
DJI_motor_status_e DJI_motor_setFeedbcak(DJI_motor_motorID_s motorID,float Position,float Speed);

/**
  * @brief 设置闭环模式
  * @param motorID 电机身份
  * @param pidMode 速度单环或位置+速度双环
  */
DJI_motor_status_e DJI_motor_setPidMode(DJI_motor_motorID_s motorID,DJI_motor_pidMode_e pidMode);

/**
  * @brief 在线修改 PID 参数
  * @param motorID 电机身份
  * @param kpPosition 位置环 kp
  * @param kdPosition 位置环 kd
  * @param kpSpeed 速度环 kp
  * @param kiSpeed 速度环 ki
  */
DJI_motor_status_e DJI_motor_setPidConfig(DJI_motor_motorID_s motorID,float kpPosition,float kdPosition,float kpSpeed,float kiSpeed);

/**
  * @brief 闭环计算并发送 CAN 帧
  * @return ok=完成；error=计算或发送失败
  */
DJI_motor_status_e DJI_Motor_work(void);

/**
  * @brief 读取反馈快照
  * @param motorID 电机身份
  * @param data 出参
  */
DJI_motor_status_e DJI_motor_getData(DJI_motor_motorID_s motorID,DJI_motor_rxData_s *data);

#endif
