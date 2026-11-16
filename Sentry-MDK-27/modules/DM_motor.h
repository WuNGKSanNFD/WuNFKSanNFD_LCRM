/**
  * @file    DM_motor.h
  * @brief   达妙 DM 电机 CAN 控制：寄存器读写 + MIT/PVT + PID 闭环
  * @note    MIT/PVT/闭环控制须 2ms 周期发帧，否则触发通信超时保护
  */

#ifndef __DM_MOTOR_H
#define __DM_MOTOR_H

#include "bsp_can.h"
#include "pid.h"
#include <string.h> 
#include <stdlib.h> 

/** @brief 寄存器地址（RID），读写命令发往 CAN ID 0x7FF */
typedef enum {
    REG_UV_Value   = 0x00,//欠压保护值 V
    REG_KT_Value   = 0x01,//转矩系数 Kt
    REG_OT_Value   = 0x02,//过温保护阈值 ℃
    REG_OC_Value   = 0x03,//过流保护值（归一化）
    REG_ACC        = 0x04,//加速度限制
    REG_DEC        = 0x05,//减速度限制
    REG_MAX_SPD    = 0x06,//最大转速 rad/s
    REG_MST_ID     = 0x07,//反馈 CAN ID
    REG_ESC_ID     = 0x08,//电机接收 CAN ID
    REG_TIMEOUT    = 0x09,//通信超时保护（50us）
    REG_CTRL_MODE  = 0x0A,//控制模式 1=MIT 2=位置速度 3=速度 4=PVT
    REG_Damp       = 0x0B,//粘性阻尼系数
    REG_Inertia    = 0x0C,//转动惯量
    REG_hw_ver     = 0x0D,//硬件版本号
    REG_sw_ver     = 0x0E,//软件版本号
    REG_SN         = 0x0F,//产品序列号
    REG_NPP        = 0x10,//电机极对数
    REG_Rs         = 0x11,//相电阻 Ω
    REG_Ls         = 0x12,//相电感 H
    REG_Flux       = 0x13,//磁链值
    REG_Gr         = 0x14,//减速比
    REG_PMAX       = 0x15,//位置映射范围 rad
    REG_VMAX       = 0x16,//速度映射范围 rad/s
    REG_TMAX       = 0x17,//力矩映射范围 N·m
    REG_I_BW       = 0x18,//电流环控制带宽
    REG_KP_ASR     = 0x19,//速度环 Kp
    REG_KI_ASR     = 0x1A,//速度环 Ki
    REG_KP_APR     = 0x1B,//位置环 Kp
    REG_KI_APR     = 0x1C,//位置环 Ki
    REG_OV_Value   = 0x1D,//过压保护值 V
    REG_GREF       = 0x1E,//减速器力矩传递效率
    REG_Beta       = 0x1F,//速度环阻尼系数
    REG_V_BW       = 0x20,//速度环滤波带宽
    REG_IQ_c1      = 0x21,//电流环增强系数
    REG_VL_c1      = 0x22,//速度环增强系数
    REG_can_br     = 0x23,//CAN 波特率代码 0=125K~4=1M
    REG_sub_ver    = 0x24,//子版本号
    REG_Boot_ver   = 0x25,//Bootloader 版本号
    REG_dir        = 0x37,//转动方向
    REG_m_off      = 0x38,//电角度零位偏置
    REG_Imax       = 0x3B,//最大电流 A
    REG_VBus       = 0x3C,//母线电压 V
    REG_Tpcb       = 0x3D,//驱动板温度 ℃
    REG_Tmtr       = 0x3E,//线圈温度 ℃
    REG_Iu_off     = 0x3F,//U 相电流采样零偏
    REG_Iv_off     = 0x40,//V 相电流采样零偏
    REG_Iw_off     = 0x41,//W 相电流采样零偏
    REG_p_m        = 0x50,//转子位置 rad
    REG_xout       = 0x51//输出轴位置 rad
} DM_motor_registerAddress_e;

/** @brief 寄存器读回值缓存（字段含义见 DM_motor_registerAddress_e） */
typedef struct {
    float   UV_Value;
    float   KT_Value;
    float   OT_Value;
    float   OC_Value;
    float   ACC;
    float   DEC;
    float   MAX_SPD;
    uint32_t MST_ID;
    uint32_t ESC_ID;
    uint32_t TIMEOUT;
    uint32_t CTRL_MODE;
    float   Damp;
    float   Inertia;
    uint32_t hw_ver;
    uint32_t sw_ver;
    uint32_t SN;
    uint32_t NPP;
    float   Rs;
    float   Ls;
    float   Flux;
    float   Gr;
    float   PMAX;
    float   VMAX;
    float   TMAX;
    float   I_BW;
    float   KP_ASR;
    float   KI_ASR;
    float   KP_APR;
    float   KI_APR;
    float   OV_Value;
    float   GREF;
    float   Beta;
    float   V_BW;
    float   IQ_c1;
    float   VL_c1;
    uint32_t can_br;
    uint32_t sub_ver;
    uint32_t Boot_ver;
    float   dir;
    float   m_off;
    float   Imax;
    float   VBus;
    float   Tpcb;
    float   Tmtr;
    float   Iu_off;
    float   Iv_off;
    float   Iw_off;
    float   p_m;
    float   xout;
} DM_motor_registerData_s;

/** @brief 使能状态（失能时才允许读写寄存器） */
typedef enum
{
  DM_disable,
  DM_enable,
}DM_motor_workstatus_e;

/** @brief 电机型号 */
typedef enum
{
  DM_J4310,
  DM_J4340,
}DM_motor_type_e;

/** @brief MIT 参数（力矩与位置/速度/kp/kd 不可混用） */
typedef struct
{
  float kp;       //位置刚度 [0,500]
  float kd;       //速度阻尼 [0,5]
  float position; //目标位置 rad
  float speed;    //目标速度 rad/s
  float torque;   //前馈力矩 N·m
}DM_motor_MITconfig_s;

/** @brief PVT 参数 */
typedef struct
{
  float position;   //目标位置 rad
  float max_speed;  //限定速度 rad/s
  float max_torque; //力矩比例 (0,1]
}DM_motor_PVTconfig_s;

/** @brief 控制模式（寄存器 0x0A） */
typedef enum
{
  DM_MIT = 0x01,
  DM_PVT = 0x04
}DM_motor_controlMode_e;

/** @brief 闭环模式 */
typedef enum
{
  DM_motor_pidSpeed,   //速度单环
  DM_motor_pidPosition //位置环 + 速度环
}DM_motor_pidMode_e;

/** @brief 反馈数据（CAN 中断更新，读取按快照使用） */
typedef struct
{
  uint8_t err;           //状态码
  float position;        //输出轴位置 rad
  float speed;           //输出轴转速 rad/s
  float torque;          //输出轴力矩 N·m
  uint16_t T_MOS;        //MOS 温度 ℃
  uint16_t T_Rotor;      //线圈温度 ℃
  float pidSpeed_output; //速度环输出 N·m
  float pidAngle_output; //位置环输出 rad/s
}DM_motor_rxData_s;

/** @brief 电机注册参数（init 按值拷贝；PMAX/VMAX/TMAX 须与寄存器一致） */
typedef struct
{
  DM_motor_type_e motor_type;
  uint8_t ID;            //电机接收 ID
  uint8_t MST_ID;        //反馈 CAN ID
  DM_motor_controlMode_e control_mode; //切换需调用 DM_Motor_mode()
  DM_motor_pidMode_e pidMode;          //闭环模式：速度单环 / 位置+速度双环
  float PMAX;            //位置映射范围 rad
  float VMAX;            //速度映射范围 rad/s
  float TMAX;            //力矩映射范围 N·m
  
  pid_instance_s pid_speed;
  pid_instance_s pid_position;
  
  CAN_HandleTypeDef* hcan;
}DM_motor_config_s;

/** @brief 函数状态 */
typedef enum
{
  DM_motor_ok,
  DM_motor_error,
}DM_motor_status_e;

/***************************************外部调用***************************************/

/** @brief 注册电机并启动 CAN 接收（最多 4 台） */
DM_motor_status_e DM_motor_init(DM_motor_config_s *config);

/** @brief 使能电机 */
DM_motor_status_e DM_motor_enable(uint16_t id);

/** @brief 失能电机 */
DM_motor_status_e DM_motor_disable(uint16_t id);

/** @brief 读取寄存器（结果稍后从 reg_data 查看） */
DM_motor_status_e DM_motor_ReadParameters(uint8_t id,DM_motor_registerAddress_e motor_reg);

/** @brief 写寄存器（RAM 生效，掉电丢失） */
DM_motor_status_e DM_Motor_writeParameters(uint8_t id,DM_motor_registerAddress_e motor_reg,uint32_t regData);

/** @brief 保存零点（会改变位置参考系） */
DM_motor_status_e DM_Motor_saveZeroPosition(uint8_t id);

/** @brief 保存参数到 Flash */
DM_motor_status_e DM_Motor_saveParameters(uint8_t id);

/** @brief 切换控制模式 */
DM_motor_status_e DM_Motor_mode(uint8_t id,DM_motor_controlMode_e motor_mode);

/** @brief 设置 MIT 参数 */
DM_motor_status_e DM_motor_setMITconfig(uint8_t id,float position, float speed, float torque, float kp, float kd);

/** @brief 发送 MIT 控制帧 */
DM_motor_status_e DM_Motor_MITcontrol(void);

/** @brief 设置 PVT 参数 */
DM_motor_status_e DM_motor_setPVTconfig(uint8_t id,float position, float max_speed, float max_torque);

/** @brief 发送 PVT 控制帧 */
DM_motor_status_e DM_Motor_PVTcontrol(void);

/** @brief 设置 PID 目标值 */
DM_motor_status_e DM_Motor_setPidConfig(uint8_t motor_ID,float targetPosition,float targetSpeed);

/**
  * @brief 设置外部反馈，调用后不再使用 CAN 反馈作为测量值
  * @param id 电机 ID
  * @param position 测量位置 rad
  * @param speed 测量速度 rad/s
  */
DM_motor_status_e DM_motor_setFeedbcak(uint8_t id,float position,float speed);

/**
  * @brief 在线修改闭环模式
  * @param motor_ID 电机 ID
  * @param pidMode 速度单环或位置+速度双环
  */
DM_motor_status_e DM_Motor_setPidMode(uint8_t motor_ID,DM_motor_pidMode_e pidMode);

/**
  * @brief PID 闭环控制（速度环/位置环合并入口）
  * @return ok=完成；error=模式非法/PID 计算失败/CAN 发送失败
  * @note 闭环模式由 DM_motor_init() 注册参数决定，此处不再接收模式参数；
  *       先计算所有使能电机，再统一发 MIT 帧，未使能电机跳过
  */
DM_motor_status_e DM_Motor_pidWork(void);

/**
  * @brief 读取反馈快照
  * @param motor_ID 电机 ID
  * @param data 出参，不可为 NULL
  */
DM_motor_status_e DM_motor_getData(uint8_t motor_ID,DM_motor_rxData_s *data);

#endif
