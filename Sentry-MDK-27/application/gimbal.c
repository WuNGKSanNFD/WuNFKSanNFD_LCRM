#include "gimbal.h"

/**********************************私有函数及其变量***********************************/
typedef struct
{
  uint8_t workStatus;
  DM_motor_config_s yaw_motor;
  DM_motor_config_s big_pitch_motor;
  DM_motor_config_s small_pitch_motor;
  gimbal_data_s data;
}gimbal_instance_s;
gimbal_instance_s *gimbal = NULL;

/***********************************外部调用*************************************/
/**
 * @brief 初始化三轴 DM 电机（MIT 模式 + 位置环）并注册，结束后处于失能态
 * @note  三轴 ID/MST_ID 需按实际接线区分，当前均为 0x05/0x01
 */
void gimbal_init(void)
{
  gimbal = (gimbal_instance_s *)malloc(sizeof(gimbal_instance_s));
  memset(gimbal,0,sizeof(gimbal_instance_s));
  
  gimbal->yaw_motor = (DM_motor_config_s)
  {
    .motor_type   = DM_J4310,
    .ID           = 0x01,
    .MST_ID       = 0x11,
    .control_mode = DM_MIT,
    .pidMode      = DM_motor_pidPosition, //闭环模式：位置环（目标/反馈均为 rad）
    .hcan         = &hcan2,
    .PMAX         = 3.1415926f,
    .VMAX         = 10.0f,
    .TMAX         = 28.0f,
    
    //速度环：小陀螺用
    .pid_speed = {
      .kp     = 1.0f,
      .ki     = 0.0f,
      .kd     = 0,
      .max_i  = 4.0f,      //积分限幅
      .maxout = 28.0f,     //输出限幅
      .dead   = 0,         //死区
      .pid_mode = pid_mode_normal,
    },
    
    //位置环：底盘反扭/跟随用
    .pid_position = {
      .kp     = 1.0f,
      .ki     = 0,
      .kd     = 0,
      .max_i  = 2.0f,      //积分限幅
      .maxout = 100.0f,    //输出限幅
      .dead   = 0,         //死区
      .pid_mode = pid_mode_i_separation,
    },
  };
  
  gimbal->big_pitch_motor = (DM_motor_config_s)
  {
    .motor_type   = DM_J4310,
    .ID           = 0x02,
    .MST_ID       = 0x12,
    .control_mode = DM_MIT,
    .pidMode      = DM_motor_pidPosition, //闭环模式：位置环（目标/反馈均为 rad）
    .hcan         = &hcan2,
    .PMAX         = 3.1415926f,
    .VMAX         = 30.0f,
    .TMAX         = 10.0f,
    
    //速度环：小陀螺用
    .pid_speed = {
      .kp     = 1.0f,
      .ki     = 0.05f,
      .kd     = 0,
      .max_i  = 4.0f,      //积分限幅
      .maxout = 10.0f,     //输出限幅
      .dead   = 0,         //死区
      .pid_mode = pid_mode_normal,
    },
    
    //位置环：底盘反扭/跟随用
    .pid_position = {
      .kp     = 6.0f,
      .ki     = 0,
      .kd     = 0,
      .max_i  = 2.0f,      //积分限幅
      .maxout = 3.0f,    //输出限幅
      .dead   = 0,         //死区
      .pid_mode = pid_mode_i_separation,
    },
  };
  
  gimbal->small_pitch_motor = (DM_motor_config_s)
  {
    .motor_type   = DM_J4310,
    .ID           = 0x03,
    .MST_ID       = 0x013,
    .control_mode = DM_MIT,
    .pidMode      = DM_motor_pidPosition, //闭环模式：位置环（目标/反馈均为 rad）
    .hcan         = &hcan2,
    .PMAX         = 3.1415926f,
    .VMAX         = 30.0f,
    .TMAX         = 10.0f,
    
    //速度环：小陀螺用
    .pid_speed = {
      .kp     = 1.0f,
      .ki     = 0.01f,
      .kd     = 0,
      .max_i  = 1.0f,      //积分限幅
      .maxout = 10.0f,     //输出限幅
      .dead   = 0,         //死区
      .pid_mode = pid_mode_normal,
    },
    
    //位置环：底盘反扭/跟随用
    .pid_position = {
      .kp     = 22.0f,
      .ki     = 0,
      .kd     = 0,
      .max_i  = 2.0f,      //积分限幅
      .maxout = 9.0f,    //输出限幅
      .dead   = 0,         //死区
      .pid_mode = pid_mode_i_separation,
    },
  };
  
  DM_motor_init(&gimbal->yaw_motor);
  DM_motor_init(&gimbal->big_pitch_motor);
  DM_motor_init(&gimbal->small_pitch_motor);
  
  gimbal_disable();
}

/** @brief 使能三轴电机（未检查 DM_motor_enable 返回值） */
void gimbal_enable(void)
{
  DM_motor_disable(gimbal->yaw_motor.ID);
  DM_motor_enable(gimbal->big_pitch_motor.ID);
  DM_motor_enable(gimbal->small_pitch_motor.ID);
  gimbal->workStatus = 1;
}

/** @brief 失能三轴电机 */
void gimbal_disable(void)
{
  DM_motor_disable(gimbal->yaw_motor.ID);
  DM_motor_disable(gimbal->big_pitch_motor.ID);
  DM_motor_disable(gimbal->small_pitch_motor.ID);
  
  gimbal->workStatus = 0;
}

/**
 * @brief 云台周期任务：读 cmd 与 IMU，维护使能状态并刷新三轴目标/反馈
 * @note  失能请求时先回零位、到位后才真正失能；IMU 角度为 deg，需换算成 rad
 */
gimbal_status_e gimbal_work(void)
{
  //获取云台信息
  cmd_gimbalCMD_s cmd = cmd_gimbalGetCMD();
  
  //获取电机信息
  DM_motor_rxData_s yaw_data; 
  DM_motor_rxData_s bigPitch_data; 
  DM_motor_rxData_s smallPitch_data; 
  DM_motor_getData(gimbal->yaw_motor.ID,&yaw_data);
  DM_motor_getData(gimbal->big_pitch_motor.ID,&bigPitch_data);
  DM_motor_getData(gimbal->small_pitch_motor.ID,&smallPitch_data);
  
  //IMU数据
  IMU_data_s IMU_data = IMU_getData();
  
  static uint8_t last_workStatus = 0;   //上一周期使能状态，用于使能沿检测
  if(cmd.workFlag == 0)
  {
    //清空数据
    memset(&gimbal->data, 0, sizeof(gimbal_data_s));
    
    //失能前先回零位（低趴），三轴都进入判定范围才真正失能
    if(gimbal->workStatus == 1)
    {
      //small pitch：用 IMU 姿态闭环，目标 0 rad
      DM_motor_setFeedbcak(gimbal->small_pitch_motor.ID, IMU_data.filter_roll * GIMBAL_DEG_TO_RAD, smallPitch_data.speed);
      DM_Motor_setPidConfig(gimbal->small_pitch_motor.ID, 0, 0);
      
      
      if(fabsf(yaw_data.position) >= GIMBAL_DISABLE_RANGE * GIMBAL_DEG_TO_RAD)
      {
        //yaw：反馈切回编码器（rad），目标 0
        DM_motor_setFeedbcak(gimbal->yaw_motor.ID, yaw_data.position, yaw_data.speed);
        DM_Motor_setPidConfig(gimbal->yaw_motor.ID, 0, 0);
      }
      //big pitch：目标 0 rad
      else if(fabsf(bigPitch_data.position) >= GIMBAL_DISABLE_RANGE * GIMBAL_DEG_TO_RAD)
      {
        DM_Motor_setPidConfig(gimbal->big_pitch_motor.ID, 0, 0);
      }
      else
      {
        gimbal_disable();
      }
    }
    return gimbal_error;
  }
  //使能云台
  else if(cmd.workFlag == 1 && last_workStatus == 0)
  {
    gimbal_enable();
    DM_Motor_setPidConfig(gimbal->big_pitch_motor.ID, 86.0f * GIMBAL_DEG_TO_RAD, 0);
  }
  
  //传参（cmd 为 deg，统一换算成 rad）
  gimbal->data.bigPitch_angle = 86.0f * GIMBAL_DEG_TO_RAD;
  gimbal->data.smallPitch_angle = -cmd.pitch * GIMBAL_DEG_TO_RAD;
  gimbal->data.yaw_angle = cmd.small_yaw * GIMBAL_DEG_TO_RAD;
  
  //设置反馈：位置用 IMU 姿态角（deg→rad），速度仍用编码器 rad/s
  DM_motor_setFeedbcak(gimbal->yaw_motor.ID, IMU_data.filter_yaw * GIMBAL_DEG_TO_RAD, yaw_data.speed);
  DM_motor_setFeedbcak(gimbal->small_pitch_motor.ID, IMU_data.filter_roll  * GIMBAL_DEG_TO_RAD, smallPitch_data.speed);
  
  //更新三轴目标（rad）
  DM_Motor_setPidConfig(gimbal->yaw_motor.ID, gimbal->data.yaw_angle, 0);
  DM_Motor_setPidConfig(gimbal->big_pitch_motor.ID, gimbal->data.bigPitch_angle, 0);
  DM_Motor_setPidConfig(gimbal->small_pitch_motor.ID, gimbal->data.smallPitch_angle, 0);
  
  last_workStatus = gimbal->workStatus;   //记录状态，供下一周期使能沿判断
  return gimbal_ok;
}

/** @brief 读云台使能状态：1=使能，0=失能 */
uint8_t gimbal_getStatus(void)
{
  return gimbal->workStatus;
}
