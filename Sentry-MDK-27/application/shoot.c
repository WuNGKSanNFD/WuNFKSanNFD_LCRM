#include "shoot.h"

/*********************************私有函数及变量************************************/
typedef struct
{ 
  DJI_motor_motorID_s fric_left_motor_ID;
  DJI_motor_motorID_s fric_right_motor_ID;
  DJI_motor_motorID_s dial_motor_ID;
  DJI_Motor_config_s fric_motor;
  DJI_Motor_config_s dial_motor;
  
  
  shoot_workStatus_e workFlag;
}shoot_instance_s;
shoot_instance_s *shoot = NULL;

/*********************************外部调用************************************/
/** @brief 初始化摩擦轮 M3508×2（CAN2 ID3/4）与拨盘 M2006（CAN2 ID2），初始全部失能 */
void shoot_init(void)
{
  shoot = (shoot_instance_s *)malloc(sizeof(shoot_instance_s));
  memset(shoot,0,sizeof(shoot_instance_s));
  
  shoot->fric_left_motor_ID = (DJI_motor_motorID_s){DJI_M3508, 0x03, &hcan2};
  shoot->fric_right_motor_ID = (DJI_motor_motorID_s){DJI_M3508, 0x04, &hcan2};
  shoot->dial_motor_ID = (DJI_motor_motorID_s){DJI_M2006, 0x02, &hcan2};
  
  //摩擦轮
  shoot->fric_motor = (DJI_Motor_config_s)
  {
    .motorID  = shoot->fric_left_motor_ID,
    .pidMode  = DJI_motor_pidSpeed,
    
    //速度环
    .pid_speed = {
      .kp     = 60.0f,
      .ki     = 0.1f,
      .kd     = 0,
      .max_i  = 500.0f,   //积分限幅
      .maxout = 8000.0f,   //输出限幅
      .dead   = 0,         //死区
      .pid_mode = pid_mode_normal,
    },
  };
  
  DJI_motor_init(&shoot->fric_motor);
  shoot->fric_motor.motorID = shoot->fric_right_motor_ID;
  DJI_motor_init(&shoot->fric_motor);
  
  //拨盘
  shoot->dial_motor = (DJI_Motor_config_s)
  {
    .motorID  = shoot->dial_motor_ID,
    .pidMode  = DJI_motor_pidSpeed,
    
    //速度环
    .pid_speed = {
      .kp     = 140.0f,
      .ki     = 1.0f,
      .kd     = 0,
      .max_i  = 3000.0f,   //积分限幅
      .maxout = 30000.0f,   //输出限幅
      .dead   = 0,         //死区
      .pid_mode = pid_mode_normal,
    },
  };
  DJI_motor_init(&shoot->dial_motor);
  
  shoot_FricDisable();
  shoot_dialDisable();
}

/** @brief 摩擦轮使能，并置位状态位 bit0 */
void shoot_FricEnable(void)
{
  shoot->workFlag = (shoot_workStatus_e)((shoot->workFlag&0xF0) | shoot_fric_on);  
  DJI_motor_enable(shoot->fric_left_motor_ID);
  DJI_motor_enable(shoot->fric_right_motor_ID);
}

/** @brief 摩擦轮失能，清除状态位 bit0 */
void shoot_FricDisable(void)
{
  shoot->workFlag = (shoot_workStatus_e)((shoot->workFlag&0xF0) | shoot_off); 
  DJI_motor_disable(shoot->fric_left_motor_ID);
  DJI_motor_disable(shoot->fric_right_motor_ID);
}

/** @brief 拨盘使能，并置位状态位 bit4 */
void shoot_dialEnable(void)
{
  DJI_motor_enable(shoot->dial_motor_ID);
  shoot->workFlag = (shoot_workStatus_e)((shoot->workFlag&0x0F) | shoot_dial_on); 
}

/** @brief 拨盘失能，清除状态位 bit4 */
void shoot_dialDisable(void)
{
  DJI_motor_disable(shoot->dial_motor_ID);
  shoot->workFlag = (shoot_workStatus_e)((shoot->workFlag&0x0F) | shoot_off); 
}

/**
 * @brief 发射周期任务：0=失能（摩擦轮降速后才断电）1=仅使能 2=摩擦轮 3=连发 4=单发（待实现）
 */
shoot_status_e shoot_work(void)
{
  //获取命令
  cmd_shootCMD_s cmd = cmd_shootGetCMD();
  
  //获取电机反馈
  DJI_motor_rxData_s fric_l;
  DJI_motor_rxData_s fric_r;
  DJI_motor_getData(shoot->fric_left_motor_ID, &fric_l);
  DJI_motor_getData(shoot->fric_right_motor_ID, &fric_r);
  
  //判断是否使能
  if(cmd.workFlag == 0)
  {
    //摩擦轮仍在高速，先给 0 目标降速，低于阈值再失能
    if(fabsf(fric_l.speed) >= SHOOT_FRIC_DISABLE_SPEED || fabsf(fric_r.speed) >= SHOOT_FRIC_DISABLE_SPEED)
    {
      DJI_motor_setTarget(shoot->fric_left_motor_ID, 0, 0);
      DJI_motor_setTarget(shoot->fric_right_motor_ID, 0, 0); 
    }
    else
    {
      shoot_FricDisable();
    }
    
    shoot_dialDisable();
    return shoot_error;
  }
  else
  {
    shoot_FricEnable();
    shoot_dialEnable();
  }
  
  //使能发射机构
  if(cmd.workFlag == 1)
  {
    DJI_motor_setTarget(shoot->fric_left_motor_ID, 0, 0);
    DJI_motor_setTarget(shoot->fric_right_motor_ID, 0, 0);
    DJI_motor_setTarget(shoot->dial_motor_ID, 0, 0);
  }
  //开启摩擦轮
  else if(cmd.workFlag == 2)
  {
    DJI_motor_setTarget(shoot->fric_left_motor_ID, 0, -cmd.fric_speed);
    DJI_motor_setTarget(shoot->fric_right_motor_ID, 0,  cmd.fric_speed);
    DJI_motor_setTarget(shoot->dial_motor_ID, 0, 0);
  }
  //连发
  else if(cmd.workFlag == 3)
  {
    DJI_motor_setTarget(shoot->dial_motor_ID, 0, cmd.dial_speed * SHOOT_DIAL_HZ_TO_RPM);
  }
  //单发
  else if(cmd.workFlag == 4)
  {
    //todo
    DJI_motor_setTarget(shoot->fric_left_motor_ID, 0 ,0);
    DJI_motor_setTarget(shoot->fric_right_motor_ID, 0 ,0);
    DJI_motor_setTarget(shoot->dial_motor_ID, 0, 0);
  }
  
  return shoot_ok;
}

/** @brief 读机构状态：任一机构使能返回 1，否则 0 */
uint8_t shoot_getStatus(void)
{
  uint8_t workSatus = 0;
  if(shoot->workFlag == shoot_off)
    workSatus = 0;
  else
    workSatus = 1;
  
  return workSatus;
}
