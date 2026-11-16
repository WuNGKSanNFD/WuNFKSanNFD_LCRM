#include "chassis.h"


/****************************************私有函数*********************************************/
typedef struct
{
  chassis_type_e type;//底盘类型
  DJI_motor_motorID_s motor_ID[4];//四个 M3508 电调 ID，下标见 CHASSIS_UL/DL/DR/UR
  DJI_Motor_config_s chassis_motor;
  
//  DM_motor_config_s yaw_motor;
  
  //底盘数据
  float chassis_radius;//底盘半径，单位 mm
  float tyre_radius;//轮毂半径，单位 mm
  float gyroRotate;//小陀螺目标转速，单位 rad/s
  
  //输出数据
  float angle;
  float forward;//前进速度，单位 m/s（由 cmd 指令写入）
  float turn;//横移速度，单位 m/s
  float rotate;//自转速度，单位 rad/s
  float motor_vector[4];//各轮目标转速，单位 rpm（解算结果）
  
  //状态
  uint8_t workStatus;
}chassis_instance_s;
chassis_instance_s *chassis = NULL;


//底盘解算
static chassis_status_e chassis_soulution(void)
{
  if(chassis->type == chassis_type_omni || chassis->type == chassis_type_mecanum)
  {
    //得到yaw轴转速
    float yaw_rotate = chassis->rotate;
    
    //角度解算
    float forward  = cos(chassis->angle) * chassis->forward + (-sin(chassis->angle) * chassis->turn);
    float turn = sin(chassis->angle) * chassis->forward + (cos(chassis->angle) * chassis->turn);
    
    
    //将输入值转换为解算到各电机输入值的绝对值
    float vector_f = sqrtf(2.0f) * forward * 60.0f / (chassis->tyre_radius / 1000.0f) / (2 * 3.1415926);
    float vector_t = sqrtf(2.0f) * turn * 60.0f / (chassis->tyre_radius / 1000.0f) / (2 * 3.1415926);
    float vector_r = yaw_rotate * (chassis->chassis_radius / 1000.0f) * 60.0f / (chassis->tyre_radius / 1000.0f) / (2 * 3.1415926);
    
    //各电机输入值
    chassis->motor_vector[CHASSIS_UL] =    vector_f + vector_t + vector_r;
    chassis->motor_vector[CHASSIS_DL] =    vector_f - vector_t + vector_r;
    chassis->motor_vector[CHASSIS_DR] =  - vector_f - vector_t + vector_r;
    chassis->motor_vector[CHASSIS_UR] =  - vector_f + vector_t + vector_r;
    
    return chassis_ok;
  }
  else if(chassis->type == chassis_type_steering)
  {
    //todo
    return chassis_error;
  }
  return chassis_error;
}

/***************************************外部调用**************************************/
//初始化
void chassis_init(void)
{
  //给实例分配内存
  chassis = (chassis_instance_s *)malloc(sizeof(chassis_instance_s));
  memset(chassis,0,sizeof(chassis_instance_s));
  
  /****************************底盘参数****************************/
  chassis->type           = chassis_type_omni;   //全向轮
  chassis->chassis_radius = 560 / 2;             //底盘半径 280mm
  chassis->tyre_radius    = 78.9f;               //轮毂半径 78.9mm
  chassis->gyroRotate     = 5.0f;                //小陀螺目标转速 rad/s
  
  //四个底盘 M3508（CAN1）；下标顺序见 CHASSIS_UL/DL/DR/UR
  chassis->motor_ID[0] = (DJI_motor_motorID_s){DJI_M3508, 0x01, &hcan1};//左上
  chassis->motor_ID[1] = (DJI_motor_motorID_s){DJI_M3508, 0x02, &hcan1};//左下
  chassis->motor_ID[2] = (DJI_motor_motorID_s){DJI_M3508, 0x03, &hcan1};//右下
  chassis->motor_ID[3] = (DJI_motor_motorID_s){DJI_M3508, 0x04, &hcan1};//右上
  
  //底盘电机注册模板：速度单环，速度环 PID
  chassis->chassis_motor = (DJI_Motor_config_s)
  {
    .motorID  = chassis->motor_ID[0],
    .pidMode  = DJI_motor_pidSpeed,
    
    //速度环
    .pid_speed = {
      .kp     = 400.0f,
      .ki     = 5.0f,
      .kd     = 0,
      .max_i  = 1000.0f,   //积分限幅
      .maxout = 8000.0f,   //输出限幅
      .dead   = 0,         //死区
      .pid_mode = pid_mode_normal,
    },
    
    //位置环（底盘不使用）
    .pid_position = {
      .kp     = 0,
      .ki     = 0,
      .kd     = 0,
      .max_i  = 0,         //积分限幅
      .maxout = 0,         //输出限幅
      .dead   = 0,         //死区
      .pid_mode = pid_mode_i_separation,
    },
  };
  
  DJI_motor_init(&chassis->chassis_motor);
  chassis->chassis_motor.motorID = chassis->motor_ID[1];
  DJI_motor_init(&chassis->chassis_motor);
  chassis->chassis_motor.motorID = chassis->motor_ID[2];
  DJI_motor_init(&chassis->chassis_motor);
  chassis->chassis_motor.motorID = chassis->motor_ID[3];
  DJI_motor_init(&chassis->chassis_motor);
  
  
  /****************************yaw轴电机****************************/
//  //达妙 DM-J4340：接收 ID 0x05、反馈 ID 0x01，走 CAN1
//  chassis->yaw_motor = (DM_motor_config_s)
//  {
//    .motor_type   = DM_J4340,
//    .ID           = 0x05,
//    .MST_ID       = 0x01,
//    .control_mode = DM_MIT,
//    .pidMode      = DM_motor_pidPosition, //默认速度环(小陀螺)，跟随模式可在线切换
//    .hcan         = &hcan1,
//    .PMAX         = 3.1415926f,
//    .VMAX         = 10.0f,
//    .TMAX         = 28.0f,
//    
//    //速度环：小陀螺用
//    .pid_speed = {
//      .kp     = 8.0f,
//      .ki     = 0.5f,
//      .kd     = 0,
//      .max_i  = 4.0f,      //积分限幅
//      .maxout = 28.0f,     //输出限幅
//      .dead   = 0,         //死区
//      .pid_mode = pid_mode_normal,
//    },
//    
//    //位置环：底盘反扭/跟随用
//    .pid_position = {
//      .kp     = 2.0f,
//      .ki     = 0,
//      .kd     = 0,
//      .max_i  = 2.0f,      //积分限幅
//      .maxout = 100.0f,    //输出限幅
//      .dead   = 0,         //死区
//      .pid_mode = pid_mode_i_separation,
//    },
//  };
//  
//  DM_motor_init(&chassis->yaw_motor);
}

//底盘使能
chassis_status_e chassis_enable(void)
{
  //给每个电机上电
  for(uint8_t i = 0;i < 4;i++)
    if(DJI_motor_enable(chassis->motor_ID[i]) != DJI_motor_ok)
    {
      chassis->workStatus = 0;
      return chassis_error;
    }
  
//  if(DM_motor_enable(chassis->yaw_motor.ID) != DM_motor_ok)
//    return chassis_error;
  chassis->workStatus = 1;
  return chassis_ok;
}

//底盘失能
chassis_status_e chassis_disable(void)
{
  chassis->forward = 0;
  chassis->turn = 0;
  chassis->rotate = 0;
  chassis->angle = 0;
  memset(chassis->motor_vector,0,16);
  
  //给每个电机上电
  for(uint8_t i = 0;i < 4;i++)
    if(DJI_motor_disable(chassis->motor_ID[i]) != DJI_motor_ok)
    {
      chassis->workStatus = 0;
      return chassis_error;
    }
  
//  if(DM_motor_disable(chassis->yaw_motor.ID) != DM_motor_ok)
//    return chassis_error;
  chassis->workStatus = 0;
  return chassis_ok;
}

//底盘工作
chassis_status_e chassis_work(void)
{
  //得到cmd数据
  cmd_chassisCMD_s cmd = cmd_chassisGetCMD();
  
  //得到大yaw数据
//  DM_motor_rxData_s yaw_data;
//  DM_motor_getData(chassis->yaw_motor.ID, &yaw_data);
  
  
  //传值
  chassis->forward = cmd.forward;
  chassis->turn = cmd.turn;
//  chassis->rotate = yaw_data.speed + cmd.chassis_rotate;
  chassis->rotate = cmd.chassis_rotate;
  chassis->angle = cmd.angle;
  
  //失能
  if(cmd.workFlag == 0)
  {
    chassis_disable();
    return chassis_error;
  }
  
  //行进
  if(cmd.workFlag == 1)
  {
    chassis_enable();
//    DM_Motor_setPidMode(chassis->yaw_motor.ID, DM_motor_pidPosition);
//    DM_Motor_setPidConfig(chassis->yaw_motor.ID, chassis->angle, 0);
  }
  //小陀螺
  else if(cmd.workFlag == 2)
  {
    chassis_enable();
//    DM_Motor_setPidMode(chassis->yaw_motor.ID, DM_motor_pidSpeed);
//    DM_Motor_setPidConfig(chassis->yaw_motor.ID, 0, cmd.top_speed - cmd.chassis_rotate);
//    chassis->angle -= yaw_data.position;
  }
  
  //底盘解算
  if(chassis_soulution() != chassis_ok)
  {
    chassis_disable();
    return chassis_error;
  }
  
  for(uint8_t i = 0;i < 4;i++)
  {      
    if(DJI_motor_setTarget(chassis->motor_ID[i],0,chassis->motor_vector[i]) != DJI_motor_ok)
    {
      chassis->workStatus = 0;
      return chassis_error;
    }
  }
  
  return chassis_ok;
}

uint8_t chassis_getStatus(void)
{
  return chassis->workStatus;
}

chassis_data_s chassis_getData(void)
{
  chassis_data_s data;
  data.chassis_speed[0] = chassis->motor_vector[0];
  data.chassis_speed[1] = chassis->motor_vector[1];
  data.chassis_speed[2] = chassis->motor_vector[2];
  data.chassis_speed[3] = chassis->motor_vector[3];
  return data;
}
