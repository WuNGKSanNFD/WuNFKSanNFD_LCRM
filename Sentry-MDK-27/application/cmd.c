#include "cmd.h"

/***********************私有变量及函数***********************/
typedef struct
{
  //速度与限幅参数
  float forward_speed;        //底盘前进最大速度
  float turn_speed;           //底盘横移最大速度
  float chassis_rotate_speed; //底盘旋转最大速度
  float gimbal_rotate_speed;  //云台最大旋转速度
  float top_rotate_speed;     //小陀螺云台旋转速度
  float pitch_speed;          //pitch 轴旋转速度
  float chassis_top_speed;   //小陀螺旋转最大速度限幅
  float fric_speed;
  float dial_speed;
  
  //角度基准与限幅
  float small_yaw_init;      //小 yaw电机电角度基准
  float pitch_init;          //pitch电机电角度基准
  cmd_ParaRange_s yawAngle;  //小 yaw 角度范围
  cmd_ParaRange_s pitchAngle;//pitch 角度范围
  
  //发射机构阈值
  float friction_range;  //拨轮启动摩擦轮范围
  float fire_range;      //拨轮开火区间
  
  //遥控器死区
  float remote_dead;
  
  //底盘、云台、发射机构控制量
  cmd_chassisCMD_s chassisCMD;
  cmd_gimbalCMD_s gimbalCMD;
  cmd_shootCMD_s shootCMD;
  
  //底盘跟随pid
  pid_instance_s pid_chassisFollow;//底盘跟随PID
  pid_instance_s pid_smallTopFollow;//小陀螺跟随PID
  
  //整车状态
  uint8_t workStatus;
  uint8_t last_workStatus;
}cmd_instance_s;

cmd_instance_s *cmd = NULL;

/**********遥控按键映射到功能**********/
static cmd_mode_e cmd_remote_to_mode(remote_data_s *remote)
{
  cmd_mode_e cmd_mode = CMD_MODE_OFF;
  //左拨杆
  switch(remote->s1)
  {
    //上
    case 1:
        cmd_mode = (cmd_mode_e)(CMD_MODE_SHOOT_ON | CMD_MODE_REMOTE);
      break;
    //下
    case 2:
      //视觉
      cmd_mode = CMD_MODE_VISION;
   
      //此键位应给键鼠,哨兵不使用键鼠，则给视觉
//      cmd_mode = (cmd_mode&0xF000) | CMD_MODE_KEYBOARD;
      break;
    //中
    case 3:
      //遥控器控制
      cmd_mode = CMD_MODE_REMOTE;
      break;
  }
  
  //右拨杆
  switch(remote->s2)
  {
    //上
    case 1:
      //小陀螺
      cmd_mode = (cmd_mode_e)((cmd_mode & 0xFF0F) | CMD_MODE_TOP);
      break;
    //下
    case 2:
      //关闭
      cmd_mode = CMD_MODE_OFF;
      break;
    //中
    case 3:
      //底盘跟随
#ifdef CMD_CHASSIS_FOLLOWING
      cmd_mode = (cmd_mode_e)((cmd_mode & 0xFFF0) | CMD_MODE_CHASSIS_FOLLOW);
#else
      cmd_mode = (cmd_mode_e)((cmd_mode & 0xFFF0) | CMD_MODE_CHASSIS_NO_FOLLOW);
#endif
      cmd_mode = (cmd_mode_e)((cmd_mode & 0xFF0F) | CMD_MODE_GIMBAL_ON);
    
      break;
  }
  
  //拨轮
  if((cmd_mode & 0x0F00) == CMD_MODE_SHOOT_ON)
  {
  //摩擦轮
    if(abs(remote->dial) >= cmd->friction_range && abs(remote->dial) < cmd->fire_range)
      cmd_mode = (cmd_mode_e)((cmd_mode & 0xF0FF) | CMD_MODE_FRICTION_ON);
    //连发
    else if(remote->dial >= cmd->fire_range)
      cmd_mode = (cmd_mode_e)((cmd_mode & 0xF0FF) | CMD_MODE_FIRE_AUTO);
    //单发
    else if(remote->dial <= -cmd->fire_range)
      cmd_mode = (cmd_mode_e)((cmd_mode & 0xF0FF) | CMD_MODE_FIRE_SINGLE);
   }
  
  return cmd_mode;
}

/**********更新底盘数据**********/
static void cmd_chassisUpdata(cmd_chassisCMD_s *raw)
{
  //工作状态更新
  cmd->chassisCMD.workFlag = raw->workFlag;
  
  if(cmd->chassisCMD.workFlag == 0)
  {
    cmd_chassis_disable();
    return;
  }
  
  //移动方向
  cmd->chassisCMD.angle = raw->angle;
  
  //进退限幅
  if(raw->forward >= cmd->forward_speed)
    cmd->chassisCMD.forward = cmd->forward_speed;
  else if(raw->forward <= -cmd->forward_speed)
    cmd->chassisCMD.forward = -cmd->forward_speed;
  else
    cmd->chassisCMD.forward = raw->forward;
  
  //横移限幅
  if(raw->turn >= cmd->turn_speed)
    cmd->chassisCMD.turn = cmd->turn_speed;
  else if(raw->turn <= -cmd->turn_speed)
    cmd->chassisCMD.turn = -cmd->turn_speed;
  else
    cmd->chassisCMD.turn = raw->turn;
  
  //底盘旋转限幅
  if(raw->chassis_rotate >= cmd->chassis_rotate_speed)
    cmd->chassisCMD.chassis_rotate = cmd->chassis_rotate_speed;
  else if(raw->chassis_rotate <= -cmd->chassis_rotate_speed)
    cmd->chassisCMD.chassis_rotate = -cmd->chassis_rotate_speed;
  else
    cmd->chassisCMD.chassis_rotate = raw->chassis_rotate;
  
  //小陀螺旋转限幅
  if(raw->top_speed >= cmd->chassis_top_speed)
    cmd->chassisCMD.top_speed = cmd->chassis_top_speed;
  else if(raw->top_speed <= -cmd->chassis_top_speed)
    cmd->chassisCMD.top_speed = -cmd->chassis_top_speed;
  else
    cmd->chassisCMD.top_speed = raw->top_speed;
  
}


/**********更新云台数据**********/
static void cmd_gimbalUpdata(cmd_gimbalCMD_s *raw)
{ 
  //工作状态更新
  cmd->gimbalCMD.workFlag = raw->workFlag;
  
  if(cmd->gimbalCMD.workFlag == 0)
  {
    cmd_gimbal_disable();
    return;
  }
  
  //yaw轴
  cmd->gimbalCMD.small_yaw = raw->small_yaw;
  
  //pitch轴
  if(raw->pitch >= cmd->pitchAngle.max - cmd->pitch_init)
    cmd->gimbalCMD.pitch = cmd->pitchAngle.max - cmd->pitch_init;
  else if(raw->pitch <= cmd->pitchAngle.min - cmd->pitch_init)
    cmd->gimbalCMD.pitch = cmd->pitchAngle.min - cmd->pitch_init;
  else
    cmd->gimbalCMD.pitch = raw->pitch;
}

/**********更新发射机构数据**********/
static void cmd_shootUpdata(cmd_shootCMD_s *raw)
{
  //工作状态更新
  cmd->shootCMD.workFlag = raw->workFlag;
  
  if(cmd->shootCMD.workFlag == 0)
  {
    cmd_shoot_disable();
    return;
  }
  
  
  //摩擦轮速度
  cmd->shootCMD.fric_speed = raw->fric_speed;
  
  //拨盘速度
  cmd->shootCMD.dial_speed = raw->dial_speed;
  
}

/**********底盘跟随模式**********/
//返回底盘旋转速度
static float cmd_chassisFollowMode(cmd_mode_e *cmd_mode)
{
  DJI_motor_motorID_s small_yaw = {DJI_G6020, 0x01, &hcan2};
  DJI_motor_rxData_s yaw_motor;
  DJI_motor_getData(small_yaw, &yaw_motor);
  float error = cmd->small_yaw_init - yaw_motor.angle;
  
  if(fabsf(error) <= 3.0f)
    return 0.0f;
  
  //相对角按±180°取最短路径,避免编码器圈数差异导致整圈误差
  while(error > 180.0f)
    error -= 360.0f;
  while(error < -180.0f)
    error += 360.0f;

  if((*cmd_mode & 0x00F0) == CMD_MODE_GIMBAL_ON)
  {
    if(pid_calculate(yaw_motor.angle + error, yaw_motor.angle,&cmd->pid_chassisFollow) != pid_ok)
      return 0;
    else
      return cmd->pid_chassisFollow.output;
  }
  else if((*cmd_mode & 0x00F0) == CMD_MODE_TOP)
  {
    if(pid_calculate(yaw_motor.angle + error, yaw_motor.angle,&cmd->pid_smallTopFollow) != pid_ok)
      return 0;
    else
      return cmd->pid_smallTopFollow.output;
  }
  else if((*cmd_mode & 0x00F0) == CMD_MODE_OFF)
  {
    return 0.0f;
  }
  
  return 0.0f;
}

/**********遥控控制**********/
static void cmd_remoteControl(cmd_mode_e *cmd_mode, remote_data_s *remote)
{
  cmd_chassisCMD_s chassisCMD;
  cmd_gimbalCMD_s gimbalCMD;
  cmd_shootCMD_s shootCMD;
  remote_data_s remote_ch;
  
  //处理遥控器死区
  //ch0
  if(remote->ch0 <= cmd->remote_dead && remote->ch0 >= -cmd->remote_dead)
    remote_ch.ch0 = 0;
  else if(remote->ch0 > cmd->remote_dead)
    remote_ch.ch0 = remote->ch0 - cmd->remote_dead;
  else
    remote_ch.ch0 = remote->ch0 + cmd->remote_dead;
  //ch1
  if(remote->ch1 <= cmd->remote_dead && remote->ch1 >= -cmd->remote_dead)
    remote_ch.ch1 = 0;
  else if(remote->ch1 > cmd->remote_dead)
    remote_ch.ch1 = remote->ch1 - cmd->remote_dead;
  else
    remote_ch.ch1 = remote->ch1 + cmd->remote_dead;
  //ch2
  if(remote->ch2 <= cmd->remote_dead && remote->ch2 >= -cmd->remote_dead)
    remote_ch.ch2 = 0;
  else if(remote->ch2 > cmd->remote_dead)
    remote_ch.ch2 = remote->ch2 - cmd->remote_dead;
  else
    remote_ch.ch2 = remote->ch2 + cmd->remote_dead;
  //ch3
  if(remote->ch3 <= cmd->remote_dead && remote->ch3 >= -cmd->remote_dead)
    remote_ch.ch3 = 0;
  else if(remote->ch3 > cmd->remote_dead)
    remote_ch.ch3 = remote->ch3 - cmd->remote_dead;
  else
    remote_ch.ch3 = remote->ch3 + cmd->remote_dead;
  
  
  
  //传输命令
  chassisCMD.workFlag       = 1;
  chassisCMD.angle          = 0;
  chassisCMD.forward        = -cmd->forward_speed * remote_ch.ch3 / (660.0f - cmd->remote_dead);
  chassisCMD.turn           = -cmd->turn_speed * remote_ch.ch2 / (660.0f - cmd->remote_dead);
  chassisCMD.top_speed      = 0;
  chassisCMD.chassis_rotate = 0;
  
  gimbalCMD.workFlag = 0;
  gimbalCMD.small_yaw = cmd->gimbalCMD.small_yaw - cmd->gimbal_rotate_speed * remote_ch.ch0 / (660.0f - cmd->remote_dead); 
  gimbalCMD.pitch = cmd->gimbalCMD.pitch + cmd->pitch_speed * remote_ch.ch1 / (660.0f - cmd->remote_dead);
   
  shootCMD.workFlag = 0;
  shootCMD.dial_speed = 0;
  shootCMD.fric_speed = 0;
  
  //云台命令处理
  if((*cmd_mode & 0x00F0) == CMD_MODE_TOP)
  {
    chassisCMD.workFlag = 2;
    chassisCMD.top_speed = cmd->top_rotate_speed;
  }
  else if((*cmd_mode & 0x00F0) == CMD_MODE_GIMBAL_ON)
  {
    //todo
    gimbalCMD.workFlag = 1;
  }
  else
  {
    gimbalCMD.workFlag = 0;
  }
  
  //底盘命令处理
  if((*cmd_mode & 0x000F) == CMD_MODE_CHASSIS_FOLLOW)
  {
    chassisCMD.chassis_rotate = cmd_chassisFollowMode(cmd_mode);
  }
  else if((*cmd_mode & 0x000F) == CMD_MODE_CHASSIS_NO_FOLLOW)
  {
    chassisCMD.chassis_rotate = cmd->chassis_rotate_speed * remote_ch.ch0 / (660.0f - cmd->remote_dead);;
  }
  else if((*cmd_mode & 0x000F) == CMD_MODE_OFF)
  {
    chassisCMD.workFlag = 0;
  }
  

  //发射机构命令处理
  if((*cmd_mode & 0x0F00) == CMD_MODE_SHOOT_ON)
  {
    shootCMD.workFlag = 1;
    shootCMD.dial_speed = 0;
    shootCMD.fric_speed = 0;
  }
  else if((*cmd_mode & 0x0F00) == CMD_MODE_FRICTION_ON)
  {
    shootCMD.workFlag = 2;
    shootCMD.fric_speed = cmd->fric_speed; 
  }
  else if((*cmd_mode & 0x0F00) == CMD_MODE_FIRE_AUTO)
  {
    shootCMD.workFlag = 3;
    shootCMD.dial_speed = cmd->dial_speed;
    shootCMD.fric_speed = cmd->fric_speed;  
  }
  else if((*cmd_mode & 0x0F00) == CMD_MODE_FIRE_SINGLE)
  {
    shootCMD.workFlag = 4;
    shootCMD.dial_speed = cmd->dial_speed;
    shootCMD.fric_speed = cmd->fric_speed; 
  }
  
  
  //更新命令
  cmd_chassisUpdata(&chassisCMD);
  cmd_gimbalUpdata(&gimbalCMD);
  cmd_shootUpdata(&shootCMD);
}

/**********视觉控制**********/
static void cmd_visionControl(cmd_mode_e *cmd_mode)
{
  cmd_chassisCMD_s chassisCMD;
  cmd_gimbalCMD_s gimbalCMD;
  cmd_shootCMD_s shootCMD;
  cmd_mode_e cmd_mode_in;
  
  //读取数据
  vision_rx_s vision_rxData = vision_getData();
  
  if(vision_rxData.state == 1)
  {
    chassisCMD.workFlag       = 1;
    chassisCMD.angle          = 0;
    chassisCMD.chassis_rotate = 0;
    chassisCMD.forward        = vision_rxData.forward;
    chassisCMD.top_speed      = 0;
    chassisCMD.turn           = vision_rxData.turn;
    
    gimbalCMD.workFlag = 1;
    gimbalCMD.small_yaw = vision_rxData.yaw;
    gimbalCMD.pitch     = vision_rxData.pitch;
    
    shootCMD.workFlag = 0; 
    shootCMD.fric_speed = 0;
    shootCMD.dial_speed = 0;
    
    //云台指令(bit4-7)
    if((vision_rxData.cmd_id & 0x00F0) == vision_gimbal_enable)
    {
      cmd_mode_in = CMD_MODE_GIMBAL_ON;
    }
    else if((vision_rxData.cmd_id & 0x00F0) == vision_gimbal_small_top)
    { 
      chassisCMD.workFlag = 2;
      cmd_mode_in = CMD_MODE_TOP;
    }
    else
    {
      gimbalCMD.workFlag = 0;
      cmd_mode_in = CMD_MODE_OFF;
    }
    
    //底盘指令(bit0-3)
    if((vision_rxData.cmd_id & 0x000F) == vision_chassis_following)
    {
      //底盘跟随
      chassisCMD.chassis_rotate = cmd_chassisFollowMode(&cmd_mode_in);
    }
    else if((vision_rxData.cmd_id & 0x000F) == vision_chassis_no_following)
    {
      //底盘不跟随
      chassisCMD.chassis_rotate = vision_rxData.rotate;
    }
    else
    {
      //未下发底盘指令(或为0)时不驱动底盘平移/旋转
      chassisCMD.workFlag = 0;
    }
    
    //发射机构(视觉侧占bit8-15, cmd_mode_e 占bit8-11)
    if((vision_rxData.cmd_id & vision_shoot_fire) == vision_shoot_fire && (*cmd_mode & 0x0F00) == CMD_MODE_FIRE_AUTO)
    {
      shootCMD.workFlag = 3;
      shootCMD.dial_speed = cmd->dial_speed;
      shootCMD.fric_speed = cmd->fric_speed;
      
    }
    else if((vision_rxData.cmd_id & vision_shoot_friction_on) == vision_shoot_friction_on)
    {
      //只开摩擦轮
      shootCMD.workFlag = 2;
      shootCMD.fric_speed = cmd->fric_speed;
    }
    else if((vision_rxData.cmd_id & 0x0F00) == vision_cmd_off)
    {
      shootCMD.workFlag = 0;
    }
    
  }
  else
  {
    chassisCMD.workFlag = 0;
    gimbalCMD.workFlag = 0;
    shootCMD.workFlag = 0;    
  }

  //更新命令
  cmd_chassisUpdata(&chassisCMD);
  cmd_gimbalUpdata(&gimbalCMD);
  cmd_shootUpdata(&shootCMD);
}

/**********键鼠控制**********/
static void cmd_keyboardControl(cmd_mode_e *cmd_mode, remote_data_s *remote)
{
  //todo
}

/***********************************外部调用************************************/
void cmd_init(void)
{
  cmd = (cmd_instance_s *)malloc(sizeof(cmd_instance_s));
  memset(cmd,0,sizeof(cmd_instance_s));
  
  cmd->forward_speed        = 1;
  cmd->turn_speed           = 1;
  cmd->chassis_rotate_speed = 5;
  cmd->gimbal_rotate_speed  = 1;
  cmd->top_rotate_speed     = 0.5;
  cmd->pitch_speed          = 1;
  cmd->chassis_top_speed    = 5;
  cmd->fric_speed           = 80;
  cmd->dial_speed           = 30;
  
  cmd->small_yaw_init       = 0;
  cmd->pitch_init           = 0;
  cmd->yawAngle.max         = 120 + 90;
  cmd->yawAngle.min         = 120 - 90;
  cmd->pitchAngle.max       = 60;
  cmd->pitchAngle.min       = -60;
  
  cmd->friction_range       = 100;
  cmd->fire_range           = 600;
  
  cmd->remote_dead          = 50;
  
  
  pid_instance_s chassis_follow_pid_default =
  {
    .kp = 0.1f,//比例:1°偏差约0.03rad/s
    .ki = 0.00000f,//积分:2ms周期下不宜过大
    .kd = 0.0f,//微分:编码器角度噪声较大,暂不使用
    .max_i = 1.0f,//积分限幅,单位rad/s
    .maxout = 10.0f,//输出限幅,底盘最大自转5rad/s
    .dead = 0.0,//死区,单位deg
    .pid_mode = pid_mode_normal,//普通位置式PID
  };  
  
  pid_instance_s small_top_follow_pid_default =
  {
    .kp = 0.1f,//比例:1°偏差约0.002rad/s(0.11°/s)
    .ki = 0.000002f,//积分:2ms周期下不宜过大
    .kd = 0,//微分暂不使用
    .max_i = 0.0087266f,//积分限幅0.5°/s
    .maxout = 1.0f,//输出限幅1°/s
    .dead = 0,//死区,单位deg
    .pid_mode = pid_mode_normal,//普通位置式PID
  };
  
  cmd->pid_chassisFollow = chassis_follow_pid_default;
  cmd->pid_smallTopFollow = small_top_follow_pid_default;
  
  cmd_all_disable();
}


//底盘失能
void cmd_chassis_disable(void)
{
  memset(&cmd->chassisCMD,0,sizeof(cmd_chassisCMD_s));
}

//云台失能
void cmd_gimbal_disable(void)
{
  memset(&cmd->gimbalCMD,0,sizeof(cmd_gimbalCMD_s));
}

//发射机构失能
void cmd_shoot_disable(void)
{
  memset(&cmd->shootCMD,0,sizeof(cmd_shootCMD_s));
}

//全车失能
void cmd_all_disable(void)
{
  cmd_chassis_disable();
  cmd_gimbal_disable();
  cmd_shoot_disable();
  cmd->workStatus = 0;
}

cmd_mode_e cmd_mode;
void cmd_work(void)
{
  //IMU上电校准
  if(IMU_isCalibrated() == 0)
  {
    cmd_all_disable();
    return;
  }
  
  //校准完成：读yaw电机编码器，把“编码器角-机械中位角”作为修正量交给IMU； 
  static uint8_t calib_applied = 0;  
  if(calib_applied == 0)  
  {  
    DJI_motor_rxData_s yaw_data;  
    DJI_motor_motorID_s smallYaw = {DJI_G6020, 0x01, &hcan2};
    //读不到编码器时修正量为0（云台停在校准位置），检查电机ID与接线  
    if(DJI_motor_getData(smallYaw, &yaw_data) == DJI_motor_ok)  
      IMU_setYawOffset(yaw_data.angle - cmd->small_yaw_init);  
    calib_applied = 1;  
  }  
  
  //向上位机发送数据
  uint16_t chassisStatus = chassis_getStatus();
  uint16_t gimbalStatus = gimbal_getStatus();
  uint16_t shootStatus = shoot_getStatus();
  IMU_data_s IMU_data = IMU_getData();
  vision_tx_s vision_tx = {
    .cmd_id = chassisStatus | (gimbalStatus << 4) | (shootStatus << 8),
    .yaw = IMU_data.filter_yaw,
    .pitch = IMU_data.filter_pitch,
    .free_data = 0,
  };
  vision_sendData(vision_tx);
  
  //获取遥控器数据(须先取数据,再做按键/拨杆到模式的映射)
  remote_data_s remote;
  if(remote_getData(&remote) != remote_ok)
  {
    cmd_all_disable();
    return;
  }
  
  //得到对应功能
  cmd_mode = cmd_remote_to_mode(&remote);
  
  //遥控器是否开启判断：只取优先级位域，s2 拨下(OFF)时整机失能
  uint16_t priority = (uint16_t)cmd_mode & 0xF000;
  if(priority == CMD_MODE_OFF)
  {
    cmd_all_disable();
    return;
  }
  else
    cmd->workStatus = 1;
  
  if(cmd->last_workStatus == 0)
  {
    cmd->gimbalCMD.pitch = 0;
    cmd->gimbalCMD.small_yaw = 0;
  }
  
  //模式判断
  switch(priority)
  {
    case CMD_MODE_REMOTE:
      cmd_remoteControl(&cmd_mode,&remote);
      break;
    case CMD_MODE_VISION:
      cmd_visionControl(&cmd_mode);
      break;
    case CMD_MODE_KEYBOARD:
      cmd_keyboardControl(&cmd_mode,&remote);
      break;
  }
  
  cmd->last_workStatus = cmd->workStatus;
}


//读取底盘命令
cmd_chassisCMD_s cmd_chassisGetCMD(void)
{
  if(cmd == NULL)
  {
    cmd_chassisCMD_s none;
    memset(&none,0,sizeof(cmd_chassisCMD_s));
    return none;
  }
  
  return cmd->chassisCMD;
}

//读取云台命令
cmd_gimbalCMD_s cmd_gimbalGetCMD(void)
{
  if(cmd == NULL)
  {
    cmd_gimbalCMD_s none;
    memset(&none,0,sizeof(cmd_gimbalCMD_s));
    return none;
  }
  
  return cmd->gimbalCMD;
}

//读取发射机构命令
cmd_shootCMD_s cmd_shootGetCMD(void)
{
  if(cmd == NULL)
  {
    cmd_shootCMD_s none;
    memset(&none,0,sizeof(cmd_shootCMD_s));
    return none;
  }
  
  return cmd->shootCMD;
}
