#include "DJI_motor.h"

/*************************************私有函数及变量****************************************/
//2 个 CAN 口 × 3 种控制帧
#define DJI_MOTOR_GROUP_CNT  6

//群组是否启用标志位,0~2=hcan1的0x200/0x1FF/0x2FF,3~5=hcan2的0x200/0x1FF/0x2FF
static uint8_t groupFlag[DJI_MOTOR_GROUP_CNT] = {0};

typedef struct
{
  float speed;
  float position;
}DJI_motor_refData_s;

typedef struct
{
  DJI_motor_motorID_s motorID;
  
  uint16_t rx_id;
  uint8_t group;
  uint8_t tx_offset;
  
  DJI_motor_rxData_s rx_data;
  DJI_motor_workstatus_e workstatus;
  
  DJI_motor_pidMode_e pidMode;
  pid_instance_s pid_speed;
  pid_instance_s pid_position;
  
  DJI_motor_refData_s target;
  DJI_motor_refData_s measure;
  
  volatile uint8_t feedBackChangeFlag;
  
}DJI_motor_instance_s;

//电机实例指针及注册索引
static uint8_t idx = 0;
DJI_motor_instance_s *motor_instance[DJI_MOTOR_CNT] = {NULL};

//发送组
can_txInstance_s motor_sendgroup[DJI_MOTOR_GROUP_CNT] = 
{
  [0] = {.hcan = &hcan1, .tx_id = 0x200, .len = 8, .tx_data = {0,}},
  [1] = {.hcan = &hcan1, .tx_id = 0x1FF, .len = 8, .tx_data = {0,}},
  [2] = {.hcan = &hcan1, .tx_id = 0x2FF, .len = 8, .tx_data = {0,}},
  
  [3] = {.hcan = &hcan2, .tx_id = 0x200, .len = 8, .tx_data = {0,}},
  [4] = {.hcan = &hcan2, .tx_id = 0x1FF, .len = 8, .tx_data = {0,}},
  [5] = {.hcan = &hcan2, .tx_id = 0x2FF, .len = 8, .tx_data = {0,}}
};

//根据can口和控制帧ID获取发送组索引,无效时返回0xFF
static uint8_t DJI_motor_getGroup(CAN_HandleTypeDef* hcan,uint16_t tx_id)
{
  uint8_t base = 0;
  
  if(hcan == &hcan2)
    base = 3;
  
  if(tx_id == 0x200)
    return base;
  else if(tx_id == 0x1FF)
    return base + 1;
  else if(tx_id == 0x2FF)
    return base + 2;
  else
    return 0xFF;
}

//根据型号和电调ID计算反馈帧ID
static uint16_t DJI_motor_getRxID(DJI_motor_type_e motor_type,uint8_t motor_ID)
{
  if(motor_type == DJI_G6020)
    return 0x204 + motor_ID;
  else
    return 0x200 + motor_ID;
}

//根据型号和电调ID计算控制帧ID,每帧最多承载4个电机
static uint16_t DJI_motor_getTxID(DJI_motor_type_e motor_type,uint8_t motor_ID)
{
  if(motor_type == DJI_G6020)
  {
    if(motor_ID <= 4)
      return 0x1FF;
    else
      return 0x2FF;
  }
  else
  {
    if(motor_ID <= 4)
      return 0x200;
    else
      return 0x1FF;
  }
}

//填充发送缓存
static void DJI_motor_stuffData(const DJI_motor_instance_s *motor,const float output)
{
  int16_t output_temp = (int16_t)output;
  
  //将输出填充到发送数据包,每个电机占2字节,高字节在前
  motor_sendgroup[motor->group].tx_data[motor->tx_offset]     = output_temp >> 8;
  motor_sendgroup[motor->group].tx_data[motor->tx_offset + 1] = output_temp;
}

//按身份查找实例
static DJI_motor_status_e DJI_motor_pairInstance(DJI_motor_motorID_s motorID,uint8_t *index_id)
{
  for(uint8_t i = 0;i < idx;i++)
  {
    if(motor_instance[i]->motorID.hcan == motorID.hcan &&
       motor_instance[i]->motorID.motor_type == motorID.motor_type &&
       motor_instance[i]->motorID.ID == motorID.ID)
    {
      *index_id = i;
      return DJI_motor_ok;
    }
  }
  return DJI_motor_error;
}

//CAN 接收回调
static void DJI_motor_recevieData(can_Rxdata_s *rxdata)
{
  uint8_t index_id = 0;
  uint8_t findFlag = 0;
  
  //按 CAN 口 + 反馈 ID 配对实例
  for(uint8_t i = 0;i < idx;i++)
  {
    if(motor_instance[i]->motorID.hcan == rxdata->hcan && motor_instance[i]->rx_id == rxdata->rx_id)
    {
      index_id = i;
      findFlag = 1;
      break;
    }
  }
  
  if(findFlag == 0)
    return;
  
  DJI_motor_instance_s *motor = motor_instance[index_id];
  
  //M2006
  if(motor->motorID.motor_type == DJI_M2006)
  {
    motor->rx_data.rotor_angle    =  ((rxdata->rx_data[0]<<8) | (rxdata->rx_data[1])) * 360.0f / 8191.0f;
    motor->rx_data.rotor_speed    =  (int16_t)(rxdata->rx_data[2]<<8) | (rxdata->rx_data[3]);
    
    int16_t rotor_angle_error = motor->rx_data.rotor_angle - motor->rx_data.last_rotor_angle;
    if(rotor_angle_error > 180.0f)
      motor->rx_data.count --;
    else if(rotor_angle_error < -180.0f)
      motor->rx_data.count ++;
    
    motor->rx_data.last_rotor_angle = motor->rx_data.rotor_angle;
    
    motor->rx_data.angle = motor->rx_data.count * 10 + motor->rx_data.rotor_angle / M2006_REDUCTION_RATIO;
    motor->rx_data.speed = motor->rx_data.rotor_speed / M2006_REDUCTION_RATIO;
    
    return;
  }
  
  //M3508
  else if(motor->motorID.motor_type == DJI_M3508)
  {
    motor->rx_data.rotor_angle    =  ((rxdata->rx_data[0]<<8) | (rxdata->rx_data[1])) * 360.0f / 8191.0f;
    motor->rx_data.rotor_speed    =  (int16_t)(rxdata->rx_data[2]<<8) | (rxdata->rx_data[3]);
         
    int16_t rotor_angle_error = motor->rx_data.rotor_angle - motor->rx_data.last_rotor_angle;
    if(rotor_angle_error > 180.0f)
      motor->rx_data.count --;
    else if(rotor_angle_error < -180.0f)
      motor->rx_data.count ++;
    
    motor->rx_data.last_rotor_angle = motor->rx_data.rotor_angle;
    
    motor->rx_data.angle = (motor->rx_data.count * 360 + motor->rx_data.rotor_angle) / M2508_REDUCTION_RATIO;
    motor->rx_data.speed = motor->rx_data.rotor_speed / M2508_REDUCTION_RATIO;
    
    return;
  }
  
  //GM6020
  else if(motor->motorID.motor_type == DJI_G6020)
  {
    motor->rx_data.rotor_angle    =  ((rxdata->rx_data[0]<<8) | (rxdata->rx_data[1])) * 360.0f / 8191.0f;
    motor->rx_data.rotor_speed    =  (int16_t)(rxdata->rx_data[2]<<8) | (rxdata->rx_data[3]);
         
    int16_t rotor_angle_error = motor->rx_data.rotor_angle - motor->rx_data.last_rotor_angle;
    if(rotor_angle_error > 180.0f)
      motor->rx_data.count --;
    else if(rotor_angle_error < -180.0f)
      motor->rx_data.count ++;
    
    motor->rx_data.last_rotor_angle = motor->rx_data.rotor_angle;
    
    motor->rx_data.temperature    =  rxdata->rx_data[6];
    motor->rx_data.angle = (motor->rx_data.count * 360 + motor->rx_data.rotor_angle);
    motor->rx_data.speed = motor->rx_data.rotor_speed;
  }
  else
    return;
  
  
  return;
}

//发送所有启用组
static DJI_motor_status_e DJI_motor_sendData(void)
{ 
  uint8_t sendFlag = 0;
  
  for(uint8_t i = 0;i < DJI_MOTOR_GROUP_CNT;i++)
  {
    if(groupFlag[i] == 1)
    {
      if(can_SendData(&(motor_sendgroup[i])) != can_ok)
        return DJI_motor_error;
      else
        sendFlag = 1;
    }
  }
  
  if(sendFlag == 1)
    return DJI_motor_ok;
  else
    return DJI_motor_false;
}

/*****************************************外部调用函数*******************************************/

//初始化注册电机
DJI_motor_status_e DJI_motor_init(DJI_Motor_config_s *config)
{  
  if(config == NULL)
    return DJI_motor_error;
  
  DJI_motor_motorID_s motorID = config->motorID;
  
  //ID 1~8，CAN 口仅支持 hcan1/hcan2
  if(motorID.hcan == NULL || motorID.ID < 1 || motorID.ID > 8)
    return DJI_motor_error;
  
  if(motorID.hcan != &hcan1 && motorID.hcan != &hcan2)
    return DJI_motor_error;
  
  if(motorID.motor_type != DJI_M2006 && motorID.motor_type != DJI_M3508 && motorID.motor_type != DJI_G6020)
    return DJI_motor_error;
  
  if(idx >= DJI_MOTOR_CNT)
    return DJI_motor_error;
  
  //同口反馈 ID 不可冲突
  uint16_t rx_id = DJI_motor_getRxID(motorID.motor_type,motorID.ID);
  for(uint8_t i = 0;i < idx;i++)
  {
    if(motor_instance[i]->motorID.hcan == motorID.hcan && motor_instance[i]->rx_id == rx_id)
      return DJI_motor_error;
  }
  
  uint8_t group = DJI_motor_getGroup(motorID.hcan,DJI_motor_getTxID(motorID.motor_type,motorID.ID));
  if(group == 0xFF)
    return DJI_motor_error;
  
  DJI_motor_instance_s *motor = (DJI_motor_instance_s *)malloc(sizeof(DJI_motor_instance_s));
  if(motor == NULL)
    return DJI_motor_error;
  memset(motor,0,sizeof(DJI_motor_instance_s));
  motor_instance[idx] = motor;
  
  motor->motorID = motorID;
  motor->rx_id = rx_id;
  motor->group = group;
  motor->tx_offset = (uint8_t)(((motorID.ID - 1) % 4) * 2);
  
  motor->pidMode = config->pidMode;
  motor->pid_speed = config->pid_speed;
  motor->pid_position = config->pid_position;
  
  //注册 CAN 接收
  can_rxConfig_s can_rxConfig;
  memset(&can_rxConfig,0,sizeof(can_rxConfig));
  can_rxConfig.callback = DJI_motor_recevieData; 
  can_rxConfig.hcan = motor->motorID.hcan;
  can_rxConfig.rx_id = motor->rx_id;
  
  idx++;
  
  //启动 CAN 服务（每个口一次）
  static uint8_t canStartFlag[2] = {0,0};
  if(motor->motorID.hcan == &hcan1 && canStartFlag[0] == 0)
  {
    can_Serviceinit(&hcan1);
    canStartFlag[0] = 1;
  }
  else if(motor->motorID.hcan == &hcan2 && canStartFlag[1] == 0)
  {
    can_Serviceinit(&hcan2);
    canStartFlag[1] = 1;
  }
  
  can_rxConfigInit(&can_rxConfig);
  groupFlag[group] = 1;
  
  //初始失能
  if(DJI_motor_disable(motor->motorID) != DJI_motor_ok)
    return DJI_motor_error;
  
  return DJI_motor_ok;
}

//使能电机
DJI_motor_status_e DJI_motor_enable(DJI_motor_motorID_s motorID)
{
  uint8_t index_id = 0;
  if(DJI_motor_pairInstance(motorID,&index_id) != DJI_motor_ok)
    return DJI_motor_error;
  
  
  motor_instance[index_id]->workstatus = motor_enable;
  return DJI_motor_ok;
}

//失能电机
DJI_motor_status_e DJI_motor_disable(DJI_motor_motorID_s motorID)
{
  
  uint8_t index_id = 0;
  if(DJI_motor_pairInstance(motorID,&index_id) != DJI_motor_ok)
    return DJI_motor_error;
  
  DJI_motor_stuffData(motor_instance[index_id],0);
  motor_instance[index_id]->workstatus = motor_disable;
  return DJI_motor_ok;
}

//设置目标参数
DJI_motor_status_e DJI_motor_setTarget(DJI_motor_motorID_s motorID,float targetPosition,float targetSpeed)
{
  uint8_t index_id = 0;
  if(DJI_motor_pairInstance(motorID,&index_id) != DJI_motor_ok)
    return DJI_motor_error;
  
  motor_instance[index_id]->target.speed    = targetSpeed;
  motor_instance[index_id]->target.position = targetPosition;
  
  return DJI_motor_ok;
};

//输入反馈
DJI_motor_status_e DJI_motor_setFeedbcak(DJI_motor_motorID_s motorID,float Position,float Speed)
{
  uint8_t index_id = 0;
  if(DJI_motor_pairInstance(motorID,&index_id) != DJI_motor_ok)
    return DJI_motor_error;
  
  motor_instance[index_id]->measure.speed    = Speed;
  motor_instance[index_id]->measure.position = Position;
  
  motor_instance[index_id]->feedBackChangeFlag = 1;
  
  return DJI_motor_ok;
};

//设置pid模式
DJI_motor_status_e DJI_motor_setPidMode(DJI_motor_motorID_s motorID,DJI_motor_pidMode_e pidMode)
{
  uint8_t index_id = 0;
  if(DJI_motor_pairInstance(motorID,&index_id) != DJI_motor_ok)
    return DJI_motor_error;
  
  if(pidMode != DJI_motor_pidSpeed && pidMode != DJI_motor_pidPosition)
    return DJI_motor_error;
  
  motor_instance[index_id]->pidMode = pidMode;
  
  return DJI_motor_ok;
}

//设置pid参数
DJI_motor_status_e DJI_motor_setPidConfig(DJI_motor_motorID_s motorID,float kpPosition,float kdPosition,float kpSpeed,float kiSpeed)
{
  uint8_t index_id = 0;
  if(DJI_motor_pairInstance(motorID,&index_id) != DJI_motor_ok)
    return DJI_motor_error;
  
  motor_instance[index_id]->pid_position.kp = kpPosition;
  motor_instance[index_id]->pid_position.kd = kdPosition;
  motor_instance[index_id]->pid_speed.kp    = kpSpeed;
  motor_instance[index_id]->pid_speed.ki    = kiSpeed;
  
  return DJI_motor_ok;
};

//闭环计算并发送
DJI_motor_status_e DJI_Motor_work(void)
{
  for(uint8_t i = 0; i< idx ; i++)
  {
    if(motor_instance[i]->workstatus == motor_disable)
      continue;
    
    
    //未使用外部反馈时取 CAN 反馈
    if(motor_instance[i]->feedBackChangeFlag == 0)
    {
      motor_instance[i]->measure.speed    = motor_instance[i]->rx_data.speed;
      motor_instance[i]->measure.position = motor_instance[i]->rx_data.angle;
    }
    
    if(motor_instance[i]->pidMode == DJI_motor_pidPosition)
    {
      if(pid_calculate(motor_instance[i]->target.position, motor_instance[i]->measure.position, &motor_instance[i]->pid_position) == pid_ok)
      {
        motor_instance[i]->target.speed = motor_instance[i]->pid_position.output; 
      }
      else
        return DJI_motor_error; 
    }
    
    if(pid_calculate(motor_instance[i]->target.speed, motor_instance[i]->measure.speed, &motor_instance[i]->pid_speed) == pid_ok)
    {
      DJI_motor_stuffData(motor_instance[i],motor_instance[i]->pid_speed.output);
    }
    else
      return DJI_motor_error;  
    
    
  }
  
  if(DJI_motor_sendData() != DJI_motor_ok)
    return DJI_motor_error; 
  
  return DJI_motor_ok;
}


//读取反馈快照
DJI_motor_status_e DJI_motor_getData(DJI_motor_motorID_s motorID,DJI_motor_rxData_s *data)
{
  uint8_t index_id = 0;
  if(DJI_motor_pairInstance(motorID,&index_id) != DJI_motor_ok)
    return DJI_motor_error;
  
  
  motor_instance[index_id]->rx_data.pidSpeed_output = motor_instance[index_id]->pid_speed.output;
  motor_instance[index_id]->rx_data.pidAngle_output = motor_instance[index_id]->pid_position.output;
  
  *data = motor_instance[index_id]->rx_data;
  
  return DJI_motor_ok;
}
