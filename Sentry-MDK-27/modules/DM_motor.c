/**
  * @file    DM_motor.c
  * @brief   达妙 DM 电机 CAN 控制
  */

#include "DM_motor.h"


/***********************************私有变量及函数**************************************/
typedef struct
{
  float speed;
  float position;
}DM_motor_refData_s;

typedef struct
{
  DM_motor_type_e motor_type; //电机型号
  uint8_t ID;                 //电机发送ID
  uint8_t MST_ID;             //电机反馈ID
  DM_motor_workstatus_e workstatus;   //电机工作状态
  DM_motor_controlMode_e control_mode;//电机控制模式
  DM_motor_rxData_s rx_data;          //反馈数据
  DM_motor_registerData_s reg_data;   //寄存器数据
  
  CAN_HandleTypeDef* hcan;  //can口
  can_txInstance_s tx_data; //发送数据
  uint8_t callbackFlag;     //发送是否接收标志位
  
  DM_motor_MITconfig_s MITconfig; //mit模式参数
  DM_motor_PVTconfig_s PVTconfig; //pvt模式参数
  
  //pid参数
  DM_motor_pidMode_e pidMode; //闭环模式
  float targetSpeed;
  pid_instance_s pid_speed;//速度环
  float targetPosition;
  pid_instance_s pid_position;//位置环(角度环)
  
  DM_motor_refData_s measure;           //测量值：CAN 反馈或外部反馈
  volatile uint8_t feedBackChangeFlag;  //0=用 CAN 反馈，1=用外部反馈
  
}DM_motor_instance_s;

//实例
DM_motor_instance_s *DM_instance[4] = {NULL}; 
static uint8_t idx = 0;

//写入寄存器缓存
static void DM_motor_writeAddress(DM_motor_registerData_s *regs, DM_motor_registerAddress_e addr, uint32_t raw) 
{
    float f_val;
    
    memcpy(&f_val, &raw, sizeof(float));

    switch (addr) {
        case REG_UV_Value:   regs->UV_Value   = f_val; break;
        case REG_KT_Value:   regs->KT_Value   = f_val; break;
        case REG_OT_Value:   regs->OT_Value   = f_val; break;
        case REG_OC_Value:   regs->OC_Value   = f_val; break;
        case REG_ACC:        regs->ACC        = f_val; break;
        case REG_DEC:        regs->DEC        = f_val; break;
        case REG_MAX_SPD:    regs->MAX_SPD    = f_val; break;
        case REG_MST_ID:     regs->MST_ID     = raw;   break;  // uint32_t
        case REG_ESC_ID:     regs->ESC_ID     = raw;   break;
        case REG_TIMEOUT:    regs->TIMEOUT    = raw;   break;
        case REG_CTRL_MODE:  regs->CTRL_MODE  = raw;   break;
        case REG_Damp:       regs->Damp       = f_val; break;
        case REG_Inertia:    regs->Inertia    = f_val; break;
        case REG_hw_ver:     regs->hw_ver     = raw;   break;
        case REG_sw_ver:     regs->sw_ver     = raw;   break;
        case REG_SN:         regs->SN         = raw;   break;
        case REG_NPP:        regs->NPP        = raw;   break;
        case REG_Rs:         regs->Rs         = f_val; break;
        case REG_Ls:         regs->Ls         = f_val; break;
        case REG_Flux:       regs->Flux       = f_val; break;
        case REG_Gr:         regs->Gr         = f_val; break;
        case REG_PMAX:       regs->PMAX       = f_val; break;
        case REG_VMAX:       regs->VMAX       = f_val; break;
        case REG_TMAX:       regs->TMAX       = f_val; break;
        case REG_I_BW:       regs->I_BW       = f_val; break;
        case REG_KP_ASR:     regs->KP_ASR     = f_val; break;
        case REG_KI_ASR:     regs->KI_ASR     = f_val; break;
        case REG_KP_APR:     regs->KP_APR     = f_val; break;
        case REG_KI_APR:     regs->KI_APR     = f_val; break;
        case REG_OV_Value:   regs->OV_Value   = f_val; break;
        case REG_GREF:       regs->GREF       = f_val; break;
        case REG_Beta:       regs->Beta       = f_val; break;
        case REG_V_BW:       regs->V_BW       = f_val; break;
        case REG_IQ_c1:      regs->IQ_c1      = f_val; break;
        case REG_VL_c1:      regs->VL_c1      = f_val; break;
        case REG_can_br:     regs->can_br     = raw;   break;
        case REG_sub_ver:    regs->sub_ver    = raw;   break;
        case REG_Boot_ver:   regs->Boot_ver   = raw;   break;
        case REG_dir:        regs->dir        = f_val; break;
        case REG_m_off:      regs->m_off      = f_val; break;
        case REG_Imax:       regs->Imax       = f_val; break;
        case REG_VBus:       regs->VBus       = f_val; break;
        case REG_Tpcb:       regs->Tpcb       = f_val; break;
        case REG_Tmtr:       regs->Tmtr       = f_val; break;
        case REG_Iu_off:     regs->Iu_off     = f_val; break;
        case REG_Iv_off:     regs->Iv_off     = f_val; break;
        case REG_Iw_off:     regs->Iw_off     = f_val; break;
        case REG_p_m:        regs->p_m        = f_val; break;
        case REG_xout:       regs->xout       = f_val; break;
        default:
            break;
    }
}




//按 ID 查找实例
static DM_motor_status_e DM_motor_pairInstance(uint8_t motor_ID,uint8_t *index_id)
{
  for(uint8_t i = 0;i < idx;i++)
  {
    if(DM_instance[i]->ID == motor_ID)
    {
      *index_id = i;
      return DM_motor_ok;
    }
    
    if(i == idx)
      return DM_motor_error;
  }
  return DM_motor_error;
}


static void DM_motor_canCallback(can_Rxdata_s * can_rxdata)
{
  for(uint8_t i = 0; i < idx; i++)
  {
    if(can_rxdata->hcan == DM_instance[i]->hcan && can_rxdata->rx_id == DM_instance[i]->MST_ID && can_rxdata->len == 8)
    {
      //读参数应答
      if(can_rxdata->rx_data[0] ==  DM_instance[i]->ID && can_rxdata->rx_data[1] ==  (DM_instance[i]->ID >> 8) && can_rxdata->rx_data[2] == 0x33)
      {
        uint32_t regData = can_rxdata->rx_data[4]     
                         +(can_rxdata->rx_data[5]<<8) 
                         +(can_rxdata->rx_data[6]<<16)
                         +(can_rxdata->rx_data[7]<<24);
        uint8_t regAress = can_rxdata->rx_data[3];   
        
        DM_motor_writeAddress(&DM_instance[i]->reg_data,(DM_motor_registerAddress_e)regAress,regData);
        DM_instance[i]->callbackFlag = 0;
        
      }
      //写参数应答
      else if(can_rxdata->rx_data[0] ==  DM_instance[i]->ID && can_rxdata->rx_data[1] ==  (DM_instance[i]->ID >> 8) && can_rxdata->rx_data[2] == 0x55)
      {
        uint32_t regData = can_rxdata->rx_data[4]     
                         +(can_rxdata->rx_data[5]<<8) 
                         +(can_rxdata->rx_data[6]<<16)
                         +(can_rxdata->rx_data[7]<<24);
        uint8_t regAress = can_rxdata->rx_data[3];   
        
        DM_motor_writeAddress(&DM_instance[i]->reg_data,(DM_motor_registerAddress_e)regAress,regData);
        DM_instance[i]->callbackFlag = 0;
      }
      //控制帧
      else
      {   
        uint16_t position = ((can_rxdata->rx_data[1] << 8) | can_rxdata->rx_data[2]) & 0xFFFF;
        uint16_t speed    = ((can_rxdata->rx_data[3] << 4) | (can_rxdata->rx_data[4] >> 4)) & 0x7FFF;
        uint16_t torque   = ((can_rxdata->rx_data[4] << 8) | (can_rxdata->rx_data[5])) & 0x7FFF;
        
        DM_instance[i]->rx_data.err     = can_rxdata->rx_data[0] >> 4;
        DM_instance[i]->rx_data.position = (position / 65535.0f) * 2.0f * DM_instance[i]->reg_data.PMAX - DM_instance[i]->reg_data.PMAX;
        DM_instance[i]->rx_data.speed   = (speed    / 4095.0f) * 2.0f * DM_instance[i]->reg_data.VMAX - DM_instance[i]->reg_data.VMAX;
        DM_instance[i]->rx_data.torque  = (torque / 4095.0f) * 2.0f * DM_instance[i]->reg_data.TMAX - DM_instance[i]->reg_data.TMAX;
        DM_instance[i]->rx_data.T_MOS   = can_rxdata->rx_data[6];
        DM_instance[i]->rx_data.T_Rotor = can_rxdata->rx_data[7];
        
        DM_instance[i]->callbackFlag = 0;
      }
    }
  }
}


//按 MIT 协议校验参数、打包并发送单台电机
static DM_motor_status_e DM_motor_sendMIT(DM_motor_instance_s *motor)
{
  //检测发送指令是否合法
  if(motor->MITconfig.torque != 0 && (motor->MITconfig.position != 0 
                                      || motor->MITconfig.speed != 0 
                                      || motor->MITconfig.kp != 0 
                                      || motor->MITconfig.kd != 0))
  {
    return DM_motor_error;
  }
  else if(motor->MITconfig.speed != 0 && (motor->MITconfig.position != 0 || motor->MITconfig.kp != 0))
    return DM_motor_error;
  
  if(motor->MITconfig.position > motor->reg_data.PMAX 
    || motor->MITconfig.speed > motor->reg_data.VMAX 
    || motor->MITconfig.torque > motor->reg_data.TMAX 
    || motor->MITconfig.kp > 500.0f 
    || motor->MITconfig.kd > 5.0f)
    return DM_motor_error;
  
  uint16_t p_des = (uint16_t)((motor->MITconfig.position+motor->reg_data.PMAX)/(2.0f*motor->reg_data.PMAX)*65535.0f);
  uint16_t v_des = (uint16_t)((motor->MITconfig.speed+motor->reg_data.VMAX)/(2.0f*motor->reg_data.VMAX)*4095.0f);
  uint16_t t_des = (uint16_t)((motor->MITconfig.torque+motor->reg_data.TMAX)/(2.0f*motor->reg_data.TMAX)*4095.0f);
  uint16_t kp_des = (uint16_t)(motor->MITconfig.kp/500.0f*4095.0f);
  uint16_t kd_des = (uint16_t)(motor->MITconfig.kd/5.0f*4095.0f);
  
  motor->tx_data.hcan  = motor->hcan;
  motor->tx_data.len   = 8;
  motor->tx_data.tx_id = motor->ID;
  
  motor->tx_data.tx_data[0] = (p_des>>8) & 0xFF;
  motor->tx_data.tx_data[1] = p_des      & 0xFF;
  motor->tx_data.tx_data[2] = (v_des>>4) & 0xFF;
  motor->tx_data.tx_data[3] = ((v_des<<4) & 0xF0)|((kp_des>>8) & 0x0F);
  motor->tx_data.tx_data[4] = kp_des     & 0xFF;
  motor->tx_data.tx_data[5] = (kd_des>>4)& 0xFF;
  motor->tx_data.tx_data[6] = ((kd_des<<4) & 0x0F)|((t_des>>8) & 0x0F);
  motor->tx_data.tx_data[7] = t_des      & 0xFF;
  
  if(can_SendData(&motor->tx_data) != can_ok)
    return DM_motor_error;
  
  return DM_motor_ok;
}

/***********************************外部调用**************************************/
DM_motor_status_e DM_motor_init(DM_motor_config_s *config)
{
  //注册实例
  DM_instance[idx] = (DM_motor_instance_s *)malloc(sizeof(DM_motor_instance_s));
  memset(DM_instance[idx],0,sizeof(DM_motor_instance_s));
  
  DM_instance[idx]->ID            = config->ID;          
  DM_instance[idx]->MST_ID        = config->MST_ID;      
  DM_instance[idx]->control_mode  = config->control_mode;
  DM_instance[idx]->motor_type    = config->motor_type;  
  DM_instance[idx]->hcan          = config->hcan;
  
  DM_instance[idx]->reg_data.PMAX = config->PMAX;
  DM_instance[idx]->reg_data.VMAX = config->VMAX;
  DM_instance[idx]->reg_data.TMAX = config->TMAX;
  
  DM_instance[idx]->pid_position  = config->pid_position;
  DM_instance[idx]->pid_speed     = config->pid_speed;
  DM_instance[idx]->pidMode       = config->pidMode;
  
  //注册 CAN 接收
  can_rxConfig_s *can_config = (can_rxConfig_s *)malloc(sizeof(can_rxConfig_s));
  memset(can_config,0,sizeof(can_rxConfig_s));
  
  can_config->hcan      = DM_instance[idx]->hcan;
  can_config->rx_id     = DM_instance[idx]->MST_ID;
  can_config->callback  = DM_motor_canCallback;
  
  can_rxConfigInit(can_config);
  free(can_config);
  
  can_Serviceinit(config->hcan);
  
  idx ++;
  
  return DM_motor_ok;
}


//使能
DM_motor_status_e DM_motor_enable(uint16_t id)
{
  uint8_t index_id = 0;
  if(DM_motor_pairInstance(id,&index_id) != DM_motor_ok)
    return DM_motor_error;
  
  DM_instance[index_id]->tx_data.hcan  = DM_instance[index_id]->hcan;
  DM_instance[index_id]->tx_data.len   = 8;
  DM_instance[index_id]->tx_data.tx_id = DM_instance[index_id]->ID;
              
  DM_instance[index_id]->tx_data.tx_data[0] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[1] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[2] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[3] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[4] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[5] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[6] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[7] = 0xFC;

  can_SendData(&DM_instance[index_id]->tx_data);
  DM_instance[index_id]->workstatus = DM_enable;
  
  return DM_motor_ok;
}


//失能
DM_motor_status_e DM_motor_disable(uint16_t id)
{
  uint8_t index_id = 0;
  if(DM_motor_pairInstance(id,&index_id) != DM_motor_ok)
    return DM_motor_error;
  
  DM_instance[index_id]->tx_data.hcan  = DM_instance[index_id]->hcan;
  DM_instance[index_id]->tx_data.len   = 8;
  DM_instance[index_id]->tx_data.tx_id = DM_instance[index_id]->ID;
              
  DM_instance[index_id]->tx_data.tx_data[0] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[1] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[2] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[3] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[4] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[5] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[6] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[7] = 0xFD;

  can_SendData(&DM_instance[index_id]->tx_data);
  DM_instance[index_id]->workstatus = DM_disable;
  return DM_motor_ok;
}


DM_motor_status_e DM_motor_ReadParameters(uint8_t id,DM_motor_registerAddress_e motor_reg)
{
  uint8_t index_id = 0;
  if(DM_motor_pairInstance(id,&index_id) != DM_motor_ok)
    return DM_motor_error;
  
  if(DM_instance[index_id]->workstatus == DM_enable)
    return DM_motor_error;
  
  DM_instance[index_id]->tx_data.hcan  = DM_instance[index_id]->hcan;
  DM_instance[index_id]->tx_data.len   = 4;
  DM_instance[index_id]->tx_data.tx_id = 0x7FF;
              
  DM_instance[index_id]->tx_data.tx_data[0] = DM_instance[index_id]->ID & 0xFF;
  DM_instance[index_id]->tx_data.tx_data[1] = (DM_instance[index_id]->ID >> 8) & 0xFF;
  DM_instance[index_id]->tx_data.tx_data[2] = 0x33;
  DM_instance[index_id]->tx_data.tx_data[3] = motor_reg;

  can_SendData(&DM_instance[index_id]->tx_data);
  
  HAL_Delay(10);
  return DM_motor_ok;
}


//写参数
DM_motor_status_e DM_Motor_writeParameters(uint8_t id,DM_motor_registerAddress_e motor_reg,uint32_t regData)
{
  uint8_t index_id = 0;
  if(DM_motor_pairInstance(id,&index_id) != DM_motor_ok)
    return DM_motor_error;
  
  if(DM_instance[index_id]->workstatus == DM_enable)
    return DM_motor_error;
  
  DM_instance[index_id]->tx_data.hcan  = DM_instance[index_id]->hcan;
  DM_instance[index_id]->tx_data.len   = 8;
  DM_instance[index_id]->tx_data.tx_id = 0x7FF;
              
  DM_instance[index_id]->tx_data.tx_data[0] = DM_instance[idx]->ID & 0xFF;
  DM_instance[index_id]->tx_data.tx_data[1] = (DM_instance[idx]->ID >> 8) & 0xFF;
  DM_instance[index_id]->tx_data.tx_data[2] = 0x55;
  DM_instance[index_id]->tx_data.tx_data[3] = motor_reg;
  DM_instance[index_id]->tx_data.tx_data[4] = regData & 0xFF;
  DM_instance[index_id]->tx_data.tx_data[5] = (regData>>8) & 0xFF;
  DM_instance[index_id]->tx_data.tx_data[6] = (regData>>16) & 0xFF;
  DM_instance[index_id]->tx_data.tx_data[7] = (regData>>24) & 0xFF;
  
  can_SendData(&DM_instance[index_id]->tx_data);
  return DM_motor_ok;
}
  
//保存零点
DM_motor_status_e DM_Motor_saveZeroPosition(uint8_t id)
{
  uint8_t index_id = 0;
  if(DM_motor_pairInstance(id,&index_id) != DM_motor_ok)
    return DM_motor_error;
  
  DM_instance[index_id]->tx_data.hcan  = DM_instance[index_id]->hcan;
  DM_instance[index_id]->tx_data.len   = 8;
  DM_instance[index_id]->tx_data.tx_id = DM_instance[index_id]->ID;
              
  DM_instance[index_id]->tx_data.tx_data[0] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[1] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[2] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[3] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[4] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[5] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[6] = 0xFF;
  DM_instance[index_id]->tx_data.tx_data[7] = 0xFE;

  can_SendData(&DM_instance[index_id]->tx_data);
  
  return DM_motor_ok;
}

//保存参数
DM_motor_status_e DM_Motor_saveParameters(uint8_t id)
{
  uint8_t index_id = 0;
  if(DM_motor_pairInstance(id,&index_id) != DM_motor_ok)
    return DM_motor_error;
  
   if(DM_instance[index_id]->workstatus == DM_enable)
    return DM_motor_error;
  
  DM_instance[index_id]->tx_data.hcan  = DM_instance[index_id]->hcan;
  DM_instance[index_id]->tx_data.len   = 4;
  DM_instance[index_id]->tx_data.tx_id = 0x7FF;
              
  DM_instance[index_id]->tx_data.tx_data[0] = DM_instance[idx]->ID & 0xFF;
  DM_instance[index_id]->tx_data.tx_data[1] = (DM_instance[idx]->ID >> 8) & 0xFF;
  DM_instance[index_id]->tx_data.tx_data[2] = 0xAA;
  DM_instance[index_id]->tx_data.tx_data[3] = 0x01;

  can_SendData(&DM_instance[index_id]->tx_data);
  
  return DM_motor_ok;
}


//电机模式选择
DM_motor_status_e DM_Motor_mode(uint8_t id,DM_motor_controlMode_e motor_mode)
{
  DM_motor_disable(id);
  HAL_Delay(10);
  return DM_Motor_writeParameters(id,REG_CTRL_MODE,motor_mode);
}

//设置MIT参数值
DM_motor_status_e DM_motor_setMITconfig(uint8_t id,float position, float speed, float torque, float kp, float kd)
{
  uint8_t index_id = 0;
  if(DM_motor_pairInstance(id,&index_id) != DM_motor_ok)
    return DM_motor_error;
  
  DM_instance[index_id]->MITconfig.position = position;
  DM_instance[index_id]->MITconfig.speed    = speed;   
  DM_instance[index_id]->MITconfig.torque   = torque;  
  DM_instance[index_id]->MITconfig.kp       = kp;      
  DM_instance[index_id]->MITconfig.kd       = kd;      
  return DM_motor_ok;
}

//MIT
DM_motor_status_e DM_Motor_MITcontrol(void)
{
  for(uint8_t i = 0; i < idx; i++)
  {
      if(DM_instance[i]->workstatus != DM_enable)
      return DM_motor_error;
    
    if(DM_motor_sendMIT(DM_instance[i]) != DM_motor_ok)
      return DM_motor_error;
  }
  
  return DM_motor_ok;
}

//设置PVT参数值
DM_motor_status_e DM_motor_setPVTconfig(uint8_t id,float position, float max_speed, float max_torque)
{
  uint8_t index_id = 0;
  if(DM_motor_pairInstance(id,&index_id) != DM_motor_ok)
    return DM_motor_error;
  
  DM_instance[index_id]->PVTconfig.position   = position;
  DM_instance[index_id]->PVTconfig.max_speed  = max_speed;   
  DM_instance[index_id]->PVTconfig.max_torque = max_torque;  
  
  return DM_motor_ok;
}


//PVT模式
DM_motor_status_e DM_Motor_PVTcontrol(void) 
{
  for(uint8_t i = 0; i < idx; i++)
  {
      if(DM_instance[i]->workstatus == DM_disable)
      return DM_motor_error;
    
    if(DM_instance[i]->PVTconfig.max_speed == 0 || DM_instance[i]->PVTconfig.max_torque == 0)
      return DM_motor_error;
    if(DM_instance[i]->PVTconfig.max_torque > 1 ||DM_instance[i]->PVTconfig.max_speed > 100)
      return DM_motor_error;
    
    uint32_t p_des = *(uint32_t*)&DM_instance[i]->PVTconfig.position;
    uint16_t v_des = (uint16_t)(DM_instance[i]->PVTconfig.max_speed * 100);
    uint16_t i_des = (uint16_t)(DM_instance[i]->PVTconfig.max_torque * 10000);
    
    DM_instance[i]->tx_data.tx_data[0] = p_des;
    DM_instance[i]->tx_data.tx_data[1] = p_des >> 8;
    DM_instance[i]->tx_data.tx_data[2] = p_des >> 16;
    DM_instance[i]->tx_data.tx_data[3] = p_des >> 24;
    DM_instance[i]->tx_data.tx_data[4] = v_des;
    DM_instance[i]->tx_data.tx_data[5] = v_des >> 8;
    DM_instance[i]->tx_data.tx_data[6] = i_des;
    DM_instance[i]->tx_data.tx_data[7] = i_des >> 8;
     
    can_SendData(&DM_instance[i]->tx_data);
  } 
  return DM_motor_ok;
}

//设置 PID 目标
DM_motor_status_e DM_Motor_setPidConfig(uint8_t motor_ID,float targetPosition,float targetSpeed)
{
  uint8_t index_id = 0;
  if(DM_motor_pairInstance(motor_ID,&index_id) != DM_motor_ok)
    return DM_motor_error;
  
  DM_instance[index_id]->targetSpeed    = targetSpeed;
  DM_instance[index_id]->targetPosition = targetPosition;
  
  return DM_motor_ok;
}

//输入反馈
DM_motor_status_e DM_motor_setFeedbcak(uint8_t id,float position,float speed)
{
  uint8_t index_id = 0;
  if(DM_motor_pairInstance(id,&index_id) != DM_motor_ok)
    return DM_motor_error;
  
  DM_instance[index_id]->measure.speed    = speed;
  DM_instance[index_id]->measure.position = position;
  
  DM_instance[index_id]->feedBackChangeFlag = 1;
  
  return DM_motor_ok;
}

//在线修改闭环模式
DM_motor_status_e DM_Motor_setPidMode(uint8_t motor_ID,DM_motor_pidMode_e pidMode)
{
  if(pidMode != DM_motor_pidSpeed && pidMode != DM_motor_pidPosition)
    return DM_motor_error;
  
  uint8_t index_id = 0;
  if(DM_motor_pairInstance(motor_ID,&index_id) != DM_motor_ok)
    return DM_motor_error;
  
  DM_instance[index_id]->pidMode = pidMode;
  
  return DM_motor_ok;
}


//PID 闭环控制（速度环/位置环合并入口）
DM_motor_status_e DM_Motor_pidWork(void)
{
  //计算所有使能电机
  for(uint8_t i = 0; i < idx; i++)
  {
    if(DM_instance[i]->workstatus != DM_enable)
      continue;
    
    //未使用外部反馈时取 CAN 反馈
    if(DM_instance[i]->feedBackChangeFlag == 0)
    {
      DM_instance[i]->measure.speed    = DM_instance[i]->rx_data.speed;
      DM_instance[i]->measure.position = DM_instance[i]->rx_data.position;
    }
    
    //按注册时设定的闭环模式计算
    if(DM_instance[i]->pidMode == DM_motor_pidPosition)
    {
      //位置环输出作为速度环目标
      if(pid_calculate(DM_instance[i]->targetPosition, DM_instance[i]->measure.position, &DM_instance[i]->pid_position) != pid_ok)
        return DM_motor_error;
      
      DM_instance[i]->targetSpeed = DM_instance[i]->pid_position.output;
    }
    
    if(pid_calculate(DM_instance[i]->targetSpeed, DM_instance[i]->measure.speed, &DM_instance[i]->pid_speed) != pid_ok)
      return DM_motor_error;
    
    //速度环输出作为 MIT 力矩项
    DM_instance[i]->MITconfig.position = 0.0f;
    DM_instance[i]->MITconfig.speed    = 0.0f;
    DM_instance[i]->MITconfig.torque   = DM_instance[i]->pid_speed.output;
    DM_instance[i]->MITconfig.kp       = 0.0f;
    DM_instance[i]->MITconfig.kd       = 0.0f;
  }
  
  //统一发帧
  for(uint8_t i = 0; i < idx; i++)
  {
    if(DM_instance[i]->workstatus != DM_enable)
      continue;
    
    if(DM_motor_sendMIT(DM_instance[i]) != DM_motor_ok)
      return DM_motor_error;
  }
  
  return DM_motor_ok;
}

//读取反馈快照
DM_motor_status_e DM_motor_getData(uint8_t motor_ID,DM_motor_rxData_s *data)
{
  if(data == NULL)
    return DM_motor_error;
  
  uint8_t index_id = 0;
  if(DM_motor_pairInstance(motor_ID,&index_id) != DM_motor_ok)
    return DM_motor_error;
  
  DM_instance[index_id]->rx_data.pidSpeed_output = DM_instance[index_id]->pid_speed.output;
  DM_instance[index_id]->rx_data.pidAngle_output = DM_instance[index_id]->pid_position.output;
  
  *data = DM_instance[index_id]->rx_data;
  
  return DM_motor_ok;
}
