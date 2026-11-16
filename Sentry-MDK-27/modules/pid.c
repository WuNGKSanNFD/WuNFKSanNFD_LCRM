#include "pid.h"

/************************优化算法***********************/

//梯形积分
static void pid_i_trapezoidal(pid_instance_s *pid_instance)
{
  pid_instance->i_term += (pid_instance->error + pid_instance->last_e) / 2.0f;
}

//积分分离
static void pid_i_separation(pid_instance_s *pid_instance)
{
  if(fabsf(pid_instance->error) > pid_instance->epsilon)
    pid_instance->i_term = 0.0f;
  else
    pid_instance->i_term += pid_instance->error;
    
}

//变速积分
static void pid_i_variableSpeed(pid_instance_s *pid_instance)
{
  if(fabsf(pid_instance->error) <= pid_instance->B)
  {
    pid_instance->i_term *= 1.0f;
  }
  else if(fabsf(pid_instance->error) < pid_instance->A)
  {
    pid_instance->i_term *= (pid_instance->A - fabsf(pid_instance->error))/(pid_instance->A - pid_instance->B);
  }
  else
  {
    pid_instance->i_term = 0.0f;
  }
}

//不完全微分(微分滤波)
static void pid_d_incomplete(pid_instance_s *pid_instance)
{
  pid_instance->d_term = pid_instance->d_f_alpha * pid_instance->last_d_term + (1-pid_instance->d_f_alpha) * pid_instance->d_term;
}

//微分先行
static void pid_d_onMeasurement(pid_instance_s *pid_instance)
{
  pid_instance->d_term =  - (pid_instance->measure - pid_instance->last_measure);
}


//输出滤波
static void pid_output_filter(pid_instance_s *pid_instance)
{
  pid_instance->output = pid_instance->output_f_alpha * pid_instance->last_output + (1 - pid_instance->output_f_alpha) * pid_instance->output;
}

/*********************计算输出*******************/
//计算
pid_status_e pid_calculate(float target,float measure,pid_instance_s *pid_instance)
{
  
  //传递参数
  pid_instance->measure = measure;
  pid_instance->target = target;

  
  
  /***模式判断***/
  
  //判断模式是否合法
  if((pid_instance->pid_mode&pid_mode_error)==0)
  {
    return pid_error;
  }
  
   //误差计算
  pid_instance->error = pid_instance->target - pid_instance->measure;
  
  
  //普通模式
  if(pid_instance->pid_mode&pid_mode_normal)
  {
    pid_instance->i_term += pid_instance->error;
    pid_instance->d_term = (pid_instance->error - pid_instance->last_e);
  }
  else
  {
    //梯形积分
    if(pid_instance->pid_mode&pid_mode_i_trapezoidal)
    {
      pid_i_trapezoidal(pid_instance);
    }
    else
      pid_instance->i_term += pid_instance->error;

    //变速积分
    if(pid_instance->pid_mode&pid_mode_i_variableSpeed)
    {
      pid_i_variableSpeed(pid_instance);
    }
    //积分分离
    else if(pid_instance->pid_mode&pid_mode_i_separation)
      pid_i_separation(pid_instance);

    
    //微分先行
    if(pid_instance->pid_mode&pid_mode_d_onMeasurement)
      pid_d_onMeasurement(pid_instance);
    else
      pid_instance->d_term = - (pid_instance->error - pid_instance->last_e);
    
    //不完全微分(微分滤波)
    if(pid_instance->pid_mode&pid_mode_d_incomplete)
      pid_d_incomplete(pid_instance);
  }
  
  
  //各环节计算
  pid_instance->p_out = pid_instance->kp * pid_instance->error;
  pid_instance->i_out = pid_instance->ki * pid_instance->i_term;
  pid_instance->d_out = pid_instance->kd * pid_instance->d_term;
  
  //保存参数
  pid_instance->last_output = pid_instance->output;
  pid_instance->last_e = pid_instance->error;
  pid_instance->last_d_term = pid_instance->d_term;
  pid_instance->last_measure = pid_instance->measure;
  
  
  //积分限幅
  if(pid_instance->i_term > pid_instance->max_i / pid_instance->ki)
    pid_instance->i_term = pid_instance->max_i / pid_instance->ki;
  else if(pid_instance->i_term < -pid_instance->max_i / pid_instance->ki)
    pid_instance->i_term = -pid_instance->max_i / pid_instance->ki;
  
  
  //输出计算（f_out 由外部前馈/重力补偿模块写入，此处只参与求和）
  if(pid_instance->dead < fabsf(pid_instance->error))
    pid_instance->output = pid_instance->p_out + pid_instance->i_out +  pid_instance->d_out + pid_instance->f_out;
  else
    pid_instance->output = pid_instance->last_output;
  
  //输出限幅
  if(pid_instance->output > pid_instance->maxout)
    pid_instance->output = pid_instance->maxout;
  else if(pid_instance->output < -pid_instance->maxout)
    pid_instance->output = -pid_instance->maxout;
  
  //输出滤波
  if(pid_instance->pid_mode&pid_mode_output_filter)
  {
    pid_output_filter(pid_instance);
  }
  
  //保存输出(供输出滤波/死区使用)
  pid_instance->last_output = pid_instance->output;
  
  return pid_ok;
}
