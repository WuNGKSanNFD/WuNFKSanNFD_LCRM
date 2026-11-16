#include "feedback.h"

/************************前馈与补偿***********************/

//速度前馈
static void feedback_v_feedforward(feedback_instance_s *feedback_instance)
{
  feedback_instance->v_out = feedback_instance->k_v * feedback_instance->target_vel;
}

//加速度前馈
static void feedback_a_feedforward(feedback_instance_s *feedback_instance)
{
  feedback_instance->a_out = feedback_instance->k_a * feedback_instance->target_acc;
}

//摩擦补偿：|v| > v_eps 时按速度方向输出 ±k_s，阈值内线性过渡，避免零速附近符号抖振
static void feedback_friction(feedback_instance_s *feedback_instance)
{
  if(fabsf(feedback_instance->target_vel) > feedback_instance->v_eps)
  {
    if(feedback_instance->target_vel > 0.0f)
      feedback_instance->s_out = feedback_instance->k_s;
    else
      feedback_instance->s_out = -feedback_instance->k_s;
  }
  else if(feedback_instance->v_eps > 0.0f)
  {
    feedback_instance->s_out = feedback_instance->k_s * feedback_instance->target_vel / feedback_instance->v_eps;
  }
  else
  {
    feedback_instance->s_out = 0.0f;
  }
}

/*********************计算输出*******************/
//重力补偿：单连杆模型，重力矩随角度按余弦变化
float feedback_gravity_calculate(float angle,feedback_instance_s *feedback_instance)
{
  feedback_instance->angle = angle;
  feedback_instance->g_out = feedback_instance->gravity_gain * cosf(angle - feedback_instance->gravity_phase);
  
  return feedback_instance->g_out;
}

//前馈+重力补偿
feedback_status_e feedback_calculate(float target_vel,float target_acc,float angle,feedback_instance_s *feedback_instance)
{
  //传递参数
  feedback_instance->target_vel = target_vel;
  feedback_instance->target_acc = target_acc;
  feedback_instance->angle = angle;
  
  
  /***模式判断***/
  
  //判断模式是否合法
  if((feedback_instance->mode&feedback_mode_error)==0)
  {
    return feedback_error;
  }
  
  //各分项清零，避免模式切换后残留上一轮输出
  feedback_instance->v_out = 0.0f;
  feedback_instance->a_out = 0.0f;
  feedback_instance->s_out = 0.0f;
  feedback_instance->g_out = 0.0f;
  
  
  //速度前馈
  if(feedback_instance->mode&feedback_mode_velocity)
  {
    feedback_v_feedforward(feedback_instance);
  }
  
  //加速度前馈
  if(feedback_instance->mode&feedback_mode_acceleration)
  {
    feedback_a_feedforward(feedback_instance);
  }
  
  //摩擦补偿
  if(feedback_instance->mode&feedback_mode_friction)
  {
    feedback_friction(feedback_instance);
  }
  
  //重力补偿
  if(feedback_instance->mode&feedback_mode_gravity)
  {
    feedback_gravity_calculate(angle,feedback_instance);
  }
  
  
  //输出求和
  feedback_instance->output = feedback_instance->v_out + feedback_instance->a_out
                            + feedback_instance->s_out + feedback_instance->g_out;
  
  //输出限幅
  if(feedback_instance->output > feedback_instance->maxout)
    feedback_instance->output = feedback_instance->maxout;
  else if(feedback_instance->output < -feedback_instance->maxout)
    feedback_instance->output = -feedback_instance->maxout;
  
  return feedback_ok;
}
