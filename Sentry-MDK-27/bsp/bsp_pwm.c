#include "bsp_pwm.h"

/************************私有函数****************************/
/**
 * @brief pwm实例
 */
typedef struct
{
  uint8_t id;
  TIM_HandleTypeDef *htim;
  uint32_t Channel;
  uint16_t ccr;
  pwm_moduleCallback_f pwm_moduleCallback;
  
}pwm_instance_s;

//pwm实例
pwm_instance_s *pwm_instance[PWM_CNT] = {NULL};
//pwm实例检索
static uint8_t idx = 0;

/*************************外部调用函数**************************/

//初始化
void pwm_init(pwm_config_s *instance)
{
  pwm_instance[idx] = (pwm_instance_s *)malloc(sizeof(pwm_instance_s));
  memset(pwm_instance[idx],0,sizeof(pwm_instance_s));
  
  
  pwm_instance[idx]->id      = instance->id;
  pwm_instance[idx]->htim    = instance->htim;   
  pwm_instance[idx]->Channel = instance->Channel;
  pwm_instance[idx]->pwm_moduleCallback = instance->pwm_moduleCallback;
  //启用tim定时器pwm比较中断模式
  HAL_TIM_PWM_Start_IT(pwm_instance[idx]->htim,pwm_instance[idx]->Channel);
  //初始化CCR为0
  __HAL_TIM_SET_COMPARE(pwm_instance[idx]->htim,pwm_instance[idx]->Channel,0);
  
  idx++;
      
}

//修改占空比
void pwm_dutyCycle(uint8_t id,float percent)
{
  for(uint8_t i = 0; i < idx; i++)
  {
    if(id == pwm_instance[i]->id)
    {
      pwm_instance[i]->ccr = (uint16_t)(percent * (pwm_instance[i]->htim->Instance->ARR));
      __HAL_TIM_SET_COMPARE(pwm_instance[i]->htim,pwm_instance[i]->Channel,pwm_instance[i]->ccr);
    }
  }
}



/**************************中断回调***********************/

//比较中断回调
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  for(uint8_t i = 0; i < idx; i++)
  {
    if(pwm_instance[i]->htim == htim)
    {
      if(pwm_instance[i]->pwm_moduleCallback != NULL)
        pwm_instance[i]->pwm_moduleCallback();
      break;
    }
  }
}
