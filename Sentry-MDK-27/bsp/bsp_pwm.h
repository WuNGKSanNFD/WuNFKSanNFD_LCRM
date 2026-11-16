#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include "tim.h"
#include <stdlib.h>
#include <string.h>

/*******************宏***********************/
//pwm通道数量
#define PWM_CNT   4

/*******************结构体*******************/

/**
 * @brief pwm回调函数
 */
typedef void (*pwm_moduleCallback_f)(); 


/**
 * @brief pwm注册实例
 */
typedef struct
{
  uint8_t id;
  TIM_HandleTypeDef *htim;
  uint32_t Channel;
  pwm_moduleCallback_f pwm_moduleCallback;
}pwm_config_s;


/*********************外部调用函数*************************/

/**
 * @brief pwm初始化
 * @param instance    注册实例结构体
 */
void pwm_init(pwm_config_s *instance);

/**
 * @brief pwm修改占空比
 * @param id      pwm实例id号
 * @param percent 占空比（0~1，1 表示 100%）
 */
void pwm_dutyCycle(uint8_t id,float percent);

#endif
