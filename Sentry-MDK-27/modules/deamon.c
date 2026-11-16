#include "deamon.h"

/***************************私有函数*****************************/

/**
  * @brief 实例
  */
typedef struct
{
  uint16_t reload_count;
  uint8_t id;  //编号
  void *owner_id;
  deamon_callback_f deamon_callback; 
  
  //当前计数值
  uint16_t cnt;
  
}deamon_instance_s;

deamon_instance_s *deamon_instance[8] = {NULL};
static uint8_t idx = 0;

static int8_t deamon_pairInstance(uint8_t id)
{
  for(uint8_t i = 0; i < idx; i++)
  {
    if(deamon_instance[i]->id == id)
      return i;
  }
  //返回-1表示没搜索到实例
  return -1;
}


/****************************外部调用****************************/
//初始化
void deamon_init(deamon_config_s *config)
{
  deamon_instance[idx] = (deamon_instance_s *)malloc(sizeof(deamon_instance_s));
  memset(deamon_instance[idx],0,sizeof(deamon_instance_s));
  
  deamon_instance[idx]->reload_count      = config->reload_count;    
  deamon_instance[idx]->id                = config->id; 
  deamon_instance[idx]->owner_id          = config->owner_id;       
  deamon_instance[idx]->deamon_callback   = config->deamon_callback;  
  
  idx ++;
}

//修改计数时间
void deamon_Reload(uint8_t id,uint16_t reload_count)
{
  //实例匹配
  int8_t index_id = deamon_pairInstance(id);

  if(index_id == -1)
    return;
  
  deamon_instance[index_id]->reload_count = reload_count;
}

//喂狗
void deamon_WDOG(uint8_t id)
{
  //实例匹配
  int8_t index_id = deamon_pairInstance(id);
  if(index_id == -1)
    return;
  
  deamon_instance[index_id]->cnt = 0;
}

void deamonStart(void)
{
  for(uint8_t i = 0; i < idx; i++)
  {
    if(deamon_instance[i]->cnt <= deamon_instance[i]->reload_count)
    {
      deamon_instance[i]->cnt ++;
    }
    else
    {
      deamon_instance[i]->deamon_callback(deamon_instance[i]->owner_id);
    }
    
  }
}
