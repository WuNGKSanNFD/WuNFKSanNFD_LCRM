#include "bsp_spi.h"

/*********************私有函数及变量**************************/
typedef struct
{
  uint8_t ID;
  SPI_HandleTypeDef *SPIx;
  GPIO_TypeDef *CS_GPIOx;
  uint16_t CS_GPIO_PIN;
  uint8_t rx_len;
  spi_rxCallBack_f callback;
  
  spi_data_s data;
  spi_csSet_e cs_status;
}spi_instance_s;

spi_instance_s *spi_instance[SPI_CNT] = {NULL};
static uint8_t idx = 0;

volatile uint8_t spi_active_id;


//片选器拉高拉低
static void spi_csSet(uint8_t in_idx, spi_csSet_e set)
{
  HAL_GPIO_WritePin(spi_instance[in_idx]->CS_GPIOx, spi_instance[in_idx]->CS_GPIO_PIN, (GPIO_PinState)set);
  spi_instance[in_idx]->cs_status = set;
}

static spi_csSet_e spi_csRead(uint8_t in_idx)
{
  spi_instance[in_idx]->cs_status = (spi_csSet_e)HAL_GPIO_ReadPin(spi_instance[in_idx]->CS_GPIOx, spi_instance[in_idx]->CS_GPIO_PIN);
  return spi_instance[in_idx]->cs_status;
}


/***********************外部调用***************************/

//初始化
spi_status_e spi_init(spi_config_s *config)
{
  spi_instance[idx] = (spi_instance_s *)malloc(sizeof(spi_instance_s));
  memset(spi_instance[idx],0,sizeof(spi_instance_s));
  
  spi_instance[idx]->ID          = config->ID;         
  spi_instance[idx]->SPIx        = config->SPIx;       
  spi_instance[idx]->CS_GPIOx    = config->CS_GPIOx;   
  spi_instance[idx]->CS_GPIO_PIN = config->CS_GPIO_PIN;
  spi_instance[idx]->callback    = config->callback;
  spi_instance[idx]->rx_len      = config->rx_len;
  
  
  idx++;
  
  //拉高片选器
  spi_csSet(idx-1, SPI_CS_STOP);
  
  return spi_ok;
}


//发送数据
spi_status_e spi_sendData(uint8_t id,uint8_t *tx_data, uint8_t len)
{
  //寻址
  for(uint8_t i = 0; i < idx; i++)
  {
    if(id == spi_instance[i]->ID)
    {
      //拉低片选器,开始通信
      spi_csSet(i, SPI_CS_START);
      
      HAL_SPI_Transmit(spi_instance[i]->SPIx, tx_data, len, SPI_TIMEOUT);
      
      //拉高片选器，通信结束
      spi_csSet(i, SPI_CS_STOP);
      
      return spi_ok;
    }    
  }

  return spi_error;
}


//接收
spi_status_e spi_receiveData(uint8_t id)
{
  for(uint8_t i = 0; i < idx; i++)
  {
    if(id == spi_instance[i]->ID)
    {
      //拉低片选器,开始通信
      spi_csSet(i, SPI_CS_START);
      
      //开启接收中断
      if(HAL_SPI_Receive_IT(spi_instance[i]->SPIx, spi_instance[i]->data.raw, spi_instance[i]->rx_len) != HAL_OK)
        return spi_error;
      
      return spi_ok;
    }  
  }
  return spi_ok;
}


//发送接收
spi_status_e spi_sendReceiveData(uint8_t id, uint8_t *tx_data)
{
  //寻址
  for(uint8_t i = 0; i < idx; i++)
  {
    if(id == spi_instance[i]->ID)
    {
      //拉低片选器,开始通信
      spi_csSet(i, SPI_CS_START);
      
      spi_active_id = i;
      
      if(HAL_SPI_TransmitReceive(spi_instance[i]->SPIx, tx_data, spi_instance[i]->data.raw, spi_instance[i]->rx_len,SPI_TIMEOUT) != HAL_OK)
      {
        spi_csSet(i, SPI_CS_STOP);
        return spi_error;
      }
      else 
      {
        spi_csSet(i, SPI_CS_STOP);
        if(spi_instance[spi_active_id]->callback != NULL)
          spi_instance[spi_active_id]->callback();
      }
      
      return spi_ok;
    }    
  }

  return spi_error;
}

//中断发送回调
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) 
{
  if(spi_active_id >= idx)
    return;
  
  if(spi_instance[spi_active_id]->SPIx == hspi)
  {
    //拉高片选器
    spi_csSet(spi_active_id,SPI_CS_STOP);
    
    //执行上层回调
    if(spi_instance[spi_active_id]->callback != NULL)
      spi_instance[spi_active_id]->callback();
    
    spi_active_id = 0xFF;
    
    return;
  }   
  
}
  


//中断接收回调
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  for(uint8_t i = 0; i < idx; i++)
  {
    if(spi_csRead(i) == SPI_CS_START && spi_instance[i]->SPIx == hspi)
    {
      //拉高片选器
      spi_csSet(i,SPI_CS_STOP);
      
      //执行上层回调
      if(spi_instance[i]->callback != NULL)
        spi_instance[i]->callback();
      
      return;
    }   

  }
}

//错误处理


//数据接口
spi_data_s spi_getData(uint8_t id)
{
  for(uint8_t i = 0; i < idx; i++)
  {
    if(id == spi_instance[i]->ID)
    {
      return spi_instance[i]->data;
    }
  }
  
  spi_data_s zone_data;
  return zone_data;
}
