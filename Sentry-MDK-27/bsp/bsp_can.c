#include "bsp_can.h"
/********************************私有函数***********************************/
typedef struct 
{
  can_Rxdata_s Rxdata;
  can_moduleCallback_f  callback;
}can_rxInstance_s;
//实例
//can_txInstance_s *can_txInstance[CAN_CNT] = {NULL};
can_rxInstance_s *can_rxInstance[CAN_CNT] = {NULL};
//uint8_t tx_idx = 0;
uint8_t rx_idx = 0;

/********************************外部调用***********************************/
//启用can通信服务
void can_Serviceinit(CAN_HandleTypeDef* hcan)
{  
  //配置过滤器
  CAN_FilterTypeDef can_filter;
  
  if(hcan != &hcan1)
    can_filter.FilterBank = 14;      
  else
    can_filter.FilterBank = 0;  
  can_filter.FilterMode   = CAN_FILTERMODE_IDMASK;  
  can_filter.FilterScale  = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0;
  can_filter.FilterIdLow  = 0;
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow  = 0;                
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; 
  can_filter.FilterActivation = ENABLE;      
  can_filter.SlaveStartFilterBank  = 14;  
    
  HAL_CAN_ConfigFilter(hcan, &can_filter); 
  HAL_CAN_Start(hcan); 
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); 
  
}

//接收实例初始化
void can_rxConfigInit(can_rxConfig_s *rxConfig)
{
  can_rxInstance[rx_idx] = (can_rxInstance_s *)malloc(sizeof(can_rxInstance_s));
  memset(can_rxInstance[rx_idx],0,sizeof(can_rxInstance_s));
  
  //传值
  can_rxInstance[rx_idx]->Rxdata.hcan     = rxConfig->hcan;    
  can_rxInstance[rx_idx]->Rxdata.rx_id    = rxConfig->rx_id;    
  can_rxInstance[rx_idx]->callback = rxConfig->callback;

  rx_idx ++;
}

//发送实例初始化
//void can_txConfigInit(can_txConfig_s *txConfig)
//{
//  can_txInstance[tx_idx] = (can_txInstance_s *)malloc(sizeof(can_txInstance_s));
//  memset(can_rxInstance[tx_idx],0,sizeof(can_txInstance_s));
//  
//  //传值
//  can_txInstance[tx_idx]->hcan  = txConfig->hcan;    
//  can_txInstance[tx_idx]->tx_id = txConfig->tx_id;    
//  can_txInstance[tx_idx]->len   = txConfig->len;

//  tx_idx ++;
//}

//发送
can_status_e can_SendData(can_txInstance_s *txInstance)
{
  CAN_TxHeaderTypeDef tx_header;
  uint32_t used_mailbox;

  tx_header.StdId = txInstance->tx_id;
  tx_header.ExtId = 0;
  tx_header.IDE = 0;
  tx_header.RTR = 0;
  tx_header.DLC = txInstance->len;

  if(HAL_CAN_AddTxMessage(txInstance->hcan, &tx_header, txInstance->tx_data, &used_mailbox) != HAL_OK)
    return can_error;
  
  return can_ok;
}


/*******************************回调************************************/


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t callback_data[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,callback_data);
  for(uint8_t i = 0; i < rx_idx; i++)
  {
    if(can_rxInstance[i]->Rxdata.hcan == hcan && can_rxInstance[i]->Rxdata.rx_id == rx_header.StdId)
    {
      memcpy(can_rxInstance[i]->Rxdata.rx_data,callback_data,rx_header.DLC);
      can_rxInstance[i]->Rxdata.len = rx_header.DLC;
      can_rxInstance[i]->callback(&can_rxInstance[i]->Rxdata);
      break;
    }
  }  
}

