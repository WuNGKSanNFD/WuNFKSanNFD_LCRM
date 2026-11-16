#include "bsp_usb.h"

/*************************私有变量及函数****************************/
typedef struct
{
  uint8_t initFlag;
//  usb_data_s data;
  usb_moduleCallback_f moduleCallback;
}usb_instance_s;
usb_instance_s *usb_instance = NULL;
/*************************外部调用****************************/
//初始化
void usb_init(usb_moduleCallback_f moduleCallback)
{
  usb_instance = (usb_instance_s*)malloc(sizeof(usb_instance_s));
  memset(usb_instance,0,sizeof(usb_instance_s));
  
  usb_instance->moduleCallback = moduleCallback;
  usb_instance->initFlag = 1;
}

//关闭usb通信
void usb_stop(void)
{
  usb_instance->initFlag = 0;
}

//启用usb通信
void usb_start(void)
{
  usb_instance->initFlag = 1;
}


//发送
usb_status_e usb_sendData(uint8_t *tx_data,uint16_t len)
{
  if(usb_instance->initFlag != 1)
    return usb_error;
  
  CDC_Transmit_FS(tx_data,len);
  return usb_ok;
}

//接收回调
void BSP_CDC_ReceviceCallBack(uint8_t* Buf, uint32_t *Len)
{
  if(usb_instance->initFlag != 1)
    return;
  
  usb_instance->moduleCallback(Buf,*Len);
}
