#include "vision.h"

/*******************************私有变量及函数**********************************/
typedef struct
{
  uint8_t SOF;
  uint8_t workFlag;
  vision_rx_s rx_data;
}vision_instance_s;
vision_instance_s *vision_instance = NULL;

uint8_t a[40];
static void vision_usbCallback(uint8_t *data,uint32_t len)
{
  static protocol_frame_s protocol_rx;
  
  memcpy(a,data,len);
  
  //解包并判断检验是否正确
  if(protocol_array_to_struct(data, &protocol_rx) == protocol_ok)
  {
    //判断帧头是否正确
    if(protocol_rx.header.SOF == vision_instance->SOF)
    {
      //喂狗
      deamon_WDOG(deamon_module_VISION);
      
      vision_instance->rx_data.cmd_id = protocol_rx.cmd_id;
      memcpy(&vision_instance->rx_data.forward, protocol_rx.data, 4);
      memcpy(&vision_instance->rx_data.turn, protocol_rx.data+4, 4);
      memcpy(&vision_instance->rx_data.rotate, protocol_rx.data+8, 4);
      memcpy(&vision_instance->rx_data.yaw, protocol_rx.data+12, 4);
      memcpy(&vision_instance->rx_data.pitch, protocol_rx.data+16, 4);
      
      vision_instance->workFlag = 1;
    }
    else
      return;
      
  }
  else
    return;
}

//离线处理
static void vision_deamonCallback(void *id)
{
  vision_instance->workFlag = 0;
}

/*******************************外部调用**********************************/
//初始化
void vision_init(void)
{
  vision_instance = (vision_instance_s*)malloc(sizeof(vision_instance_s));
  memset(vision_instance,0,sizeof(vision_instance_s));
  
  vision_instance->SOF = 0x11;
  vision_instance->workFlag = 1;
  
  //初始化usb
  usb_init(vision_usbCallback);
  
  //注册线程守护
  deamon_config_s deamon;
  deamon.id               = deamon_module_VISION;
  deamon.owner_id         = NULL;
  deamon.reload_count     = 100;
  deamon.deamon_callback  = vision_deamonCallback;
  deamon_init(&deamon);
}

//开启视觉通信
void vision_start(void)
{
  vision_instance->workFlag = 1;
  usb_start();
}

//停止通信
void vision_stop(void)
{
  vision_instance->workFlag = 0;
  usb_stop();
}

//发送数据

vision_status_e vision_sendData(vision_tx_s tx_struct)
{
  if(vision_instance->workFlag != 1)
    return vision_error;
  
  protocol_frame_s protocol_strcut;
  protocol_strcut.header.SOF = vision_instance->SOF;
  protocol_strcut.header.data_len = 12;
  protocol_strcut.header.seq = 0x00;
  protocol_strcut.cmd_id = tx_struct.cmd_id;
  
  memcpy(protocol_strcut.data,&tx_struct.yaw,4);
  memcpy(protocol_strcut.data+4,&tx_struct.pitch,4);
  memcpy(protocol_strcut.data+8,&tx_struct.free_data,4);
  
  static uint8_t tx_array[22];
  static uint16_t len;
  protocol_struct_to_array(&protocol_strcut, tx_array, &len);
  
  if(usb_sendData(tx_array, len)==usb_ok)
    return vision_ok;
  else
    return vision_error;
}

//数据接口
vision_rx_s vision_getData(void)
{
  vision_instance->rx_data.state = vision_instance->workFlag;
  return vision_instance->rx_data;
}
