#include "remote.h"
/*******************私有函数********************/
remote_rawData_s remote_rawData = {0,};

//串口回调：喂狗和解析数据
static void remote_rxCallback(void)
{
  remote_rawData.startFlag = 1;
  deamon_WDOG(deamon_module_remote);
  remote_receive();
}

//离线处理
static void remore_offline(void *id)
{
  remote_rawData.startFlag = 0;
}
/*********************外部调用*******************/
//初始化
remote_status_e remote_init(void)
{
  //串口初始化，该遥控器指定用DJI DT7 接收机用DR16
  serial_config_s serial;
  serial.mode = serial_mode_receive;
  serial.huart = &huart3;
  serial.rx_DLC = 18;
  serial.moduleCallback = remote_rxCallback;
  
  Serial_init(&serial);
  
  //离线检测
  deamon_config_s deamon_remote;
  deamon_remote.deamon_callback = remore_offline;
  deamon_remote.id = deamon_module_remote;
  deamon_remote.owner_id = NULL;
  deamon_remote.reload_count = 10;
  deamon_init(&deamon_remote);
  
  //开启串口接收
  Serial_receive(&huart3);
  
  return remote_ok;
}

//遥控数据传递
uint8_t serial_rxdata[18];
remote_status_e remote_receive(void)
{  
  //获取串口接收信息
  
  Serial_getRxdata(&huart3,serial_rxdata);
  
  
  /**************接收数据****************/
  
  //遥控器
  remote_rawData.DT7.ch0 = (serial_rxdata[0] | (serial_rxdata[1] << 8)) & 0x07FF;
  remote_rawData.DT7.ch1 = ((serial_rxdata[1] >> 3) | (serial_rxdata[2] << 5)) & 0x07FF;
  remote_rawData.DT7.ch2 = ((serial_rxdata[2] >> 6) | (serial_rxdata[3] << 2) | (serial_rxdata[4] << 10)) & 0x07FF;
  remote_rawData.DT7.ch3 = ((serial_rxdata[4] >> 1) | (serial_rxdata[5] << 7)) & 0x07FF;
  remote_rawData.DT7.s1  = ((serial_rxdata[5] >> 4) & 0x000C) >> 2;
  remote_rawData.DT7.s2  = ((serial_rxdata[5] >> 4) & 0x0003);

  //拨轮
  remote_rawData.DT7.dial = (serial_rxdata[16] | (serial_rxdata[17] << 8)) & 0x07FF;
  
  //鼠标
  remote_rawData.client.mouse_x       = (serial_rxdata[6] | (serial_rxdata[7] << 8));
  remote_rawData.client.mouse_y       = (serial_rxdata[8] | (serial_rxdata[9] << 8));
  remote_rawData.client.mouse_z       = (serial_rxdata[10] | (serial_rxdata[11] << 8));
  remote_rawData.client.mouse_press_l = serial_rxdata[12];
  remote_rawData.client.mouse_press_r = serial_rxdata[13];

  //按键
  remote_rawData.client.key_w     = (serial_rxdata[14] & 0x0001);
  remote_rawData.client.key_s     = ((serial_rxdata[14] & 0x0002) >> 1);
  remote_rawData.client.key_a     = ((serial_rxdata[14] & 0x0004) >> 2);
  remote_rawData.client.key_d     = ((serial_rxdata[14] & 0x0008) >> 3);
  remote_rawData.client.key_shift = ((serial_rxdata[14] & 0x0010) >> 4);
  remote_rawData.client.key_ctrl  = ((serial_rxdata[14] & 0x0020) >> 5);
  remote_rawData.client.key_q     = ((serial_rxdata[14] & 0x0040) >> 6);
  remote_rawData.client.key_e     = ((serial_rxdata[14] & 0x0080) >> 7);
  
  return remote_ok;
}


//数据调用接口
remote_status_e remote_getData(remote_data_s *remote_data)
{
  //判断遥控是否开启
  if(remote_rawData.startFlag == 0)
  {
    memset(remote_data,0,sizeof(remote_data_s));
    return remote_error;
  }
  
  //传值
  remote_data->ch0  = remote_rawData.DT7.ch0 - 1024;
  remote_data->ch1  = remote_rawData.DT7.ch1 - 1024;
  remote_data->ch2  = remote_rawData.DT7.ch2 - 1024;
  remote_data->ch3  = remote_rawData.DT7.ch3 - 1024;
  
  remote_data->s1   = remote_rawData.DT7.s1;
  remote_data->s2   = remote_rawData.DT7.s2;
  remote_data->dial = remote_rawData.DT7.dial - 1024;
  
  remote_data->client.mouse_x       = remote_rawData.client.mouse_x;      
  remote_data->client.mouse_y       = remote_rawData.client.mouse_y;      
  remote_data->client.mouse_z       = remote_rawData.client.mouse_z;      
  remote_data->client.mouse_press_l = remote_rawData.client.mouse_press_l;
  remote_data->client.mouse_press_r = remote_rawData.client.mouse_press_r;
  
  remote_data->client.key_w     = remote_rawData.client.key_w;    
  remote_data->client.key_s     = remote_rawData.client.key_s;    
  remote_data->client.key_a     = remote_rawData.client.key_a;    
  remote_data->client.key_d     = remote_rawData.client.key_d;    
  remote_data->client.key_shift = remote_rawData.client.key_shift;
  remote_data->client.key_ctrl  = remote_rawData.client.key_ctrl; 
  remote_data->client.key_q     = remote_rawData.client.key_q;    
  remote_data->client.key_e     = remote_rawData.client.key_e;    
  
  return remote_ok;
}
