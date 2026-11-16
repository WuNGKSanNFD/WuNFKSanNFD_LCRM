#include "BMI088.h"

/*************************私有函数及变量*************************/
typedef struct
{
  uint8_t ACC_ID;
  uint8_t GYRO_ID;
  SPI_HandleTypeDef *SPIx;
  GPIO_TypeDef *ACC_CS_GPIOx;
  uint16_t ACC_CS_GPIO_PIN;
  GPIO_TypeDef *GYRO_CS_GPIOx;
  uint16_t GYRO_CS_GPIO_PIN;
   
  volatile uint8_t RxBusyFlag;
  volatile uint8_t TempReadFlag;//置1表示本次读取的是温度寄存器（温度与加速度计共用一路SPI）
  volatile uint8_t TempUpdateFlag;//置1表示刚采集到新的温度值，由 BMI_getTempUpdate() 读取后清零
  BMI_data_s data;
}BMI_instance_s;

BMI_instance_s *BMI_instance = NULL;


//写寄存器
static void BMI_txReg(uint8_t spi_id,uint8_t address, uint8_t data)
{
  uint8_t send_data[2] = {address,data};
  spi_sendData(spi_id,send_data,2);
}


//读取寄存器
static BMI_status_e BMI_rxReg(uint8_t spi_id,uint8_t address)
{
  BMI_instance->RxBusyFlag = 1;
  uint8_t send_data[8] ={address | 0x80,0,};
  if(spi_sendReceiveData(spi_id,send_data) != spi_ok)
  {
    BMI_instance->RxBusyFlag = 0;
    return BMI_error;
  }
 
  return BMI_ok;
}


//读取温度寄存器
static BMI_status_e BMI_rxTemp(void)
{
  BMI_instance->TempReadFlag = 1;
  if(BMI_rxReg(BMI_instance->ACC_ID,BMI_TEMP_ADDR) != BMI_ok)
  {
    BMI_instance->TempReadFlag = 0;
    return BMI_error;
  }
  
  return BMI_ok;
}


//线程守护回调 
//static void BMI_deamonCallback(void *owner_id)
//{
//  
//}


//加速度计回调
static void BMI_accCallback(void)
{
  BMI_instance->RxBusyFlag = 0;
  spi_data_s acc_data = spi_getData(BMI_instance->ACC_ID); 
  
  //温度与加速度共用一路SPI，按标志区分本次帧内容
  if(BMI_instance->TempReadFlag)
  {
    BMI_instance->TempReadFlag = 0;
    
    //温度原始值为11位补码：0x22为高8位，0x23的高3位为低3位
    int16_t temp_raw = (int16_t)((acc_data.raw[2] << 3) | (acc_data.raw[3] >> 5));
    if(temp_raw > 1023)
      temp_raw -= 2048;
    
    BMI_instance->data.temp = temp_raw / BMI_TEMP_LSB + BMI_TEMP_OFFSET;
    BMI_instance->TempUpdateFlag = 1;
    return;
  }
  
  BMI_instance->data.acc_x = (int16_t)((acc_data.raw[3] << 8) | acc_data.raw[2]) / BMI_ACC_LSB;
  BMI_instance->data.acc_y = (int16_t)((acc_data.raw[5] << 8) | acc_data.raw[4]) / BMI_ACC_LSB;
  BMI_instance->data.acc_z = (int16_t)((acc_data.raw[7] << 8) | acc_data.raw[6]) / BMI_ACC_LSB;
  
}


//陀螺仪回调
static void BMI_gyroCallback(void)
{
  BMI_instance->RxBusyFlag = 0;
  spi_data_s gyro_data = spi_getData(BMI_instance->GYRO_ID);
  
  BMI_instance->data.gyro_x = (int16_t)((gyro_data.raw[2] << 8) | gyro_data.raw[1]) / BMI_GYRO_LSB;
  BMI_instance->data.gyro_y = (int16_t)((gyro_data.raw[4] << 8) | gyro_data.raw[3]) / BMI_GYRO_LSB;
  BMI_instance->data.gyro_z = (int16_t)((gyro_data.raw[6] << 8) | gyro_data.raw[5]) / BMI_GYRO_LSB;
}

/******************************外部调用******************************/
//初始化
BMI_status_e BMI_init(BMI_config_s *config)
{
  BMI_instance = (BMI_instance_s *)malloc(sizeof(BMI_instance_s));
  memset(BMI_instance,0,sizeof(BMI_instance_s));
  
  BMI_instance->SPIx              = config->SPIx;            
  BMI_instance->ACC_ID            = config->ACC_ID;          
  BMI_instance->GYRO_ID           = config->GYRO_ID;         
  BMI_instance->ACC_CS_GPIOx      = config->ACC_CS_GPIOx;    
  BMI_instance->ACC_CS_GPIO_PIN   = config->ACC_CS_GPIO_PIN; 
  BMI_instance->GYRO_CS_GPIOx     = config->GYRO_CS_GPIOx;  
  BMI_instance->GYRO_CS_GPIO_PIN  = config->GYRO_CS_GPIO_PIN;
  
  //初始化加速度计spi
  spi_config_s spi_acc;
  spi_acc.ID          = BMI_instance->ACC_ID;
  spi_acc.CS_GPIOx    = BMI_instance->ACC_CS_GPIOx;
  spi_acc.CS_GPIO_PIN = BMI_instance->ACC_CS_GPIO_PIN;
  spi_acc.rx_len      = 8;
  spi_acc.SPIx        = BMI_instance->SPIx;
  spi_acc.callback    = BMI_accCallback;
  
  //初始化陀螺仪spi
  spi_config_s spi_gyro; 
  spi_gyro.ID          = BMI_instance->GYRO_ID;
  spi_gyro.CS_GPIOx    = BMI_instance->GYRO_CS_GPIOx;
  spi_gyro.CS_GPIO_PIN = BMI_instance->GYRO_CS_GPIO_PIN;
  spi_gyro.rx_len      = 7;
  spi_gyro.SPIx        = BMI_instance->SPIx;
  spi_gyro.callback    = BMI_gyroCallback;
  
  
  //线程守护
//  deamon_config_s deamon_bmi;
//  deamon_bmi.id = deamon_module_BMI | 0;
//  deamon_bmi.owner_id = NULL;
//  deamon_bmi.reload_count = 200;
//  deamon_bmi.deamon_callback = BMI_deamonCallback;
//  deamon_init(&deamon_bmi);
  
  
  
  if(spi_init(&spi_gyro) != spi_ok)
    return BMI_error;
  if(spi_init(&spi_acc) != spi_ok)
    return BMI_error;

  //自检
  BMI_rxReg(BMI_instance->ACC_ID,0x00);
  HAL_Delay(1);
  
  //配置加速度计参数
  BMI_txReg(BMI_instance->ACC_ID,0x7D,0x04);
  HAL_Delay(1);
  
  BMI_txReg(BMI_instance->ACC_ID,0x40,0xAB);//频率
  HAL_Delay(1);
  BMI_txReg(BMI_instance->ACC_ID,0x41,0x03);//量程24g
  HAL_Delay(1);
  
  //配置陀螺仪参数
  BMI_txReg(BMI_instance->GYRO_ID,0x0F,0x00);//量程2000dps
  HAL_Delay(1);
  BMI_txReg(BMI_instance->GYRO_ID,0x10,0x03);//频率 400Hz/47Hz
  HAL_Delay(1);
  
  return BMI_ok;
}

//读取数据
BMI_status_e BMI_work(void)
{
  static uint8_t seq = 0;//轮换读取
  static uint16_t temp_count = BMI_TEMP_DIV - 1;//温度分频计数，首次调用即采集一次温度
  if(BMI_instance->RxBusyFlag == 0)
  {
    //温度变化慢，分频采集，避免挤占加速度计与陀螺仪的刷新率
    if(++temp_count >= BMI_TEMP_DIV)
    {
      temp_count = 0;
      //读取温度数据
      BMI_rxTemp();
    }
    else
    {
      seq ^= 1;
      if(seq)
        //读取加速度计数据
        BMI_rxReg(BMI_instance->ACC_ID, 0x12);
      else
        //读取陀螺仪数据
        BMI_rxReg(BMI_instance->GYRO_ID, 0x02);
    }
    
    return BMI_ok;
  }
  else
    return BMI_busy;
 
}

//数据接口
BMI_data_s BMI_getData(void)
{
  return BMI_instance->data;
}

//温度更新接口（读取后清零）
uint8_t BMI_getTempUpdate(void)
{
  if(BMI_instance->TempUpdateFlag == 0)
    return 0;
  
  BMI_instance->TempUpdateFlag = 0;
  return 1;
}


