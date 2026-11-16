#include "mahony.h"

/*************************私有函数及变量*************************/
#define MAHONY_DEG2RAD  0.0174532925f//π/180，角速度 °/s 换成 rad/s
#define MAHONY_RAD2DEG  57.2957795f//180/π，欧拉角 rad 换成 deg

typedef struct
{
  float kp;
  float ki;
  float dt;
  
  float q0;//四元数 w
  float q1;//四元数 x
  float q2;//四元数 y
  float q3;//四元数 z
  
  float integral_x;//x轴零偏积分
  float integral_y;//y轴零偏积分
  float integral_z;//z轴零偏积分
  
  mahony_data_s data;
}mahony_instance_s;

mahony_instance_s *mahony[MAHONY_CNT] = {NULL};
static uint8_t idx = 0;


//四元数归一化
static void mahony_normalize(mahony_instance_s *instance)
{
  float norm = sqrtf(instance->q0*instance->q0 + instance->q1*instance->q1 + instance->q2*instance->q2 + instance->q3*instance->q3);
  
  //长度异常时退回单位四元数
  if(norm < 0.000001f)
  {
    instance->q0 = 1.0f;
    instance->q1 = 0.0f;
    instance->q2 = 0.0f;
    instance->q3 = 0.0f;
    return;
  }
  
  norm = 1.0f / norm;
  instance->q0 *= norm;
  instance->q1 *= norm;
  instance->q2 *= norm;
  instance->q3 *= norm;
}


/******************************外部调用******************************/
//初始化
uint8_t mahony_init(mahony_config_s *config)
{
  mahony[idx] = (mahony_instance_s *)malloc(sizeof(mahony_instance_s));
  memset(mahony[idx],0,sizeof(mahony_instance_s));
  
  mahony[idx]->kp = config->kp;
  mahony[idx]->ki = config->ki;
  mahony[idx]->dt = config->dt;
  
  //四元数初值为单位四元数（姿态角为0）
  mahony[idx]->q0 = 1.0f;
  mahony[idx]->data.q0 = 1.0f;
  
  idx++;
  
  return idx-1;
}

//更新
mahony_status_e mahony_update(uint8_t id,float gx,float gy,float gz,float ax,float ay,float az)
{
  if(id >= MAHONY_CNT || mahony[id] == NULL)
    return mahony_error;
  
  mahony_instance_s *instance = mahony[id];
  
  //陀螺仪 °/s -> rad/s
  gx *= MAHONY_DEG2RAD;
  gy *= MAHONY_DEG2RAD;
  gz *= MAHONY_DEG2RAD;
  
  //加速度计归一化，得到测量重力方向；模长过小（自由落体/异常）时本次只积分陀螺仪
  float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
  if(acc_norm > 0.1f)
  {
    ax /= acc_norm;
    ay /= acc_norm;
    az /= acc_norm;
    
    //估计的重力方向（四元数旋转矩阵第三列）
    float vx = 2.0f*(instance->q1*instance->q3 - instance->q0*instance->q2);
    float vy = 2.0f*(instance->q0*instance->q1 + instance->q2*instance->q3);
    float vz = instance->q0*instance->q0 - instance->q1*instance->q1 - instance->q2*instance->q2 + instance->q3*instance->q3;
    
    //误差 = 测量重力方向 × 估计重力方向
    float ex = ay*vz - az*vy;
    float ey = az*vx - ax*vz;
    float ez = ax*vy - ay*vx;
    
    //零偏积分修正
    if(instance->ki > 0)
    {
      instance->integral_x += instance->ki*ex*instance->dt;
      instance->integral_y += instance->ki*ey*instance->dt;
      instance->integral_z += instance->ki*ez*instance->dt;
      
      gx += instance->integral_x;
      gy += instance->integral_y;
      gz += instance->integral_z;
    }
    
    //比例修正
    gx += instance->kp*ex;
    gy += instance->kp*ey;
    gz += instance->kp*ez;
  }
  
  //四元数积分 q = q + 0.5*q*ω*dt
  float half_dt = 0.5f*instance->dt;
  float q0 = instance->q0;
  float q1 = instance->q1;
  float q2 = instance->q2;
  float q3 = instance->q3;
  
  instance->q0 += (-q1*gx - q2*gy - q3*gz)*half_dt;
  instance->q1 += ( q0*gx + q2*gz - q3*gy)*half_dt;
  instance->q2 += ( q0*gy - q1*gz + q3*gx)*half_dt;
  instance->q3 += ( q0*gz + q1*gy - q2*gx)*half_dt;
  
  mahony_normalize(instance);
  
  //四元数输出
  instance->data.q0 = instance->q0;
  instance->data.q1 = instance->q1;
  instance->data.q2 = instance->q2;
  instance->data.q3 = instance->q3;
  
  //欧拉角（ZYX）
  instance->data.roll = atan2f(2.0f*(instance->q0*instance->q1 + instance->q2*instance->q3),
                               1.0f - 2.0f*(instance->q1*instance->q1 + instance->q2*instance->q2)) * MAHONY_RAD2DEG;
  
  float sin_pitch = 2.0f*(instance->q0*instance->q2 - instance->q3*instance->q1);
  if(sin_pitch > 1.0f)
    sin_pitch = 1.0f;
  else if(sin_pitch < -1.0f)
    sin_pitch = -1.0f;
  instance->data.pitch = asinf(sin_pitch) * MAHONY_RAD2DEG;
  
  instance->data.yaw = atan2f(2.0f*(instance->q0*instance->q3 + instance->q1*instance->q2),
                              1.0f - 2.0f*(instance->q2*instance->q2 + instance->q3*instance->q3)) * MAHONY_RAD2DEG;
  
  return mahony_ok;
}

//数据接口
mahony_data_s mahony_getData(uint8_t id)
{
  mahony_data_s data =
  {
    .q0 = 1.0f,
    .q1 = 0.0f,
    .q2 = 0.0f,
    .q3 = 0.0f,
    .roll = 0.0f,
    .pitch = 0.0f,
    .yaw = 0.0f,
  };
  
  if(id < MAHONY_CNT && mahony[id] != NULL)
    data = mahony[id]->data;
  
  return data;
}
