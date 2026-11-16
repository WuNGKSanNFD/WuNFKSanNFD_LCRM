#include "robotTask.h"

/************************初始化****************************/
void robot_init(void)
{
  //remote
  remote_init();
 
  
  //串口调试初始化
  serial_config_s serial_p;
  serial_p.mode = serial_mode_send;
  serial_p.huart = &huart6;
  serial_p.rx_DLC = 0;
  Serial_init(&serial_p);
    
  //bmi初始化
  BMI_config_s BMI;
  BMI.SPIx = &hspi1;
  BMI.ACC_ID = 1;
  BMI.GYRO_ID = 2;
  BMI.ACC_CS_GPIOx = GPIOA;
  BMI.ACC_CS_GPIO_PIN = GPIO_PIN_4;
  BMI.GYRO_CS_GPIOx = GPIOB;
  BMI.GYRO_CS_GPIO_PIN = GPIO_PIN_0;
  
  BMI_init(&BMI);
  
  //imu初始化
  IMU_config_s IMU = {
    .dt = 0.001f,
    .alpha = 0,
    .kal_roll.A = {1,0.001f,0,1},
    .kal_roll.B = {0,0},
    .kal_roll.H = {1,0,0,1},
    .kal_roll.R = {0.0329769f, 0.0f, 0.0f, 0.00782716f},
    .kal_roll.Q = {3.33333e-7f, 0.0005f, 0.0005f, 1.0f},
    
    .kal_pitch.A = {1,0.001f,0,1},
    .kal_pitch.B = {0,0},
    .kal_pitch.H = {1,0,0,1},
    .kal_pitch.R = {0.035019f, -0.000263f,  -0.000263f, 0.013151f},
    .kal_pitch.Q = {3.33333e-06f, 0.005f, 0.005f, 10.0f},
    
    .kal_yaw.A = {1,0.0011f,0,1},
    .kal_yaw.B = {0,0},
    .kal_yaw.H = {0,1},
    .kal_yaw.R = {0.008364f},
    .kal_yaw.Q = {3.33333e-06f, 0.005f, 0.005f, 10.0f},
    
    .yaw_compensate = 0.000009f,
    
    //姿态滤波算法：IMU_filter_kalman（默认，已整定）/ IMU_filter_mahony（6轴Mahony）
    .filter_type = IMU_filter_mahony,
    //Mahony参数（filter_type 为 IMU_filter_mahony 时使用）
    .mahony = {
      .kp = 1.0f,//重力修正比例增益
      .ki = 0.0f,//零偏积分增益（0=关闭）
      .dt = 0.00114f,//更新周期，与IMU控制周期一致
    },
    
    //yaw机械中位与编码器的修正由 cmd 在校准完成后经 IMU_setYawOffset() 写入（须与 gimbal.initYawAngle 一致）
    
    //加热电阻PWM（TIM10 CH1 -> PF6）
    .heater_pwm = {
      .id = 0,
      .htim = &htim10,
      .Channel = TIM_CHANNEL_1,
      .pwm_moduleCallback = NULL,
    },
    .target_temp = 40.0f,//温控目标温度
    
    //温控PID参数
    .pid_heat = {
      .kp = 0.20f,//1600/5000
      .ki = 0.0032f,//0.2×800Hz×100ms/5000
      .kd = 0,//示例 Kd=0
      .max_i = 0.88f,//4400/5000
      .maxout = 0.9f,//4500/5000
      .dead = 0,//死区
      .pid_mode = pid_mode_normal,//普通位置式PID，与示例一致
      .epsilon = 0,//积分分离参数（普通模式不使用）
      .A = 0,//变速积分参数
      .B = 0,
      .d_f_alpha = 0,//不完全微分参数
      .input_f_alpha = 0,//输入滤波参数
      .output_f_alpha = 0,//输出滤波参数
    },
  };
  IMU_init(&IMU);
  
  //视觉通信
  vision_init();
  
  //cmd初始化
  cmd_init();
  
  //底盘初始化（实例参数在 chassis_init() 内赋值）
  chassis_init();
  
  //云台
  gimbal_init();
  
  //发射机构
  shoot_init();
 
}


/************************底盘****************************/
void chassisTask(void *argument)
{
  
  for(;;)
  {
    chassis_work();
    
    osDelay(5);
  }
}

/************************模块监测****************************/
void deamonTask(void *argument)
{
  for(;;)
  {
    deamonStart();
    
    osDelay(10);
  }
}


/************************电机****************************/
void motorTask(void *argument)
{
  for(;;)
  {   
    DJI_Motor_work();//内部已完成位置环/速度环计算与发帧
    //大 yaw 关节的闭环方式由注册参数决定，需切换时调用 DM_Motor_setPidMode()
    DM_Motor_pidWork();
    osDelay(2);
  }
}

/************************云台****************************/
void gimbalTask(void *argument)
{
  for(;;)
  {
    gimbal_work();
//    //cmd 的底盘跟随环需要与云台编码器同周期（2ms），在此按 2ms 节拍驱动
    osDelay(2);
  }

}

/************************CMD****************************/
void cmdTask(void *argument)
{
  for(;;)
  {
    cmd_work();
    osDelay(4);
  }
}

/************************IMU****************************/
void imuTask(void *argument)
{
  for(;;)
  {
    IMU_work();
    osDelay(1);
  }
}

/*************************发射机构****************************/
void shootTask(void *argument)
{
  for(;;)
  {
    shoot_work();
    osDelay(5);
  }
}


/************************串口****************************/
void serialTask(void *argument)
{
  
  DJI_motor_rxData_s chassis_1;
  DJI_motor_rxData_s chassis_2;
  DJI_motor_rxData_s chassis_3;
  DJI_motor_rxData_s chassis_4;
  
  for(;;)
  { 
    //底盘
    DJI_motor_motorID_s chassis_1_ID   = {DJI_M3508,1,&hcan1};
    DJI_motor_motorID_s chassis_2_ID   = {DJI_M3508,2,&hcan1};
    DJI_motor_motorID_s chassis_3_ID   = {DJI_M3508,3,&hcan1};
    DJI_motor_motorID_s chassis_4_ID   = {DJI_M3508,4,&hcan1};
    DJI_motor_getData(chassis_1_ID,&chassis_1);
    DJI_motor_getData(chassis_2_ID,&chassis_2);
    DJI_motor_getData(chassis_3_ID,&chassis_3);
    DJI_motor_getData(chassis_4_ID,&chassis_4);
    chassis_data_s chassis = chassis_getData();
    
    Serial_Printf(&huart6,"%.6f,%.6f\n",chassis_2.speed, chassis.chassis_speed[1]);



   
    
    osDelay(5);
  }
}

