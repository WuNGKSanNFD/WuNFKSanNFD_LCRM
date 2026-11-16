#include "IMU.h"

/**************************私有函数及变量***************************/
typedef struct
{
  float alpha;
  float dt;
  kalman_config2_s kal_roll;
  kalman_config2_s kal_pitch;
  kalman_config1_s kal_yaw;
  uint8_t kalman_id[3];//0为roll,pitch为1,yaw为2
  IMU_filter_e filter_type;//姿态滤波算法：卡尔曼/Mahony
  mahony_config_s mahony;//Mahony参数
  uint8_t mahony_id;//Mahony实例id
  float yaw_compensate;//yaw轴漂移补偿
  
  IMU_calibState_e calib_state;//上电校准状态
  uint8_t calib_temp_ok;//1=芯片温度已到温控目标
  uint32_t calib_timer;//状态计时（一次IMU_work调用）
  uint32_t calib_motion_timer;//采样重试计时
  uint32_t calib_count;//采样计数
  float gyro_sum[3];//零偏累加
  float gyro_min[3];//采样窗口最小值
  float gyro_max[3];//采样窗口最大值
  IMU_calibData_s calib;//校准结果
  float yaw_zero;//yaw输出零点（校准完成时记录）
  float yaw_offset;//yaw修正量（外部写入，叠加到 filter_yaw 输出）
  uint8_t calib_zero_pending;//校准完成标志：本周期滤波输出后记录yaw零点
  
  uint8_t heater_pwm_ID;//加热电阻PWM实例编号
  float target_temp;//温控目标温度，单位℃
  pid_instance_s pid_heat;//温控PID
  
  IMU_data_s data;
}IMU_instance_s;
IMU_instance_s *IMU_instance = NULL;


//温控计算：芯片温度 -> 加热占空比
static void IMU_heatCalculate(float temperature)
{
  if(pid_calculate(IMU_instance->target_temp, temperature, &IMU_instance->pid_heat) != pid_ok)
    return;
  
  //pid 模块只限幅 i_out、不限幅积分累计值；这里按示例 PID（LimitMax(Iout, max_iout)）的做法
  //把积分状态本身限住，防止升温阶段积分饱和导致超调
  if(IMU_instance->pid_heat.ki > 0)
  {
    float i_term_max = IMU_instance->pid_heat.max_i / IMU_instance->pid_heat.ki;
    if(IMU_instance->pid_heat.i_term > i_term_max)
      IMU_instance->pid_heat.i_term = i_term_max;
    else if(IMU_instance->pid_heat.i_term < -i_term_max)
      IMU_instance->pid_heat.i_term = -i_term_max;
  }
  
  //加热只能单向输出，负值（温度高于目标）截断为0
  float duty = IMU_instance->pid_heat.output;
  if(duty < 0)
    duty = 0;
  
  IMU_instance->data.heat_duty = duty;
  pwm_dutyCycle(IMU_instance->heater_pwm_ID, duty);
}


//温控工作：温度每更新一次计算一次，长时间未更新则关闭加热
static void IMU_heatWork(void)
{
  static uint16_t timeout_count = 0;//温度更新超时计数
  
  if(BMI_getTempUpdate())
  {
    timeout_count = 0;
    IMU_heatCalculate(IMU_instance->data.temp);
  }
  else if(timeout_count < IMU_HEAT_TIMEOUT)
  {
    timeout_count++;
  }
  else if(IMU_instance->data.heat_duty != 0)
  {
    //温度长时间未更新（如SPI异常），关闭加热，防止持续加热失控
    IMU_instance->data.heat_duty = 0;
    pwm_dutyCycle(IMU_instance->heater_pwm_ID, 0);
  }
}


//上电校准：开始一轮零偏采样（清空累加）
static void IMU_calibResetCollect(void)
{
  IMU_instance->calib_state = IMU_calib_collecting;
  IMU_instance->calib_count = 0;
  IMU_instance->calib_timer = 0;
  IMU_instance->calib_motion_timer = 0;
  IMU_instance->gyro_sum[0] = 0.0f;
  IMU_instance->gyro_sum[1] = 0.0f;
  IMU_instance->gyro_sum[2] = 0.0f;
}


//上电校准：采样结束，计算零偏并结束校准
static void IMU_calibFinish(uint8_t steady)
{
  //陀螺零偏平均值
  if(IMU_instance->calib_count > 0)
  {
    IMU_instance->calib.gyro_bias_x = IMU_instance->gyro_sum[0] / IMU_instance->calib_count;
    IMU_instance->calib.gyro_bias_y = IMU_instance->gyro_sum[1] / IMU_instance->calib_count;
    IMU_instance->calib.gyro_bias_z = IMU_instance->gyro_sum[2] / IMU_instance->calib_count;
  }
  
  //到温且采样时静止，才认为校准有效
  if(IMU_instance->calib_temp_ok == 1 && steady == 1)
    IMU_instance->calib.valid = 1;
  else
    IMU_instance->calib.valid = 0;
  
  IMU_instance->calib_state = IMU_calib_done;
  IMU_instance->calib_zero_pending = 1;
}


//上电校准状态机：等温 -> 稳定 -> 采集陀螺零偏 -> 完成
static void IMU_calibWork(BMI_data_s *bmi)
{
  float gyro[3] = {bmi->gyro_x, bmi->gyro_y, bmi->gyro_z};
  
  switch(IMU_instance->calib_state)
  {
    //等待芯片温度到达温控目标
    case IMU_calib_heating:
      if(fabsf(IMU_instance->data.temp - IMU_instance->target_temp) <= IMU_CALIB_TEMP_TOLERANCE)
      {
        IMU_instance->calib_temp_ok = 1;
        IMU_instance->calib_state = IMU_calib_settling;
        IMU_instance->calib_timer = 0;
      }
      else if(++IMU_instance->calib_timer >= IMU_CALIB_HEAT_TIMEOUT)
      {
        //等温超时：用当前温度继续校准，结果标记为不可信
        IMU_instance->calib_temp_ok = 0;
        IMU_instance->calib.temp = IMU_instance->data.temp;
        IMU_calibResetCollect();
      }
      break;
      
    //到温后再稳定一段时间，避免温控仍在调节
    case IMU_calib_settling:
      if(fabsf(IMU_instance->data.temp - IMU_instance->target_temp) > IMU_CALIB_TEMP_TOLERANCE)
      {
        IMU_instance->calib_temp_ok = 0;
        IMU_instance->calib_state = IMU_calib_heating;
        IMU_instance->calib_timer = 0;
      }
      else if(++IMU_instance->calib_timer >= IMU_CALIB_SETTLE_TIME)
      {
        IMU_instance->calib.temp = IMU_instance->data.temp;
        IMU_calibResetCollect();
      }
      break;
      
    //采集陀螺零偏
    case IMU_calib_collecting:
      //窗口内三轴极值
      if(IMU_instance->calib_count == 0)
      {
        for(uint8_t i = 0; i < 3; i++)
        {
          IMU_instance->gyro_min[i] = gyro[i];
          IMU_instance->gyro_max[i] = gyro[i];
        }
      }
      else
      {
        for(uint8_t i = 0; i < 3; i++)
        {
          if(gyro[i] < IMU_instance->gyro_min[i])
            IMU_instance->gyro_min[i] = gyro[i];
          if(gyro[i] > IMU_instance->gyro_max[i])
            IMU_instance->gyro_max[i] = gyro[i];
        }
      }
      
      IMU_instance->gyro_sum[0] += gyro[0];
      IMU_instance->gyro_sum[1] += gyro[1];
      IMU_instance->gyro_sum[2] += gyro[2];
      IMU_instance->calib_count++;
      IMU_instance->calib_motion_timer++;
      
      if(IMU_instance->calib_count >= IMU_CALIB_SAMPLE_TIME)
      {
        //静止判断：三轴陀螺峰峰值 + 加速度模长
        float acc_norm = sqrtf(bmi->acc_x*bmi->acc_x + bmi->acc_y*bmi->acc_y + bmi->acc_z*bmi->acc_z);
        uint8_t steady = 1;
        for(uint8_t i = 0; i < 3; i++)
        {
          if(IMU_instance->gyro_max[i] - IMU_instance->gyro_min[i] > IMU_CALIB_GYRO_RANGE)
            steady = 0;
        }
        if(acc_norm < IMU_CALIB_ACC_MIN || acc_norm > IMU_CALIB_ACC_MAX)
          steady = 0;
        
        //静止或重试超时：结束采样
        if(steady == 1 || IMU_instance->calib_motion_timer >= IMU_CALIB_MOTION_TIMEOUT)
        {
          IMU_calibFinish(steady);
        }
        else
        {
          //有运动，丢弃本窗口重新采样
          IMU_instance->calib_count = 0;
          IMU_instance->gyro_sum[0] = 0.0f;
          IMU_instance->gyro_sum[1] = 0.0f;
          IMU_instance->gyro_sum[2] = 0.0f;
        }
      }
      break;
      
    default:
      break;
  }
}


//卡尔曼滤波（roll/pitch双观测，yaw单观测）
static IMU_status_e IMU_kalmanWork(void)
{
  //roll
  matrix_instance_s measure_roll, u_roll = {{0,},1,1};
  measure_roll.matrix[0][0] = IMU_instance->data.roll;
  measure_roll.matrix[1][0] = IMU_instance->data.gyro_x * 6.0f;
  measure_roll.row = 2;
  measure_roll.column = 1;
  kalman_setMeasure(IMU_instance->kalman_id[0], measure_roll, u_roll);
  
  //pitch
  matrix_instance_s measure_pitch, u_pitch = {{0,},1,1};
  measure_pitch.matrix[0][0] = IMU_instance->data.pitch;
  measure_pitch.matrix[1][0] = IMU_instance->data.gyro_y * 6.0f;
  measure_pitch.row = 2;
  measure_pitch.column = 1;
  kalman_setMeasure(IMU_instance->kalman_id[1], measure_pitch, u_pitch);
  
  //yaw
  matrix_instance_s measure_yaw, u_yaw = {{0,},1,1};
  measure_yaw.matrix[0][0] = IMU_instance->data.gyro_z * 6.0f;
  measure_yaw.row = 1;
  measure_yaw.column = 1;
  kalman_setMeasure(IMU_instance->kalman_id[2], measure_yaw, u_yaw);
  
  //work
  if(kalman_work(IMU_instance->kalman_id[0]) != kalman_ok)
    return IMU_error;
  if(kalman_work(IMU_instance->kalman_id[1]) != kalman_ok)
    return IMU_error;
  if(kalman_work(IMU_instance->kalman_id[2]) != kalman_ok)
    return IMU_error;
  
  kalman_data_s kalman_rollData = kalman_getData(IMU_instance->kalman_id[0]);
  kalman_data_s kalman_pitchData = kalman_getData(IMU_instance->kalman_id[1]);
  kalman_data_s kalman_yawData = kalman_getData(IMU_instance->kalman_id[2]);
  
  IMU_instance->data.filter_roll = kalman_rollData.X_out[0][0];
  IMU_instance->data.filter_pitch = kalman_pitchData.X_out[0][0];
  IMU_instance->data.filter_yaw = kalman_yawData.X_out[0][0];
  
  return IMU_ok;
}


//6轴Mahony滤波
static IMU_status_e IMU_mahonyWork(void)
{
  //陀螺仪rpm->°/s，加速度单位g
  if(mahony_update(IMU_instance->mahony_id,
                   IMU_instance->data.gyro_x * 6.0f,
                   IMU_instance->data.gyro_y * 6.0f,
                   IMU_instance->data.gyro_z * 6.0f,
                   IMU_instance->data.acc_x,
                   IMU_instance->data.acc_y,
                   IMU_instance->data.acc_z) != mahony_ok)
    return IMU_error;
  
  mahony_data_s mahony_data = mahony_getData(IMU_instance->mahony_id);
  
  //Mahony的yaw是-180~180的欧拉角，展开成连续角，与卡尔曼的yaw输出保持一致
  static float last_yaw = 0.0f;
  static float yaw_continuous = 0.0f;
  float yaw_delta = mahony_data.yaw - last_yaw;
  if(yaw_delta > 180.0f)
    yaw_delta -= 360.0f;
  else if(yaw_delta < -180.0f)
    yaw_delta += 360.0f;
  yaw_continuous += yaw_delta;
  last_yaw = mahony_data.yaw;
  
  IMU_instance->data.filter_roll  = mahony_data.roll;
  IMU_instance->data.filter_pitch = -mahony_data.pitch;//工程pitch为atan2(acc_x,acc_z)，与标准ZYX定义反向
  IMU_instance->data.filter_yaw   = yaw_continuous;
  
  return IMU_ok;
}

/***************************外部调用******************************/

//初始化
IMU_status_e IMU_init(IMU_config_s *config)
{
  IMU_instance = (IMU_instance_s *)malloc(sizeof(IMU_instance_s));
  memset(IMU_instance,0,sizeof(IMU_instance_s));
  
  IMU_instance->alpha = config->alpha;
  IMU_instance->dt = config->dt;
  IMU_instance->kal_roll = config->kal_roll;
  IMU_instance->kal_pitch = config->kal_pitch;
  IMU_instance->kal_yaw = config->kal_yaw;
  IMU_instance->yaw_compensate = config->yaw_compensate; 
  IMU_instance->filter_type = config->filter_type;
  IMU_instance->mahony = config->mahony;
  IMU_instance->target_temp = config->target_temp;
  IMU_instance->pid_heat = config->pid_heat;
  
  //上电校准：先等温控到 target_temp，再由 IMU_work() 推进
  IMU_instance->calib_state = IMU_calib_heating;
  
  //加热电阻PWM初始化（占空比先置0，上电不加热）
  pwm_config_s pwm_heater = config->heater_pwm;
  pwm_init(&pwm_heater);
  IMU_instance->heater_pwm_ID = pwm_heater.id;
  
  //姿态滤波初始化：只注册初始化时选择的算法
  if(IMU_instance->filter_type == IMU_filter_mahony)
  {
    //6轴Mahony
    IMU_instance->mahony_id = mahony_init(&IMU_instance->mahony);
  }
  else
  {
    //三轴卡尔曼
    //roll轴
    IMU_instance->kalman_id[0] = kalman_init2(&IMU_instance->kal_roll);
    //pitch轴
    IMU_instance->kalman_id[1] = kalman_init2(&IMU_instance->kal_pitch);
    //yaw轴
    IMU_instance->kalman_id[2] = kalman_init1(&IMU_instance->kal_yaw);
  }
  
  return IMU_ok;
}

//IMU工作
IMU_status_e IMU_work(void)
{
 
  //启用BMI加速度计陀螺仪
  if(BMI_work() != BMI_ok)
    return IMU_error;
  
  BMI_data_s BMI_data = BMI_getData();
  
  //芯片温度与温控（温度每更新一次计算一次PID，本工程约100ms）
  IMU_instance->data.temp = BMI_data.temp;
  IMU_heatWork();
  
  //上电校准：等温 -> 稳定 -> 采集陀螺零偏
  IMU_calibWork(&BMI_data);
  
  //传值，角速度单位°/s->rpm（扣除上电校准得到的陀螺零偏）
  IMU_instance->data.gyro_x = (BMI_data.gyro_x - IMU_instance->calib.gyro_bias_x) / 6.0f;
  IMU_instance->data.gyro_y = (BMI_data.gyro_y - IMU_instance->calib.gyro_bias_y) / 6.0f;
  IMU_instance->data.gyro_z = (BMI_data.gyro_z - IMU_instance->calib.gyro_bias_z) / 6.0f;
  IMU_instance->data.acc_x  = BMI_data.acc_x; 
  IMU_instance->data.acc_y  = BMI_data.acc_y; 
  IMU_instance->data.acc_z  = BMI_data.acc_z; 
  
  IMU_instance->data.roll   = atan2f(IMU_instance->data.acc_y,IMU_instance->data.acc_z) / IMU_PI * 180.0f;
  IMU_instance->data.pitch  = atan2f(IMU_instance->data.acc_x,IMU_instance->data.acc_z) / IMU_PI * 180.0f;
  IMU_instance->data.yaw    += IMU_instance->data.gyro_z * IMU_instance->dt;
  
  
  //姿态滤波：按初始化时选择的算法更新 filter_roll/filter_pitch/filter_yaw
  if(IMU_instance->filter_type == IMU_filter_mahony)
  {
    if(IMU_mahonyWork() != IMU_ok)
      return IMU_error;
  }
  else
  {
    if(IMU_kalmanWork() != IMU_ok)
      return IMU_error;
  }
  
  //yaw零点：上电校准完成时把当前yaw输出记为0，之后yaw从0开始（云台以该位置为初始角基准）
  if(IMU_instance->calib_zero_pending == 1)
  {
    IMU_instance->yaw_zero = IMU_instance->data.filter_yaw;
    IMU_instance->calib_zero_pending = 0;//yaw已归零后才允许 cmd 检测到校准完成
  }
  IMU_instance->data.filter_yaw -= IMU_instance->yaw_zero;
  
  //yaw修正量：由外部写入（本工程 cmd 读编码器后换算），叠加后外部读到的 filter_yaw 即为修正后的角度
  IMU_instance->data.filter_yaw += IMU_instance->yaw_offset;
  
  
  return IMU_ok;
}

//数据接口
IMU_data_s IMU_getData(void)
{
  return IMU_instance->data;
}

//yaw修正量接口：外部（本工程cmd）把编码器角-机械中位角换算后写入，IMU_work()下一周期生效
void IMU_setYawOffset(float offset)
{
  IMU_instance->yaw_offset = offset;
}

//上电校准是否完成（yaw 零点记录完成才算完成，避免云台用未归零的 yaw 使能）
uint8_t IMU_isCalibrated(void)
{
  if(IMU_instance->calib_state == IMU_calib_done && IMU_instance->calib_zero_pending == 0)
    return 1;
  
  return 0;
}

//上电校准状态
IMU_calibState_e IMU_getCalibState(void)
{
  return IMU_instance->calib_state;
}

//上电校准结果
IMU_calibData_s IMU_getCalibData(void)
{
  return IMU_instance->calib;
}

