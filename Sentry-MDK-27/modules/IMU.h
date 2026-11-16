/** @file   IMU.h
 *  @brief  姿态解算与 BMI088 温控接口：姿态滤波可选三轴卡尔曼或 6 轴 Mahony，同时用加热电阻把芯片温度稳定在目标值。
 *  @note   调用顺序 BMI_init() -> IMU_init() -> 周期 IMU_work()（周期须等于 config.dt）；不要在中断中调用。
 */

#ifndef __IMU_H
#define __IMU_H

#include "BMI088.h"
#include "bsp_pwm.h"
#include "mahony.h"
#include "pid.h"
#include "math.h"
#include "kalman_filter.h"

/********************宏**********************/
#define IMU_PI    3.1415926f    ///< 角度换算用的 π 近似值（rad × 180 / IMU_PI = deg）
#define IMU_HEAT_TIMEOUT  500   ///< 温度更新超时阈值：超过该次数未更新温度则关闭加热，单位：一次 IMU_work 调用（本工程 1ms）

#define IMU_CALIB_TEMP_TOLERANCE  1.0f     ///< 等温判定：芯片温度与 target_temp 的误差小于该值视为到温，单位 ℃
#define IMU_CALIB_SETTLE_TIME     500      ///< 到温后继续稳定该时间再采样，单位：一次 IMU_work 调用（本工程约 1ms）
#define IMU_CALIB_SAMPLE_TIME     1000     ///< 陀螺零偏采样窗口长度，单位：一次 IMU_work 调用（本工程约 1ms）
#define IMU_CALIB_GYRO_RANGE      2.0f     ///< 静止判定：采样窗口内三轴 gyro 峰峰值阈值，超过视为未静止，单位 °/s
#define IMU_CALIB_ACC_MIN         0.8f     ///< 静止判定：加速度模长下限，单位 g
#define IMU_CALIB_ACC_MAX         1.2f     ///< 静止判定：加速度模长上限，单位 g
#define IMU_CALIB_HEAT_TIMEOUT    600000   ///< 等温超时（单位：一次 IMU_work 调用，约 10min）：超时后用当前温度校准并继续，结果标记 valid=0
#define IMU_CALIB_MOTION_TIMEOUT  10000    ///< 采样因运动反复重试的总时限（单位：一次 IMU_work 调用，约 10s）


/******************枚举*********************/
/** @brief 姿态滤波算法选择（IMU_config_s.filter_type 取值） */
typedef enum
{
  IMU_filter_kalman,              ///< 0：三轴卡尔曼（roll/pitch 双观测 + yaw 单观测，本工程默认）
  IMU_filter_mahony,              ///< 1：6 轴 Mahony（加速度计重力修正 + 四元数积分）
}IMU_filter_e;


/******************结构体*********************/
/** @brief IMU 初始化配置（本工程在 robot_init() 中以指定初始化方式赋值） */
typedef struct
{
  float alpha;                    ///< 历史遗留的互补滤波系数，IMU_work() 未使用
  float dt;                       ///< 姿态解算周期，单位 s（本工程 0.001f，对应 imuTask 的 1ms）
  kalman_config2_s kal_roll;      ///< roll 轴卡尔曼参数（加速度计角度 + 陀螺仪角速度双观测）
  kalman_config2_s kal_pitch;     ///< pitch 轴卡尔曼参数（双观测）
  kalman_config1_s kal_yaw;       ///< yaw 轴卡尔曼参数（单观测：陀螺仪 z 轴角速度）
  float yaw_compensate;           ///< yaw 漂移补偿，单位 deg/次（每调用一次扣除，本工程 0.000041f）
  
  IMU_filter_e filter_type;       ///< 姿态滤波算法：IMU_filter_kalman=三轴卡尔曼（默认）；IMU_filter_mahony=6 轴 Mahony
  mahony_config_s mahony;         ///< Mahony 参数（filter_type 为 IMU_filter_mahony 时使用；dt 须与 IMU 控制周期一致）
  
  pwm_config_s heater_pwm;        ///< 加热电阻 PWM 配置（本工程 TIM10 CH1 → PF6）
  float target_temp;              ///< 温控目标温度，单位 ℃（本工程 40℃）
  pid_instance_s pid_heat;        ///< 温控 PID 参数：反馈为 BMI088 温度 ℃，输出为占空比 0~1，每约 100ms 计算一次
}IMU_config_s;

/** @brief IMU 解算输出数据（角度 deg、角速度 rpm、加速度 g、温度 ℃，按值返回） */
typedef struct
{
  float roll;                     ///< 加速度计解算横滚角，单位 deg
  float pitch;                    ///< 加速度计解算俯仰角，单位 deg
  float yaw;                      ///< 陀螺仪 z 轴积分角，单位 deg（未做 rpm→deg 换算，为真实角度的 1/6）
  float gyro_x;                   ///< X 轴角速度，单位 rpm（= BMI088 的 °/s ÷ 6）
  float gyro_y;                   ///< Y 轴角速度，单位 rpm
  float gyro_z;                   ///< Z 轴角速度，单位 rpm
  float acc_x;                    ///< X 轴加速度，单位 g（±24g）
  float acc_y;                    ///< Y 轴加速度，单位 g（±24g）
  float acc_z;                    ///< Z 轴加速度，单位 g（±24g）
  float temp;                     ///< BMI088 芯片温度，单位 ℃（约 100ms 更新一次）
  float filter_roll;              ///< 姿态滤波后的 roll，单位 deg（卡尔曼或 Mahony）
  float filter_pitch;             ///< 姿态滤波后的 pitch，单位 deg（卡尔曼或 Mahony）
  float filter_yaw;               ///< 姿态滤波后的 yaw，单位 deg（连续角，已扣 yaw 零点并叠加 IMU_setYawOffset() 的修正量）
  float heat_duty;                ///< 加热电阻 PWM 占空比 0~1（温控输出，0 表示不加热）
}IMU_data_s;


/** @brief IMU 上电校准状态（IMU_getCalibState() 返回值） */
typedef enum
{
  IMU_calib_heating,              ///< 0：等待芯片温度到达温控目标
  IMU_calib_settling,             ///< 1：温度已到，等待稳定时间
  IMU_calib_collecting,           ///< 2：采集陀螺仪零偏
  IMU_calib_done,                 ///< 3：校准完成
}IMU_calibState_e;

/** @brief IMU 上电校准结果（IMU_getCalibData() 按值返回） */
typedef struct
{
  float gyro_bias_x;              ///< X 轴陀螺零偏，单位 °/s（校准完成后 IMU_work() 已扣除）
  float gyro_bias_y;              ///< Y 轴陀螺零偏，单位 °/s
  float gyro_bias_z;              ///< Z 轴陀螺零偏，单位 °/s
  float temp;                     ///< 校准时的芯片温度，单位 ℃（用于确认是否热校准）
  uint8_t valid;                  ///< 1=到温且采样静止；0=等温超时/运动中完成，结果仅供参考
}IMU_calibData_s;


/** @brief IMU 模块状态 */
typedef enum
{
  IMU_ok,                         ///< 0：正常
  IMU_error,                      ///< 1：错误（BMI_work() 未返回 BMI_ok，或某滤波器 kalman_work() 失败）
}IMU_status_e;

/*****************外部调用**********************/
/**
 * @brief  初始化 IMU 姿态解算、温控与上电校准：保存参数，按 filter_type 注册卡尔曼或 Mahony，并初始化加热电阻 PWM
 * @param  config 配置（滤波算法/Mahony 参数、yaw 补偿、加热 PWM 与 PID 参数等），不能为 NULL
 * @return IMU_ok=初始化完成（当前实现恒返回 IMU_ok）
 * @note   须先调用 BMI_init()；只初始化 config.filter_type 选中的算法（卡尔曼会占满 kalman 模块 3 个实例槽位）；
 *         加热 PWM 初始化后占空比为 0，上电不加热；上电校准在 IMU_work() 中推进（先等温到 target_temp 再采样）
 */
IMU_status_e IMU_init(IMU_config_s *config);

/**
 * @brief  周期姿态解算、温控与上电校准：采样 BMI088，按选择的算法更新姿态角，控温，并在校准阶段采集陀螺零偏
 * @return IMU_ok=本周期解算完成且数据已更新；IMU_error=本周期未完成，姿态数据不更新
 * @note   须在 IMU_init() 之后按 dt 周期调用（本工程 imuTask 约 1ms）；含 static 计数器，不可重入；
 *         温控在温度更新（约 100ms）时计算一次，温度超过 IMU_HEAT_TIMEOUT 未更新则自动关闭加热；
 *         校准完成前 IMU_isCalibrated()=0，cmd 应保持失能；完成后扣除陀螺零偏，yaw 输出在校准时刻归零
 */
IMU_status_e IMU_work(void);

/**
 * @brief  读取最近一次解算结果（姿态角 deg、角速度 rpm、加速度 g、芯片温度 ℃ 与加热占空比）
 * @return IMU_data_s 数据副本，按值返回
 * @note   须在 IMU_init() 且至少执行过一次 IMU_work() 之后调用；跨任务读取未加临界区
 */
IMU_data_s IMU_getData(void);

/**
 * @brief  设置 yaw 修正量：IMU_work() 每个周期把它叠加到 filter_yaw 输出上，外部读到的 filter_yaw 即为修正后的角度
 * @param  offset 修正量，单位 deg（与 filter_yaw 同向累加，0 表示不修正）
 * @note   用于把云台机械中位映射为 filter_yaw=0：外部读 yaw 电机编码器后传入“编码器角-中位角”；
 *         可随时调用，下一周期 IMU_work() 生效；本工程由 cmd_work() 在校准完成后写入
 */
void IMU_setYawOffset(float offset);

/**
 * @brief  查询上电校准是否完成
 * @return 1=校准完成（陀螺零偏已扣除、yaw 零点已记录）；0=校准中（等温/稳定/采样）
 * @note   校准完成前不应使能底盘/云台；本工程由 cmd_work() 在校准期间强制全部执行机构失能实现
 */
uint8_t IMU_isCalibrated(void);

/**
 * @brief  查询上电校准状态（主要用于串口打印/调试）
 * @return IMU_calib_heating=等待到温；IMU_calib_settling=到温后稳定；IMU_calib_collecting=采样；IMU_calib_done=完成
 */
IMU_calibState_e IMU_getCalibState(void);

/**
 * @brief  读取上电校准结果（陀螺零偏、校准时温度、有效标志）
 * @return IMU_calibData_s 数据副本，按值返回；校准完成前为零
 * @note   yaw 机械中位与编码器的换算由外部（cmd）完成后经 IMU_setYawOffset() 写入，IMU 内不读编码器
 */
IMU_calibData_s IMU_getCalibData(void);

#endif
