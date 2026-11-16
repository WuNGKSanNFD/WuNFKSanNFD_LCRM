/** @file   BMI088.h
 *  @brief  BMI088 六轴 IMU 的 SPI 驱动接口：初始化、周期采样（加速度、角速度、温度）与数据读取。
 *  @note   BMI_work()/BMI_getData() 须在同一任务中调用；BMI_init() 含 HAL_Delay()，禁止在中断中调用。
 */

#ifndef __BMI088_H
#define __BMI088_H

#include "bsp_spi.h"
#include "deamon.h"

/*****************************宏********************************/
#define BMI_ACC_LSB   1365.0f   ///< 加速度计灵敏度，单位 LSB/g（量程 ±24g）
#define BMI_GYRO_LSB  16.384f   ///< 陀螺仪灵敏度，单位 LSB/(°/s)（量程 ±2000dps）
#define BMI_TEMP_LSB  8.0f      ///< 温度计灵敏度，单位 LSB/℃（温度数据为 11 位，即 0.125℃/LSB）
#define BMI_TEMP_OFFSET 23.0f   ///< 温度零点偏置，单位 ℃（原始值为 0 时对应 23℃）
#define BMI_TEMP_ADDR 0x22      ///< 温度数据起始寄存器，0x22 为 MSB、0x23 为 LSB（温度传感器在加速度计内）
#define BMI_TEMP_DIV  100       ///< 温度采集分频：BMI_work() 每调用 100 次采集一次温度（取值 1~65535）


/*****************************结构体********************************/
/** @brief BMI088 初始化配置（由调用方填写，本工程在 robot_init() 中赋值） */
typedef struct
{
  uint8_t ACC_ID;                 ///< 加速度计 SPI 设备编号（spi_config_s.ID），本工程为 1
  uint8_t GYRO_ID;                ///< 陀螺仪 SPI 设备编号，本工程为 2，须与 ACC_ID 不同
  SPI_HandleTypeDef *SPIx;        ///< 两路共用的 SPI 句柄，本工程为 &hspi1
  GPIO_TypeDef *ACC_CS_GPIOx;     ///< 加速度计片选端口，本工程为 GPIOA
  uint16_t ACC_CS_GPIO_PIN;       ///< 加速度计片选引脚，本工程为 GPIO_PIN_4
  GPIO_TypeDef *GYRO_CS_GPIOx;    ///< 陀螺仪片选端口，本工程为 GPIOB
  uint16_t GYRO_CS_GPIO_PIN;      ///< 陀螺仪片选引脚，本工程为 GPIO_PIN_0
}BMI_config_s; 

/** @brief BMI088 解算后的物理量数据（由 SPI 收发回调换算后写入） */
typedef struct
{
  float acc_x;                    ///< 加速度 X，单位 g（±24g）
  float acc_y;                    ///< 加速度 Y，单位 g（±24g）
  float acc_z;                    ///< 加速度 Z，单位 g（±24g）
  float temp;                     ///< 芯片温度，单位 ℃（由加速度计内的温度传感器采集，BMI_work() 分频更新）
  float gyro_x;                   ///< 角速度 X，单位 °/s（IMU.c 中再 ÷6 变 rpm）
  float gyro_y;                   ///< 角速度 Y，单位 °/s
  float gyro_z;                   ///< 角速度 Z，单位 °/s
}BMI_data_s;

/** @brief BMI088 驱动状态 */
typedef enum
{
  BMI_ok,                         ///< 0：成功
  BMI_error,                      ///< 1：失败（spi_init() 或 SPI 收发失败）
  BMI_busy,                       ///< 2：上一路未读完，本次未发起新读取
}BMI_status_e;


/*****************************外部调用********************************/
/**
 * @brief  初始化 BMI088：绑定 SPI 设备并配置加速度计与陀螺仪
 * @param  config 初始化配置，不能为 NULL；内部 malloc 私有实例并拷贝
 * @return BMI_ok=初始化完成；BMI_error=任一路 spi_init() 失败
 * @note   须在 MX_SPI1_Init() 之后、调度器启动前调用（内部 HAL_Delay() 共约 6ms）
 */
BMI_status_e BMI_init(BMI_config_s *config);

/**
 * @brief  周期读取 BMI088（加速度计与陀螺仪交替各读一次，并按分频采集温度）
 * @return BMI_ok=已发起并完成一路读取；BMI_busy=上一路未结束，本次未读取
 * @note   须在 BMI_init() 之后周期调用（本工程 imuTask 每 1ms）；加速度计/陀螺仪单路刷新率约为调用频率的一半，
 *         温度每 BMI_TEMP_DIV 次读取一次（本工程约 100ms），温度变化慢，无需更高频率
 */
BMI_status_e BMI_work(void);

/**
 * @brief  读取最近一帧解算数据（加速度 g、角速度 °/s、温度 ℃）
 * @return BMI_data_s 数据副本，按值返回；SPI 失败时可能返回旧值
 * @note   须在 BMI_init() 之后、与 BMI_work() 同任务调用
 */
BMI_data_s BMI_getData(void);

/**
 * @brief  查询温度是否已更新（读取后自动清零）
 * @return 1=自上次调用以来采集到了新的温度值；0=没有新温度
 * @note   须在 BMI_init() 之后、与 BMI_work() 同任务调用；供上层按温度采样周期执行温控
 */
uint8_t BMI_getTempUpdate(void);

#endif
