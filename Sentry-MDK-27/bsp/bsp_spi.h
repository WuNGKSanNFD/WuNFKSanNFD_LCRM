/******************************************************************************
 * @file        bsp_spi.h
 * @brief       SPI 从设备驱动：按逻辑 ID 注册 SPI 外设与片选，提供发送、接收与数据读取接口。
 * @note        上层回调可能在中断上下文执行，必须短小且不可阻塞。
 ******************************************************************************/

#ifndef  __BSP_SPI_H
#define  __BSP_SPI_H

#include "spi.h"
#include "string.h"
#include "stdlib.h"

/*************************宏***********************/

/** @brief SPI 实例表容量：spi_instance[SPI_CNT] 的数组长度，实现无容量检查 */
#define SPI_CNT   4   ///< 实例数量上限，单位：个

/** @brief 阻塞式收发的 HAL 超时时间（依赖 HAL_GetTick() 毫秒时基） */
#define SPI_TIMEOUT   100  ///< 超时时间，单位：ms

/** @brief 单次接收的数据长度上限：spi_config_s.rx_len 不得超过该值 */
#define SPI_MAX_RX_LEN  10  ///< 接收缓冲区长度，单位：字节

/*************************结构体***********************/

/** @brief SPI 接收/收发完成回调函数指针类型（需在回调内用 spi_getData() 取数据） */
typedef void (* spi_rxCallBack_f)(void);

/** @brief SPI 注册实例（内容会被 spi_init() 拷贝到内部实例） */
typedef struct
{
  uint8_t ID;                 ///< 逻辑设备 ID（0~255），需在已注册实例中唯一
  SPI_HandleTypeDef *SPIx;    ///< SPI 外设句柄（如 &hspi1）
  GPIO_TypeDef *CS_GPIOx;     ///< 片选引脚所属 GPIO 端口（如 GPIOA）
  uint16_t CS_GPIO_PIN;       ///< 片选引脚号（如 GPIO_PIN_4），低电平选中
  uint8_t rx_len;             ///< 每次接收字节数（1~SPI_MAX_RX_LEN），决定全双工长度
  
  spi_rxCallBack_f callback;  ///< 接收/收发完成回调，可为 NULL
}spi_config_s;


/** @brief 数据集（每个实例内部各持一份，存放最近一次接收的原始字节） */
typedef struct
{
  uint8_t raw[SPI_MAX_RX_LEN];///< 最近一次接收到的原始数据，有效长度 = rx_len
}spi_data_s;

/** @brief 片选器上下拉（借用 HAL 的 GPIO_PinState，低电平选中） */
typedef enum
{
  SPI_CS_STOP = GPIO_PIN_SET,//拉高：片选无效/空闲
  SPI_CS_START = GPIO_PIN_RESET,//拉低：片选有效
}spi_csSet_e;

/** @brief 函数状态（仅表示接口是否按预期完成） */
typedef enum
{
  spi_ok,     ///< 成功，隐式取值 0：传输已完成或已成功启动
  spi_error,  ///< 失败，隐式取值 1：HAL 返回非 HAL_OK（超时/忙）或未找到实例
}spi_status_e;


/*************************外部调用*******************************/


/**
 * @brief  SPI 初始化（须在任何收发调用之前执行）
 * @param  config 实例注册信息（ID/SPIx/片选/接收长度/回调），不可为 NULL
 * @return spi_ok=注册成功；spi_error=当前实现不会返回
 * @note   注册后立即把片选拉高（空闲为高）；ID 须唯一
 */
spi_status_e spi_init(spi_config_s *config);


/**
 * @brief  SPI 发送数据（阻塞，禁止在中断中调用）
 * @param  id      设备 ID（与 spi_init() 注册的一致）
 * @param  tx_data 发送数据缓冲区首地址，长度至少 len 字节
 * @param  len     发送字节数（1~255），不受注册的 rx_len 限制
 * @return spi_ok=发送完成；spi_error=未找到该 ID，无总线动作
 */
spi_status_e spi_sendData(uint8_t id,uint8_t *tx_data, uint8_t len);


/**
 * @brief  SPI 开启接收数据（中断方式，数据须在回调中用 spi_getData() 取走）
 * @param  id 设备 ID（与 spi_init() 注册的一致）
 * @return spi_ok=已启动接收（未找到该 ID 也返回 spi_ok）；spi_error=上一次接收尚未结束
 * @note   接收完成前重复调用会失败，且失败分支不拉高片选
 */
spi_status_e spi_receiveData(uint8_t id);


/**
 * @brief  全双工阻塞收发：同时发送与接收 rx_len 个字节（读寄存器常用）
 * @param  id      设备 ID（与 spi_init() 注册的一致）
 * @param  tx_data 发送数据缓冲区首地址，长度须 ≥ 该实例注册的 rx_len
 * @return spi_ok=收发完成并已存入实例；spi_error=未找到该 ID 或 HAL 返回非 HAL_OK
 * @note   不可重入：内部按 spi_active_id 寻址，多任务并发调用同一 SPI 会互相破坏状态
 */
spi_status_e spi_sendReceiveData(uint8_t id, uint8_t *tx_data);

/**
 * @brief  SPI 数据接口：取该实例最近一次接收到的数据（按值拷贝）
 * @param  id 设备 ID（与 spi_init() 注册的一致）
 * @return 该实例最近一次接收到的数据包，有效长度为该实例的 rx_len
 * @note   只做内存拷贝、不加锁，可在任务或中断中调用
 * @warning 未找到该 ID 时返回未初始化的局部变量，内容不确定
 */
spi_data_s spi_getData(uint8_t id);
#endif
