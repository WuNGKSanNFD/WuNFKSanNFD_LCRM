/** @file   remote.h
 *  @brief  遥控/键鼠输入模块：解析 DT7 接收机（DR16 协议，UART3）的 18 字节帧，输出去零偏后的遥控数据。
 *  @note   remote_receive() 在串口中断中调用，不可阻塞；remote_getData() 在任务中调用（本工程 cmdTask 5ms）。
 */

#ifndef __REMOTE_H
#define __REMOTE_H

#include "serial.h"
#include "string.h"
#include "deamon.h"


/******************结构体*******************/

/**
 * @brief DT7 遥控器原始数据（未去零偏）
 * @note  摇杆/拨轮为 11 位量，中位 1024
 */
typedef struct
{
  uint16_t ch0;         ///< 通道0（右摇杆水平）原始值
  uint16_t ch1;         ///< 通道1（右摇杆竖直）原始值
  uint16_t ch2;         ///< 通道2（左摇杆水平）原始值
  uint16_t ch3;         ///< 通道3（左摇杆竖直）原始值
  uint8_t  s1;          ///< 左三档开关：1=上、3=中、2=下
  uint8_t  s2;          ///< 右三档开关：1=上、3=中、2=下
  uint16_t dial;        ///< 拨轮原始值
}remote_DT7data_s;


/**
 * @brief PC 客户端鼠标键盘数据
 * @note  与摇杆数据在同一 18 字节帧内下发，本工程未消费
 */
typedef struct
{
  int16_t mouse_x;      ///< 鼠标 X 增量，单位 count
  int16_t mouse_y;      ///< 鼠标 Y 增量，单位 count
  int16_t mouse_z;      ///< 鼠标滚轮增量，单位 count
  uint8_t mouse_press_l;///< 鼠标左键：0=松开，1=按下
  uint8_t mouse_press_r;///< 鼠标右键：0=松开，1=按下
  uint8_t key_w;        ///< 键盘 W：0=松开，1=按下
  uint8_t key_s;        ///< 键盘 S：0=松开，1=按下
  uint8_t key_a;        ///< 键盘 A：0=松开，1=按下
  uint8_t key_d;        ///< 键盘 D：0=松开，1=按下
  uint8_t key_q;        ///< 键盘 Q：0=松开，1=按下
  uint8_t key_e;        ///< 键盘 E：0=松开，1=按下
  uint8_t key_shift;    ///< 键盘 Shift：0=松开，1=按下
  uint8_t key_ctrl;     ///< 键盘 Ctrl：0=松开，1=按下
}remote_clientData_s;


/**
 * @brief 遥控原始数据（模块内部缓存）
 * @note  由串口中断写入、remote_getData() 在任务中读取；startFlag 是掉线判定的唯一依据
 */
typedef struct
{
  remote_DT7data_s DT7;        ///< DT7 遥控器原始数据（未去零偏）
  remote_clientData_s client;  ///< PC 客户端鼠标键盘数据
  uint8_t startFlag;           ///< 在线标志：1=最近收到过完整帧，0=掉线
  
}remote_rawData_s;

/**
 * @brief 遥控数据（对外输出，摇杆/拨轮已去零偏）
 * @note  remote_getData() 只做中位换算，不做死区处理（死区由 application/cmd.c 负责）
 */
typedef struct
{
  int16_t ch0;              ///< 通道0（右摇杆水平），用于云台/底盘旋转
  int16_t ch1;              ///< 通道1（右摇杆竖直），用于 pitch 角度增量
  int16_t ch2;              ///< 通道2（左摇杆水平），用于底盘 turn
  int16_t ch3;              ///< 通道3（左摇杆竖直），用于底盘 forward
  uint8_t  s1;              ///< 左三档开关：1=上、3=中、2=下
  uint8_t  s2;              ///< 右三档开关：1=上、3=中、2=下
  int16_t dial;             ///< 拨轮，去零偏后依次触发摩擦轮、开火
  
  remote_clientData_s client;  ///< PC 客户端鼠标键盘数据（原样透传）
  
}remote_data_s;


/**
 * @brief 遥控状态枚举
 */
typedef enum
{
  remote_ok,                ///< 0：在线/成功
  remote_error              ///< 1：离线/失败（startFlag 为 0，输出被清零）
}remote_status_e;

/******************外部调用********************/

/**
 * @brief  遥控初始化：注册 UART3 接收（DR16，18 字节）、注册离线守护并开启接收
 * @return remote_ok=初始化完成（当前实现恒返回 remote_ok）
 * @note   须在 MX_USART3_UART_Init() 与 DMA 配置完成后调用（本工程 cmd_init()）
 */
remote_status_e remote_init(void);

/**
 * @brief  解析一帧 DR16 数据（18 字节）到模块内部原始缓存
 * @return remote_ok=解析完成（只做位域拆分，不校验帧内容）
 * @note   由串口中断中的 remote_rxCallback() 调用，不要在任务中直接调用
 */
remote_status_e remote_receive(void);

/**
 * @brief  获取遥控数据：把原始值减去中位 1024 后写入输出缓存
 * @param  remote_data 输出缓存，不能为 NULL；无需预先初始化，离线时会被清零
 * @return remote_ok=在线并已写入；remote_error=离线（startFlag==0），缓存被清零
 * @note   本工程在 cmdTask 中每 5ms 调用；读取期间未关中断，可能读到半新半旧的一帧
 */
remote_status_e remote_getData(remote_data_s *remote_data);

#endif
