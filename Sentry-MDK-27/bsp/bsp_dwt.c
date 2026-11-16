#include "bsp_dwt.h"

// 寄存器地址（通常适用于Cortex-M3/M4/M7）
#define DWT_CYCCNT  *(volatile uint32_t *)0xE0001004
#define DWT_CTRL    *(volatile uint32_t *)0xE0001000
#define DEMCR       *(volatile uint32_t *)0xE000EDFC

#define DEMCR_TRCENA           (1 << 24)
#define DWT_CTRL_CYCCNTENA     (1 << 0)



/**
 * @brief 使能dwt
 */
void DWT_Init(void)
{
  // 1. 使能DWT
  DEMCR |= DEMCR_TRCENA;
  // 2. 清零计数器
  DWT_CYCCNT = 0;
  // 3. 使能计数器
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;
}

/**
 * @brief 
 */
float DWT_getData(uint32_t cnt_old)
{
  //获取当前寄存器计数值
  uint32_t cnt_now = DWT->CYCCNT;
  //读取HCLK时钟频率
  float ticks_MHz = SystemCoreClock;
  //得到时间
  float dt = (cnt_now-cnt_old)/ticks_MHz;
  
  cnt_old = cnt_now;
  
  return dt;
}
