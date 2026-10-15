/**
  ******************************************************************************
  * @file    bsp_time.h
  * @brief   时间基准: 微秒级延时(DWT) + 单调递增时间戳(TIM2)
  *
  *  设计说明:
  *   - 延时用 Cortex-M3 的 DWT->CYCCNT 周期计数器, 精度到 1 个 CPU 周期
  *     (72MHz 下 13.9ns), 适合 WS2812 这类需要亚微秒级时序的用途。
  *     若 DWT 不可用(极少见), 自动退化为 NOP 空循环(精度略差)。
  *   - 时间戳: STM32F1 的 TIM2 是 16 位定时器, 单靠它无法表达秒级时间。
  *     这里把 TIM2 配成 1ms 周期中断(PSC=71, ARR=999), 在中断里累加软件计数器,
  *     得到 32 位毫秒时间戳(49.7 天回绕)与微秒时间戳。1ms 一次中断的开销约
  *     0.2% CPU, 换来所有时间比较都只是简单的减法。
  *   - 所有"过了多久"的判断请使用 bsp_elapsed_ms()/bsp_elapsed_us(), 跨回绕安全。
  ******************************************************************************
  */
#ifndef __BSP_TIME_H
#define __BSP_TIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 初始化时间基准(必须在其它驱动之前调用) */
void bsp_time_init(void);

/** CPU 周期计数(DWT), 未使能时返回 0 */
uint32_t bsp_cycles(void);

/** 微秒时间戳(1us 分辨率, 约 71 分钟回绕) */
uint32_t bsp_time_us(void);

/** 毫秒时间戳(约 49.7 天回绕) */
uint32_t bsp_time_ms(void);

/** 计算从 start 到现在的流逝时间(自动处理回绕) */
uint32_t bsp_elapsed_us(uint32_t start_us);
uint32_t bsp_elapsed_ms(uint32_t start_ms);

/** 忙等待延时(周期精度) */
void bsp_delay_cycles(uint32_t cycles);

/** 微秒级延时(精度约 ±1us) */
void bsp_delay_us(uint32_t us);

/** 毫秒级延时 */
void bsp_delay_ms(uint32_t ms);

/** DWT 是否可用 */
uint8_t bsp_time_dwt_ok(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TIME_H */
