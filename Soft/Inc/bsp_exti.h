/**
  ******************************************************************************
  * @file    bsp_exti.h
  * @brief   外部中断分发器
  *
  *  STM32 的 EXTI0~EXTI4 各有独立中断向量, EXTI5~9 共用一个, EXTI10~15 共用
  *  一个。为了让"按键"和"电源键"这些模块都能用外部中断又不重复定义中断服务
  *  函数, 这里统一实现 ISR, 只做: 清标志 + 回调(回调里只允许记录时间戳/置标志,
  *  绝不能做耗时操作)。
  ******************************************************************************
  */
#ifndef __BSP_EXTI_H
#define __BSP_EXTI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_exti.h"

/** EXTI 回调: 参数为触发的 EXTI_Linex */
typedef void (*bsp_exti_cb_t)(uint16_t exti_line);

/**
  * @brief  注册某条 EXTI 线的回调(只注册, 不配置硬件)
  * @param  exti_line  EXTI_Line0 ~ EXTI_Line15
  */
void bsp_exti_attach(uint16_t exti_line, bsp_exti_cb_t cb);

/**
  * @brief  一步完成: GPIO 作为 EXTI 源 + 边沿配置 + NVIC 使能 + 回调注册
  * @param  exti_line    EXTI_Linex
  * @param  port_source  GPIO_PortSourceGPIOx
  * @param  pin_source   GPIO_PinSourcex
  * @param  trigger      EXTI_Trigger_Rising / Falling / Rising_Falling
  * @param  preempt_prio 抢占优先级(0~3, 数值越小越高)
  * @note   同一 EXTI5~9 / EXTI10~15 组内请使用相同优先级
  */
void bsp_exti_config(uint16_t exti_line, uint8_t port_source, uint8_t pin_source,
                     uint32_t trigger, uint8_t preempt_prio, bsp_exti_cb_t cb);

/** 手动清除某条线的挂起标志 */
void bsp_exti_clear(uint16_t exti_line);

/** 读取某条线累计触发次数(调试用) */
uint32_t bsp_exti_count(uint16_t exti_line);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_EXTI_H */
