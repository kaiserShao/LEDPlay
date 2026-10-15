/**
  ******************************************************************************
  * @file    app_task.h
  * @brief   任务的划分与创建 (RTOS 模式下的"调度表")
  *
  *  划分原则: 按"实时性要求"分组, 而不是按文件分组 —— 任务越少, RAM 与切换
  *  开销越小。本工程 4 个任务(优先级/周期可在 bsp_config.h 第 9 节调整):
  *
  *  ---------------------------------------------------------------------------
  *  任务                优先级          周期/唤醒方式        跑哪些 *_process()
  *  ---------------------------------------------------------------------------
  *  app_task_input      AboveNormal    5ms                  drv_key, drv_power
  *  app_task_light      Normal         10ms                 app_wsgame, app_led_fx, drv_keyled, drv_ws2812
  *  app_task_comm       Normal         10ms 或串口有数据     drv_esp32, app_cmd
  *  app_task_voice      BelowNormal    10ms                 drv_wt2003h
  *  ---------------------------------------------------------------------------
  *
  *  注意: 真正"精确时序"的工作(WS2812 的 PWM+DMA 与 HT/TC 中断、按键灯 DWT
  *  位打、串口收发(默认 DMA1_Channel5 环形接收)、TIM2 时间基准、EXTI)全部留在
  *  中断/DMA 里, 任务只做业务逻辑, 因此任务周期的抖动不会影响灯带/按键灯的波形。
  ******************************************************************************
  */
#ifndef __APP_TASK_H
#define __APP_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief  创建所有任务并启动调度器
  * @note   RTX 模式下本函数不返回(调度器接管); 未使能 RTOS 时立即返回, 此时
  *         各模块的 *_process() 由 main 的主循环轮询(功能完全一样)。
  *         必须在所有硬件初始化 + app_cmd_init() 之后调用。
  */
void app_task_start(void);

/** 是否已经进入多任务模式(4 个任务都创建成功) */
uint8_t app_task_started(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_H */
