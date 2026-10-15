/**
  ******************************************************************************
  * @file    bsp_rtos.h
  * @brief   CMSIS-RTOS(Keil RTX) 适配层 —— 延时 / 互斥锁 / 事件标志 / 消息队列
  *
  *  为什么要有这一层:
  *    "不需要精确时序"的等待(等按键稳定、等语音播完、等串口发完、等数据到来)
  *    如果用忙等, 会白吃 CPU 并且让别的任务没机会跑; 用 RTOS 原语则只阻塞当前
  *    任务。为了不在每个驱动文件里塞满 #if, 这里统一提供 4 组接口:
  *
  *      bsp_sleep_ms()   RTOS: osDelay() 让出 CPU        | 无 RTOS: 忙等
  *      bsp_mutex_*()    RTOS: RTX 互斥锁                | 无 RTOS: 临界区
  *      bsp_signal_*()   RTOS: RTX 事件标志(中断可置位)  | 无 RTOS: 标志轮询
  *      bsp_queue_*()    RTOS: RTX 消息队列              | 无 RTOS: 环形缓冲
  *
  *  使用原则
  *  ---------------------------------------------------------------------------
  *  * 需要"微秒级精确"的地方(WS2812 时序 / 74HC595 移位 / DWT 延时)继续用
  *    bsp_time.h 的 bsp_delay_us() / bsp_delay_cycles(), 不要用本文件。
  *  * 所有"过了多久"的判断继续用 bsp_elapsed_ms(bsp_time_ms()): 时间基准来自
  *    TIM2 中断, 与 RTX 的 SysTick 完全独立, 跨任务/跨中断都安全。
  *  * bsp_rtos_init() 必须在"会创建锁/队列"的模块 *_init() 之前调用(见 main.c)。
  *  * 互斥锁: 谁加锁谁解锁; 可以把返回值忽略(失败时退化为临界区, 仍安全)。
  ******************************************************************************
  */
#ifndef __BSP_RTOS_H
#define __BSP_RTOS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_config.h"

#if (BSP_USE_RTOS != 0U)
#include "cmsis_os.h"      /* Keil RTX 4.x, CMSIS-RTOS API v1 (RTE 里已选中) */
#endif

/*==============================================================================
 *                             1. 内核控制
 *============================================================================*/
/** 初始化内核对象池: 必须最先调用(之后各模块 _init() 里才能创建锁/队列) */
void bsp_rtos_init(void);

/** 启动调度器: 不返回(RTX 模式下); 未使能 RTOS 时立即返回, 由 main 轮询 */
void bsp_rtos_start(void);

/** 调度器是否已经跑起来(在 main 里判断"现在是 RTOS 模式还是轮询模式") */
uint8_t bsp_rtos_running(void);

/*==============================================================================
 *                                2. 延时
 *============================================================================*/
/**
  * @brief  "不需要精确"的毫秒级延时
  * @param  ms 毫秒; 传 0 表示"让出一次 CPU"(等价于 osThreadYield)
  * @note   任务里调用 -> osDelay(只阻塞当前任务); 中断里调用 -> 自动改为忙等;
  *         未使能 RTOS 时 -> 忙等(与 bsp_delay_ms 相同)
  */
void bsp_sleep_ms(uint32_t ms);

/*==============================================================================
 *                              3. 互斥锁
 *  用于保护"会被多个任务同时访问"的资源: 串口发送、WS2812 颜色缓冲、按键灯等。
 *============================================================================*/
typedef struct
{
#if (BSP_USE_RTOS != 0U)
    uint32_t  cb[4];          /*!< RTX 互斥锁控制块(放在最前面保证对齐) */
    osMutexId id;             /*!< NULL = 创建失败, 退化为临界区 */
#endif
    uint32_t  primask;        /*!< 退化模式: 保存的中断状态 */
    uint8_t   inited;
} bsp_mutex_t;

/** 初始化(创建)一把锁 —— 在模块 _init() 里调用一次即可 */
void bsp_mutex_init(bsp_mutex_t *m);

/** 加锁: RTOS 模式下会阻塞到拿到锁为止; 退化模式下关中断 */
void bsp_mutex_lock(bsp_mutex_t *m);

/** 解锁 */
void bsp_mutex_unlock(bsp_mutex_t *m);

/*==============================================================================
 *                            4. 事件标志(信号)
 *  用于"中断 -> 任务"的轻量通知: 中断里置位, 任务里等待(不占用数据通道)。
 *  需要传递数据时请用下面的消息队列。
 *============================================================================*/
typedef struct
{
#if (BSP_USE_RTOS != 0U)
    osThreadId owner;         /*!< 等待者(第一次 wait 时登记) */
    int32_t    flag;          /*!< 使用的信号位 */
#endif
    volatile uint8_t pending;
    uint8_t          inited;
} bsp_signal_t;

/**
  * @brief  初始化一个事件标志
  * @param  index 信号位序号 0~15(同一个任务里不同用途请用不同序号)
  */
void bsp_signal_init(bsp_signal_t *s, uint8_t index);

/** 置位(可在中断中调用; 没有任务在等时也能"记住", 下次 wait 立刻返回 1) */
void bsp_signal_set(bsp_signal_t *s);

/** 清除(任务里调用) */
void bsp_signal_clear(bsp_signal_t *s);

/**
  * @brief  等待事件
  * @param  timeout_ms 超时(ms), 0 = 只检查不等待(非阻塞)
  * @retval 1=收到事件 0=超时
  */
uint8_t bsp_signal_wait(bsp_signal_t *s, uint32_t timeout_ms);

/*==============================================================================
 *                              5. 消息队列
 *  固定深度(uint32_t 元素), 结构体自带存储, 不依赖动态内存。
 *============================================================================*/
#define BSP_QUEUE_DEPTH      8U                                     /* 队列深度 */
#define BSP_QUEUE_STORAGE_WORDS   (4U + BSP_QUEUE_DEPTH)            /* RTX 需要 4 个字头 */

typedef struct
{
    uint32_t   storage[BSP_QUEUE_STORAGE_WORDS];   /*!< RTX 的消息缓冲(前 4 字是队列头) */
    volatile uint16_t head;                        /*!< 退化模式的环形缓冲 */
    volatile uint16_t tail;
    uint8_t    inited;
#if (BSP_USE_RTOS != 0U)
    osMessageQId id;                               /*!< NULL = 未创建, 退化为环形缓冲 */
#endif
} bsp_queue_t;

/** 初始化: 在模块 _init() 里调用(必须在 bsp_rtos_init() 之后) */
void bsp_queue_init(bsp_queue_t *q);

/** 入队(满则丢弃 + 返回 0; 可在中断中调用) */
uint8_t bsp_queue_put(bsp_queue_t *q, uint32_t value);

/**
  * @brief  出队
  * @param  value      取到的值
  * @param  timeout_ms 超时(ms), 0 = 非阻塞
  * @retval 1=取到 0=超时/空
  */
uint8_t bsp_queue_get(bsp_queue_t *q, uint32_t *value, uint32_t timeout_ms);

/** 队列是否为空 */
uint8_t bsp_queue_is_empty(const bsp_queue_t *q);

/** 清空队列 */
void bsp_queue_flush(bsp_queue_t *q);

/*==============================================================================
 *                                6. 线程
 *============================================================================*/
/** 线程入口函数原型(与 RTX 一致) */
typedef void (*bsp_thread_fn_t)(void const *argument);

/**
  * @brief  创建一个任务(默认优先级范围 -3 ~ +3, 见 APP_TASK_PRIO_xxx)
  * @param  fn        线程入口(死循环 + 必须主动让出 CPU: bsp_sleep_ms / bsp_signal_wait ...)
  * @param  arg       传给入口的参数(不需要就传 NULL)
  * @param  priority  优先级: 越大越高(-1=BelowNormal 0=Normal 1=AboveNormal 2=High)
  * @retval 1=成功 0=失败(失败时该任务的 *_process() 需要由 main 轮询兜底)
  * @note   任务栈大小由 RTE\CMSIS\RTX_Conf_CM.c 的 OS_STKSIZE 决定(默认 1KB)
  */
uint8_t bsp_thread_create(bsp_thread_fn_t fn, void *arg, int32_t priority);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_RTOS_H */
