/**
  ******************************************************************************
  * @file    drv_key.h
  * @brief   按键驱动 (PA0~PA6, EXTI + 主循环状态机)
  *
  *  设计目标 —— 中断里尽量少干活:
  *    中断服务程序(由 bsp_exti 统一分发)只做两件事:
  *       1) 记录本次边沿的时间戳(微秒级, 来自 TIM2 1MHz 计数器)
  *       2) 置位一个标志位
  *    真正的消抖、短按/长按判定全部在 drv_key_process() 里完成, 主循环里
  *    每个周期调用一次即可 (内部自动限速到 KEY_SCAN_PERIOD_MS)。
  *
  *  对外输出: 事件队列 (按下/松开/短按/长按/长按保持), 业务层轮询取用;
  *           也可注册回调, 在 process() 中"立即"处理。
  ******************************************************************************
  */
#ifndef __DRV_KEY_H
#define __DRV_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_config.h"

/** 按键编号: 对应 PA0~PA6, KEY_1 即 PA0 */
#define KEY_1                        0U
#define KEY_2                        1U
#define KEY_3                        2U
#define KEY_4                        3U
#define KEY_5                        4U
#define KEY_6                        5U
#define KEY_7                        6U

/** 按键事件 */
typedef enum
{
    KEY_EV_NONE   = 0,   /*!< 无事件 */
    KEY_EV_DOWN   = 1,   /*!< 消抖后确认按下 */
    KEY_EV_UP     = 2,   /*!< 消抖后确认松开 */
    KEY_EV_SHORT  = 3,   /*!< 短按 (松开时上报, 时长 < KEY_LONG_MS) */
    KEY_EV_LONG   = 4,   /*!< 长按 (按住达到 KEY_LONG_MS 时上报一次) */
    KEY_EV_HOLD   = 5    /*!< 长按保持 (每 KEY_HOLD_REPEAT_MS 上报一次) */
} key_event_t;

/** 按键消息 */
typedef struct
{
    uint8_t  id;           /*!< 按键编号 KEY_1 ~ KEY_7 */
    uint8_t  event;        /*!< key_event_t */
    uint16_t duration_ms;  /*!< DOWN/UP/SHORT: 按住时长; LONG/HOLD: 当前按住时长 */
} key_msg_t;

/** 事件回调(在 drv_key_process() 内被调用, 不要做耗时操作) */
typedef void (*key_callback_t)(const key_msg_t *msg);

/** 初始化: GPIO(内部上拉) + EXTI(下降沿) + 状态机复位 */
void drv_key_init(void);

/** 主循环调用, 建议 >=1ms 调用一次 (内部限速到 KEY_SCAN_PERIOD_MS) */
void drv_key_process(void);

/** 从事件队列取一条消息, 返回 1=取到, 0=队列空 */
uint8_t drv_key_get_event(key_msg_t *msg);

/**
  * @brief  阻塞等待一条按键事件(业务任务里用, 不再需要轮询)
  * @param  msg        取到的消息
  * @param  timeout_ms 超时(ms): 0=只检查不等待, 0xFFFFFFFF=一直等
  * @retval 1=取到 0=超时
  * @note   RTOS 模式下由按键中断/事件置位唤醒(不占 CPU);
  *         无 RTOS 模式下按 1ms 粒度轮询等待。
  */
uint8_t drv_key_wait_event(key_msg_t *msg, uint32_t timeout_ms);

/** 清空事件队列 */
void drv_key_flush_queue(void);

/** 注册事件回调(可传 NULL 取消) */
void drv_key_set_callback(key_callback_t cb);

/** 按键数量 */
uint8_t drv_key_get_count(void);

/** 当前是否处于按下状态(已消抖) */
uint8_t drv_key_is_down(uint8_t id);

/** 当前按住时长(ms), 未按下返回 0 */
uint32_t drv_key_press_ms(uint8_t id);

/** 读取引脚原始电平: 1=按下(低电平) */
uint8_t drv_key_raw_pressed(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_KEY_H */
