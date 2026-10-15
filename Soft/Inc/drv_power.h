/**
  ******************************************************************************
  * @file    drv_power.h
  * @brief   一键开关机驱动 (PC3 = 电源保持, PA7 = 电源键)
  *
  *  典型自锁电路:
  *      按下电源键 -> 硬件先把电池供电接到 MCU -> MCU 启动后在初始化里立刻把
  *      PC3 拉到 POWER_HOLD_ON_LEVEL 维持供电(否则松手就断电);
  *      要关机时把 PC3 拉到 POWER_HOLD_OFF_LEVEL, 硬件断电。
  *
  *  电源键 PA7: 内部上拉 + EXTI 下降沿中断(只记时间戳), 状态机在主循环:
  *      * 长按 POWER_OFF_LONG_MS(默认 3s)   -> 回调 app_shutdown_handler() 后断电
  *        (业务层可在回调里播提示音、保存数据、熄灭灯带)
  *      * 长按 POWER_FORCE_OFF_MS(默认 6s)  -> 不等待回调, 立即断电(强力兜底)
  *      * 短按                               -> 通过 short press 回调上报, 由业务决定
  ******************************************************************************
  */
#ifndef __DRV_POWER_H
#define __DRV_POWER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_config.h"

/** 初始化: 先"保持供电", 再配置电源键 EXTI */
void power_init(void);

/** 主循环调用(建议 10ms 一次) */
void power_process(void);

/** 电源键当前是否按下(已消抖) */
uint8_t power_key_is_down(void);

/** 电源键当前按住时长(ms) */
uint32_t power_key_press_ms(void);

/** 立即断电: 把 PC3 拉到 POWER_HOLD_OFF_LEVEL(不返回) */
void power_shutdown_now(void);

/** 请求关机: 先调用关机回调(播提示音等), 然后断电 */
void power_request_shutdown(void);

/** 注册"关机前处理"回调(例如语音提示 + 关灯 + 保存参数) */
void power_set_shutdown_handler(void (*handler)(void));

/** 注册短按回调 */
void power_set_short_press_handler(void (*handler)(void));

/** 是否已经进入关机流程 */
uint8_t power_is_shutting_down(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_POWER_H */
