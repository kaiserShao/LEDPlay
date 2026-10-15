/**
  ******************************************************************************
  * @file    app_led_fx.h
  * @brief   主灯带(120 颗)效果层: 常亮/彩虹/跑马灯/呼吸/闪烁/追踪
  *
  *  效果层只在色带缓冲上算颜色, 然后调用 ws2812_show() 交给 DMA 输出,
  *  本身不做任何时序工作, 全部在 main 的主循环里调用 led_fx_process()。
  ******************************************************************************
  */
#ifndef __APP_LED_FX_H
#define __APP_LED_FX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_config.h"

/** 灯带效果编号 */
typedef enum
{
    LED_FX_OFF      = 0,   /*!< 全灭 */
    LED_FX_SOLID    = 1,   /*!< 常亮(单色) */
    LED_FX_RAINBOW  = 2,   /*!< 彩虹渐变流动 */
    LED_FX_COMET    = 3,   /*!< 彗星跑马灯(带拖尾) */
    LED_FX_BREATHE  = 4,   /*!< 整体呼吸 */
    LED_FX_BLINK    = 5,   /*!< 整体闪烁 */
    LED_FX_CHASE    = 6,   /*!< 亮点追踪 */
    LED_FX_COUNT    = 7
} led_fx_t;

/** 初始化(默认效果: 全灭) */
void led_fx_init(void);

/** 切换效果 */
void led_fx_set(led_fx_t fx);

/** 取当前效果 */
led_fx_t led_fx_get(void);

/** 设置效果主色(常亮/彗星/呼吸/闪烁/追踪使用) */
void led_fx_set_color(uint8_t r, uint8_t g, uint8_t b);

/** 取主色 */
void led_fx_get_color(uint8_t *r, uint8_t *g, uint8_t *b);

/** 设置动画速度: 一次完整动画的时间(ms), 越大越慢 */
void led_fx_set_speed(uint16_t period_ms);
uint16_t led_fx_get_speed(void);

/** 主循环调用 */
void led_fx_process(void);

/** 效果名称(用于命令应答) */
const char *led_fx_name(led_fx_t fx);

#ifdef __cplusplus
}
#endif

#endif /* __APP_LED_FX_H */
