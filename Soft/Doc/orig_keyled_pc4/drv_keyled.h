/**
  ******************************************************************************
  * @file    drv_keyled.h
  * @brief   按键灯 WS2812 驱动 (4 路, 单线串在 PC4, 软件模拟时序)
  *
  *  为什么不用 PWM: 只有 4 颗灯(96 位), 用 DWT 周期计数做"软件延时 + 管脚翻转"
  *  更省资源, 不占用任何定时器/DMA 通道, 输出一次约 110us(96 位 x 约 1.1us/位)。
  *
  *  时序: 每位 = "高电平 + 低电平" 两段, 0 码/1 码各自独立(T0H/T0L/T1H/T1L),
  *  不设"固定 1.25us 位周期" —— 规格书只规定这四段各自的上下限, 位周期只是参考。
  *  上电时用 DWT 实测这四段实际有多长, 把延时函数/循环的固定开销扣掉
  *  (keyled_calibrate), 所以 KEYLED_T0H_NS/T0L_NS/T1H_NS/T1L_NS 填什么, 灯珠就真看到什么。
  *  现场还能用 keyled_set_timing() / keyled_set_invert() 实时改(串口 KEYLED T0H ...)。
  *
  *  用法:
  *      keyled_init();
  *      keyled_set_fx(0, KEYLED_FX_BREATHE, 0, 128, 255, 1500);  // 按键1 呼吸
  *      keyled_set_color(1, 255, 0, 0);                          // 按键2 常亮红
  *      主循环: keyled_process();
  ******************************************************************************
  */
#ifndef __DRV_KEYLED_H
#define __DRV_KEYLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_config.h"

#define KEYLED_COUNT          KEYLED_NUM

/** 按键灯效果 */
typedef enum
{
    KEYLED_FX_OFF     = 0,   /*!< 熄灭 */
    KEYLED_FX_STATIC  = 1,   /*!< 常亮 */
    KEYLED_FX_BLINK   = 2,   /*!< 闪烁 (period_ms 为亮灭周期) */
    KEYLED_FX_BREATHE = 3    /*!< 呼吸 (period_ms 为一次呼吸时间) */
} keyled_fx_t;

/** 初始化 GPIO (PC4 推挽输出) */
void keyled_init(void);

/** 设置某路效果 + 颜色 + 周期(ms)。idx: 0 ~ 3 */
void keyled_set_fx(uint8_t idx, keyled_fx_t fx, uint8_t r, uint8_t g, uint8_t b, uint16_t period_ms);

/** 设置某路为常亮颜色 */
void keyled_set_color(uint8_t idx, uint8_t r, uint8_t g, uint8_t b);

/** 全部设置为同一常亮颜色 */
void keyled_fill(uint8_t r, uint8_t g, uint8_t b);

/** 全部熄灭并立即刷新 */
void keyled_off(void);

/** 设置全局亮度 0~255 */
void keyled_set_brightness(uint8_t brightness);

/** 立即输出当前颜色(阻塞约 110us, 期间关中断) */
void keyled_show(void);

/** 主循环调用: 处理闪烁/呼吸等动画(内部限速刷新) */
void keyled_process(void);

/** 取当前实际输出的颜色(考虑效果与亮度之后) */
void keyled_get_output(uint8_t idx, uint8_t *r, uint8_t *g, uint8_t *b);

/** 取某路"设定值"(未经效果/亮度处理), 配合 keyled_get_output() 排查用 */
void keyled_get_set(uint8_t idx, uint8_t *r, uint8_t *g, uint8_t *b);

/** 位时序信息(ns / 实际用的周期数 / 自标定扣掉的固定开销), 调试用 */
typedef struct
{
    uint16_t t0h_ns;    /*!< 0 码高电平目标(ns) */
    uint16_t t0l_ns;    /*!< 0 码低电平目标(ns) */
    uint16_t t1h_ns;    /*!< 1 码高电平目标(ns) */
    uint16_t t1l_ns;    /*!< 1 码低电平目标(ns) */
    uint32_t t0h_cyc;   /*!< 实际使用的周期数(目标值已扣掉固定开销) */
    uint32_t t0l_cyc;
    uint32_t t1h_cyc;
    uint32_t t1l_cyc;
    uint32_t comp_t0h;  /*!< 自标定扣掉的固定开销(周期数) */
    uint32_t comp_t0l;
    uint32_t comp_t1h;
    uint32_t comp_t1l;
} keyled_timing_t;

/** 重新设置位时序(ns, 传 0 = 用回配置默认值), 立即生效并重发一帧 */
void keyled_set_timing(uint16_t t0h_ns, uint16_t t0l_ns, uint16_t t1h_ns, uint16_t t1l_ns);

/** 取当前时序信息(调试用, 见串口 "KEYLED DIAG") */
void keyled_get_timing_info(keyled_timing_t *info);

/** 上电自标定是否生效(1 = 当前时序是按实测校准过的) */
uint8_t keyled_calib_ok(void);

/** 数据线极性: 1 = 反相(数据线上串了反相器/三极管), 立即生效并重发一帧 */
void keyled_set_invert(uint8_t invert);

/** 当前是否反相 */
uint8_t keyled_get_invert(void);

/** 现场核实数据线通不通: 驱动低/高时分别读回引脚电平(调试用, 见 "KEYLED DIAG") */
void keyled_probe_line(uint8_t *low_read, uint8_t *high_read);

/** 批量写入 4 路颜色(R,G,B 顺序, count 路) */
void keyled_set_buffer(const uint8_t *rgb, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_KEYLED_H */
