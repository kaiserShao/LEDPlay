/**
  ******************************************************************************
  * @file    drv_keyled.h
  * @brief   按键灯 WS2812 驱动 (4 颗, 单线接 PC9 = TIM3_CH4, PWM + DMA 发帧)
  *
  *  输出方式: 和主灯带同一套(见 bsp_config.h 第 5 节) —— TIM3_CH4 输出 800kHz PWM,
  *  1 个 PWM 周期 = 1 个数据位, 这一位的高电平长度(DMA 搬进 CCR4 的值)决定它是 0 码
  *  还是 1 码; 整帧 = 96 个数据位 + 240 个复位位(CCR4 = 0, 整位低电平)先编进缓冲区,
  *  由 DMA1_Channel3 在后台一次搬完。所以发帧期间**不用关中断**, 也不需要高优先级中断:
  *  波形是硬件搬的, 中断只负责收尾(记帧数、把颜色认成"已显示")。
  *
  *  时序: 默认值直接抄主灯带那一套(见 bsp_config.h 第 5 节: KEYLED_T?H_NS = WS2812_* 换算出来),
  *  所以按键灯 DIN 上看到的波形与主灯带逐位相同。段长由比较寄存器决定(是硬件级的),
  *  不存在"延时函数/循环固定开销"这类误差; PWM 方式下位周期由定时器 ARR 定死
  *  (KEYLED_BIT_NS = 1250ns), 四条参数里只有"0 码高 / 1 码高"能独立改, 低电平按
  *  "位周期 - 高电平"推出来(所以串口传低电平时会自动换算成对应的高电平)。
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

/** 初始化 GPIO/TIM3/DMA (PC9 = TIM3_CH4 完全重映射, 复用推挽 + DMA1_Channel3) */
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

/** 立即输出当前颜色(会等这一帧真的发完: 336 位 x 1.25us ≈ 420us) */
void keyled_show(void);

/** 主循环调用: 处理闪烁/呼吸等动画(内部限速刷新) */
void keyled_process(void);

/** 取当前实际输出的颜色(考虑效果与亮度之后) */
void keyled_get_output(uint8_t idx, uint8_t *r, uint8_t *g, uint8_t *b);

/** 取某路"设定值"(未经效果/亮度处理), 配合 keyled_get_output() 排查用 */
void keyled_get_set(uint8_t idx, uint8_t *r, uint8_t *g, uint8_t *b);

/** 位时序信息(ns / 实际用的拍数), 调试用 */
typedef struct
{
    uint16_t bit_ns;    /*!< 1 位的时间长度(位周期): PWM 方式由定时器 ARR 定死 */
    uint16_t t0h_ns;    /*!< 0 码高电平(ns) */
    uint16_t t0l_ns;    /*!< 0 码低电平(ns) = 位周期 - 0 码高电平(推出来的) */
    uint16_t t1h_ns;    /*!< 1 码高电平(ns) */
    uint16_t t1l_ns;    /*!< 1 码低电平(ns) = 位周期 - 1 码高电平(推出来的) */
    uint32_t t0h_cyc;   /*!< 实际使用的拍数 = 写进 CCR4 的值 */
    uint32_t t0l_cyc;
    uint32_t t1h_cyc;
    uint32_t t1l_cyc;
    uint32_t comp_t0h;  /*!< 老方式(软件翻转)自标定扣掉的固定开销(拍);
                             PWM 方式恒为 0: 段长就是 CCR 里的值本身 */
    uint32_t comp_t0l;
    uint32_t comp_t1h;
    uint32_t comp_t1l;
    uint32_t meas_t0h;  /*!< 实测 - 目标(拍): PWM 方式恒为 0(段长是硬件保证的,
                             要对照就用示波器量 PC9) */
    uint32_t meas_t0l;
    uint32_t meas_t1h;
    uint32_t meas_t1l;
} keyled_timing_t;

/** 重新设置位时序(ns, 传 0 = 用回配置默认值), 立即生效并重发一帧。
    PWM 方式下位周期由定时器定死, 低电平 = 位周期 - 高电平: 传低电平进来会换算成高电平 */
void keyled_set_timing(uint16_t t0h_ns, uint16_t t0l_ns, uint16_t t1h_ns, uint16_t t1l_ns);

/** 取当前时序信息(调试用, 见串口 "KEYLED DIAG") */
void keyled_get_timing_info(keyled_timing_t *info);

/** 时序是否已就绪(1 = 可以发帧; PWM 方式恒为 1 = 段长由比较寄存器保证) */
uint8_t keyled_calib_ok(void);

/** 真正发完过多少帧(调试用 —— 发一条颜色命令后再敲 "KEYLED DIAG",
    这个数变大就说明"发帧"这条路径是活的) */
uint32_t keyled_frames(void);

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
