/**
  ******************************************************************************
  * @file    drv_ws2812.h
  * @brief   主灯带 WS2812 驱动 (120 颗, PB6 = TIM4_CH1, PWM + DMA)
  *
  *  实现要点
  *  ---------------------------------------------------------------------------
  *  * TIM4 产生 800kHz PWM: 1 个 PWM 周期(1.25us) = 1 个 WS2812 数据位,
  *    占空比 300ns -> 数据 "0", 600ns -> 数据 "1", 0 -> 复位/低电平。
  *  * DMA1_Channel1 工作在循环模式, 每次比较匹配自动把下一个 CCR 值写入,
  *    全硬件发送, 完全不占用 CPU。
  *  * 缓冲区结构: [前半帧 1440 位 = 第 0~59 颗] [后半帧 1440 位 = 第 60~119 颗]
  *                [复位段 240 位 = 300us 低电平]
  *    半传输(HT)中断里刷新"前半帧", 传输完成(TC)中断里刷新"后半帧 + 复位段",
  *    每个中断有约 1.95ms 的充裕时间, 因此灯带输出永不断流。
  *  * 应用层只写自己的颜色缓冲(show_pending), 中断里按帧作一次快照, 保证
  *    每帧颜色数据"整帧一致", 不会出现撕裂。
  *
  *  内存占用: PWM 缓冲 3120 x 2 = 6240 字节 + 颜色缓冲 2 x 360 字节
  *  ---------------------------------------------------------------------------
  *  用法:
  *      ws2812_init();                // 初始化并启动(默认 WS2812_AUTO_START=1)
  *      ws2812_set_pixel(0, 255, 0, 0);
  *      ws2812_show();                // 提交, 下一帧生效(非阻塞)
  ******************************************************************************
  */
#ifndef __DRV_WS2812_H
#define __DRV_WS2812_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_config.h"

/** 缓冲区长度相关(可按需引用) */
#define WS2812_DATA_BITS      (WS2812_LED_NUM * 24U)                         /* 2880 */
#define WS2812_RESET_BITS     ((WS2812_RESET_US * 1000U) / WS2812_BIT_NS)    /* 240  */
#define WS2812_BUF_LEN        (WS2812_DATA_BITS + WS2812_RESET_BITS)         /* 3120 */

/** 初始化 (GPIO/TIM4/DMA/NVIC), 成功后按 WS2812_AUTO_START 自动开始输出 */
void ws2812_init(void);

/** 启动 / 停止 DMA 连续输出 (停止后灯带保持最后一帧颜色) */
void ws2812_start(void);
void ws2812_stop(void);

/** 是否正在输出 */
uint8_t ws2812_is_running(void);

/** 主循环调用(当前实现里仅用于内部状态维护, 可直接调用不必需) */
void ws2812_process(void);

/*------------------------------- 多任务保护 ------------------------------*/
/**
  * @brief  加锁 / 解锁: 保护颜色缓冲
  * @note   多个任务同时改灯带颜色时(例如"动画任务"+"命令任务"), 请把
  *         "连续改多颗 + ws2812_show()"整段用 ws2812_lock()/ws2812_unlock()
  *         包起来, 这样每一帧的颜色都是成套的, 不会出现半新半旧。
  *         只改 1 颗灯时可以不锁(3 字节写入本身不会破坏缓冲区结构)。
  */
void ws2812_lock(void);
void ws2812_unlock(void);

/*------------------------------- 颜色操作 --------------------------------*/
/** 设置单颗灯珠颜色(写入待显示缓冲, 需 ws2812_show() 生效) */
void ws2812_set_pixel(uint16_t idx, uint8_t r, uint8_t g, uint8_t b);

/** 读取待显示缓冲中的颜色 */
void ws2812_get_pixel(uint16_t idx, uint8_t *r, uint8_t *g, uint8_t *b);

/** 全部灯珠设置成同一颜色 */
void ws2812_fill(uint8_t r, uint8_t g, uint8_t b);

/** 从 start 开始连续 count 颗设置成同一颜色(自动裁剪到 120) */
void ws2812_fill_range(uint16_t start, uint16_t count, uint8_t r, uint8_t g, uint8_t b);

/** 批量写入 RGB 数组(count 颗, 每颗 3 字节 R,G,B) */
void ws2812_set_buffer(const uint8_t *rgb, uint16_t count);

/** 全灭 */
void ws2812_clear(void);

/** 设置全局亮度 0~255 (在中断里做缩放, 不额外占内存) */
void ws2812_set_brightness(uint8_t brightness);
uint8_t ws2812_get_brightness(void);

/** 提交颜色缓冲: 下一帧开始生效(非阻塞, 立即返回) */
void ws2812_show(void);

/** 是否有未提交的修改 */
uint8_t ws2812_is_dirty(void);

/*------------------------------- 帧同步信息 ------------------------------*/
/** 已输出帧计数(在 TC 中断里递增) */
uint32_t ws2812_frame_count(void);

/** 调试用: 取初始化时实际算出来的"一位拍数 / 0 码 / 1 码"比较值(和 cfg 行的 ns 值对照) */
void ws2812_get_timing(uint16_t *arr, uint16_t *ccr0, uint16_t *ccr1);

/** 注册帧回调(在 TC 中断里每帧调用一次, 用于驱动动画节拍) */
void ws2812_set_frame_callback(void (*cb)(void));

#ifdef __cplusplus
}
#endif

#endif /* __DRV_WS2812_H */
