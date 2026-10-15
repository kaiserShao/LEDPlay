/**
  ******************************************************************************
  * @file    drv_hc595.h
  * @brief   4 位静态数码管驱动 (4 片 74HC595 级联, PC6=SDI, PC7=SCLK, PC8=LOAD)
  *
  *  段码位定义(与 74HC595 的输出 Q0~Q7 对应):
  *      bit0 = a   bit1 = b   bit2 = c   bit3 = d
  *      bit4 = e   bit5 = f   bit6 = g   bit7 = dp(小数点)
  *  有效电平由 HC595_ACTIVE_LEVEL 决定: 1 = 段码高电平点亮(共阴),
  *  0 = 段码低电平点亮(共阳, 内部自动取反)。
  *
  *  位序: 级联时"先移入的字节"最终停在最远端的 595 上。如果实际焊接顺序与
  *        逻辑位号不一致, 修改 HC595_DIGIT_MAP 即可。
  *
  *  用法:
  *      hc595_init();
  *      hc595_show_int(1234);          // 显示 1234
  *      hc595_show_digits("A1-2");     // 显示字符串(4 位)
  ******************************************************************************
  */
#ifndef __DRV_HC595_H
#define __DRV_HC595_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_config.h"

#define HC595_DIGITS      HC595_DIGIT_NUM

/** 逻辑位号 -> 移位顺序(数组下标为逻辑位, 值为移出的先后顺序索引) */
extern const uint8_t hc595_digit_map[HC595_DIGITS];

/** 初始化 GPIO 并清屏 */
void hc595_init(void);

/** 直接写入 4 位段码(下标 0 = 第 1 位) 并立即锁存输出 */
void hc595_write(const uint8_t seg[HC595_DIGITS]);

/** 只修改内部缓冲中的某一位段码(不输出), 之后调用 hc595_flush() 生效 */
void hc595_write_index(uint8_t digit, uint8_t seg);

/** 把内部缓冲输出到数码管 */
void hc595_flush(void);

/** 显示 4 个字符(不足补空格), 支持 0-9 A-F H L P r U - _ 空格 */
void hc595_show_digits(const char *text);

/** 显示整数 (-999 ~ 9999), 负数用第 1 位显示 '-', 超出范围显示 "----" */
void hc595_show_int(int32_t value);

/** 设置某一位的小数点 */
void hc595_set_dp(uint8_t digit, uint8_t on);

/** 按位掩码设置小数点 (bit0 = 第 1 位) */
void hc595_set_dp_mask(uint8_t mask);

/** 全部熄灭 */
void hc595_clear(void);

/** 字符 -> 段码 (无法显示的字符返回 0) */
uint8_t hc595_char_to_seg(char c);

/** 读取当前缓冲段码 */
uint8_t hc595_get_seg(uint8_t digit);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_HC595_H */
