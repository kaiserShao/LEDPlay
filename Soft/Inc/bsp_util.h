/**
  ******************************************************************************
  * @file    bsp_util.h
  * @brief   通用小工具: 临界区、字符串/数字转换
  ******************************************************************************
  */
#ifndef __BSP_UTIL_H
#define __BSP_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f10x.h"

/** 系统级初始化(中断优先级分组等), 必须最先调用一次 */
void bsp_util_init(void);

/*--------------------------- 临界区 (可嵌套保存/恢复) -----------------------*/
/** 关中断并返回之前的 PRIMASK 状态 */
uint32_t bsp_irq_save(void);

/** 按 bsp_irq_save() 的返回值恢复中断状态 */
void bsp_irq_restore(uint32_t primask);

#define BSP_CRITICAL_BEGIN()   do { uint32_t __pm = bsp_irq_save()
#define BSP_CRITICAL_END()       bsp_irq_restore(__pm); } while (0)

/*--------------------------- 字符串 / 数字 --------------------------------*/
/** 无符号整数 -> 十进制字符串, 返回写入长度(不含 '\0') */
uint8_t bsp_utoa(uint32_t value, char *buf);

/** 有符号整数 -> 十进制字符串, 返回写入长度(不含 '\0') */
uint8_t bsp_itoa(int32_t value, char *buf);

/** 尽量把 0~255 的整数转成字符串, 返回写入长度(不含 '\0') */
uint8_t bsp_u8toa(uint8_t value, char *buf);

/** 十六进制(不带 0x, 低位在右), len 为需要输出的位数, 返回写入长度 */
uint8_t bsp_hex_to_str(uint32_t value, uint8_t digits, char *buf);

/** 十进制字符串 -> uint32(遇到非数字停止), 返回解析出的值 */
uint32_t bsp_atoi(const char *s);

/** 环形字符串比较(不区分大小写), 相等返回 1 */
uint8_t bsp_stricmp_eq(const char *a, const char *b);

/** 把字符串转为大写(原地) */
void bsp_str_upper(char *s);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UTIL_H */
