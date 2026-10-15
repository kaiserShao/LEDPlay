/**
  ******************************************************************************
  * @file    drv_ringbuf.h
  * @brief   通用字节环形缓冲区 (头文件实现, 单生产者/单消费者)
  *
  *  典型用法:
  *    - 生产者: 串口接收中断中调用 ringbuf_push()
  *    - 消费者: 主循环中调用 ringbuf_pop() / ringbuf_read()
  *  push/pop 都是无锁的单步操作, 中断与主循环同时访问是安全的。
  ******************************************************************************
  */
#ifndef __DRV_RINGBUF_H
#define __DRV_RINGBUF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>

/** 环形缓冲控制块 */
typedef struct
{
    uint8_t *buf;       /*!< 缓冲区      */
    uint16_t size;      /*!< 缓冲区大小(字节), 建议取 2 的幂 */
    volatile uint16_t head; /*!< 写指针(仅 ISR 修改) */
    volatile uint16_t tail; /*!< 读指针(仅主循环修改) */
    volatile uint32_t overflow; /*!< 溢出(丢弃)字节计数 */
} ringbuf_t;

/** 初始化环形缓冲 */
static inline void ringbuf_init(ringbuf_t *rb, uint8_t *buf, uint16_t size)
{
    rb->buf      = buf;
    rb->size     = (size == 0U) ? 1U : size;
    rb->head     = 0U;
    rb->tail     = 0U;
    rb->overflow = 0U;
}

/** 已缓存字节数 */
static inline uint16_t ringbuf_count(const ringbuf_t *rb)
{
    return (uint16_t)((rb->head - rb->tail) & 0xFFFFU);
}

/** 剩余空间字节数 */
static inline uint16_t ringbuf_space(const ringbuf_t *rb)
{
    if (rb->size == 0U)
    {
        return 0U;
    }
    return (uint16_t)(rb->size - 1U - ringbuf_count(rb));
}

/** 清空缓冲 */
static inline void ringbuf_flush(ringbuf_t *rb)
{
    rb->tail = rb->head;
}

/** 写入 1 字节(中断安全), 返回 1=成功 0=满 */
static inline uint8_t ringbuf_push(ringbuf_t *rb, uint8_t data)
{
    uint16_t next = (uint16_t)((rb->head + 1U) % rb->size); /* 保留 1 字节区分满/空 */

    if (next == rb->tail)
    {
        rb->overflow++;
        return 0U;
    }
    rb->buf[rb->head] = data;
    rb->head = next;
    return 1U;
}

/** 读取 1 字节, 返回 1=成功 0=空 */
static inline uint8_t ringbuf_pop(ringbuf_t *rb, uint8_t *data)
{
    if (rb->head == rb->tail)
    {
        return 0U;
    }
    *data = rb->buf[rb->tail];
    rb->tail = (uint16_t)((rb->tail + 1U) % rb->size);
    return 1U;
}

/** 批量读取, 返回实际读取字节数 */
static inline uint16_t ringbuf_read(ringbuf_t *rb, uint8_t *dst, uint16_t len)
{
    uint16_t n = 0U;

    while ((n < len) && (ringbuf_pop(rb, &dst[n]) != 0U))
    {
        n++;
    }
    return n;
}

#ifdef __cplusplus
}
#endif

#endif /* __DRV_RINGBUF_H */
