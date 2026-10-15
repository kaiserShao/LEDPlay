/**
  ******************************************************************************
  * @file    drv_esp32.h
  * @brief   ESP32-C3 (WiFi 转串口) 通信驱动  USART1  PA9=TX / PA10=RX
  *
  *  分工:
  *      * 本驱动只负责"链路": 初始化、接收中断 + 环形缓冲、按行拆包、发送。
  *      * 具体命令的解析与响应放在 app_cmd.c (业务层), 通过 esp32_set_rx_handler()
  *        注册回调即可。
  *
  *  数据格式: 默认 115200 8N1, ASCII 文本行, 以 '\n' 或 '\r\n' 结束。
  *           上位机发完命令不跟换行也能用: 最后一个字节之后静默 ESP32_LINE_IDLE_MS,
  *           就把这一段当成一整行(见 bsp_config.h 第 7 节)。
  ******************************************************************************
  */
#ifndef __DRV_ESP32_H
#define __DRV_ESP32_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_config.h"

/** 收到一整行(不含 \r\n)时的回调 */
typedef void (*esp32_line_cb_t)(const char *line);

/** 初始化 USART1 + GPIO + NVIC + 环形缓冲 */
void esp32_init(void);

/**
  * @brief  主循环/任务调用: 取数据并拆分成完整命令行, 触发回调
  * @note   ESP32_RX_USE_DMA=1 时它还要负责"把 DMA 窗口里硬件已收到的字节搬进环形
  *         缓冲/原始缓冲", 所以**必须被周期性调用**(通信任务每轮 + 裸机主循环每圈)。
  */
void esp32_process(void);

/**
  * @brief  等待串口收到数据
  * @param  timeout_ms 超时(ms): 0=只检查不等待
  * @retval 1=有数据可处理 0=超时
  * @note   ESP32_RX_USE_DMA=1: 收数不经过中断, 这里退化成"最多睡 timeout_ms"
  *         (所以 APP_TASK_COMM_PERIOD_MS 就是命令的最大响应延迟, 别设大);
  *         ESP32_RX_USE_DMA=0: 由 USART1 接收中断置事件标志唤醒, 无 RTOS 时轮询。
  */
uint8_t esp32_wait_data(uint32_t timeout_ms);

/** 注册整行接收回调(在 esp32_process() 中被调用) */
void esp32_set_rx_handler(esp32_line_cb_t cb);

/** 发送原始数据 */
void esp32_send(const uint8_t *data, uint16_t len);

/** 发送字符串(不追加换行) */
void esp32_send_str(const char *s);

/** 发送一行(自动追加 "\r\n") */
void esp32_send_line(const char *s);

/** 格式化发送(内部缓冲 128 字节) */
int esp32_printf(const char *fmt, ...);

/** 环形缓冲中可读字节数 */
uint16_t esp32_rx_available(void);

/** 读取原始字节, 返回实际读取数 */
uint16_t esp32_read(uint8_t *dst, uint16_t len);

/** 丢弃尚未处理的接收数据(同时丢掉半条行) */
void esp32_flush_rx(void);

/** 统计: 累计收到的字节数 / 溢出丢弃的字节数 / 收到的命令行数 / 错误次数 */
uint32_t esp32_rx_total(void);
uint32_t esp32_rx_overflows(void);
uint32_t esp32_rx_lines(void);
uint32_t esp32_rx_errors(void);

/** 统计: 被丢掉的字节数(超长行 + 控制字符/噪声) / 因超长整行作废的次数 */
uint32_t esp32_rx_dropped(void);
uint32_t esp32_rx_long_lines(void);

/**
  * @brief  行缓冲里是否有"还没组成整行"的字节
  * @retval 1=正在收一条命令(或正在丢弃超长行) 0=空闲
  * @note   给调度层用: 按键灯输出要关中断约 120us, 正好会把正在收的字节挤掉。
  *         所以"正在收命令"时先不发按键灯(见 app_task / main 主循环)。
  */
uint8_t esp32_line_pending(void);

/** 最近一次真正解析的整行(原样, 未切词; 排查"到底收到了什么"用, 见串口命令 UART) */
const char *esp32_last_line(void);

/** 原始字节透传: 1 = 收到的每个字节都打一行 "RX hh c"(排查链路多/少字节用) */
void    esp32_set_raw_trace(uint8_t on);
uint8_t esp32_get_raw_trace(void);

/*===========================================================================
 * 原始接收数据全局缓冲区(调试用, 一直开 —— 收到的字节在这里一份都不会少)
 *
 *  收到的**每个字节**(没经过任何过滤)按到达顺序存进 g_esp32_rx_cap[],
 *  同一个下标在 g_esp32_rx_flg[] 里记这个字节的标记(ESP32_CAP_FLAG_xxx)。
 *  是环形: 满了就覆盖最老的, g_esp32_rx_cap_idx = 下一个要写的位置,
 *  也就是说"最新收到的那个字节"在 (g_esp32_rx_cap_idx + ESP32_CAP_SIZE - 1) & (SIZE-1)。
 *
 *  三种看法(哪个方便用哪个):
 *    1. Keil 里 Watch 窗口加 g_esp32_rx_cap / g_esp32_rx_flg / g_esp32_rx_cap_idx
 *       (配合 ESP32_CAP_SIZE 看环形, 或直接把 Memory 窗口定到数组首地址);
 *    2. 串口发 UART DUMP [n] —— 十六进制 + ASCII + 标记打出来, 最直观;
 *    3. 串口发 UART RAW 1 —— 边收边打(看实时)。
 *=========================================================================*/
#define ESP32_CAP_FLAG_NONE    0x00U
#define ESP32_CAP_FLAG_ERR     0x01U   /* 这个字节到达时 USART 报了错(ORE/FE/NE/PE):
                                          丢字节/噪声/波特率偏差, 都往这里看 */
#define ESP32_CAP_FLAG_EOL     0x02U   /* 是行结束符 \r 或 \n */
#define ESP32_CAP_FLAG_GAP     0x04U   /* 与上一个字节之间静默 >= ESP32_CAP_GAP_MS:
                                          说明这里是一"批"数据的开头(上位机换命令了) */
#define ESP32_CAP_FLAG_FILTER  0x08U   /* 被行过滤器丢掉(控制字符 / 高位字节 / 超长行) */
#define ESP32_CAP_FLAG_BUFOVF  0x10U   /* 软件环形缓冲满, 这个字节没能进缓冲(会被丢) */
#define ESP32_CAP_FLAG_DMAOVF  0x20U   /* DMA 窗口被覆盖: 这里少了若干字节(取数太慢) */

extern uint8_t  g_esp32_rx_cap[ESP32_CAP_SIZE];   /* 收到的原始字节(环形) */
extern uint8_t  g_esp32_rx_flg[ESP32_CAP_SIZE];   /* 每个字节的标记(与 g_esp32_rx_cap 同下标) */
extern volatile uint32_t g_esp32_rx_cap_idx;      /* 下一个写入位置(0 ~ SIZE-1) */
extern volatile uint32_t g_esp32_rx_cap_total;    /* 累计收到的字节数(不清零) */

/** 原始缓冲: 累计收到的字节数(同 g_esp32_rx_cap_total) */
uint32_t esp32_cap_total(void);

/** 原始缓冲: 已经被覆盖掉的旧字节数(= 累计 - 缓冲大小, 最小 0) */
uint32_t esp32_cap_wraps(void);

/** 原始缓冲: 因为"没及时取"而丢掉的字节数(DMA 窗口被覆盖); >0 就说明任务被饿着了 */
uint32_t esp32_cap_dma_lost(void);

/** 原始缓冲: 缓冲大小(字节) */
uint32_t esp32_cap_size(void);

/**
  * @brief  把原始缓冲打印到串口(调试用)
  * @param  n 打印最近 n 个字节; 传 0(或大于缓冲大小) = 全部
  * @note   输出形如(每行 16 字节, 第二行 FLG 是每字节的标记, '.' = 无):
  *           CAP total=1234 idx=186
  *           CAP 00BA 4B 45 59 4C 45 44 20 30 20 32 35 35 20 30 20 30 |KEYLED 0 255 0 0|
  *           FLG      L  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
  *         标记: E=ERR(线上报错) L=换行 G=新一批 F=被过滤 O=缓冲满 D=DMA丢数
  */
void esp32_cap_dump(uint32_t n);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_ESP32_H */
