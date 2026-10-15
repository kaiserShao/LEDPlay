/**
  ******************************************************************************
  * @file    drv_esp32.c
  * @brief   ESP32-C3 WiFi 转串口驱动 (USART1)
  ******************************************************************************
  */
#include "drv_esp32.h"
#include "drv_ringbuf.h"
#include "bsp_rtos.h"
#include "bsp_time.h"
#include "bsp_util.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "misc.h"
#include <stdio.h>
#include <stdarg.h>

static ringbuf_t      s_rx;
static uint8_t        s_rx_buf[ESP32_RX_BUF_SIZE];
static char           s_line[ESP32_LINE_MAX + 1U];
static uint16_t       s_line_len = 0U;

#if (ESP32_RX_USE_DMA != 0U)
/* USART1_RX 由 DMA 循环搬到这里; esp32_process() 再把它们搬进环形缓冲/原始缓冲 */
static uint8_t        s_dma_buf[ESP32_RX_DMA_SIZE];
static uint16_t       s_dma_rd      = 0U;   /*!< 下一个要搬走的字节下标 */
static uint16_t       s_dma_wr_prev = 0U;   /*!< 上次看到的 DMA 写指针(判断有没有被套圈) */
#endif

/*------------------------- 原始数据全局缓冲(调试用) ------------------------*/
uint8_t  g_esp32_rx_cap[ESP32_CAP_SIZE];
uint8_t  g_esp32_rx_flg[ESP32_CAP_SIZE];
volatile uint32_t g_esp32_rx_cap_idx   = 0U;
volatile uint32_t g_esp32_rx_cap_total = 0U;

static volatile uint32_t s_cap_dma_lost = 0U;   /*!< DMA 窗口被覆盖丢掉的字节数 */
static uint32_t          s_cap_last_ms  = 0U;   /*!< 上一个字节的到达时刻(判断"新一批") */

/* 1 = 这一行已经超长(废了), 后面的字节全部丢到下一个换行为止。
   不能"清空计数接着存": 一串没有换行的数据(上位机连发/桥接芯片的回显)会被
   拼成一条乱七八糟的命令, 现象是 "ERR UNKNOWN <三四段残缺命令拼在一起>"。 */
static uint8_t        s_line_discard  = 0U;

/* 本行最后一个字节的到达时刻: 有些上位机发完命令不跟 \r\n, 隔得够久就当整行收完 */
static uint32_t       s_last_byte_ms  = 0U;

/* 最近一次真正交给业务层的整行(业务层会就地改写 s_line, 所以另留一份),
   串口命令 UART 会把它打出来 —— 排查"到底收到了什么"最直接的一招 */
static char           s_last_line[ESP32_LINE_KEEP_MAX + 1U] = {0};

/* 1 = 收到的每个字节都打一行十六进制(串口命令 UART RAW 1 打开, 默认关) */
static uint8_t        s_raw_trace     = 0U;

static esp32_line_cb_t s_line_cb = NULL;

/* 拆行用的两个内部函数(实现在后面; esp32_process() 要在它们之前调用, 所以先声明) */
static void    esp32_line_finish(void);
static uint8_t esp32_line_feed(uint8_t data);
#if (ESP32_RX_USE_DMA != 0U)
static void    esp32_dma_drain(void);  /* 打印原始缓冲时也要顺手把 DMA 取空, 免得被覆盖 */
#endif

/* 多个任务可能同时发数据(命令应答 + 心跳上报), 用互斥锁保证不被插队 */
static bsp_mutex_t    s_tx_lock;

/* 接收中断 -> 通信任务 的唤醒标志(收到换行符 / 一批数据开头时置位) */
static bsp_signal_t   s_rx_sig;

static volatile uint32_t s_rx_total    = 0U;
static volatile uint32_t s_rx_lines    = 0U;
static volatile uint32_t s_rx_errors   = 0U;
static volatile uint32_t s_rx_dropped  = 0U;   /*!< 被丢掉的字节(超长行 + 控制字符/噪声) */
static volatile uint32_t s_rx_long     = 0U;   /*!< 因超长整行作废的次数 */

/*---------------------------------------------------------------------------*/
void esp32_init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef uart;
#if (ESP32_RX_USE_DMA == 0U)
    NVIC_InitTypeDef  nvic;
#else
    DMA_InitTypeDef   dma;
#endif

    RCC_APB2PeriphClockCmd(ESP32_GPIO_CLK | RCC_APB2Periph_AFIO | ESP32_UART_CLK, ENABLE);

    /* PA9 = TX 复用推挽 */
    gpio.GPIO_Pin   = ESP32_TX_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(ESP32_TX_PORT, &gpio);

    /* PA10 = RX 浮空输入(外部由 ESP32 驱动) */
    gpio.GPIO_Pin   = ESP32_RX_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ESP32_RX_PORT, &gpio);

    ringbuf_init(&s_rx, s_rx_buf, ESP32_RX_BUF_SIZE);

    /* RTOS 资源(必须在 bsp_rtos_init() 之后创建, main 里已保证顺序) */
    bsp_mutex_init(&s_tx_lock);
    bsp_signal_init(&s_rx_sig, BSP_SIGNAL_ESP32_RX);

    uart.USART_BaudRate            = ESP32_BAUD;
    uart.USART_WordLength          = USART_WordLength_8b;
    uart.USART_StopBits            = USART_StopBits_1;
    uart.USART_Parity              = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(ESP32_UART, &uart);

#if (ESP32_RX_USE_DMA != 0U)
    /*--------------------------- 接收: DMA1_Channel5 循环搬运 ---------------
      为什么不用接收中断: 主灯带的 DMA 中断里要跑 ws2812_encode(), 按键灯发帧还要关
      中断 ~120us; F103 的 USART 接收寄存器只有 1 个字节, 这期间到的字节必丢。
      交给 DMA 后, 字节由硬件直接写进 RAM —— CPU 关不关中断、被谁抢占都照收。 */
    RCC_AHBPeriphClockCmd(ESP32_RX_DMA_CLK, ENABLE);
    DMA_DeInit(ESP32_RX_DMA_CHANNEL);

    dma.DMA_PeripheralBaseAddr = (uint32_t)(&ESP32_UART->DR);
    dma.DMA_MemoryBaseAddr     = (uint32_t)s_dma_buf;
    dma.DMA_DIR                = DMA_DIR_PeripheralSRC;       /* 外设 -> 内存 */
    dma.DMA_BufferSize         = (uint16_t)ESP32_RX_DMA_SIZE;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode               = DMA_Mode_Circular;           /* 循环: 永远在收 */
    dma.DMA_Priority           = DMA_Priority_High;
    dma.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(ESP32_RX_DMA_CHANNEL, &dma);
    DMA_Cmd(ESP32_RX_DMA_CHANNEL, ENABLE);

    USART_DMACmd(ESP32_UART, USART_DMAReq_Rx, ENABLE);

    s_dma_rd      = 0U;
    s_dma_wr_prev = 0U;
#else
    /*--------------------------- 接收: 每字节一次 RXNE 中断 ----------------*/
    USART_ITConfig(ESP32_UART, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel                   = ESP32_UART_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = BSP_IRQ_PRIO_ESP32_UART;
    nvic.NVIC_IRQChannelSubPriority        = 0U;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
#endif

    USART_Cmd(ESP32_UART, ENABLE);

    ringbuf_flush(&s_rx);
    s_line_len        = 0U;
    s_line_discard    = 0U;
    s_last_byte_ms    = bsp_time_ms();
    s_last_line[0]    = '\0';

    g_esp32_rx_cap_idx   = 0U;
    g_esp32_rx_cap_total = 0U;
    s_cap_dma_lost       = 0U;
    s_cap_last_ms        = bsp_time_ms();
}

/*===========================================================================
 *                    原始接收数据全局缓冲(调试用, 一直开)
 *
 * 为什么要有它: "串口少字节 / 多字节"这种问题, 只看命令行的结果永远说不清是
 * 哪一段链路出的问题。这里把**链路上刚收到的每个字节**原样留一份(带标记),
 * 拿去和上位机发出去的内容逐字节比, 一眼就能看出:
 *     * 少了字节            -> 看这个位置的标记: E(线上报错) / D(DMA 丢) / O(缓冲满)
 *     * 多了字节            -> 桥接芯片把自己的回显/日志混进来了
 *     * 全都在但命令还是错  -> 那就不是"收"的问题(是拆行/参数/逻辑)
 * 保存形态: 环形, 只留最近 ESP32_CAP_SIZE 个字节(见 drv_esp32.h 的说明)。
 *=========================================================================*/
static void esp32_cap_add(uint8_t ch, uint8_t flags)
{
    uint32_t idx = g_esp32_rx_cap_idx;

    /* 与上一个字节之间隔了 >= ESP32_CAP_GAP_MS: 这是一"批"数据的开头 */
    if ((g_esp32_rx_cap_total != 0U) &&
        (bsp_elapsed_ms(s_cap_last_ms) >= ESP32_CAP_GAP_MS))
    {
        flags |= (uint8_t)ESP32_CAP_FLAG_GAP;
    }
    s_cap_last_ms = bsp_time_ms();

    g_esp32_rx_cap[idx] = ch;
    g_esp32_rx_flg[idx] = flags;

    g_esp32_rx_cap_idx    = (idx + 1U) & (ESP32_CAP_SIZE - 1U);
    g_esp32_rx_cap_total++;
}

/*---------------------------------------------------------------------------*/
/** 给"最新存进去的那个字节"补标记(取数时才判定的标记: 被过滤 / 缓冲满) */
static void esp32_cap_mark(uint8_t flags)
{
    uint32_t idx = (g_esp32_rx_cap_idx + ESP32_CAP_SIZE - 1U) & (ESP32_CAP_SIZE - 1U);

    g_esp32_rx_flg[idx] |= flags;
}

/*---------------------------------------------------------------------------*/
uint32_t esp32_cap_total(void)
{
    return g_esp32_rx_cap_total;
}

/*---------------------------------------------------------------------------*/
uint32_t esp32_cap_wraps(void)
{
    uint32_t total = g_esp32_rx_cap_total;

    return (total > (uint32_t)ESP32_CAP_SIZE) ? (total - (uint32_t)ESP32_CAP_SIZE) : 0U;
}

/*---------------------------------------------------------------------------*/
uint32_t esp32_cap_dma_lost(void)
{
    return s_cap_dma_lost;
}

/*---------------------------------------------------------------------------*/
uint32_t esp32_cap_size(void)
{
    return (uint32_t)ESP32_CAP_SIZE;
}

/*---------------------------------------------------------------------------*/
/** 一个字节的标记 -> 一个字符(多种标记同时存在时按"最要紧"的一个显示) */
static char esp32_cap_flag_char(uint8_t f)
{
    if ((f & (uint8_t)ESP32_CAP_FLAG_ERR)    != 0U) { return 'E'; }   /* 线上报错 */
    if ((f & (uint8_t)ESP32_CAP_FLAG_BUFOVF) != 0U) { return 'O'; }   /* 缓冲满  */
    if ((f & (uint8_t)ESP32_CAP_FLAG_DMAOVF) != 0U) { return 'D'; }   /* DMA 丢  */
    if ((f & (uint8_t)ESP32_CAP_FLAG_FILTER) != 0U) { return 'F'; }   /* 被过滤  */
    if ((f & (uint8_t)ESP32_CAP_FLAG_GAP)    != 0U) { return 'G'; }   /* 新一批  */
    if ((f & (uint8_t)ESP32_CAP_FLAG_EOL)    != 0U) { return 'L'; }   /* 换行    */
    return '.';
}

/*---------------------------------------------------------------------------*/
void esp32_cap_dump(uint32_t n)
{
    char     row[160];
    char     flg[160];
    uint32_t total = g_esp32_rx_cap_total;
    uint32_t cnt   = (total > (uint32_t)ESP32_CAP_SIZE) ? (uint32_t)ESP32_CAP_SIZE : total;
    uint32_t start;
    uint32_t i;

    if ((n == 0U) || (n > cnt))
    {
        n = cnt;
    }

    esp32_printf("CAP total=%lu show=%lu idx=%lu lost=%lu wraps=%lu\r\n",
                 (unsigned long)total, (unsigned long)n,
                 (unsigned long)g_esp32_rx_cap_idx,
                 (unsigned long)s_cap_dma_lost,
                 (unsigned long)esp32_cap_wraps());

    if (n == 0U)
    {
        return;
    }

    /* 从"最老的保留字节"开始(环形绕回来的情况也算对) */
    start = (g_esp32_rx_cap_idx + (uint32_t)ESP32_CAP_SIZE - n) & (uint32_t)(ESP32_CAP_SIZE - 1U);

    for (i = 0U; i < n; i += 16U)
    {
        uint32_t k;
        uint32_t len = ((n - i) < 16U) ? (n - i) : 16U;
        int      p   = 0;
        int      q   = 0;
        uint8_t  any = 0U;

#if (ESP32_RX_USE_DMA != 0U)
        /* 打印是慢动作(每行 16 字节 ≈ 1.4ms): 先把 DMA 取空, 免得窗口被覆盖丢数 */
        esp32_dma_drain();
#endif

        p += sprintf(&row[p], "CAP %04lu ", (unsigned long)(i + 1U));
        for (k = 0U; k < 16U; k++)
        {
            uint32_t a = (start + i + k) & (uint32_t)(ESP32_CAP_SIZE - 1U);

            if (k < len)
            {
                p += sprintf(&row[p], "%02X ", (unsigned)g_esp32_rx_cap[a]);
            }
            else
            {
                p += sprintf(&row[p], "   ");
            }
        }

        row[p] = '|';
        p++;
        for (k = 0U; k < 16U; k++)
        {
            uint32_t a = (start + i + k) & (uint32_t)(ESP32_CAP_SIZE - 1U);
            char     c = (k < len) ? (char)g_esp32_rx_cap[a] : ' ';

            row[p] = ((c >= 0x20) && (c <= 0x7E)) ? c : '.';
            p++;
        }
        row[p] = '\0';
        esp32_send_line(row);

        /* 标记行: 每字节一个字符, 与上面十六进制列对齐, 全是 '.' 就不打了 */
        q += sprintf(&flg[q], "FLG      ");
        for (k = 0U; k < 16U; k++)
        {
            uint32_t a = (start + i + k) & (uint32_t)(ESP32_CAP_SIZE - 1U);
            char     c = (k < len) ? esp32_cap_flag_char(g_esp32_rx_flg[a]) : ' ';

            if (c != '.')
            {
                any = 1U;
            }
            flg[q] = c;
            q++;
            flg[q] = ' ';
            q++;
            flg[q] = ' ';
            q++;
        }
        flg[q] = '\0';

        if (any != 0U)
        {
            esp32_send_line(flg);
        }
    }

    esp32_send_line("CAP end (E=err L=eol G=new batch F=filtered O=ringfull D=dma_lost)");
}

/*---------------------------------------------------------------------------*/
/** 发送 1 字节(带超时保护) */
static void esp32_tx_byte(uint8_t data)
{
    uint32_t t0 = bsp_time_ms();

    while (USART_GetFlagStatus(ESP32_UART, USART_FLAG_TXE) == RESET)
    {
        if (bsp_elapsed_ms(t0) > ESP32_TX_TIMEOUT_MS)
        {
            return;         /* 超时放弃, 避免死等 */
        }
    }
    USART_SendData(ESP32_UART, (uint16_t)data);
}

/*---------------------------------------------------------------------------*/
void esp32_send(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    uint32_t t0;

    if ((data == NULL) || (len == 0U))
    {
        return;
    }

    /* 加锁: 防止两个任务(命令应答 / 心跳上报)把字节交错在一起 */
    bsp_mutex_lock(&s_tx_lock);

    for (i = 0U; i < len; i++)
    {
        esp32_tx_byte(data[i]);
    }

    /* 等最后一个字节真正移位完成(带超时, 避免硬件异常时死等) */
    t0 = bsp_time_ms();
    while (USART_GetFlagStatus(ESP32_UART, USART_FLAG_TC) == RESET)
    {
        if (bsp_elapsed_ms(t0) > ESP32_TX_TIMEOUT_MS)
        {
            break;
        }
    }

    bsp_mutex_unlock(&s_tx_lock);
}

/*---------------------------------------------------------------------------*/
void esp32_send_str(const char *s)
{
    if (s == NULL)
    {
        return;
    }

    /* 走 esp32_send(): 内部加锁 + 等移位完成, 保证整串"一次发完"不被别的任务插队
       (以前这里逐字节发、且没加锁, 和命令应答并发时会在串口上互相插字符) */
    esp32_send((const uint8_t *)s, (uint16_t)strlen(s));
}

/*---------------------------------------------------------------------------*/
void esp32_send_line(const char *s)
{
    char    buf[152];
    size_t  n;
    uint16_t i;

    if (s == NULL)
    {
        return;
    }

    /* 整行拼好再"一次"发出去: 分多次发会被别的任务(命令应答/语音帧跟踪)插队,
       串口上看到的就是两条命令的字符混在一行里 */
    n = strlen(s);
    if (n > (sizeof(buf) - 2U))
    {
        n = sizeof(buf) - 2U;
    }

    for (i = 0U; (size_t)i < n; i++)
    {
        buf[i] = s[i];
    }
    buf[n]      = '\r';
    buf[n + 1U] = '\n';

    esp32_send((const uint8_t *)buf, (uint16_t)(n + 2U));
}

/*---------------------------------------------------------------------------*/
int esp32_printf(const char *fmt, ...)
{
    char    buf[128];
    va_list ap;
    int     n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n < 0)
    {
        return n;
    }
    if (n > (int)(sizeof(buf) - 1U))
    {
        n = (int)(sizeof(buf) - 1U);
    }

    esp32_send((const uint8_t *)buf, (uint16_t)n);
    return n;
}

/*===========================================================================
 *                          接收: 拆行 + 分发
 *=========================================================================*/
#if (ESP32_RX_USE_DMA != 0U)
/*---------------------------------------------------------------------------*/
/** DMA 当前写指针(硬件已经收到第几个字节了) */
static uint16_t esp32_dma_wr(void)
{
    uint16_t left = (uint16_t)DMA_GetCurrDataCounter(ESP32_RX_DMA_CHANNEL);

    return (uint16_t)(((uint32_t)ESP32_RX_DMA_SIZE - (uint32_t)left) &
                      (uint32_t)(ESP32_RX_DMA_SIZE - 1U));
}

/*---------------------------------------------------------------------------*/
/** 把 DMA 窗口里"硬件已经收到的"字节搬进环形缓冲 + 原始缓冲(不阻塞, 纯内存操作) */
static void esp32_dma_drain(void)
{
    uint32_t sr    = (uint32_t)ESP32_UART->SR;      /* 顺带看一眼线上有没有报错 */
    uint8_t  err   = (((sr & (uint32_t)USART_FLAG_ORE) != 0U) ||
                      ((sr & (uint32_t)USART_FLAG_FE)  != 0U) ||
                      ((sr & (uint32_t)USART_FLAG_NE)  != 0U) ||
                      ((sr & (uint32_t)USART_FLAG_PE)  != 0U)) ? 1U : 0U;
    uint16_t wr    = esp32_dma_wr();
    uint8_t  first = 1U;

    /* 读完 SR, 后面 DMA 读 DR 的顺序正好把这几个错误标志清掉 */

    if (err != 0U)
    {
        s_rx_errors++;      /* DMA 方式没有错误中断, 在这里补计一次 */
    }

    /* 生产者把消费者"套圈"了: 一次调用间隔里来了 >= SIZE 个字节, 中间那段没了。
       正常收命令永远不该发生(命令才十几个字节); 真出现就是任务被饿着, 要查调度。 */
    if ((wr == s_dma_rd) && (wr != s_dma_wr_prev))
    {
        s_cap_dma_lost += (uint32_t)ESP32_RX_DMA_SIZE;
        esp32_cap_add(s_dma_buf[s_dma_rd], (uint8_t)ESP32_CAP_FLAG_DMAOVF);
    }
    s_dma_wr_prev = wr;

    while (s_dma_rd != wr)
    {
        uint8_t  ch    = s_dma_buf[s_dma_rd];
        uint8_t  flags = 0U;

        if ((err != 0U) && (first != 0U))
        {
            flags |= (uint8_t)ESP32_CAP_FLAG_ERR;   /* 报错加在这批的第一个字节上 */
        }
        if ((ch == (uint8_t)'\r') || (ch == (uint8_t)'\n'))
        {
            flags |= (uint8_t)ESP32_CAP_FLAG_EOL;
        }

        s_dma_rd = (uint16_t)(((uint32_t)s_dma_rd + 1U) & (uint32_t)(ESP32_RX_DMA_SIZE - 1U));
        first    = 0U;

        esp32_cap_add(ch, flags);       /* 原始缓冲: 一个字节都不会漏 */
        s_rx_total++;

        if (ringbuf_push(&s_rx, ch) == 0U)
        {
            esp32_cap_mark((uint8_t)ESP32_CAP_FLAG_BUFOVF);
        }
    }
}
#endif /* ESP32_RX_USE_DMA */

/*---------------------------------------------------------------------------*/
uint8_t esp32_wait_data(uint32_t timeout_ms)
{
    if (ringbuf_count(&s_rx) != 0U)          /* 已经有数据: 不用等 */
    {
        return 1U;
    }
#if (ESP32_RX_USE_DMA != 0U)
    if (s_dma_rd != esp32_dma_wr())          /* DMA 里已经有字节了: 不用等 */
    {
        return 1U;
    }
#endif
    return bsp_signal_wait(&s_rx_sig, timeout_ms);
}

/*---------------------------------------------------------------------------*/
void esp32_process(void)
{
    uint8_t  data;

    /* 每轮处理上限, 避免长时间占用主循环 */
    uint16_t budget = 256U;

#if (ESP32_RX_USE_DMA != 0U)
    /* 先把 DMA 里已经收到的字节搬出来(每轮都要做, 否则 DMA 窗口会被覆盖) */
    esp32_dma_drain();
#endif

    while ((budget > 0U) && (ringbuf_pop(&s_rx, &data) != 0U))
    {
        budget--;

        if (s_raw_trace != 0U)       /* UART RAW 1: 每个字节都打出来(排查链路丢/多字节) */
        {
            esp32_printf("RX %02X %c\r\n", (unsigned)data,
                         ((data >= 0x20U) && (data <= 0x7EU)) ? (char)data : (char)'.');
        }

        if (esp32_line_feed(data) != 0U)
        {
            /* 这个字节被行过滤器丢了(控制字符/噪声/超长行): 在原始缓冲上标出来 */
            esp32_cap_mark((uint8_t)ESP32_CAP_FLAG_FILTER);
        }
    }

    /* 兼容"发完命令不跟换行"的上位机/工具: 隔够久(ESP32_LINE_IDLE_MS)就当这一行收完了。
       注意本函数必须被周期性调用(通信任务每轮都会调, 无 RTOS 时主循环每圈都调),
       否则这里的"收尾"不会发生。ESP32_LINE_IDLE_MS = 0 表示关掉这条规则(只认 \r\n)。 */
#if (ESP32_LINE_IDLE_MS != 0U)
    if ((s_line_len > 0U) && (bsp_elapsed_ms(s_last_byte_ms) >= ESP32_LINE_IDLE_MS))
    {
        esp32_line_finish();
    }
#endif
}

/*---------------------------------------------------------------------------*/
/** 把"已经收满的这一行"交给业务层; 同时留一份副本给串口命令 UART 回看 */
static void esp32_line_finish(void)
{
    uint8_t i;

    if (s_line_discard != 0U)      /* 超长行: 整行作废(绝不把两段拼在一起) */
    {
        s_line_discard = 0U;
        s_line_len     = 0U;
        return;
    }

    if (s_line_len == 0U)          /* 空行(CRLF / LFCR 里的第二个, 或用户直接回车) */
    {
        return;
    }

    s_line[s_line_len] = '\0';
    s_rx_lines++;

    /* 先留副本: 业务层会就地切词(cmd_tokenize 把空格改成 '\0'), 之后就没法回看了 */
    for (i = 0U; (i < s_line_len) && (i < (uint8_t)ESP32_LINE_KEEP_MAX); i++)
    {
        s_last_line[i] = s_line[i];
    }
    s_last_line[i] = '\0';

    if (s_line_cb != NULL)
    {
        s_line_cb(s_line);
    }

    s_line_len     = 0U;
    s_last_byte_ms = bsp_time_ms();
}

/*---------------------------------------------------------------------------*/
/** 收下 1 个字节(只负责"拼行", 不做解析)
    @retval 1 = 这个字节被丢掉了(控制字符/噪声/超长行), 0 = 正常收下/作为换行收尾 */
static uint8_t esp32_line_feed(uint8_t data)
{
    /* '\r' / '\n' 都当行结束: CRLF 或 LFCR 里的第二个会走空行分支被忽略 */
    if ((data == (uint8_t)'\r') || (data == (uint8_t)'\n'))
    {
        esp32_line_finish();
        return 0U;
    }

    if (s_line_discard != 0U)      /* 这行已经废了: 只等换行符 */
    {
        s_rx_dropped++;
        return 1U;
    }

    /* 只收可打印 ASCII: 控制字符 / 高位字节通常是线路噪声或桥接芯片自己的日志,
       混进来只会把命令拼成乱码, 直接丢掉(制表符当空格, 方便上位机对齐参数)。 */
    if (data == (uint8_t)'\t')
    {
        data = (uint8_t)' ';
    }
    else if ((data < 0x20U) || (data > 0x7EU))
    {
        s_rx_dropped++;
        return 1U;
    }

    if (s_line_len < ESP32_LINE_MAX)
    {
        s_line[s_line_len] = (char)data;
        s_line_len++;
        s_last_byte_ms = bsp_time_ms();
    }
    else
    {
        /* 超长命令: 本行作废, 丢弃到下一个换行(见 s_line_discard 的说明) */
        s_line_discard = 1U;
        s_line_len     = 0U;
        s_rx_dropped++;
        s_rx_long++;
        s_rx_errors++;
        return 1U;
    }

    return 0U;
}

/*---------------------------------------------------------------------------*/
void esp32_set_rx_handler(esp32_line_cb_t cb)
{
    s_line_cb = cb;
}

/*---------------------------------------------------------------------------*/
uint16_t esp32_rx_available(void)
{
    return ringbuf_count(&s_rx);
}

/*---------------------------------------------------------------------------*/
uint16_t esp32_read(uint8_t *dst, uint16_t len)
{
    return ringbuf_read(&s_rx, dst, len);
}

/*---------------------------------------------------------------------------*/
void esp32_flush_rx(void)
{
    ringbuf_flush(&s_rx);
#if (ESP32_RX_USE_DMA != 0U)
    s_dma_rd = esp32_dma_wr();      /* DMA 窗口里还没搬走的字节也一起丢掉 */
#endif
    s_line_len     = 0U;
    s_line_discard = 0U;
    s_last_byte_ms = bsp_time_ms();
}

/*---------------------------------------------------------------------------*/
uint32_t esp32_rx_total(void)
{
    return s_rx_total;
}

/*---------------------------------------------------------------------------*/
uint32_t esp32_rx_overflows(void)
{
    return s_rx.overflow;
}

/*---------------------------------------------------------------------------*/
uint32_t esp32_rx_lines(void)
{
    return s_rx_lines;
}

/*---------------------------------------------------------------------------*/
uint32_t esp32_rx_errors(void)
{
    return s_rx_errors;
}

/*---------------------------------------------------------------------------*/
uint32_t esp32_rx_dropped(void)
{
    return s_rx_dropped;
}

/*---------------------------------------------------------------------------*/
uint32_t esp32_rx_long_lines(void)
{
    return s_rx_long;
}

/*---------------------------------------------------------------------------*/
uint8_t esp32_line_pending(void)
{
#if (ESP32_RX_USE_DMA != 0U)
    /* DMA 窗口里还有"硬件已经收到、但还没被搬进环形缓冲"的字节: 也算正在收命令 */
    if (s_dma_rd != esp32_dma_wr())
    {
        return 1U;
    }
#endif
    return ((s_line_len != 0U) || (s_line_discard != 0U)) ? 1U : 0U;
}

/*---------------------------------------------------------------------------*/
const char *esp32_last_line(void)
{
    return s_last_line;
}

/*---------------------------------------------------------------------------*/
void esp32_set_raw_trace(uint8_t on)
{
    s_raw_trace = (on != 0U) ? 1U : 0U;
}

/*---------------------------------------------------------------------------*/
uint8_t esp32_get_raw_trace(void)
{
    return s_raw_trace;
}

/*===========================================================================
 *                              串口接收
 *  ESP32_RX_USE_DMA = 1: 收数由 DMA 硬件完成(见 esp32_dma_drain), 没有接收中断;
 *  ESP32_RX_USE_DMA = 0: 每个字节进一次中断 —— 只做"取字节 -> 环形缓冲 + 原始缓冲"
 *                        以及必要的错误标志清理, 尽量短。
 *  两种方式都会把收到的每个字节存进原始缓冲(g_esp32_rx_cap), 一个都不漏。
 *=========================================================================*/
#if (ESP32_RX_USE_DMA == 0U)
void USART1_IRQHandler(void)
{
    uint32_t sr = (uint32_t)ESP32_UART->SR;

    if ((sr & (uint32_t)USART_FLAG_RXNE) != 0U)
    {
        uint8_t ch;
        uint8_t burst_start;
        uint8_t flags = 0U;

        /* 环形缓冲原本是空的 -> 这是"一批数据的第一个字节": 唤醒通信任务。
           这样即使上位机发完命令不跟换行, 任务也能立刻醒过来(再等 ESP32_LINE_IDLE_MS
           把它当成整行收尾)。 */
        burst_start = (ringbuf_count(&s_rx) == 0U) ? 1U : 0U;

        /* 读 SR + 读 DR 一起把 RXNE/ORE/FE/NE/PE 全部清掉 */
        ch = (uint8_t)(ESP32_UART->DR & 0x00FFU);

        if ((sr & ((uint32_t)USART_FLAG_ORE | (uint32_t)USART_FLAG_FE |
                   (uint32_t)USART_FLAG_NE | (uint32_t)USART_FLAG_PE)) != 0U)
        {
            /* 这个字节到达时线上有错: 噪声 / 波特率偏差 / 被长时间关中断挤掉一个字节 */
            s_rx_errors++;
            flags |= (uint8_t)ESP32_CAP_FLAG_ERR;
        }

        if ((ch == (uint8_t)'\r') || (ch == (uint8_t)'\n'))
        {
            flags |= (uint8_t)ESP32_CAP_FLAG_EOL;
        }

        esp32_cap_add(ch, flags);        /* 原始缓冲: 收到的每个字节都留一份 */

        if (ringbuf_push(&s_rx, ch) == 0U)
        {
            esp32_cap_mark((uint8_t)ESP32_CAP_FLAG_BUFOVF);
        }
        s_rx_total++;

        if (((ch == (uint8_t)'\n') || (ch == (uint8_t)'\r')) || (burst_start != 0U))
        {
            bsp_signal_set(&s_rx_sig);
        }
        return;
    }

    /* 只有错误标志、没有 RXNE: 也必须"读 SR 再读 DR"才能清掉(否则中断会一直挂着) */
    if ((sr & ((uint32_t)USART_FLAG_ORE | (uint32_t)USART_FLAG_FE |
               (uint32_t)USART_FLAG_NE | (uint32_t)USART_FLAG_PE)) != 0U)
    {
        volatile uint16_t tmp = ESP32_UART->DR;

        (void)tmp;
        s_rx_errors++;
    }
}
#else
/*---------------------------------------------------------------------------*/
/** DMA 方式下收数不经过中断; 这里留个空实现: 万一有杂散中断也不会跑飞 */
void USART1_IRQHandler(void)
{
    /* 什么都不用做(字节由 DMA1_Channel5 直接搬进 s_dma_buf[]) */
}
#endif /* ESP32_RX_USE_DMA */
