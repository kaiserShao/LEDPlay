/**
 ******************************************************************************
 * @file    drv_wt2003h.c
 * @brief   WT2003HX 语音模块驱动 (USART3) —— 只保留实测可用的串口命令
 *
 *  发: wt2003_send_cmd() 按手册格式拼帧(校验码自动算),
 *      并保证两条命令之间至少 WT2003_CMD_GAP_MS(模块需要喘息);
 *  收: USART3 接收中断 -> 环形缓冲 -> wt2003_process() 解析
 *      "7E 长度 命令 数据… 校验码 EF", 把 0xC4(查文件总数)的应答存起来,
 *      并回调 wt2003_set_frame_handler() 注册的打印函数(排查用)。
 ******************************************************************************
 */
#include "drv_wt2003h.h"
#include "drv_ringbuf.h"
#include "bsp_rtos.h"
#include "bsp_time.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"

#define WT2003_FRAME_HEAD     0x7EU
#define WT2003_FRAME_TAIL     0xEFU
#define WT2003_FRAME_MAX      16U    /* 7E + 长度 + 命令 + 最多 4 字节数据 + 校验 + EF */

static ringbuf_t         s_rx;
static uint8_t           s_rx_buf[WT2003_RX_BUF_SIZE];
static bsp_mutex_t       s_tx_lock;
static volatile uint32_t s_cmd_count  = 0U;      /* 累计发出的命令帧数 */
static volatile uint32_t s_rx_total   = 0U;      /* 累计收到的字节数 */
static volatile uint16_t s_file_count = 0U;      /* 0xC4 查询结果 */
static uint8_t           s_volume     = WT2003_DEFAULT_VOLUME;
static uint32_t          s_last_cmd_ms = 0U;     /* 上一条命令发出的时刻(算命令间隔) */
static wt2003_frame_cb_t s_frame_cb   = NULL;
static wt2003_tx_cb_t    s_tx_cb      = NULL;

/*---------------------------------------------------------------------------*/
/** 发送 1 字节(带超时, 硬件异常时不会死等) */
static void wt2003_tx_byte(uint8_t data)
{
    uint32_t t0 = bsp_time_ms();

    while (USART_GetFlagStatus(WT2003_UART, USART_FLAG_TXE) == RESET)
    {
        if (bsp_elapsed_ms(t0) > WT2003_TX_TIMEOUT_MS)
        {
            return;
        }
    }
    USART_SendData(WT2003_UART, (uint16_t)data);
}

/*===========================================================================
 *                            拼帧 / 发帧
 *=========================================================================*/
/** 把一整帧(含 7E / 长度 / 校验码 / EF)原样发出去。
 *  wt2003_send_cmd() 与 wt2003_send_raw() 共用这条路, 所以"命令间隔 / 发送超时
 *  保护 / 命令计数 / WTTX 回调"对两条路都一样。 */
static uint8_t wt2003_tx_frame(const uint8_t *frame, uint8_t len)
{
    uint8_t  i;
    uint32_t t0;

    if ((frame == NULL) || (len == 0U))
    {
        return 1U;
    }

    /* 命令间隔: 距上一条命令不足 WT2003_CMD_GAP_MS 就等一下(最长 30ms)。
       发得太密模块会把命令丢掉, 表现为"命令发出去了但没反应" */
    if (s_last_cmd_ms != 0U)
    {
        uint32_t dt = bsp_elapsed_ms(s_last_cmd_ms);

        if (dt < (uint32_t)WT2003_CMD_GAP_MS)
        {
            bsp_sleep_ms((uint32_t)WT2003_CMD_GAP_MS - dt);
        }
    }

    bsp_mutex_lock(&s_tx_lock);                 /* 多任务时防止两帧被交错发出 */
    for (i = 0U; i < len; i++)
    {
        wt2003_tx_byte(frame[i]);
    }
    t0 = bsp_time_ms();
    while (USART_GetFlagStatus(WT2003_UART, USART_FLAG_TC) == RESET)
    {
        if (bsp_elapsed_ms(t0) > WT2003_TX_TIMEOUT_MS)
        {
            break;
        }
    }
    bsp_mutex_unlock(&s_tx_lock);

    s_last_cmd_ms = bsp_time_ms();
    s_cmd_count++;

    if (s_tx_cb != NULL)                        /* 调试打印放在解锁之后 */
    {
        s_tx_cb(frame, len);
    }

    return 0U;
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_send_cmd(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t frame[WT2003_FRAME_MAX];
    uint8_t n   = 0U;
    uint8_t i;
    uint8_t sum = 0U;

    /* 整帧 = 7E + 长度 + 命令 + N 数据 + 校验码 + EF = N + 5 */
    if (((uint16_t)len + 5U) > (uint16_t)WT2003_FRAME_MAX)
    {
        return 1U;
    }
    if ((data == NULL) && (len != 0U))
    {
        return 1U;
    }

    frame[n] = WT2003_FRAME_HEAD;
    n++;
    frame[n] = (uint8_t)((uint16_t)len + 3U);   /* 长度 = 自身 + 命令 + 数据 + 校验码 */
    n++;
    frame[n] = cmd;
    n++;

    for (i = 0U; i < len; i++)
    {
        frame[n] = data[i];
        n++;
    }

    for (i = 1U; i < n; i++)                    /* 校验码 = 从"长度"字节起累加 */
    {
        sum = (uint8_t)(sum + frame[i]);
    }
    frame[n] = sum;
    n++;
    frame[n] = WT2003_FRAME_TAIL;
    n++;

    return wt2003_tx_frame(frame, n);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_send_raw(const uint8_t *frame, uint8_t len)
{
    /* 最短的一帧 = 7E 03 <cmd> <ck> EF = 5 字节; 最长受 WT2003_FRAME_MAX 限制。
       只查头尾, 校验码按调用者给的字节原样发出(想故意发错码做对照也行) */
    if ((frame == NULL) || (len < 5U) || (len > (uint8_t)WT2003_FRAME_MAX))
    {
        return 1U;
    }
    if ((frame[0] != WT2003_FRAME_HEAD) || (frame[len - 1U] != WT2003_FRAME_TAIL))
    {
        return 1U;
    }

    return wt2003_tx_frame(frame, len);
}

/*===========================================================================
 *                                初始化
 *=========================================================================*/
void wt2003_init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef  nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(WT2003_UART_CLK, ENABLE);

    /*--------------------------- USART3 --------------------------------*/
    gpio.GPIO_Pin   = WT2003_TX_PIN;          /* PB10 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(WT2003_TX_PORT, &gpio);

    gpio.GPIO_Pin   = WT2003_RX_PIN;          /* PB11 */
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(WT2003_RX_PORT, &gpio);

    ringbuf_init(&s_rx, s_rx_buf, WT2003_RX_BUF_SIZE);
    bsp_mutex_init(&s_tx_lock);               /* RTOS 资源(在 bsp_rtos_init() 之后) */

    uart.USART_BaudRate            = WT2003_BAUD;
    uart.USART_WordLength          = USART_WordLength_8b;
    uart.USART_StopBits            = USART_StopBits_1;
    uart.USART_Parity              = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(WT2003_UART, &uart);

    USART_ITConfig(WT2003_UART, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel                   = WT2003_UART_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = BSP_IRQ_PRIO_WT2003_UART;
    nvic.NVIC_IRQChannelSubPriority        = 0U;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(WT2003_UART, ENABLE);

    s_volume      = WT2003_DEFAULT_VOLUME;
    s_file_count  = 0U;
    s_last_cmd_ms = 0U;
}

/*===========================================================================
 *                            4 条命令
 *=========================================================================*/
uint8_t wt2003_set_volume(uint8_t vol)
{
    uint8_t data[1];

    if (vol > (uint8_t)WT2003_VOL_MAX)
    {
        vol = (uint8_t)WT2003_VOL_MAX;
    }
    data[0] = vol;

    if (wt2003_send_cmd((uint8_t)WT2003_CMD_SET_VOLUME, data, 1U) != 0U)
    {
        return 1U;
    }
    s_volume = vol;
    return 0U;
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_get_volume(void)
{
    return s_volume;
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_play_index(uint16_t index)
{
    uint8_t data[2];

    data[0] = (uint8_t)(index >> 8);            /* 高字节在前: 索引 1 -> 00 01 */
    data[1] = (uint8_t)(index & 0x00FFU);
    return wt2003_send_cmd((uint8_t)WT2003_CMD_PLAY_INDEX, data, 2U);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_play_name(const char *name4)
{
    uint8_t data[4];
    uint8_t i;

    if (name4 == NULL)
    {
        return 1U;
    }

    for (i = 0U; i < 4U; i++)
    {
        if (name4[i] == '\0')                   /* 名字必须正好 4 个字符 */
        {
            return 1U;
        }
        data[i] = (uint8_t)name4[i];
    }
    return wt2003_send_cmd((uint8_t)WT2003_CMD_PLAY_NAME, data, 4U);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_set_mode(uint8_t mode)
{
    uint8_t data[1];

    if (mode > (uint8_t)WT2003_MODE_RANDOM)
    {
        mode = (uint8_t)WT2003_MODE_SINGLE;
    }
    data[0] = mode;

    /* 单曲循环 = 7E 04 AF 01 B4 EF(校验码由 wt2003_send_cmd() 自动算) */
    return wt2003_send_cmd((uint8_t)WT2003_CMD_SET_MODE, data, 1U);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_stop(void)
{
    /* 7E 03 AA AD EF */
    return wt2003_send_cmd((uint8_t)WT2003_CMD_STOP, NULL, 0U);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_insert_ex(uint16_t index, uint8_t flag)
{
    uint8_t data[3];

    /* 插播: 叠在正在播的背景音上放, 背景音不停也不重头。
       帧 = 7E 06 B1 <标示字> <曲目高> <曲目低> <校验码> EF(手册 4.1.2.14);
       例 标示字 00(外挂 SPI Flash) + 曲目 3 -> 7E 06 B1 00 00 03 BA EF */
    data[0] = flag;
    data[1] = (uint8_t)(index >> 8);
    data[2] = (uint8_t)(index & 0x00FFU);
    return wt2003_send_cmd((uint8_t)WT2003_CMD_INSERT, data, 3U);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_insert_index(uint16_t index)
{
    return wt2003_insert_ex(index, (uint8_t)WT2003_INSERT_FLAG);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_query_count(void)
{
    return wt2003_send_cmd((uint8_t)WT2003_CMD_QUERY_COUNT, NULL, 0U);
}

/*---------------------------------------------------------------------------*/
uint16_t wt2003_get_count(void)
{
    return s_file_count;
}

/*===========================================================================
 *                          收帧解析(主循环调用)
 *=========================================================================*/
void wt2003_process(void)
{
    static uint8_t s_state = 0U;                 /* 0 = 找帧头 / 1 = 收长度 / 2 = 收数据 */
    static uint8_t s_len   = 0U;                 /* 本帧"长度"字节的值 */
    static uint8_t s_idx   = 0U;                 /* 已收字节数(命令 + 数据 + 校验码) */
    static uint8_t s_buf[WT2003_RX_FRAME_MAX];
    uint8_t        budget  = 32U;                /* 一次最多处理 32 字节, 不长时间霸占任务 */
    uint8_t        ch;

    while ((budget > 0U) && (ringbuf_pop(&s_rx, &ch) != 0U))
    {
        budget--;

        switch (s_state)
        {
            case 0U:                             /* 找帧头 7E */
                if (ch == WT2003_FRAME_HEAD)
                {
                    s_state = 1U;
                }
                break;

            case 1U:                             /* 长度: 最小 3 = 自身 + 命令 + 校验码 */
                if (ch < 3U)
                {
                    s_state = (ch == WT2003_FRAME_HEAD) ? 1U : 0U;
                }
                else if ((uint8_t)(ch - 1U) > (uint8_t)WT2003_RX_FRAME_MAX)
                {
                    s_state = 0U;                /* 长度不合理: 丢掉重找 */
                }
                else
                {
                    s_len   = ch;
                    s_idx   = 0U;
                    s_state = 2U;
                }
                break;

            case 2U:                             /* (长度-1) 字节: 命令 + 数据 + 校验码 */
                if (s_idx < (uint8_t)(s_len - 1U))
                {
                    s_buf[s_idx] = ch;
                    s_idx++;
                }
                else
                {
                    /* 本字节应是帧尾; 对不上就把这一字节当作下一帧的帧头重新开始找 */
                    if ((ch == WT2003_FRAME_TAIL) && (s_idx >= 2U))
                    {
                        uint8_t cmd  = s_buf[0];
                        uint8_t dlen = (uint8_t)(s_idx - 2U);   /* 去掉命令码与校验码 */

                        if ((cmd == (uint8_t)WT2003_CMD_QUERY_COUNT) && (dlen >= 1U))
                        {
                            /* 文件总数: 手册 1 字节; 若模块回 2 字节则按大端处理 */
                            s_file_count = (dlen >= 2U)
                                           ? (uint16_t)(((uint16_t)s_buf[1] << 8) | (uint16_t)s_buf[2])
                                           : (uint16_t)s_buf[1];
                        }

                        if (s_frame_cb != NULL)
                        {
                            s_frame_cb(cmd, &s_buf[1], dlen);
                        }
                    }
                    s_state = (ch == WT2003_FRAME_HEAD) ? 1U : 0U;
                }
                break;

            default:
                s_state = 0U;
                break;
        }
    }
}

/*===========================================================================
 *                            回调 / 统计
 *=========================================================================*/
void wt2003_set_frame_handler(wt2003_frame_cb_t cb)
{
    s_frame_cb = cb;
}

/*---------------------------------------------------------------------------*/
void wt2003_set_tx_handler(wt2003_tx_cb_t cb)
{
    s_tx_cb = cb;
}

/*---------------------------------------------------------------------------*/
uint32_t wt2003_cmd_count(void)
{
    return s_cmd_count;
}

/*---------------------------------------------------------------------------*/
uint32_t wt2003_rx_total(void)
{
    return s_rx_total;
}

/*===========================================================================
 *                              串口中断
 *=========================================================================*/
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(WT2003_UART, USART_IT_RXNE) != RESET)
    {
        uint8_t ch = (uint8_t)(USART_ReceiveData(WT2003_UART) & 0x00FFU);

        (void)ringbuf_push(&s_rx, ch);
        s_rx_total++;
    }

    if (USART_GetFlagStatus(WT2003_UART, USART_FLAG_ORE) != RESET)
    {
        volatile uint16_t tmp;

        tmp = WT2003_UART->SR;                  /* 清溢出标志: 先读 SR 再读 DR */
        tmp = WT2003_UART->DR;
        (void)tmp;
    }
}


