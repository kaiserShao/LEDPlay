/**
  ******************************************************************************
  * @file    drv_wt2003h.c
  * @brief   WT2003H4-16S 语音模块驱动 (串口 + Key 口, 统一播放入口)
  ******************************************************************************
  */
#include "drv_wt2003h.h"
#include "drv_ringbuf.h"
#include "bsp_rtos.h"
#include "bsp_time.h"
#include "bsp_util.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"

#define WT2003_FRAME_MAX      16U
#define WT2003_FRAME_HEAD     0x7EU
#define WT2003_FRAME_TAIL     0xEFU
#if (WT2003_KEY_ENABLE != 0U)
#define WT2003_KEY_MASK       (uint16_t)(WT2003_KEY1_PIN | WT2003_KEY2_PIN | WT2003_KEY3_PIN)

/** K1/K2/K3 引脚 */
static const uint16_t s_key_pin[WT2003_KEY_COUNT] =
{
    WT2003_KEY1_PIN,
    WT2003_KEY2_PIN,
    WT2003_KEY3_PIN
};
#endif /* WT2003_KEY_ENABLE */

/** voice_id -> Key 口 绑定表(业务层可通过 wt2003_bind_key 修改) */
typedef struct
{
    uint16_t id;        /*!< 0 表示空槽 */
    uint8_t  key;       /*!< 0~2 */
} wt2003_route_t;

static wt2003_route_t s_route[WT2003_KEY_ROUTE_NUM];

static volatile uint8_t  s_key_busy[WT2003_KEY_COUNT];
static volatile uint32_t s_key_start_ms[WT2003_KEY_COUNT];

static ringbuf_t         s_rx;
static uint8_t           s_rx_buf[WT2003_RX_BUF_SIZE];

/* 最近一帧"收完整"的原始回帧(含 7E / 长度 / 校验码 / EF), 供 wt2003_last_rx_raw() 取用:
   调试器里看 s_rx_buf 会看到环形缓冲的原貌(还可能带上一帧的残留字节), 这里留一份
   从帧头到帧尾的干净副本, 可直接和手册的"发/收"示例逐字节对照。 */
static uint8_t           s_last_raw[WT2003_RX_FRAME_MAX + 3U];
static uint8_t           s_last_raw_len = 0U;

static volatile uint32_t s_cmd_count = 0U;
static volatile uint32_t s_rx_total  = 0U;
static uint8_t           s_volume    = WT2003_DEFAULT_VOLUME;
static wt2003_frame_cb_t s_frame_cb  = NULL;
static wt2003_tx_cb_t    s_tx_cb     = NULL;
static wt2003_bad_frame_cb_t s_bad_cb = NULL;   /* 结构不对的帧(帧尾非 EF / 超长)上报, 排查用 */

/* 串口发送保护: 命令应答、音量、曲目…可能来自不同任务 */
static bsp_mutex_t       s_tx_lock;

/* 播放请求队列: 业务任务投递 -> 语音任务执行(避免多任务抢 Key 口/串口) */
static bsp_queue_t       s_req_q;
static uint8_t           s_task_mode = 0U;   /*!< 1 = 由语音任务执行播放 */

/*---------------------------------------------------------------------------*/
/** 发送 1 字节(带超时) */
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

/*---------------------------------------------------------------------------*/
uint8_t wt2003_send_raw(const uint8_t *frame, uint8_t len)
{
    uint8_t  i;
    uint32_t t0;

    if ((frame == NULL) || (len == 0U))
    {
        return 1U;
    }

    bsp_mutex_lock(&s_tx_lock);          /* 防止多任务把帧交错发送 */

    for (i = 0U; i < len; i++)
    {
        wt2003_tx_byte(frame[i]);
    }

    /* 等发送完成(带超时, 避免硬件异常时死等) */
    t0 = bsp_time_ms();
    while (USART_GetFlagStatus(WT2003_UART, USART_FLAG_TC) == RESET)
    {
        if (bsp_elapsed_ms(t0) > WT2003_TX_TIMEOUT_MS)
        {
            break;
        }
    }

    bsp_mutex_unlock(&s_tx_lock);

    /* 排查用: 把"真正发出去的字节"原样回调出去。
       放在解锁之后, 免得 115200 的打印拖慢模块命令的节奏。 */
    if (s_tx_cb != NULL)
    {
        s_tx_cb(frame, len);
    }

    s_cmd_count++;
    return 0U;
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_send_cmd(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t frame[WT2003_FRAME_MAX];
    uint8_t n = 0U;
    uint8_t i;

    if ((uint8_t)(len + 5U) > WT2003_FRAME_MAX)
    {
        return 1U;
    }

    frame[n] = WT2003_FRAME_HEAD;
    n++;
    /* 长度 = 命令(1) + 数据(len) [+ 校验和(1)] [+ 长度字节自己(1), 模块方言] */
    frame[n] = (uint8_t)(1U + len
#if (WT2003_LEN_COUNT_SELF != 0U)
                         + 1U
#endif
#if (WT2003_USE_CHECKSUM != 0U)
                         + 1U
#endif
                         );
    n++;
    frame[n] = cmd;
    n++;

    for (i = 0U; i < len; i++)
    {
        frame[n] = data[i];
        n++;
    }

#if (WT2003_USE_CHECKSUM != 0U)
    {
        uint8_t sum = 0U;
        uint8_t j;

        for (j = 1U; j < n; j++)      /* 从"长度"字节开始累加 */
        {
            sum = (uint8_t)(sum + frame[j]);
        }
        frame[n] = sum;
        n++;
    }
#endif

    frame[n] = WT2003_FRAME_TAIL;
    n++;

    return wt2003_send_raw(frame, n);
}

/*===========================================================================
 *                                初始化
 *=========================================================================*/
void wt2003_init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef  nvic;
    uint8_t           i;

    RCC_APB2PeriphClockCmd(WT2003_KEY_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(WT2003_UART_CLK, ENABLE);

    /*----------------------- K1/K2/K3: 开漏输出 -------------------------
      开漏 + 输出寄存器置 1 => 引脚高阻(由模块侧上拉), 等效"松开按键";
      输出寄存器置 0 => 拉低, 等效"按下按键"。这样不会和模块内部电路打架。
      仅 WT2003_KEY_ENABLE = 1 (模块已定制 Key 口) 时才初始化。 */
#if (WT2003_KEY_ENABLE != 0U)
    GPIO_SetBits(WT2003_KEY_PORT, WT2003_KEY_MASK);
    gpio.GPIO_Pin   = WT2003_KEY_MASK;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_Init(WT2003_KEY_PORT, &gpio);
#endif

    for (i = 0U; i < WT2003_KEY_COUNT; i++)
    {
        s_key_busy[i]     = 0U;
        s_key_start_ms[i] = 0U;
    }

    /*--------------------------- USART3 --------------------------------*/
    gpio.GPIO_Pin   = WT2003_TX_PIN;          /* PB10 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(WT2003_TX_PORT, &gpio);

    gpio.GPIO_Pin   = WT2003_RX_PIN;          /* PB11 */
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(WT2003_RX_PORT, &gpio);

    ringbuf_init(&s_rx, s_rx_buf, WT2003_RX_BUF_SIZE);

    /* RTOS 资源(必须在 bsp_rtos_init() 之后创建, main 里已保证顺序) */
    bsp_mutex_init(&s_tx_lock);
    bsp_queue_init(&s_req_q);
    s_task_mode = 0U;

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

    /*----------------------- 默认绑定关系 -------------------------------
       Key 口启用时: 上电默认 语音编号 1/2/3 分别走 K1/K2/K3 (与模块上位机
       配置的按键文件对应)。业务层可用 wt2003_bind_key() 改成别的编号。
       Key 口关闭时(WT2003_KEY_ENABLE = 0, 标准品无此功能): 绑定表保持空表,
       wt2003_get_key_of() 一律返回 0xFF -> wt2003_talk() 全部走串口 A0,
       不会出现"编号 1/2/3 走 Key 口 -> 静默无反应"。 */
    for (i = 0U; i < WT2003_KEY_ROUTE_NUM; i++)
    {
        s_route[i].id  = 0U;
        s_route[i].key = 0xFFU;
    }
    s_volume = WT2003_DEFAULT_VOLUME;
#if (WT2003_KEY_ENABLE != 0U)
    (void)wt2003_bind_key(1U, 0U);
    (void)wt2003_bind_key(2U, 1U);
    (void)wt2003_bind_key(3U, 2U);
#endif
}

/*===========================================================================
 *                            Key 口(硬件按键)
 *=========================================================================*/
void wt2003_key_pulse(uint8_t key_idx)
{
#if (WT2003_KEY_ENABLE == 0U)
    /* Key 口未启用(标准品 WT2003H4-16S 没开放此功能, 需厂家定制): 不动引脚 */
    (void)key_idx;
#else
    if (key_idx >= WT2003_KEY_COUNT)
    {
        return;
    }
    if (s_key_busy[key_idx] != 0U)
    {
        return;                      /* 上一脉冲还没结束, 忽略本次 */
    }

    GPIO_ResetBits(WT2003_KEY_PORT, s_key_pin[key_idx]);   /* 拉低 = 按下 */
    s_key_start_ms[key_idx] = bsp_time_ms();
    s_key_busy[key_idx]     = 1U;
#endif
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_key_available(void)
{
    return (WT2003_KEY_ENABLE != 0U) ? 1U : 0U;
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_key_is_busy(uint8_t key_idx)
{
    if (key_idx >= WT2003_KEY_COUNT)
    {
        return 0U;
    }
    return s_key_busy[key_idx];
}

/*===========================================================================
 *                        voice_id -> Key 口 绑定表
 *=========================================================================*/
uint8_t wt2003_bind_key(uint16_t voice_id, uint8_t key_idx)
{
    uint8_t i;

    if (voice_id == 0U)
    {
        return 1U;
    }

    /* 先删除旧的绑定 */
    for (i = 0U; i < WT2003_KEY_ROUTE_NUM; i++)
    {
        if (s_route[i].id == voice_id)
        {
            if (key_idx == 0xFFU)
            {
                s_route[i].id  = 0U;
                s_route[i].key = 0xFFU;
                return 0U;
            }
            s_route[i].key = key_idx;
            return 0U;
        }
    }

    if (key_idx == 0xFFU)
    {
        return 0U;                    /* 本来就没绑定 */
    }

    for (i = 0U; i < WT2003_KEY_ROUTE_NUM; i++)
    {
        if (s_route[i].id == 0U)
        {
            s_route[i].id  = voice_id;
            s_route[i].key = key_idx;
            return 0U;
        }
    }
    return 2U;                        /* 表满 */
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_get_key_of(uint16_t voice_id)
{
    uint8_t i;

    for (i = 0U; i < WT2003_KEY_ROUTE_NUM; i++)
    {
        if (s_route[i].id == voice_id)
        {
            return s_route[i].key;
        }
    }
    return 0xFFU;
}

/*===========================================================================
 *                        统一语音播放接口
 *=========================================================================*/
uint8_t wt2003_talk(uint16_t voice_id)
{
    return wt2003_talk_src(voice_id, WT2003_SRC_AUTO);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_talk_src(uint16_t voice_id, wt2003_src_t src)
{
    uint8_t key = wt2003_get_key_of(voice_id);

#if (WT2003_KEY_ENABLE == 0U)
    /* Key 口未启用: 即使调用方点名要 Key 口, 也退回串口(A0)播放,
       否则就是"命令发了、什么都没发生"的静默故障 */
    if (src == WT2003_SRC_KEY)
    {
        src = WT2003_SRC_UART;
    }
#endif

    /* 队列模式(语音任务已运行): 任何任务调用都只是"投递请求", 立刻返回;
       真正的 Key 脉冲 / 串口发送由语音任务串行执行 -> 模块只有一个"主人" */
    if (s_task_mode != 0U)
    {
        return (bsp_queue_put(&s_req_q, WT2003_REQ_PACK(voice_id, src)) != 0U) ? 0U : 2U;
    }

    /* 非队列模式: 就地执行(无 RTOS 时的行为, 与原来完全一致) */
    if ((src == WT2003_SRC_KEY)
        || ((src == WT2003_SRC_AUTO) && (key != 0xFFU) && (key < WT2003_KEY_COUNT)))
    {
#if (WT2003_KEY_ENABLE != 0U)
        if (key >= WT2003_KEY_COUNT)
        {
            return 1U;                /* 要求走 Key 口但没绑定 */
        }
        wt2003_key_pulse(key);
        return 0U;
#else
        return 1U;                    /* Key 口未启用(标准品无此功能) */
#endif
    }

    /* 其余情况走串口, 播放指定曲目 */
    return wt2003_play_index(voice_id);
}

/*===========================================================================
 *                         播放请求队列(语音任务侧)
 *=========================================================================*/
void wt2003_use_task(uint8_t enable)
{
    s_task_mode = (enable != 0U) ? 1U : 0U;
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_take_request(uint32_t *req, uint32_t timeout_ms)
{
    if (req == NULL)
    {
        return 0U;
    }

    /* RTOS: 阻塞到有请求或有超时(不占 CPU); 无 RTOS: 轮询 */
    return bsp_queue_get(&s_req_q, req, timeout_ms);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_exec_request(uint32_t req)
{
    uint16_t voice_id = WT2003_REQ_ID(req);
    uint8_t  src      = WT2003_REQ_SRC(req);
    uint8_t  key      = wt2003_get_key_of(voice_id);

#if (WT2003_KEY_ENABLE == 0U)
    /* Key 口未启用: 点名要 Key 口也退回串口, 保证"要播就能播" */
    if (src == (uint8_t)WT2003_SRC_KEY)
    {
        src = (uint8_t)WT2003_SRC_UART;
    }
#endif

    if ((src == (uint8_t)WT2003_SRC_KEY)
        || ((src == (uint8_t)WT2003_SRC_AUTO) && (key != 0xFFU) && (key < WT2003_KEY_COUNT)))
    {
#if (WT2003_KEY_ENABLE != 0U)
        if (key >= WT2003_KEY_COUNT)
        {
            return 1U;
        }

        /* 阻塞式 Key 脉冲: 只让"当前任务"睡 WT2003_KEY_PULSE_MS,
           其它任务(灯带/串口/按键)照常运行 */
        GPIO_ResetBits(WT2003_KEY_PORT, s_key_pin[key]);   /* 拉低 = 按下 */
        s_key_busy[key]     = 1U;
        s_key_start_ms[key] = 0U;      /* 0 = 不由 wt2003_process() 释放 */
        bsp_sleep_ms(WT2003_KEY_PULSE_MS);
        GPIO_SetBits(WT2003_KEY_PORT, s_key_pin[key]);     /* 释放 */
        s_key_busy[key] = 0U;
        return 0U;
#else
        return 1U;                    /* Key 口未启用(标准品无此功能) */
#endif
    }

    /* 走串口: 播放指定曲目 */
    return wt2003_play_index(voice_id);
}

/*===========================================================================
 *                            串口命令封装
 *=========================================================================*/
uint8_t wt2003_play_index(uint16_t index)
{
#if (WT2003_PLAY_USE_UDISK != 0U)
    return wt2003_play_udisk(index);       /* A6: 播 U 盘根目录索引 */
#else
    return wt2003_play_flash(index);       /* A0: 播 SPI Flash 根目录索引 */
#endif
}

/*---------------------------------------------------------------------------*/
/** A0: 指定 SPI Flash 根目录索引播放(数据 = 曲目高位/低位)
    -> 7E 05 A0 <高> <低> <校验> EF, 例 索引1 : 7E 05 A0 00 01 A6 EF */
uint8_t wt2003_play_flash(uint16_t index)
{
    uint8_t data[2];

    data[0] = (uint8_t)(index >> 8);
    data[1] = (uint8_t)(index & 0x00FFU);
    return wt2003_send_cmd(WT2003_CMD_PLAY_INDEX, data, 2U);
}

/*---------------------------------------------------------------------------*/
/** A6: 指定 U 盘根目录索引播放(数据 = 曲目高位/低位)
    -> 7E 05 A6 <高> <低> <校验> EF, 例 索引2 : 7E 05 A6 00 02 AD EF */
uint8_t wt2003_play_udisk(uint16_t index)
{
    uint8_t data[2];

    data[0] = (uint8_t)(index >> 8);
    data[1] = (uint8_t)(index & 0x00FFU);
    return wt2003_send_cmd(WT2003_CMD_PLAY_UFLASH, data, 2U);
}

/*---------------------------------------------------------------------------*/
/** A7: 指定 U 盘根目录文件名播放(数据 = 4 字节 ASCII 文件名, 不含扩展名)
    -> 7E 07 A7 54 30 30 32 <校验> EF, 播 "T002.mp3" */
uint8_t wt2003_play_udisk_name(const char *name4)
{
    uint8_t data[4];
    uint8_t i;

    if (name4 == NULL)
    {
        return 1U;
    }
    for (i = 0U; i < 4U; i++)
    {
        data[i] = (uint8_t)name4[i];
    }
    return wt2003_send_cmd(WT2003_CMD_PLAY_UNAME, data, 4U);
}

/*---------------------------------------------------------------------------*/
/** A8: 指定 U 盘文件夹内文件索引播放(数据 = 5 字节文件夹名 + 2 字节索引)
    -> 7E 0A A8 4D 55 53 49 43 00 01 <校验> EF, 播 "MUSIC" 内第 2 首 */
uint8_t wt2003_play_udisk_folder(const char *folder5, uint16_t index)
{
    uint8_t data[7];
    uint8_t i;

    if (folder5 == NULL)
    {
        return 1U;
    }
    for (i = 0U; i < 5U; i++)
    {
        data[i] = (uint8_t)folder5[i];
    }
    data[5] = (uint8_t)(index >> 8);
    data[6] = (uint8_t)(index & 0x00FFU);
    return wt2003_send_cmd(WT2003_CMD_PLAY_UFOLDER, data, 7U);
}

/*---------------------------------------------------------------------------*/
/** D2: 切换当前工作盘符(数据 1 字节: 00 = SPI-Flash / 02 = U 盘)
    -> 7E 04 D2 02 D8 EF, 收 D2 00。
    手册 4.1.3.16: 影响 AC/AD(下一曲/上一曲)、循环播放这些"跟着当前盘走"的命令;
    想强制指定介质时仍可用 A0/A6/A7/A8。 */
uint8_t wt2003_select_drive(uint8_t drive)
{
    uint8_t data[1];

    data[0] = (drive == (uint8_t)WT2003_DRIVE_UDISK) ? (uint8_t)WT2003_DRIVE_UDISK
                                                     : (uint8_t)WT2003_DRIVE_FLASH;
    return wt2003_send_cmd(WT2003_CMD_SET_DRIVE, data, 1U);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_play_next(void)
{
    return wt2003_send_cmd(WT2003_CMD_PLAY_NEXT, NULL, 0U);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_play_prev(void)
{
    return wt2003_send_cmd(WT2003_CMD_PLAY_PREV, NULL, 0U);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_pause(void)
{
    return wt2003_send_cmd(WT2003_CMD_PAUSE, NULL, 0U);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_stop(void)
{
    return wt2003_send_cmd(WT2003_CMD_STOP, NULL, 0U);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_set_volume(uint8_t vol)
{
    uint8_t data[1];

    if (vol > WT2003_VOL_MAX)
    {
        vol = WT2003_VOL_MAX;
    }
    data[0] = vol;

    if (wt2003_send_cmd(WT2003_CMD_SET_VOLUME, data, 1U) == 0U)
    {
        s_volume = vol;
        return 0U;
    }
    return 1U;
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_get_volume(void)
{
    return s_volume;
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_set_mode(uint8_t mode)
{
    uint8_t data[1];

    data[0] = mode;
    return wt2003_send_cmd(WT2003_CMD_SET_MODE, data, 1U);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_play_combo(const uint8_t *info, uint8_t len)
{
    if ((info == NULL) || (len == 0U))
    {
        return 1U;
    }
    return wt2003_send_cmd(WT2003_CMD_PLAY_COMBO, info, len);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_insert(const uint8_t *info, uint8_t len)
{
    if ((info == NULL) || (len == 0U))
    {
        return 1U;
    }
    return wt2003_send_cmd(WT2003_CMD_INSERT, info, len);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_set_output(wt2003_out_t out)
{
    uint8_t data[1];

    data[0] = (uint8_t)out;
    return wt2003_send_cmd(WT2003_CMD_SET_OUTPUT, data, 1U);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_low_power(uint8_t mode)
{
    uint8_t data[1];

    data[0] = mode;
    return wt2003_send_cmd(WT2003_CMD_LOW_POWER, data, 1U);
}

/*---------------------------------------------------------------------------*/
/** B7: Flash 引脚加载 / 释放(数据 1 字节: 00 = 释放引脚 / 01 = 加载引脚)
    -> 7E 04 B7 00 <校验> EF(手册 4.1.4.15) */
uint8_t wt2003_flash_pin(uint8_t load)
{
    uint8_t data[1];

    data[0] = (load != 0U) ? 1U : 0U;
    return wt2003_send_cmd(WT2003_CMD_FLASH_PIN, data, 1U);
}

/*===========================================================================
 *                              查询命令(手册 4.1.2 通信查询命令)
 *  只负责发帧; 模块的应答由 wt2003_set_frame_handler() 的回调上报。
 *=========================================================================*/
uint8_t wt2003_query_version(void)
{
    return wt2003_send_cmd(WT2003_CMD_GET_VERSION, NULL, 0U);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_query_volume(void)
{
    return wt2003_send_cmd(WT2003_CMD_GET_VOLUME, NULL, 0U);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_query_status(void)
{
    return wt2003_send_cmd(WT2003_CMD_GET_STATUS, NULL, 0U);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_query_file_num(void)
{
    return wt2003_send_cmd(WT2003_CMD_GET_FILE_NUM, NULL, 0U);
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_query_cur_file(void)
{
    return wt2003_send_cmd(WT2003_CMD_GET_CUR_FILE, NULL, 0U);
}

/*---------------------------------------------------------------------------*/
/** C7: 查询 U 盘内音乐文件总数(应答 2 字节, 高字节在前)
    -> 7E 03 C7 CA EF, 收 C7 00 07 = U 盘上 7 个文件 */
uint8_t wt2003_query_udisk_num(void)
{
    return wt2003_send_cmd(WT2003_CMD_GET_UDISK_NUM, NULL, 0U);
}

/*---------------------------------------------------------------------------*/
/** C8: 查询 U 盘指定文件夹内音乐文件总数(数据 = 5 字节 ASCII 文件夹名)
    -> 7E 08 C8 4D 55 53 49 43 <校验> EF, 收 C8 00 03 = MUSIC 内 3 个文件;
       收 C8 00 00 = 没有该文件夹或里面没有音频文件 */
uint8_t wt2003_query_ufolder_num(const char *folder5)
{
    uint8_t data[5];
    uint8_t i;

    if (folder5 == NULL)
    {
        return 1U;
    }
    for (i = 0U; i < 5U; i++)
    {
        data[i] = (uint8_t)folder5[i];
    }
    return wt2003_send_cmd(WT2003_CMD_GET_UFOLDER_NUM, data, 5U);
}

/*---------------------------------------------------------------------------*/
/** CA: 查询当前外设连接状态(应答 1 字节)
    -> 7E 03 CA CD EF, 收 CA 04 = 有 U 盘(bit2=1);
       返回值低 4 位: bit3 = PC 连接, bit2 = U 盘, bit0 = SPI-Flash (1 = 存在) */
uint8_t wt2003_query_conn(void)
{
    return wt2003_send_cmd(WT2003_CMD_GET_CONN, NULL, 0U);
}

/*---------------------------------------------------------------------------*/
/** CB: 查询当前播放歌曲名(应答 8 字节 ASCII, 不足的用 0x20 补齐)
    -> 7E 03 CB CE EF, 收 CB 46 30 31 31 2E 6D 70 33 = "F011.mp3" */
uint8_t wt2003_query_cur_name(void)
{
    return wt2003_send_cmd(WT2003_CMD_GET_CUR_NAME, NULL, 0U);
}

/*===========================================================================
 *                语音链路自检序列(上电自动跑 / 远程命令触发, 排查用)
 *  用法: WT2003_PROBE_ON_BOOT = 1 时上电自动把下面 38 步跑完, 每一步发出去的
 *        原始字节(WTTX) + 模块回帧(WTRXRAW/WTRX) + 解析成人话(WTRXINFO) 都会
 *        从 ESP32 串口打出来, 不需要上位机、也不需要在手机端敲任何命令;
 *        数码管同步显示"正在试第几步"(0001~0038), 听到声音就记下数字。
 *
 *  !!! 本序列已按《WT2003H4-16S》手册(4.1.2 命令列表)全面改成"U 盘口径" !!!
 *      本工程音频放在 U 盘上, 所以:
 *        - "还剩几个文件"必须用 C7(查询 U 盘内音乐文件总数), 不是 C3
 *          —— 实测本模块 C3 也只回 1 字节、和 C7 同一个口径, 别拿它判介质;
 *        - "盘到底认到没有"用 CA(外设连接状态): bit2 = 1 才说明模块真的认到了 U 盘;
 *        - 试播一律 A6(指定 U 盘根目录索引播放), 绝不用 A0(那是 SPI-Flash);
 *        - 先发一条 D2 = 02 把"当前工作盘符"切到 U 盘, 免得 AC/AD/循环还在翻 SPI-Flash。
 *      已定位的现场事实: C0 回 WTS251016-70-A01V4.01(校验和完全吻合)
 *      => 协议/波特率/接线都是对的; 旧序列用 A0(SPI-Flash) 播索引必然全被拒
 *      (模块回 02 = 无此文件 / 目录为空) -> 全程无声。
 *
 *  本轮要定位的三件事:
 *      (甲) 介质: 第 6 步 CA 的 U 盘位 = 0, 或第 7 步 C7 回 00 00
 *           => 模块根本没认到 U 盘。先解决这个(换 U 盘 / 换 FAT/FAT32 格式 / 查
 *              USB 走线: D+ D- 与 5V, 必要时 12MHz 晶振), 否则后面发多少条 A6 都不会有声;
 *      (乙) 忙碌: 第 10 步 A6 非 00, 而第 15 步("暂停后重发") 回 00
 *           => 第 10 步是"忙碌拒绝", 即模块原本就在播;
 *      (丙) 输出: 第 20 步(DAC)/ 第 23 步(SPK) 哪一档出声, 就知道喇叭接法对不对
 *           —— SPK 是 BTL 桥接, 必须接 SPK+ 与 SPK- 两端, 一端接 GND 则永远无声。
 *
 *  步骤含义(按顺序):
 *    1     AB  停止播放    —— 清掉"卡在播放/忙"的状态
 *    2     D2  盘符 = U 盘 —— 让 AC/AD/循环这些"跟着当前盘走"的命令也作用在 U 盘
 *    3     B6  输出 = SPK  —— 本硬件喇叭接 SPK(必须接 SPK+ 与 SPK- 两端)
 *    4     AE  音量 = 31   —— 排除"音量被存成 0(静音)"
 *    5~7   C1 / CA / C7    —— 基准: 音量? 外设(有没有 U 盘)? U 盘上有几个文件?
 *    8~9   C9 / C2         —— 基准: 当前曲目号 / 工作状态(打印带 UNCHANGED/CHANGED!)
 *    10    A6  播放索引 1  —— 第 1 次试播(U 盘), 听有没有声
 *    11~12 C9 / C2         —— 当前文件号 / 状态有没有跟着变
 *    13~14 AA / C2         —— 把"自认为在播"的东西暂停掉, 再看状态
 *    15    A6  播放索引 1  —— 再来一次: 回 00 = 第 10 步的 02 是"忙碌拒绝"(其实在播)
 *    16~17 C9 / C2
 *    18    AB  停止
 *    19~20 B6 输出 = DAC / A6 再试播 —— 换另一条输出通路(喇叭若走 DAC/功放, 这步有声)
 *    21~22 AA / B6 输出 = SPK(切回)
 *    23~24 A6 再试播(SPK 模式) / C2
 *    25~28 AC / C9 / AD / C9 —— 换曲引擎: cur_file 会不会在 1 与 2 之间跳
 *    29    A6  播放索引 2  —— 扫 U 盘文件索引(回 00 = 该索引有文件可播)
 *    30    AB  停止         —— 收尾
 *    31    C3  查 Flash 文件数—— 和第 7 步 C7 对比(C3 = SPI-Flash, 不是 U 盘!)
 *    32    A6  播放索引 99  —— 必定不存在: 应回 02(EMP 无此文件), 用它验"返回码表对不对"
 *    33    C2  查状态       —— 这条"失败"的 A6 有没有影响状态
 *    34    A6  播放索引 0  —— 索引到底从 0 还是从 1 开始(听!)
 *    35    C2  查状态       —— 短间隔再查一次, 有没有变成 01 = 播放
 *    36    A0  播放索引 1  —— SPI-Flash 口径(Flash 里没文件时应回 02)
 *    37    C9  查当前文件
 *    38    C2  查状态       —— 收尾
 *
 *  探测由 wt2003_process() 驱动(每步之间等表里的 gap_ms, 0 = WT2003_PROBE_INTERVAL_MS),
 *  不阻塞、不占 CPU; 步骤编号通过 wt2003_probe_set_handler() 注册的回调抛出。
 *=========================================================================*/
typedef struct
{
    uint8_t        kind;        /* 0 = 发 Key 口脉冲(仅定制模块有), 1 = 发串口原始帧 */
    uint8_t        key;         /* kind=0 时: Key 口序号(0/1/2 = K1/K2/K3) */
    const uint8_t *frame;       /* kind=1 时: 原始帧内容 */
    uint8_t        len;         /* 原始帧长度 */
    uint16_t       gap_ms;      /* 发本步之前等的 ms(0 = 用 WT2003_PROBE_INTERVAL_MS) */
} wt2003_probe_step_t;

/* 串口候选项: 全部按手册格式 7E 长度 命令 数据 校验码 EF
   (长度含自身, 校验码 = 从长度字节起逐字节累加和) */
/* AB 停止播放 ................................. 7E 03 AB AE EF */
static const uint8_t s_probe_stop[] = {0x7EU, 0x03U, 0xABU, 0xAEU, 0xEFU};
/* AA 播放/暂停(停止状态下 = 开始播放当前曲目) ... 7E 03 AA AD EF */
static const uint8_t s_probe_pp[]   = {0x7EU, 0x03U, 0xAAU, 0xADU, 0xEFU};
/* AC 下一曲 .................................. 7E 03 AC AF EF */
static const uint8_t s_probe_next[] = {0x7EU, 0x03U, 0xACU, 0xAFU, 0xEFU};
/* AD 上一曲 .................................. 7E 03 AD B0 EF */
static const uint8_t s_probe_prev[] = {0x7EU, 0x03U, 0xADU, 0xB0U, 0xEFU};
/* B6 输出方式 = DAC .......................... 7E 04 B6 01 BB EF */
static const uint8_t s_probe_dac[]  = {0x7EU, 0x04U, 0xB6U, 0x01U, 0xBBU, 0xEFU};
/* B6 输出方式 = SPK(本硬件喇叭接 SPK) ......... 7E 04 B6 00 BA EF */
static const uint8_t s_probe_spk[]  = {0x7EU, 0x04U, 0xB6U, 0x00U, 0xBAU, 0xEFU};
/* AE 音量 = 31(最大) ......................... 7E 04 AE 1F D1 EF */
static const uint8_t s_probe_vol[]  = {0x7EU, 0x04U, 0xAEU, 0x1FU, 0xD1U, 0xEFU};
/* C1 查询当前音量 ............................ 7E 03 C1 C4 EF */
static const uint8_t s_probe_c1[]   = {0x7EU, 0x03U, 0xC1U, 0xC4U, 0xEFU};
/* C2 查询工作状态(播放中会变) ................. 7E 03 C2 C5 EF */
static const uint8_t s_probe_c2[]   = {0x7EU, 0x03U, 0xC2U, 0xC5U, 0xEFU};
/* D2 切换当前工作盘符 = U 盘(02) .............. 7E 04 D2 02 D8 EF */
static const uint8_t s_probe_d2[]   = {0x7EU, 0x04U, 0xD2U, 0x02U, 0xD8U, 0xEFU};
/* CA 查询当前外设连接状态(bit2 = 有 U 盘) ..... 7E 03 CA CD EF */
static const uint8_t s_probe_ca[]   = {0x7EU, 0x03U, 0xCAU, 0xCDU, 0xEFU};
/* C7 查询 U 盘内音乐文件总数(本工程看这条) .... 7E 03 C7 CA EF */
static const uint8_t s_probe_c7[]   = {0x7EU, 0x03U, 0xC7U, 0xCAU, 0xEFU};
/* C9 查询当前播放文件 ........................ 7E 03 C9 CC EF */
static const uint8_t s_probe_c9[]   = {0x7EU, 0x03U, 0xC9U, 0xCCU, 0xEFU};
/* A6 播放索引 1(指定 U 盘根目录索引; 本工程音频在 U 盘) 7E 05 A6 00 01 AC EF */
static const uint8_t s_probe_p1[]   = {0x7EU, 0x05U, 0xA6U, 0x00U, 0x01U, 0xACU, 0xEFU};
/* A6 播放索引 2(扫下一个索引试听) ........... 7E 05 A6 00 02 AD EF */
static const uint8_t s_probe_p2[]   = {0x7EU, 0x05U, 0xA6U, 0x00U, 0x02U, 0xADU, 0xEFU};
/* A6 播放索引 0(有的固件索引从 0 开始) ....... 7E 05 A6 00 00 AB EF */
static const uint8_t s_probe_p0[]   = {0x7EU, 0x05U, 0xA6U, 0x00U, 0x00U, 0xABU, 0xEFU};
/* A6 播放索引 99(必定不存在, 验证返回码 02) .... 7E 05 A6 00 63 0E EF */
static const uint8_t s_probe_pbad[] = {0x7EU, 0x05U, 0xA6U, 0x00U, 0x63U, 0x0EU, 0xEFU};
/* C3 查询 SPI-Flash 内音乐文件总数 ............ 7E 03 C3 C6 EF */
static const uint8_t s_probe_c3[]   = {0x7EU, 0x03U, 0xC3U, 0xC6U, 0xEFU};
/* A0 播放 SPI-Flash 根目录索引 1 .............. 7E 05 A0 00 01 A6 EF */
static const uint8_t s_probe_f1[]   = {0x7EU, 0x05U, 0xA0U, 0x00U, 0x01U, 0xA6U, 0xEFU};

/* 自检顺序(与 s_probe_hint[] 一一对应, 两表长度必须一致)
   核心思路(手册 4.1.2: 本工程音频在 U 盘, 所以全部按 U 盘口径发令):
   - "还剩几个文件"用 C7(查询 U 盘内音乐文件总数), 不是 C3 ——
     实测 C3 与 C7 同为 1 字节口径(回的都是 5), 不能当"另一种介质"的计数;
   - "模块认到 U 盘没有"用 CA(外设连接状态)的 bit2;
   - 试播一律用 A6(播 U 盘根目录索引), 绝不用 A0(那是 SPI-Flash);
   - 先发 D2 = 02 把"当前工作盘符"切到 U 盘, 免得 AC/AD/循环还在翻 SPI-Flash。
   本轮按"三件事"定位:
        (1) 介质: 第 6 步 CA 的 U 盘位 = 0, 或第 7 步 C7 回 00 00
            -> 模块根本没认到 U 盘(先解决这个, 后面发多少条 A6 都不会有声)。
        (2) 忙碌: 第 10 步 A6 非 00, 而第 15 步("暂停后重发") 回 00
            -> 第 10 步是"忙碌拒绝", 即模块原本就在播(听第 10 步到底有没有声)。
        (3) 输出: 第 20 步(DAC)/ 第 23 步(SPK) 哪一档出声
            -> 就知道喇叭接法 / B6 输出方式对不对。
   换曲引擎: 第 25/27 步 AC/AD 后紧跟 C9, cur_file 在 1 与 2 之间跳 = 引擎在跑。
   播放类步骤 gap_ms 给到 3500, 留出足够时间用耳朵判断有没有声。
   第 31~38 步是"返回码取证"(2026-07 实测 A6/D2/C7 都回 0x05 之后加的):
        - 第 31 步 C3 与第 7 步 C7 对比: C3=0 而 C7>0 = 音频确实在 U 盘, C3 不能用来看 U 盘;
        - 第 32 步发一个必定不存在的索引: 若回 02(EMP 无此文件), 说明手册 4.1.3.1 的
          返回码表在本固件上有效, 那么 0x05 只是"U 盘类命令的另一种应答", 不是失败;
          若也回 05, 说明本固件的返回码口径和手册完全不同, 一切以"听没听到声"为准;
        - 第 34 步 A6 索引 0: 索引到底从 0 还是从 1 开始(手册 4.1.4.5 的示例写"索引 0001 = 第 2 首");
        - 第 36 步 A0(Flash): Flash 里没文件时应回 02, 用来交叉验证第 31 步。 */
static const wt2003_probe_step_t s_probe_tab[] =
{
    {1U, 0U, s_probe_stop, (uint8_t)sizeof(s_probe_stop), 1500U},   /*  1: AB 停止 */
    {1U, 0U, s_probe_d2,   (uint8_t)sizeof(s_probe_d2),   1200U},   /*  2: D2 盘符 = U 盘 */
    {1U, 0U, s_probe_spk,  (uint8_t)sizeof(s_probe_spk),  1200U},   /*  3: B6 -> SPK */
    {1U, 0U, s_probe_vol,  (uint8_t)sizeof(s_probe_vol),  1200U},   /*  4: AE -> 音量 31 */
    {1U, 0U, s_probe_c1,   (uint8_t)sizeof(s_probe_c1),   1200U},   /*  5: C1 音量 */
    {1U, 0U, s_probe_ca,   (uint8_t)sizeof(s_probe_ca),   1200U},   /*  6: CA 外设连接(有 U 盘吗) */
    {1U, 0U, s_probe_c7,   (uint8_t)sizeof(s_probe_c7),   1200U},   /*  7: C7 U 盘文件数 */
    {1U, 0U, s_probe_c9,   (uint8_t)sizeof(s_probe_c9),   1200U},   /*  8: C9 当前文件(基准) */
    {1U, 0U, s_probe_c2,   (uint8_t)sizeof(s_probe_c2),   1200U},   /*  9: C2 状态(基准) */
    {1U, 0U, s_probe_p1,   (uint8_t)sizeof(s_probe_p1),   3500U},   /* 10: A6 索引1(U 盘) <-- 听! */
    {1U, 0U, s_probe_c9,   (uint8_t)sizeof(s_probe_c9),   1200U},   /* 11: C9 当前文件(动了吗) */
    {1U, 0U, s_probe_c2,   (uint8_t)sizeof(s_probe_c2),   1200U},   /* 12: C2 状态(变了吗) */
    {1U, 0U, s_probe_pp,   (uint8_t)sizeof(s_probe_pp),   2500U},   /* 13: AA 播放/暂停(把它暂停掉) */
    {1U, 0U, s_probe_c2,   (uint8_t)sizeof(s_probe_c2),   1200U},   /* 14: C2 状态(暂停了吗) */
    {1U, 0U, s_probe_p1,   (uint8_t)sizeof(s_probe_p1),   3500U},   /* 15: A6 索引1 再来 <-- 听! 回00=第10步是"忙碌" */
    {1U, 0U, s_probe_c9,   (uint8_t)sizeof(s_probe_c9),   1200U},   /* 16: C9 当前文件 */
    {1U, 0U, s_probe_c2,   (uint8_t)sizeof(s_probe_c2),   1200U},   /* 17: C2 状态 */
    {1U, 0U, s_probe_stop, (uint8_t)sizeof(s_probe_stop), 1200U},   /* 18: AB 停止 */
    {1U, 0U, s_probe_dac,  (uint8_t)sizeof(s_probe_dac),  1200U},   /* 19: B6 -> DAC */
    {1U, 0U, s_probe_p1,   (uint8_t)sizeof(s_probe_p1),   3500U},   /* 20: A6 索引1(DAC 模式) <-- 听! */
    {1U, 0U, s_probe_pp,   (uint8_t)sizeof(s_probe_pp),   2500U},   /* 21: AA 播放/暂停 */
    {1U, 0U, s_probe_spk,  (uint8_t)sizeof(s_probe_spk),  1200U},   /* 22: B6 -> SPK(切回) */
    {1U, 0U, s_probe_p1,   (uint8_t)sizeof(s_probe_p1),   3500U},   /* 23: A6 索引1(SPK 模式) <-- 听! */
    {1U, 0U, s_probe_c2,   (uint8_t)sizeof(s_probe_c2),   1200U},   /* 24: C2 状态 */
    {1U, 0U, s_probe_next, (uint8_t)sizeof(s_probe_next), 2500U},   /* 25: AC 下一曲(跟当前盘符) */
    {1U, 0U, s_probe_c9,   (uint8_t)sizeof(s_probe_c9),   1200U},   /* 26: C9 当前文件(变 2 了吗) */
    {1U, 0U, s_probe_prev, (uint8_t)sizeof(s_probe_prev), 2500U},   /* 27: AD 上一曲(跟当前盘符) */
    {1U, 0U, s_probe_c9,   (uint8_t)sizeof(s_probe_c9),   1200U},   /* 28: C9 当前文件(回 1 了吗) */
    {1U, 0U, s_probe_p2,   (uint8_t)sizeof(s_probe_p2),   3500U},   /* 29: A6 索引2(扫 U 盘) <-- 听! */
    {1U, 0U, s_probe_stop, (uint8_t)sizeof(s_probe_stop), 1200U},   /* 30: AB 停止(收尾) */
    /* ---- 31~38: 返回码取证(把"05 到底是什么"钉死) ---- */
    {1U, 0U, s_probe_c3,   (uint8_t)sizeof(s_probe_c3),   1200U},   /* 31: C3 文件数 <-- 和第 7 步 C7 对比(实测口径相同) */
    {1U, 0U, s_probe_pbad, (uint8_t)sizeof(s_probe_pbad), 1500U},   /* 32: A6 索引99(不存在) <-- 应回 02=EMP */
    {1U, 0U, s_probe_c2,   (uint8_t)sizeof(s_probe_c2),   1200U},   /* 33: C2 状态(失败的那条有影响吗) */
    {1U, 0U, s_probe_p0,   (uint8_t)sizeof(s_probe_p0),   3500U},   /* 34: A6 索引0 <-- 听! 索引是从 0 还是从 1 开始 */
    {1U, 0U, s_probe_c2,   (uint8_t)sizeof(s_probe_c2),   1200U},   /* 35: C2 状态(短间隔, 变 01=播放了吗) */
    {1U, 0U, s_probe_f1,   (uint8_t)sizeof(s_probe_f1),   1500U},   /* 36: A0 Flash 索引1 <-- Flash 没文件应回 02 */
    {1U, 0U, s_probe_c9,   (uint8_t)sizeof(s_probe_c9),   1200U},   /* 37: C9 当前文件 */
    {1U, 0U, s_probe_c2,   (uint8_t)sizeof(s_probe_c2),   1200U}    /* 38: C2 状态(收尾) */
};

static const char *const s_probe_hint[] =
{
    "AB stop play (clear stuck state)",
    "D2 drive = U-disk (so AC/AD/loop follow the U-disk)",
    "B6 output = SPK [speaker must be on SPK+ AND SPK-, never to GND]",
    "AE set volume 31 (max)",
    "C1 query volume",
    "CA query connection [bit2 = U-disk; but this module answered CA=00 while C7=05 -> trust C7 + ears over CA]",
    "C7 query U-disk file count <== 0 = module sees NO file on the U-disk (C3 is NOT a medium flag on this module)",
    "C9 query current file (baseline)",
    "C2 query status (baseline)",
    "A6 play index 1 (U-disk) --> LISTEN! (attempt 1)",
    "C9 current file (did it move?)",
    "C2 status (did it change?)",
    "AA play/pause (pause whatever is playing)",
    "C2 status (paused?)",
    "A6 play index 1 AGAIN (U-disk) --> LISTEN! (compare ret with step 10; same value = no 'busy', we know 05 still plays)",
    "C9 current file",
    "C2 status",
    "AB stop play",
    "B6 output = DAC (test the other output path)",
    "A6 play index 1 in DAC mode (U-disk) --> LISTEN!",
    "AA play/pause",
    "B6 output = SPK (back)",
    "A6 play index 1 in SPK mode (U-disk) --> LISTEN!",
    "C2 status",
    "AC next track (engine, follows the current drive = U-disk after step 2)",
    "C9 current file (moved to 2?)",
    "AD prev track (engine, follows the current drive)",
    "C9 current file (back to 1?)",
    "A6 play index 2 (U-disk index scan) --> LISTEN!",
    "AB stop play",
    "C3 query file count <== same 1-byte format as step 7 here -> NOT a Flash/U-disk discriminator",
    "A6 index 99 (must NOT exist) <== expect ret 02 = EMP(no such file); if it also answers 05, then 05 is not an error code",
    "C2 status (did that failed A6 change anything?)",
    "A6 play index 0 (some firmwares count from 0) --> LISTEN!",
    "C2 status (short delay: did it ever become 01 = PLAYING?)",
    "A0 play SPI-Flash index 1 <== answers 02 (no such file) -> the audio is NOT on the SPI-Flash",
    "C9 current file",
    "C2 status"
};

/* 编译期自检: 两张表长度必须一致(不一致时这行会因"数组大小为负"直接报错) */
typedef char wt2003_probe_hint_len_check[
    ((sizeof(s_probe_hint) / sizeof(s_probe_hint[0])) ==
     (sizeof(s_probe_tab) / sizeof(s_probe_tab[0]))) ? 1 : -1];

#define WT2003_PROBE_NUM     (sizeof(s_probe_tab) / sizeof(s_probe_tab[0]))

static uint8_t           s_probe_on    = 0U;
static uint8_t           s_probe_idx   = 0U;
static uint8_t           s_probe_round = 0U;
static uint32_t          s_probe_tick  = 0U;
static uint16_t          s_probe_wait  = 0U;    /* 当前这一步要等的间隔(见 s_probe_tab[].gap_ms) */
static wt2003_probe_cb_t s_probe_cb    = NULL;

/*---------------------------------------------------------------------------*/
/** 第 step(1..N) 步之前要等的毫秒数(gap_ms = 0 时用 WT2003_PROBE_INTERVAL_MS) */
static uint16_t wt2003_probe_gap(uint8_t step)
{
    uint16_t g;

    if ((step == 0U) || (step > (uint8_t)WT2003_PROBE_NUM))
    {
        return (uint16_t)WT2003_PROBE_INTERVAL_MS;
    }

    g = s_probe_tab[step - 1U].gap_ms;
    return (g == 0U) ? (uint16_t)WT2003_PROBE_INTERVAL_MS : g;
}

/*---------------------------------------------------------------------------*/
/** 手工发一步候选(远程命令/调试用, 不改变自动探测状态), 见 drv_wt2003h.h */
uint8_t wt2003_probe_fire(uint8_t step)
{
    const wt2003_probe_step_t *s;

    if ((step == 0U) || (step > (uint8_t)WT2003_PROBE_NUM))
    {
        return 1U;
    }

    s = &s_probe_tab[step - 1U];

    if (s->kind == 0U)
    {
        wt2003_key_pulse(s->key);       /* Key 口: 非阻塞, 由 wt2003_process() 到点释放 */
    }
    else
    {
        (void)wt2003_send_raw(s->frame, s->len);
    }
    return 0U;
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_probe_count(void)
{
    return (uint8_t)WT2003_PROBE_NUM;
}

/*---------------------------------------------------------------------------*/
const char *wt2003_probe_cfg_hint(uint8_t step)
{
    if ((step == 0U) || (step > (uint8_t)WT2003_PROBE_NUM))
    {
        return "?";
    }
    return s_probe_hint[step - 1U];
}

/*---------------------------------------------------------------------------*/
void wt2003_probe_set_handler(wt2003_probe_cb_t cb)
{
    s_probe_cb = cb;
}

/*---------------------------------------------------------------------------*/
void wt2003_probe_start(void)
{
    wt2003_name_stop();         /* 与"按文件名轮播"互斥: 两套时序交叉会互相打乱 */

    s_probe_on    = 1U;
    s_probe_idx   = 0U;
    s_probe_round = 0U;
    /* 先减掉第 1 步的间隔 => 下一次 wt2003_process() 立刻发出第 1 步, 不用干等 */
    s_probe_wait  = wt2003_probe_gap(1U);
    s_probe_tick  = bsp_time_ms() - (uint32_t)s_probe_wait;
}

/*---------------------------------------------------------------------------*/
void wt2003_probe_stop(void)
{
    s_probe_on = 0U;
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_probe_is_running(void)
{
    return s_probe_on;
}

/*---------------------------------------------------------------------------*/
/** 由 wt2003_process() 调用: 到点就发下一步(间隔 = 该步的 gap_ms, 见 s_probe_tab) */
static void wt2003_probe_poll(void)
{
    if (s_probe_on == 0U)
    {
        return;
    }
    if (bsp_elapsed_ms(s_probe_tick) < s_probe_wait)
    {
        return;
    }

    s_probe_tick = bsp_time_ms();

    (void)wt2003_probe_fire((uint8_t)(s_probe_idx + 1U));

    if (s_probe_cb != NULL)
    {
        s_probe_cb((uint8_t)(s_probe_idx + 1U), (uint8_t)WT2003_PROBE_NUM);
    }

    s_probe_idx++;
    if (s_probe_idx >= (uint8_t)WT2003_PROBE_NUM)
    {
        s_probe_idx = 0U;
        s_probe_round++;
        if (s_probe_round >= (uint8_t)WT2003_PROBE_ROUNDS)
        {
            s_probe_on = 0U;
            if (s_probe_cb != NULL)
            {
                s_probe_cb(0xFFU, (uint8_t)WT2003_PROBE_NUM);   /* 结束通知 */
            }
        }
    }

    s_probe_wait = wt2003_probe_gap((uint8_t)(s_probe_idx + 1U));   /* 下一步的间隔 */
}

/*===========================================================================
 *              按文件名轮播(U 盘 A7) —— 排查"指定索引播不出声"用
 *  ---------------------------------------------------------------------------
 *  背景(实测, 见 README 8.7/8.9): 本模块 A6(指定 U 盘根目录索引播放)收下命令、
 *  回 0x05, 但喇叭不出声; 而 AC/AD(下一曲/上一曲, 跟着当前盘符)会真的出声。
 *  => 要播指定语音就改走 A7(手册 4.1.3.5 按文件名播放): 文件放 U 盘根目录、
 *     文件名固定 4 字符(如 T001.mp3 -> 命令数据 "T001")。
 *  本段做"每 WT2003_NAME_CYCLE_MS 毫秒播下一个文件名并循环":
 *     ① 发 A7 播第 n 个名字 -> 回调通知业务层(数码管/串口显示第几个);
 *     ② 过 WT2003_NAME_CYCLE_QUERY_MS 毫秒自动发 C9(当前曲目号) + CB(当前曲目名),
 *        从串口 WTRXINFO 就能看到"模块到底在播哪一首 / 有没有动", 不用挂调试器;
 *     ③ 周期到 -> 换下一个名字, 回到 ①(循环)。
 *  配置: bsp_config.h 的 WT2003_NAME_CYCLE_*; 运行时: VNAME / VNAMEON / VNAMEOFF。
 *=========================================================================*/
static const char *const s_name_tab[] = WT2003_NAME_CYCLE_LIST;

#define WT2003_NAME_NUM     (sizeof(s_name_tab) / sizeof(s_name_tab[0]))

/* 编译期自检: "播后查询"必须早于一个播放周期, 否则这一查会被下一次播放顶掉
   (条件不满足时这行会因"数组大小为负"直接报错) */
typedef char wt2003_name_cfg_check[
    ((WT2003_NAME_CYCLE_QUERY_MS < WT2003_NAME_CYCLE_MS) ? 1 : -1)];

static uint8_t          s_name_on    = 0U;
static uint8_t          s_name_idx   = 0U;
static uint8_t          s_name_phase = 0U;   /* 0=待播(发 A7) 1=已播(等 C9/CB) 2=已查(等换下一首) */
static uint32_t         s_name_tick  = 0U;
static wt2003_name_cb_t s_name_cb    = NULL;

/*---------------------------------------------------------------------------*/
uint8_t wt2003_name_count(void)
{
    return (uint8_t)WT2003_NAME_NUM;
}

/*---------------------------------------------------------------------------*/
const char *wt2003_name_get(uint8_t n)
{
    if ((n == 0U) || (n > (uint8_t)WT2003_NAME_NUM))
    {
        return NULL;
    }
    return s_name_tab[n - 1U];
}

/*---------------------------------------------------------------------------*/
void wt2003_name_set_handler(wt2003_name_cb_t cb)
{
    s_name_cb = cb;
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_name_fire(uint8_t n)
{
    const char *name = wt2003_name_get(n);

    if (name == NULL)
    {
        return 1U;
    }
    if (wt2003_play_udisk_name(name) != 0U)     /* A7: 7E 07 A7 <4 字节名字> <校验> EF */
    {
        return 1U;
    }
    if (s_name_cb != NULL)
    {
        s_name_cb(name, n, (uint8_t)WT2003_NAME_NUM);
    }
    return 0U;
}

/*---------------------------------------------------------------------------*/
void wt2003_name_start(void)
{
    wt2003_probe_stop();        /* 与上电自检互斥: 两套时序交叉会互相打乱 */

    s_name_on    = 1U;
    s_name_idx   = 0U;
    s_name_phase = 0U;
    /* 先减掉一个周期 => 下一次 wt2003_process() 立刻播第 1 个, 不用干等一个周期 */
    s_name_tick  = bsp_time_ms() - (uint32_t)WT2003_NAME_CYCLE_MS;
}

/*---------------------------------------------------------------------------*/
void wt2003_name_stop(void)
{
    s_name_on = 0U;
}

/*---------------------------------------------------------------------------*/
uint8_t wt2003_name_is_running(void)
{
    return s_name_on;
}

/*---------------------------------------------------------------------------*/
/** 由 wt2003_process() 驱动: 到点播下一个文件名(A7), 播后自动查一次 C9/CB */
static void wt2003_name_poll(void)
{
    uint32_t dt;

    if (s_name_on == 0U)
    {
        return;
    }

    dt = bsp_elapsed_ms(s_name_tick);

    if (s_name_phase == 0U)                             /* ① 播 */
    {
        (void)wt2003_name_fire((uint8_t)(s_name_idx + 1U));
        s_name_tick  = bsp_time_ms();
        s_name_phase = 1U;
        return;
    }

    if ((s_name_phase == 1U) && (dt >= (uint32_t)WT2003_NAME_CYCLE_QUERY_MS))
    {
        (void)wt2003_query_cur_file();                  /* C9: 当前曲目号(变了吗) */
        (void)wt2003_query_cur_name();                  /* CB: 当前曲目名(是不是刚发那首) */
        s_name_phase = 2U;
        return;
    }

    if ((s_name_phase == 2U) && (dt >= (uint32_t)WT2003_NAME_CYCLE_MS))
    {
        s_name_idx++;                                   /* ③ 换下一个名字(循环) */
        if (s_name_idx >= (uint8_t)WT2003_NAME_NUM)
        {
            s_name_idx = 0U;
        }
        s_name_tick  = bsp_time_ms();
        s_name_phase = 0U;
    }
}

/*===========================================================================
 *                              主循环处理
 *=========================================================================*/
void wt2003_process(void)
{
    uint8_t        i;
    static uint8_t s_rx_state = 0U;         /* 0:找帧头 1:收长度 2:收数据 */
    static uint8_t s_rx_len   = 0U;
    static uint8_t s_rx_idx   = 0U;
    static uint8_t s_rx_buf2[WT2003_RX_FRAME_MAX];
    static uint8_t s_rx_raw[WT2003_RX_FRAME_MAX + 3U];   /* 正在收的原始帧: 7E 长度 ... 校验 EF */
    static uint8_t s_rx_raw_len = 0U;
    uint8_t        budget = 32U;
    uint8_t        ch;

    /*------------------- 上电协议探测(到点发下一种候选帧) -------------*/
    wt2003_probe_poll();

    /*------------------- 按文件名轮播(到点播下一个文件 + 查一次) --------*/
    wt2003_name_poll();

    /*------------------- Key 脉冲到点释放 --------------------
      注: s_key_start_ms == 0 表示该脉冲由语音任务自己阻塞释放(见 wt2003_exec_request),
          这里不要动它。Key 口未启用(WT2003_KEY_ENABLE = 0)时下面永远不会命中。 */
    for (i = 0U; i < WT2003_KEY_COUNT; i++)
    {
        if ((s_key_busy[i] != 0U) && (s_key_start_ms[i] != 0U)
            && (bsp_elapsed_ms(s_key_start_ms[i]) >= WT2003_KEY_PULSE_MS))
        {
#if (WT2003_KEY_ENABLE != 0U)
            GPIO_SetBits(WT2003_KEY_PORT, s_key_pin[i]);   /* 释放 */
#endif
            s_key_busy[i] = 0U;
        }
    }

    /*------------------- 解析模块返回的帧 -------------------*/
    while ((budget > 0U) && (ringbuf_pop(&s_rx, &ch) != 0U))
    {
        budget--;

        switch (s_rx_state)
        {
            case 0U:
                if (ch == WT2003_FRAME_HEAD)
                {
                    s_rx_raw[0]  = ch;          /* 原始帧从帧头开始留一份 */
                    s_rx_raw_len = 1U;
                    s_rx_state   = 1U;
                }
                break;

            case 1U:
                /* 长度 = 1(自身) + 1(命令) + N(数据) + 1(校验码) >= 3 */
                if (ch < 3U)
                {
                    s_rx_state   = 0U;
                    s_rx_raw_len = 0U;
                }
                else
                {
                    s_rx_len     = ch;
                    s_rx_idx     = 0U;
                    s_rx_raw[1]  = ch;
                    s_rx_raw_len = 2U;
                    s_rx_state   = 2U;
                }
                break;

            case 2U:
                /* 长度字节之后共有 (长度-1) 个字节: [命令][数据...][校验码] */
                if (s_rx_idx < (uint8_t)(s_rx_len - 1U))
                {
                    if (s_rx_idx < WT2003_RX_FRAME_MAX)
                    {
                        s_rx_buf2[s_rx_idx] = ch;
                        s_rx_idx++;
                        if (s_rx_raw_len < (uint8_t)sizeof(s_rx_raw))
                        {
                            s_rx_raw[s_rx_raw_len] = ch;
                            s_rx_raw_len++;
                        }
                    }
                    else
                    {
                        /* 超出缓冲: 原样报出来(能看到"到底是多长的帧"), 再重新找帧头 */
                        if ((s_bad_cb != NULL) && (s_rx_raw_len != 0U))
                        {
                            s_bad_cb(s_rx_raw, s_rx_raw_len);
                        }
                        s_rx_state   = 0U;
                        s_rx_raw_len = 0U;
                    }
                }
                else
                {
                    /* 数据已收齐, 本字节应是帧尾 */
                    if (ch == WT2003_FRAME_TAIL)
                    {
                        if (s_rx_raw_len < (uint8_t)sizeof(s_rx_raw))
                        {
                            s_rx_raw[s_rx_raw_len] = ch;   /* 连帧尾一起留档 */
                            s_rx_raw_len++;
                        }
                        for (i = 0U; i < s_rx_raw_len; i++)   /* 交给 wt2003_last_rx_raw() */
                        {
                            s_last_raw[i] = s_rx_raw[i];
                        }
                        s_last_raw_len = s_rx_raw_len;

                        if ((s_frame_cb != NULL) && (s_rx_idx >= 2U))
                        {
                            /* 上报时剥掉命令码(1)与校验码(1) => 数据长度 = idx - 2 */
                            s_frame_cb(s_rx_buf2[0], &s_rx_buf2[1], (uint8_t)(s_rx_idx - 2U));
                        }
                    }
                    else
                    {
                        /* 帧尾不是 EF: 说明长度字节口径与本驱动不同(整帧会被静默丢掉),
                           原样报出来, 便于对照手册判断"模块到底回了几字节" */
                        if ((s_bad_cb != NULL) && (s_rx_raw_len < (uint8_t)sizeof(s_rx_raw)))
                        {
                            s_rx_raw[s_rx_raw_len] = ch;
                            s_rx_raw_len++;
                            s_bad_cb(s_rx_raw, s_rx_raw_len);
                        }
                    }
                    s_rx_state   = 0U;
                    s_rx_raw_len = 0U;
                }
                break;

            default:
                s_rx_state = 0U;
                break;
        }
    }
}

/*===========================================================================
 *                            状态 / 统计
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
void wt2003_set_bad_frame_handler(wt2003_bad_frame_cb_t cb)
{
    s_bad_cb = cb;
}

/*---------------------------------------------------------------------------*/
const uint8_t *wt2003_last_rx_raw(uint8_t *len)
{
    if (len != NULL)
    {
        *len = s_last_raw_len;
    }
    return s_last_raw;
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

        tmp = WT2003_UART->SR;
        tmp = WT2003_UART->DR;
        (void)tmp;
    }
}
