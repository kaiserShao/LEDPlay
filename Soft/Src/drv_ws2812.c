/**
  ******************************************************************************
  * @file    drv_ws2812.c
  * @brief   主灯带 WS2812 驱动 (PB6 / TIM4_CH1 / DMA1_Channel1, PWM+DMA)
  *
  *  时序: 800kHz PWM, 周期 1.25us 对应 1 个数据位
  *        "0" -> 高 400ns (CCR = 29 @72MHz)
  *        "1" -> 高 800ns (CCR = 58 @72MHz)
  *        复位 -> CCR = 0 (整周期低电平)
  *
  *  缓冲区: [0 .. 1439] 第 0~59 颗   (HT 中断刷新)
  *          [1440 .. 2879] 第 60~119 颗 (TC 中断刷新)
  *          [2880 .. 3119] 复位段 300us (TC 中断刷新)
  *
  *  数据流: 应用写 s_frame_pending -> ws2812_show() 置脏标志 ->
  *          TC 中断里做一次快照 s_frame_snap (整帧一致) ->
  *          HT/TC 中断把快照编码成 CCR 序列写入 DMA 缓冲
  ******************************************************************************
  */
#include "drv_ws2812.h"
#include "bsp_rtos.h"
#include "bsp_time.h"
#include "bsp_util.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_dma.h"
#include "misc.h"

#define WS2812_LED_BITS       (24U)                            /* 每颗灯 24 位 */
#define WS2812_HALF_LED_NUM   (WS2812_LED_NUM / 2U)            /* 前半帧灯数 60 */
#define WS2812_HALF_BITS      (WS2812_HALF_LED_NUM * 24U)      /* 1440 */

/* PWM 比较值缓冲: 由 DMA 硬件读取 */
static uint16_t s_pwm[WS2812_BUF_LEN];

/* 颜色缓冲: [待显示] 和 [中断内使用的快照] */
static uint8_t  s_frame_pending[WS2812_LED_NUM * 3U];
static uint8_t  s_frame_snap[WS2812_LED_NUM * 3U];

static volatile uint8_t  s_dirty     = 0U;      /*!< 有待提交的颜色修改 */
static volatile uint32_t s_frame_cnt = 0U;      /*!< 已输出帧数 */
static volatile uint8_t  s_running   = 0U;      /*!< DMA 输出中 */
static uint8_t           s_brightness = WS2812_MAX_BRIGHTNESS;
static uint16_t          s_ccr0 = 0U;           /*!< "0" 码比较值 */
static uint16_t          s_ccr1 = 0U;           /*!< "1" 码比较值 */
static uint16_t          s_arr  = 0U;           /*!< 一位(1.25us)的定时器拍数, 仅调试取值用 */
static void            (*s_frame_cb)(void) = NULL;

/* 多任务保护: 灯带颜色缓冲可能被"灯光任务(动画)"和"命令任务(FILL/LED)"同时改,
   用锁保证一帧内容成套一致(RTX 任务里用互斥锁, 无 RTOS 时退化为临界区)。 */
static bsp_mutex_t       s_lock;

/*---------------------------------------------------------------------------*/
/** 纳秒 -> 定时器计数 */
static uint16_t ws2812_ns_to_ticks(uint32_t tim_clk, uint32_t ns)
{
    uint64_t v = (((uint64_t)tim_clk * (uint64_t)ns) + 500000000ULL) / 1000000000ULL;

    if (v > 0xFFFFU)
    {
        v = 0xFFFFU;
    }
    return (uint16_t)v;
}

/*---------------------------------------------------------------------------*/
/** 取 TIM4 的实际计数时钟: APB1 分频系数 != 1 时定时器时钟 = PCLK1 x 2 */
static uint32_t ws2812_tim_clock(void)
{
    RCC_ClocksTypeDef clocks;
    uint32_t          ppre1;

    RCC_GetClocksFreq(&clocks);
    ppre1 = (RCC->CFGR >> 8) & 0x07U;   /* 0b000:/1 0b100:/2 ... */

    if (ppre1 == 0U)
    {
        return clocks.PCLK1_Frequency;
    }
    return (clocks.PCLK1_Frequency * 2U);
}

/*---------------------------------------------------------------------------*/
/** 把快照中的一段灯珠编码成 CCR 序列(24 位/颗, 顺序 G R B, 每字节高位先出) */
static void ws2812_encode(uint16_t led_start, uint16_t led_count, uint16_t *dst)
{
    const uint8_t *p  = &s_frame_snap[(uint32_t)led_start * 3U];
    uint8_t        br = s_brightness;
    uint16_t       i;
    uint8_t        b;
    uint8_t        mask;

    for (i = 0U; i < led_count; i++)
    {
        for (b = 0U; b < 3U; b++)
        {
            uint8_t v;

            if (b == 0U)
            {
                v = p[1];           /* G */
            }
            else if (b == 1U)
            {
                v = p[0];           /* R */
            }
            else
            {
                v = p[2];           /* B */
            }

            if (br != 255U)
            {
                v = (uint8_t)(((uint16_t)v * (uint16_t)br) >> 8);   /* 亮度缩放 */
            }

            for (mask = 0x80U; mask != 0U; mask = (uint8_t)(mask >> 1))
            {
                *dst = ((v & mask) != 0U) ? s_ccr1 : s_ccr0;
                dst++;
            }
        }
        p += 3U;
    }
}

/*---------------------------------------------------------------------------*/
/** 把"待显示"颜色复制到快照(整帧一起切换, 避免半新半旧) */
static void ws2812_snapshot(void)
{
    uint32_t pm = bsp_irq_save();
    uint16_t i;

    for (i = 0U; i < (uint16_t)sizeof(s_frame_snap); i++)
    {
        s_frame_snap[i] = s_frame_pending[i];
    }
    s_dirty = 0U;
    bsp_irq_restore(pm);
}

/*===========================================================================
 *                              硬件初始化
 *=========================================================================*/
void ws2812_init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef       oc;
    DMA_InitTypeDef         dma;
    NVIC_InitTypeDef        nvic;
    uint32_t                tim_clk;
    uint32_t                arr;
    uint16_t                i;

    /*--------------------------- 时钟使能 -------------------------------*/
    RCC_APB2PeriphClockCmd(WS2812_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(WS2812_TIM_CLK, ENABLE);
    RCC_AHBPeriphClockCmd(WS2812_DMA_CLK, ENABLE);

    /*--------------------------- PB6 复用推挽 ---------------------------*/
    gpio.GPIO_Pin   = WS2812_GPIO_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(WS2812_GPIO_PORT, &gpio);

    /*--------------------------- 时序参数换算 ---------------------------*/
    tim_clk = ws2812_tim_clock();                       /* 72MHz */
    arr     = ws2812_ns_to_ticks(tim_clk, WS2812_BIT_NS);  /* 90  */
    s_ccr0  = ws2812_ns_to_ticks(tim_clk, WS2812_T0H_NS);  /* 22  */
    s_ccr1  = ws2812_ns_to_ticks(tim_clk, WS2812_T1H_NS);  /* 43  */
    if (arr < 4U)
    {
        arr = 4U;
    }
    s_arr = (uint16_t)arr;      /* 供 ws2812_get_timing() 读出实际算出的拍数 */

    /*--------------------------- 缓冲清零 -------------------------------*/
    for (i = 0U; i < WS2812_BUF_LEN; i++)
    {
        s_pwm[i] = 0U;              /* 全 0: 全灭 + 复位段满足 */
    }
    for (i = 0U; i < (uint16_t)sizeof(s_frame_pending); i++)
    {
        s_frame_pending[i] = 0U;
        s_frame_snap[i]    = 0U;
    }
    s_brightness = WS2812_MAX_BRIGHTNESS;
    s_dirty      = 0U;
    s_frame_cnt  = 0U;

    /*--------------------------- TIM4 PWM 配置 --------------------------*/
    tim.TIM_Prescaler         = 0U;                       /* 72MHz / 90 = 800kHz */
    tim.TIM_CounterMode       = TIM_CounterMode_Up;
    tim.TIM_Period            = arr - 1U;
    tim.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(WS2812_TIM, &tim);

    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse       = s_ccr0;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(WS2812_TIM, &oc);

    /* 关键: 打开 CCR 预装载。DMA 在比较匹配时写入的值会在下一个更新事件
       才生效, 否则新值会立刻影响本周期剩余的低电平长度, 导致数据位出错。 */
    TIM_OC1PreloadConfig(WS2812_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(WS2812_TIM, ENABLE);
    TIM_SetCompare1(WS2812_TIM, s_ccr0);
    TIM_Cmd(WS2812_TIM, DISABLE);

    /*--------------------------- DMA1_Channel1 --------------------------*/
    DMA_DeInit(WS2812_DMA_CHANNEL);
    DMA_ClearITPendingBit(WS2812_DMA_IT_HT | WS2812_DMA_IT_TC);

    dma.DMA_PeripheralBaseAddr = (uint32_t)(&(WS2812_TIM->CCR1));
    dma.DMA_MemoryBaseAddr     = (uint32_t)s_pwm;
    dma.DMA_DIR                = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize         = WS2812_BUF_LEN;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode               = DMA_Mode_Circular;      /* 循环: 连续输出, 永不停止 */
    dma.DMA_Priority           = DMA_Priority_VeryHigh;
    dma.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(WS2812_DMA_CHANNEL, &dma);
    DMA_ITConfig(WS2812_DMA_CHANNEL, DMA_IT_HT | DMA_IT_TC, ENABLE);

    /*--------------------------- NVIC -----------------------------------*/
    nvic.NVIC_IRQChannel                   = WS2812_DMA_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = BSP_IRQ_PRIO_WS2812_DMA;
    nvic.NVIC_IRQChannelSubPriority        = 0U;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    s_running = 0U;

    bsp_mutex_init(&s_lock);     /* 多任务保护(必须在 bsp_rtos_init() 之后) */

#if (WS2812_AUTO_START != 0U)
    ws2812_start();
#endif
}

/*===========================================================================
 *                          启动 / 停止输出
 *=========================================================================*/
void ws2812_start(void)
{
    if (s_running != 0U)
    {
        return;
    }

    /* 启动顺序很讲究: 先备好 DMA, 再开 DMA 通道, 最后放行定时器。 */
    TIM_Cmd(WS2812_TIM, DISABLE);
    TIM_SetCounter(WS2812_TIM, 0U);
    TIM_SetCompare1(WS2812_TIM, s_ccr0);

    DMA_ClearITPendingBit(WS2812_DMA_IT_HT | WS2812_DMA_IT_TC);
    DMA_SetCurrDataCounter(WS2812_DMA_CHANNEL, WS2812_BUF_LEN);
    DMA_Cmd(WS2812_DMA_CHANNEL, ENABLE);

    TIM_DMACmd(WS2812_TIM, TIM_DMA_CC1, ENABLE);
    TIM_Cmd(WS2812_TIM, ENABLE);

    s_running = 1U;
}

/*---------------------------------------------------------------------------*/
void ws2812_stop(void)
{
    if (s_running == 0U)
    {
        return;
    }
    TIM_DMACmd(WS2812_TIM, TIM_DMA_CC1, DISABLE);
    TIM_Cmd(WS2812_TIM, DISABLE);
    DMA_Cmd(WS2812_DMA_CHANNEL, DISABLE);
    DMA_ClearITPendingBit(WS2812_DMA_IT_HT | WS2812_DMA_IT_TC);
    s_running = 0U;
}

/*---------------------------------------------------------------------------*/
uint8_t ws2812_is_running(void)
{
    return s_running;
}

/*---------------------------------------------------------------------------*/
void ws2812_process(void)
{
    /* 目前不需要后台处理(全部由 DMA 中断完成), 保留接口方便以后扩展 */
}

/*---------------------------------------------------------------------------*/
void ws2812_lock(void)
{
    bsp_mutex_lock(&s_lock);
}

/*---------------------------------------------------------------------------*/
void ws2812_unlock(void)
{
    bsp_mutex_unlock(&s_lock);
}

/*===========================================================================
 *                     DMA 半传输 / 传输完成 中断
 *
 *  半传输(HT): DMA 刚写完前半帧, 现在刷新"前半帧"(第 0~59 颗)供下一轮使用,
 *              此时硬件正在发送后半帧, 有约 1.95ms 的时间余量。
 *  传输完成(TC): DMA 刚写完整个缓冲(含复位段), 现在刷新"后半帧 + 复位段",
 *              此时硬件正在发送前半帧, 同样约有 1.95ms 余量。
 *  每帧结束后更新颜色快照, 保证"整帧颜色一致"。
 *=========================================================================*/
void DMA1_Channel1_IRQHandler(void)
{
    if (DMA_GetITStatus(WS2812_DMA_IT_HT) != RESET)
    {
        DMA_ClearITPendingBit(WS2812_DMA_IT_HT);
        ws2812_encode(0U, WS2812_HALF_LED_NUM, &s_pwm[0]);
    }

    if (DMA_GetITStatus(WS2812_DMA_IT_TC) != RESET)
    {
        uint16_t i;

        DMA_ClearITPendingBit(WS2812_DMA_IT_TC);

        ws2812_encode(WS2812_HALF_LED_NUM,
                      (uint16_t)(WS2812_LED_NUM - WS2812_HALF_LED_NUM),
                      &s_pwm[WS2812_HALF_BITS]);

        /* 复位段保持低电平 */
        for (i = (uint16_t)WS2812_DATA_BITS; i < WS2812_BUF_LEN; i++)
        {
            s_pwm[i] = 0U;
        }

        /* 一帧发送完毕: 换用最新的颜色快照 */
        if (s_dirty != 0U)
        {
            ws2812_snapshot();
        }

        s_frame_cnt++;
        if (s_frame_cb != NULL)
        {
            s_frame_cb();
        }
    }
}

/*===========================================================================
 *                              颜色操作接口
 *=========================================================================*/
void ws2812_set_pixel(uint16_t idx, uint8_t r, uint8_t g, uint8_t b)
{
    if (idx >= WS2812_LED_NUM)
    {
        return;
    }
    s_frame_pending[(uint32_t)idx * 3U + 0U] = r;
    s_frame_pending[(uint32_t)idx * 3U + 1U] = g;
    s_frame_pending[(uint32_t)idx * 3U + 2U] = b;
}

/*---------------------------------------------------------------------------*/
void ws2812_get_pixel(uint16_t idx, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (idx >= WS2812_LED_NUM)
    {
        return;
    }
    if (r != NULL)
    {
        *r = s_frame_pending[(uint32_t)idx * 3U + 0U];
    }
    if (g != NULL)
    {
        *g = s_frame_pending[(uint32_t)idx * 3U + 1U];
    }
    if (b != NULL)
    {
        *b = s_frame_pending[(uint32_t)idx * 3U + 2U];
    }
}

/*---------------------------------------------------------------------------*/
void ws2812_fill(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t i;

    for (i = 0U; i < WS2812_LED_NUM; i++)
    {
        s_frame_pending[(uint32_t)i * 3U + 0U] = r;
        s_frame_pending[(uint32_t)i * 3U + 1U] = g;
        s_frame_pending[(uint32_t)i * 3U + 2U] = b;
    }
}

/*---------------------------------------------------------------------------*/
void ws2812_fill_range(uint16_t start, uint16_t count, uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t i;
    uint16_t end;

    if (start >= WS2812_LED_NUM)
    {
        return;
    }
    end = (uint16_t)(start + count);
    if (end > WS2812_LED_NUM)
    {
        end = WS2812_LED_NUM;
    }

    for (i = start; i < end; i++)
    {
        s_frame_pending[(uint32_t)i * 3U + 0U] = r;
        s_frame_pending[(uint32_t)i * 3U + 1U] = g;
        s_frame_pending[(uint32_t)i * 3U + 2U] = b;
    }
}

/*---------------------------------------------------------------------------*/
void ws2812_set_buffer(const uint8_t *rgb, uint16_t count)
{
    uint16_t i;

    if (rgb == NULL)
    {
        return;
    }
    if (count > WS2812_LED_NUM)
    {
        count = WS2812_LED_NUM;
    }
    for (i = 0U; i < count; i++)
    {
        s_frame_pending[(uint32_t)i * 3U + 0U] = rgb[(uint32_t)i * 3U + 0U];
        s_frame_pending[(uint32_t)i * 3U + 1U] = rgb[(uint32_t)i * 3U + 1U];
        s_frame_pending[(uint32_t)i * 3U + 2U] = rgb[(uint32_t)i * 3U + 2U];
    }
}

/*---------------------------------------------------------------------------*/
void ws2812_clear(void)
{
    ws2812_fill(0U, 0U, 0U);
}

/*---------------------------------------------------------------------------*/
void ws2812_set_brightness(uint8_t brightness)
{
    s_brightness = brightness;
}

/*---------------------------------------------------------------------------*/
uint8_t ws2812_get_brightness(void)
{
    return s_brightness;
}

/*---------------------------------------------------------------------------*/
void ws2812_show(void)
{
    s_dirty = 1U;      /* 真正的编码在下一个 TC 中断(帧边界)完成 */
}

/*---------------------------------------------------------------------------*/
uint8_t ws2812_is_dirty(void)
{
    return s_dirty;
}

/*---------------------------------------------------------------------------*/
/** 调试用: 取初始化时实际算出来的拍数(见 drv_ws2812.h 的说明) */
void ws2812_get_timing(uint16_t *arr, uint16_t *ccr0, uint16_t *ccr1)
{
    if (arr != NULL)
    {
        *arr = s_arr;
    }
    if (ccr0 != NULL)
    {
        *ccr0 = s_ccr0;
    }
    if (ccr1 != NULL)
    {
        *ccr1 = s_ccr1;
    }
}

/*---------------------------------------------------------------------------*/
uint32_t ws2812_frame_count(void)
{
    return s_frame_cnt;
}

/*---------------------------------------------------------------------------*/
void ws2812_set_frame_callback(void (*cb)(void))
{
    s_frame_cb = cb;
}

