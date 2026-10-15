/**
  ******************************************************************************
  * @file    drv_keyled.c
  * @brief   按键灯 WS2812 驱动 (PC9 = TIM3_CH4 完全重映射 + DMA1_Channel3, PWM+DMA)
  *
  *  为什么用这个方式: 老办法(PC4)在 STM32F103 上没有定时器通道, 96 位只能在一个
  *  定时器更新中断里"数拍翻转 GPIO"发出来 —— 中断被压住一会儿时序就歪。本板把按键灯
  *  挪到 PC9(TIM3_CH4, 完全重映射; 它的 DMA 请求固定接 DMA1_Channel3, 见
  *  bsp_config.h 第 5 节), 于是直接照抄主灯带那一套:
  *      * TIM3 输出 800kHz PWM: 1 个 PWM 周期(1.25us) = 1 个数据位, 这一位的高电平
  *        长度(写进 CCR4 的值)决定它是 0 码还是 1 码; CCR4 = 0 就是整位低电平;
  *      * DMA1_Channel3 把"一整帧的比较值"一次搬完, 波形完全由硬件产生。
  *  所以发帧期间**不用关中断**, 也不需要最高优先级: 中断只在帧尾收个尾(关掉本帧的
  *  DMA 请求、记帧数、把颜色认成"已显示"), 来得晚几十微秒也不会让任何一位变宽变窄。
  *
  *  一帧的 DMA 缓冲区(KEYLED_BUF_LEN = 336 个字 = 336 个 PWM 周期):
  *      [0 .. 95]       第 0~3 颗灯, 每颗 24 位(顺序 G R B, 每字节高位先出)
  *      [96 .. 335]     240 个复位位(CCR4 = 0): 帧尾低电平 300us
  *  定时器一直在跑(空闲时 CCR4 = 0, 线上就是连续的复位电平), 一帧 = 336 次 DMA 传输
  *  再加上帧首那一个"还没被搬进去"的周期 ≈ 337 x 1.25us ≈ 421us。
  *
  *  发帧流程:
  *      keyled_show()/keyled_process() -> keyled_tx(): 等上一帧收尾 -> 把 s_out 编成
  *      CCR 序列 -> 启动 DMA;
  *      KEYLED_DMA_IRQHANDLER(): 关掉本帧的 DMA 请求/通道, 帧数 +1, s_busy = 0。
  *  应用只改颜色缓冲, 一帧的内容在"开始搬之前"一次成型, 所以每帧颜色成套一致。
  *
  *  内存占用: PWM 缓冲 336 x 2 = 672 字节 + 颜色/效果缓冲约 100 字节
  *
  *  改这个文件时的快速通道(省得每次重新摸索):
  *      编译:     .\tools\build.ps1              增量; 想全量重编加 -Rebuild
  *      日志:     Doc\tmp\build.log             看到 "0 Error(s), 0 Warning(s)" 才算过
  *      工具链:   Keil MDK 5.40, ARM Compiler 5.06u7 (uvprojx 里 uAC6 = 0), STM32F103RC
  *      硬件自检: PC9 上量到 ~800kHz(位周期 1.25us); 上电应全灭;
  *                KEYLED DIAG 里的 frames= 每发一条颜色命令应该 +1。
  ******************************************************************************
  */
#include "drv_keyled.h"
#include "bsp_rtos.h"
#include "bsp_time.h"
#include "bsp_util.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_dma.h"
#include "misc.h"

/* 缓冲长度必须等于"数据位 + 复位位"(bsp_config.h 第 4/5 节), 对不上就是配置被改歪了 */
#if ((KEYLED_BITS + KEYLED_RESET_BITS) != KEYLED_BUF_LEN)
#error "KEYLED_BUF_LEN != KEYLED_BITS + KEYLED_RESET_BITS"
#endif

/** 每路效果状态 */
typedef struct
{
    uint8_t  fx;          /*!< keyled_fx_t */
    uint16_t period_ms;   /*!< 效果周期 */
    uint32_t base_ms;     /*!< 效果起始时间 */
} keyled_fx_state_t;

/* 现场改时序时允许的范围(ns): 只放开规格书量级, 免得参数打错把灯珠写成一片白。
   PWM 方式的位周期被定时器 ARR 定死(KEYLED_BIT_NS), 所以每条码只有"高电平"是自由量:
   低电平恒 = 位周期 - 高电平, 于是四条窗口(高下限/高上限/低下限/低上限)一起夹。 */
#define KEYLED_T0H_MIN_NS     80U
#define KEYLED_T0H_MAX_NS     600U
#define KEYLED_T0L_MIN_NS     200U
#define KEYLED_T0L_MAX_NS     2000U
#define KEYLED_T1H_MIN_NS     300U
#define KEYLED_T1H_MAX_NS     1600U
#define KEYLED_T1L_MIN_NS     150U
#define KEYLED_T1L_MAX_NS     1000U
#define KEYLED_T1H_GAP_NS     100U   /* 两个码的高电平至少要差这么多, 灯珠才分得开 */

#define KEYLED_FRAME_MS       1U     /* 一帧 ≈ 0.42ms, 上取整(等帧完成时的超时兜底) */
#define KEYLED_FRAME_WAIT_MS  20U    /* 等上一帧收尾的最长时间(正常只要 0.42ms) */

/* PWM 比较值缓冲: 由 DMA1_Channel3 读走 -> TIM3->CCR4 */
static uint16_t s_pwm[KEYLED_BUF_LEN];

/* 颜色缓冲: [应用设定] [加效果/亮度后] [最近一次真正发出去的] */
static uint8_t  s_rgb[KEYLED_COUNT * 3U];
static uint8_t  s_out[KEYLED_COUNT * 3U];
static uint8_t  s_shown[KEYLED_COUNT * 3U];
static keyled_fx_state_t s_fx[KEYLED_COUNT];
static uint8_t  s_brightness = 255U;
static uint8_t  s_inited = 0U;
static uint32_t s_last_refresh_ms = 0U;

static uint16_t s_ccr0 = 0U;    /*!< "0" 码比较值(高电平拍数), 写进 CCR4 */
static uint16_t s_ccr1 = 0U;    /*!< "1" 码比较值(高电平拍数) */
static uint16_t s_arr  = 0U;    /*!< 一位(1.25us)的定时器拍数( = ARR + 1 ) */
static uint8_t  s_invert = KEYLED_INVERT; /*!< 1 = 数据线反相(线上有反相器/三极管) */
static uint8_t  s_calib_ok = 0U;          /*!< 1 = 时序已就绪(PWM 方式恒为 1) */
static uint8_t  s_need_preamble = 1U;     /*!< 1 = 下一帧前先给一段复位电平 */

static volatile uint8_t  s_busy = 0U;     /*!< 1 = DMA 正在搬这一帧 */
static volatile uint32_t s_frames = 0U;   /*!< 已经真正发完的帧数(DIAG 用) */

/* 多任务保护: 按键灯的颜色/效果可能被"命令任务"和"灯光任务"同时修改, 而 s_rgb /
   s_out / s_shown 必须成套一致, 所以整段操作加锁(见 drv_keyled.h 的说明)。 */
static bsp_mutex_t s_lock;

/* 内部: 刷新输出(调用者必须已经持有 s_lock) */
static void keyled_show_locked(void);

/*---------------------------------------------------------------------------*/
/** 纳秒 -> 定时器计数(四舍五入) */
static uint16_t keyled_ns_to_ticks(uint32_t tim_clk, uint32_t ns)
{
    uint64_t v = (((uint64_t)tim_clk * (uint64_t)ns) + 500000000ULL) / 1000000000ULL;

    if (v > 0xFFFFU)
    {
        v = 0xFFFFU;
    }
    return (uint16_t)v;
}

/*---------------------------------------------------------------------------*/
/** 取 TIM3 的实际计数时钟: APB1 分频系数 != 1 时定时器时钟 = PCLK1 x 2 (本板 72MHz) */
static uint32_t keyled_tim_clock(void)
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
/**
  * @brief  把一帧颜色编码成 CCR 序列(24 位/颗, 顺序 G R B, 每字节高位先出)
  * @param  rgb 3 字节/颗的颜色; 帧尾再补 KEYLED_RESET_BITS 个复位位(CCR4 = 0)
  * @note   必须在"没有 DMA 在搬"的时候调用 —— 缓冲区正被 DMA 读, 中途改会让这一帧
  *         的位流花掉(keyled_tx() 已经用 keyled_wait_idle() 保证这一点)。
  */
static void keyled_encode(const uint8_t *rgb)
{
    uint16_t *dst = s_pwm;
    uint16_t  i;
    uint8_t   b;
    uint8_t   mask;

    for (i = 0U; i < KEYLED_COUNT; i++)
    {
        for (b = 0U; b < 3U; b++)
        {
            uint8_t v;

            if (b == 0U)
            {
                v = rgb[((uint32_t)i * 3U) + 1U];   /* G */
            }
            else if (b == 1U)
            {
                v = rgb[((uint32_t)i * 3U) + 0U];   /* R */
            }
            else
            {
                v = rgb[((uint32_t)i * 3U) + 2U];   /* B */
            }

            for (mask = 0x80U; mask != 0U; mask = (uint8_t)(mask >> 1))
            {
                *dst = ((v & mask) != 0U) ? s_ccr1 : s_ccr0;
                dst++;
            }
        }
    }

    /* 帧尾复位段: CCR4 = 0 -> 整位低电平, 240 位 = 300us(灯珠靠它把这一帧锁存下来) */
    for (i = KEYLED_BITS; i < KEYLED_BUF_LEN; i++)
    {
        s_pwm[i] = 0U;
    }
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  等"上一帧真的搬完"(DMA 传输完成中断把 s_busy 清掉)
  * @note   正常一帧约 0.42ms, 所以这里几乎不用等。超时兜底: 万一中断没跑起来
  *         (DMA/定时器被别的模块动过), 就把这一路关掉 —— 至少后面的帧还能发出去,
  *         不至于整个按键灯卡死。
  */
static void keyled_wait_idle(void)
{
    uint32_t t0 = bsp_time_ms();

    while (s_busy != 0U)
    {
        if (bsp_elapsed_ms(t0) > KEYLED_FRAME_WAIT_MS)
        {
            TIM_DMACmd(KEYLED_TIM, TIM_DMA_CC4, DISABLE);
            DMA_Cmd(KEYLED_DMA_CHANNEL, DISABLE);
            DMA_ClearITPendingBit(KEYLED_DMA_IT_TC);
            s_busy = 0U;
            break;
        }
    }
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  把数据线摆到"整位低电平"并保持 KEYLED_RESET_US(帧前复位)
  * @note   定时器一直在跑, CCR4 = 0 时每一位都是低电平, 所以这里只要等够时间。
  *         灯珠只有在 DIN 上看到足够长的低电平(规格书 >=280us)之后才会把刚收到的
  *         96 位真正锁存下来 —— 上电第一帧之前必须先做这件事, 否则引脚浮空期间
  *         灯珠可能把噪声当成一帧锁存成"上电默认色"(有些批次上电就是全白)。
  *         本函数内部有延时, 必须在"开中断"的状态下调用(不要放进关中断段)。
  */
static void keyled_line_reset(void)
{
    TIM_SetCompare4(KEYLED_TIM, 0U);      /* 从下一个更新事件起整位低电平 */
    bsp_delay_us(KEYLED_RESET_US);
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  启动这一帧的 DMA 搬运
  * @note   顺序很重要: 先掐掉 CC4 的 DMA 请求, 再复位计数器/清标志, 最后才开请求 ——
  *         这样第一次搬运一定落在"下一个更新事件"上, 一帧里每一位的宽度都是整的。
  *         定时器从初始化起就一直跑(空闲时输出整位低电平 = 复位电平)。
  */
static void keyled_dma_start(void)
{
    TIM_DMACmd(KEYLED_TIM, TIM_DMA_CC4, DISABLE);
    TIM_SetCompare4(KEYLED_TIM, 0U);      /* 帧首那一位是低电平(复位), 不是数据 */

    DMA_ClearITPendingBit(KEYLED_DMA_IT_TC);
    DMA_SetCurrDataCounter(KEYLED_DMA_CHANNEL, KEYLED_BUF_LEN);
    DMA_Cmd(KEYLED_DMA_CHANNEL, ENABLE);

    s_busy = 1U;
    TIM_DMACmd(KEYLED_TIM, TIM_DMA_CC4, ENABLE);
}

/*---------------------------------------------------------------------------*/
/** 发一帧(调用者必须已经持有 s_lock)。真正的位输出由 DMA 在后台完成, 这里只编码 + 启动 */
static void keyled_tx(void)
{
    uint8_t i;

    keyled_wait_idle();                  /* 缓冲区正被 DMA 读的时候不能改 */

    if (s_need_preamble != 0U)
    {
        s_need_preamble = 0U;
        keyled_line_reset();             /* 帧前复位: 上电第一帧 / 现场改完参数后需要 */
    }

    keyled_encode(s_out);

    for (i = 0U; i < (uint8_t)sizeof(s_out); i++)
    {
        s_shown[i] = s_out[i];           /* s_shown = "已经认下来的颜色" */
    }

    keyled_dma_start();
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  等 n 帧真的发完(keyled_show() / 现场自检要的是"确实发完了")
  * @return 1 = 等到了, 0 = 超时(中断没起来 / DMA 被别的模块动过)
  */
static uint8_t keyled_wait_frames(uint32_t n)
{
    uint32_t want  = s_frames + n;
    uint32_t t0    = bsp_time_ms();
    uint32_t limit = (n * (uint32_t)KEYLED_FRAME_MS) + (uint32_t)KEYLED_FRAME_WAIT_MS;

    while (s_frames < want)
    {
        if (bsp_elapsed_ms(t0) > limit)
        {
            return 0U;
        }
    }

    return 1U;
}
/*===========================================================================
 *                        DMA 传输完成中断
 *
 *  一帧的 336 个字搬完(约 0.42ms)时进来一次, 只干三件事:
 *      1) 掐掉 CC4 的 DMA 请求 + 关掉通道 —— 本帧的值已经全搬完, 再搬就是把缓冲区
 *         里的旧值重复打出去(波形会拖出一整帧);
 *      2) s_busy = 0: 告诉任务层"缓冲区可以改了";
 *      3) 帧数 +1(串口 DIAG 里看这个数涨不涨, 就知道发帧这条路是不是活的)。
 *  定时器继续跑, 输出停在整位低电平(就是复位电平), 所以帧与帧之间线上一直是最低电平。
 *  本中断不碰任何时序: 来得晚几十微秒也不会让任何一位变宽或变窄。
 *=========================================================================*/
void KEYLED_DMA_IRQHANDLER(void)
{
    if (DMA_GetITStatus(KEYLED_DMA_IT_TC) != RESET)
    {
        DMA_ClearITPendingBit(KEYLED_DMA_IT_TC);

        TIM_DMACmd(KEYLED_TIM, TIM_DMA_CC4, DISABLE);
        DMA_Cmd(KEYLED_DMA_CHANNEL, DISABLE);

        TIM_SetCompare4(KEYLED_TIM, 0U);     /* 收尾: 线上回到整位低电平 */

        s_busy = 0U;
        s_frames++;
    }
}

/*---------------------------------------------------------------------------*/
/** 按效果与亮度计算 s_out */
static void keyled_update_output(void)
{
    uint8_t  i;
    uint32_t now = bsp_time_ms();

    for (i = 0U; i < KEYLED_COUNT; i++)
    {
        uint8_t  r;
        uint8_t  g;
        uint8_t  b;
        uint8_t  scale = s_brightness;   /* 0~255 */
        uint32_t period = s_fx[i].period_ms;

        if (period == 0U)
        {
            period = KEYLED_DEFAULT_PERIOD_MS;
        }

        r = s_rgb[((uint32_t)i * 3U) + 0U];
        g = s_rgb[((uint32_t)i * 3U) + 1U];
        b = s_rgb[((uint32_t)i * 3U) + 2U];

        switch ((keyled_fx_t)s_fx[i].fx)
        {
            case KEYLED_FX_OFF:
                r = 0U;
                g = 0U;
                b = 0U;
                break;

            case KEYLED_FX_STATIC:
                break;

            case KEYLED_FX_BLINK:
            {
                uint32_t half = period / 2U;
                uint32_t k;

                if (half == 0U)
                {
                    half = 1U;
                }
                k = (now - s_fx[i].base_ms) % period;
                if (k >= half)
                {
                    r = 0U;
                    g = 0U;
                    b = 0U;
                }
                break;
            }

            case KEYLED_FX_BREATHE:
            {
                uint32_t half = period / 2U;
                uint32_t k;
                uint32_t level;

                if (half == 0U)
                {
                    half = 1U;
                }
                k = (now - s_fx[i].base_ms) % period;
                if (k < half)
                {
                    level = (k * 255U) / half;             /* 渐亮 */
                }
                else
                {
                    level = ((period - k) * 255U) / half;  /* 渐暗 */
                }
                scale = (uint8_t)((level * (uint32_t)s_brightness) / 255U);
                break;
            }

            default:
                break;
        }

        /* 亮度缩放(右移 8 位, 比除法快) */
        s_out[((uint32_t)i * 3U) + 0U] = (uint8_t)(((uint16_t)r * scale) >> 8);
        s_out[((uint32_t)i * 3U) + 1U] = (uint8_t)(((uint16_t)g * scale) >> 8);
        s_out[((uint32_t)i * 3U) + 2U] = (uint8_t)(((uint16_t)b * scale) >> 8);
    }
}
/*===========================================================================
 *                              硬件初始化
 *=========================================================================*/
void keyled_init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef       oc;
    DMA_InitTypeDef         dma;
    NVIC_InitTypeDef        nvic;
    uint32_t                tim_clk;
    uint32_t                arr;
    uint16_t                i;

    /*--------------------------- 时钟 / 重映射 ---------------------------*/
    RCC_APB2PeriphClockCmd(KEYLED_GPIO_CLK | KEYLED_GPIO_REMAP_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(KEYLED_TIM_CLK, ENABLE);
    RCC_AHBPeriphClockCmd(KEYLED_DMA_CLK, ENABLE);

    /* TIM3 完全重映射: CH1~CH4 -> PC6~PC9(只有 CH4 = PC9 会被启用; PC6/PC7/PC8 是
       数码管 595 的三个普通推挽引脚, 它们的 CRH 配的是通用输出, 所以定时器碰不到它们) */
    GPIO_PinRemapConfig(KEYLED_TIM_REMAP, ENABLE);

    /*--------------------------- PC9 复用推挽 ----------------------------*/
    gpio.GPIO_Pin   = KEYLED_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(KEYLED_PORT, &gpio);

    /*--------------------------- 时序参数换算 ----------------------------*/
    tim_clk = keyled_tim_clock();                        /* 72MHz */
    arr     = keyled_ns_to_ticks(tim_clk, KEYLED_BIT_NS); /* 90 拍 = 1.25us */
    if (arr < 8U)
    {
        arr = 8U;
    }
    s_arr  = (uint16_t)arr;
    s_ccr0 = keyled_ns_to_ticks(tim_clk, KEYLED_T0H_NS);  /* 29 拍 = 403ns */
    s_ccr1 = keyled_ns_to_ticks(tim_clk, KEYLED_T1H_NS);  /* 58 拍 = 806ns */
    /* 高电平必须 < 整位(要留出低电平), 否则这一位一直高, 灯珠就把它当复位后的噪声 */
    if (s_ccr0 > (uint16_t)(arr - 1U))
    {
        s_ccr0 = (uint16_t)(arr - 1U);
    }
    if (s_ccr1 > (uint16_t)(arr - 1U))
    {
        s_ccr1 = (uint16_t)(arr - 1U);
    }

    /*--------------------------- 缓冲区清 0 ------------------------------*/
    for (i = 0U; i < KEYLED_BUF_LEN; i++)
    {
        s_pwm[i] = 0U;                  /* 全 0: 全灭 + 复位段满足 */
    }
    for (i = 0U; i < (uint16_t)sizeof(s_rgb); i++)
    {
        s_rgb[i]   = 0U;
        s_out[i]   = 0U;
        s_shown[i] = 0xFFU;             /* 初值不同, 保证首次 process 会刷新一次 */
    }
    s_brightness = 255U;
    s_busy       = 0U;
    s_frames     = 0U;
    s_invert     = (KEYLED_INVERT != 0U) ? 1U : 0U;

    /*--------------------------- TIM3 PWM(CH4) ---------------------------*/
    tim.TIM_Prescaler         = 0U;                      /* 72MHz / 90 = 800kHz */
    tim.TIM_CounterMode       = TIM_CounterMode_Up;
    tim.TIM_Period            = (uint16_t)(arr - 1U);
    tim.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(KEYLED_TIM, &tim);

    oc.TIM_OCMode       = TIM_OCMode_PWM1;
    oc.TIM_OutputState  = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse        = 0U;                            /* 空闲 = 整位低电平(复位电平) */
    oc.TIM_OCPolarity   = (s_invert != 0U) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
    oc.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC4Init(KEYLED_TIM, &oc);

    /* 关键: 打开 CCR4 预装载。DMA 在比较匹配时写进 CCR4 的值要到下一个更新事件才
       生效, 所以每一位的高电平长度就是"CCR4 决定的那一段整周期", 搬运的快慢不影响位宽。 */
    TIM_OC4PreloadConfig(KEYLED_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(KEYLED_TIM, ENABLE);
    TIM_SetCompare4(KEYLED_TIM, 0U);
    TIM_DMACmd(KEYLED_TIM, TIM_DMA_CC4, DISABLE);

    /*--------------------------- DMA1_Channel3 ---------------------------*/
    DMA_DeInit(KEYLED_DMA_CHANNEL);
    DMA_ClearITPendingBit(KEYLED_DMA_IT_TC);

    dma.DMA_PeripheralBaseAddr = (uint32_t)(&(KEYLED_TIM->CCR4));
    dma.DMA_MemoryBaseAddr     = (uint32_t)s_pwm;
    dma.DMA_DIR                = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize         = KEYLED_BUF_LEN;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode               = DMA_Mode_Normal;        /* 单次: 一帧搬完就停, 由中断收尾 */
    dma.DMA_Priority           = DMA_Priority_Medium;    /* 比主灯带/WiFi 串口低, 给它们让路 */
    dma.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(KEYLED_DMA_CHANNEL, &dma);
    DMA_ITConfig(KEYLED_DMA_CHANNEL, DMA_IT_TC, ENABLE);

    nvic.NVIC_IRQChannel                   = KEYLED_DMA_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = BSP_IRQ_PRIO_KEYLED_DMA;
    nvic.NVIC_IRQChannelSubPriority        = 0U;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    /*--------------------------- 起表 --------------------------------*/
    /* 计数器一直跑(PSC = 0): 空闲期间 CCR4 = 0, 线上就是连续的复位电平 —— 灯珠看到的
       永远是"低电平", 既不会误锁存噪声, 也让每一帧的起点都落在整周期上。 */
    TIM_Cmd(KEYLED_TIM, ENABLE);

    for (i = 0U; i < KEYLED_COUNT; i++)
    {
        s_fx[i].fx        = (uint8_t)KEYLED_FX_OFF;
        s_fx[i].period_ms = KEYLED_DEFAULT_PERIOD_MS;
        s_fx[i].base_ms   = 0U;
    }

    s_calib_ok = 1U;    /* PWM 方式: 段长就是 CCR4 里的值本身, 不存在"上电自标定" */
    s_inited   = 1U;

    bsp_mutex_init(&s_lock);     /* 多任务保护(必须在 bsp_rtos_init() 之后) */

    keyled_off();                /* 上电先确保全灭 */

    /* 再连发几帧全零: 上电瞬间引脚是浮空的, 灯珠可能已经把噪声锁存成自己的"上电默认色"
       (有的批次就是全白); 只发一帧的话, 万一这一帧被破坏, 灯就停在白色不动了。
       多补几帧(内容仍全零)把它压回"全灭"。帧数见 KEYLED_INIT_FRAMES。 */
    for (i = 1U; i < (uint8_t)KEYLED_INIT_FRAMES; i++)
    {
        keyled_show();
    }
}
/*===========================================================================
 *                              对外接口
 *=========================================================================*/
void keyled_set_fx(uint8_t idx, keyled_fx_t fx, uint8_t r, uint8_t g, uint8_t b, uint16_t period_ms)
{
    if (idx >= KEYLED_COUNT)
    {
        return;
    }

    bsp_mutex_lock(&s_lock);

    s_fx[idx].fx        = (uint8_t)fx;
    s_fx[idx].period_ms = (period_ms == 0U) ? KEYLED_DEFAULT_PERIOD_MS : period_ms;
    s_fx[idx].base_ms   = bsp_time_ms();

    s_rgb[((uint32_t)idx * 3U) + 0U] = r;
    s_rgb[((uint32_t)idx * 3U) + 1U] = g;
    s_rgb[((uint32_t)idx * 3U) + 2U] = b;

    keyled_show_locked();        /* 立即生效 */

    bsp_mutex_unlock(&s_lock);
}

/*---------------------------------------------------------------------------*/
void keyled_set_color(uint8_t idx, uint8_t r, uint8_t g, uint8_t b)
{
    keyled_set_fx(idx, KEYLED_FX_STATIC, r, g, b, KEYLED_DEFAULT_PERIOD_MS);
}

/*---------------------------------------------------------------------------*/
void keyled_fill(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t i;

    for (i = 0U; i < KEYLED_COUNT; i++)
    {
        keyled_set_color(i, r, g, b);
    }
}

/*---------------------------------------------------------------------------*/
void keyled_off(void)
{
    uint8_t i;

    bsp_mutex_lock(&s_lock);

    for (i = 0U; i < KEYLED_COUNT; i++)
    {
        s_fx[i].fx = (uint8_t)KEYLED_FX_OFF;
        s_rgb[((uint32_t)i * 3U) + 0U] = 0U;
        s_rgb[((uint32_t)i * 3U) + 1U] = 0U;
        s_rgb[((uint32_t)i * 3U) + 2U] = 0U;
    }
    keyled_show_locked();

    bsp_mutex_unlock(&s_lock);
}

/*---------------------------------------------------------------------------*/
void keyled_set_brightness(uint8_t brightness)
{
    s_brightness = brightness;   /* 单字节, 效果刷新(每 KEYLED_REFRESH_MS)会带上它 */
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  把"(高电平, 低电平)这一对"折成一个"高电平(ns)"值
  * @param  h_ns   调用者给的高电平(0 = 这次没给)
  * @param  l_ns   调用者给的低电平(0 = 这次没给)
  * @param  cur_ns 当前正在用的高电平(用来判断调用者到底改的是哪一项)
  * @return 这一对对应的高电平(ns)
  * @note   PWM 方式的位周期被定时器定死, 每条码只有"高电平"是自由量(低电平 =
  *         位周期 - 高电平)。串口命令 "KEYLED T0L xxx" 的做法是"先把现值读出来,
  *         只改一项, 再整组传进来", 所以这里要能认出"改的是低电平": 哪个值和现用值
  *         不一样就按哪个算(两个都自洽 / 都有改动时, 以"高电平"为准)。
  */
static uint16_t keyled_pick_high(uint16_t h_ns, uint16_t l_ns, uint16_t cur_ns)
{
    uint16_t h_from_h = h_ns;
    uint16_t h_from_l = ((l_ns == 0U) || (l_ns >= (uint16_t)KEYLED_BIT_NS))
                        ? 0U : (uint16_t)((uint16_t)KEYLED_BIT_NS - l_ns);

    if (h_from_h == 0U)
    {
        return (h_from_l != 0U) ? h_from_l : cur_ns;    /* 只给了低电平: 按低电平换算 */
    }
    if (h_from_l == 0U)
    {
        return h_from_h;
    }
    if (h_from_h == h_from_l)
    {
        return h_from_h;                                /* 两个值自洽 */
    }
    if (h_from_h == cur_ns)
    {
        return h_from_l;                                /* 高电平没动 -> 那就是低电平动了 */
    }
    return h_from_h;                                    /* 高电平动了(或有冲突): 以高电平为准 */
}
/*---------------------------------------------------------------------------*/
/**
  * @brief  重新设置位时序(ns), 立即生效并重发一帧
  * @param  t0h_ns 0 码高电平(80~600ns),   传 0 = 用回 KEYLED_T0H_NS
  * @param  t0l_ns 0 码低电平(200~2000ns), 传 0 = 忽略(PWM 方式低电平是推出来的)
  * @param  t1h_ns 1 码高电平(300~1600ns), 传 0 = 用回 KEYLED_T1H_NS
  * @param  t1l_ns 1 码低电平(150~1000ns), 传 0 = 忽略
  * @note   PWM 方式的位周期由定时器 ARR 定死(KEYLED_BIT_NS), 所以低电平 = 位周期 - 高
  *         电平; 四个值都会按规格书窗口夹一遍(高下限 / 高上限 / 低下限 / 低上限),
  *         参数打错也顶多被夹到边界, 不会把灯珠写成一片白。
  */
void keyled_set_timing(uint16_t t0h_ns, uint16_t t0l_ns, uint16_t t1h_ns, uint16_t t1l_ns)
{
    keyled_timing_t cur;
    uint32_t        tim_clk = keyled_tim_clock();
    int32_t         lo;
    int32_t         hi;
    int32_t         v0;
    int32_t         v1;

    keyled_get_timing_info(&cur);        /* 现值: 用来判断调用者改的是哪一项 */

    v0 = (int32_t)keyled_pick_high(t0h_ns, t0l_ns, cur.t0h_ns);
    v1 = (int32_t)keyled_pick_high(t1h_ns, t1l_ns, cur.t1h_ns);

    /* 0 码: 高 80~600ns, 低 200~2000ns(低 = 位周期 - 高) */
    lo = (int32_t)KEYLED_T0H_MIN_NS;
    hi = (int32_t)KEYLED_T0H_MAX_NS;
    if (((int32_t)KEYLED_BIT_NS - (int32_t)KEYLED_T0L_MAX_NS) > lo)
    {
        lo = (int32_t)KEYLED_BIT_NS - (int32_t)KEYLED_T0L_MAX_NS;
    }
    if (((int32_t)KEYLED_BIT_NS - (int32_t)KEYLED_T0L_MIN_NS) < hi)
    {
        hi = (int32_t)KEYLED_BIT_NS - (int32_t)KEYLED_T0L_MIN_NS;
    }
    if (hi < lo)
    {
        hi = lo;                         /* 位周期太短时这两条窗口没交集, 兜个底 */
    }
    if (v0 < lo)
    {
        v0 = lo;
    }
    if (v0 > hi)
    {
        v0 = hi;
    }

    /* 1 码: 高 300~1600ns, 低 150~1000ns */
    lo = (int32_t)KEYLED_T1H_MIN_NS;
    hi = (int32_t)KEYLED_T1H_MAX_NS;
    if (((int32_t)KEYLED_BIT_NS - (int32_t)KEYLED_T1L_MAX_NS) > lo)
    {
        lo = (int32_t)KEYLED_BIT_NS - (int32_t)KEYLED_T1L_MAX_NS;
    }
    if (((int32_t)KEYLED_BIT_NS - (int32_t)KEYLED_T1L_MIN_NS) < hi)
    {
        hi = (int32_t)KEYLED_BIT_NS - (int32_t)KEYLED_T1L_MIN_NS;
    }
    if (hi < lo)
    {
        hi = lo;
    }
    if (v1 < (v0 + (int32_t)KEYLED_T1H_GAP_NS))   /* 两个码必须能区分 */
    {
        v1 = v0 + (int32_t)KEYLED_T1H_GAP_NS;
    }
    if (v1 > hi)
    {
        v1 = hi;
    }
    if (v1 < lo)
    {
        v1 = lo;
    }

    bsp_mutex_lock(&s_lock);

    keyled_wait_idle();                  /* 缓冲区正被 DMA 读的时候不能改 */

    s_ccr0 = keyled_ns_to_ticks(tim_clk, (uint32_t)v0);
    s_ccr1 = keyled_ns_to_ticks(tim_clk, (uint32_t)v1);
    if (s_ccr0 > (uint16_t)(s_arr - 1U))
    {
        s_ccr0 = (uint16_t)(s_arr - 1U);
    }
    if (s_ccr1 > (uint16_t)(s_arr - 1U))
    {
        s_ccr1 = (uint16_t)(s_arr - 1U);
    }
    if (s_ccr1 <= s_ccr0)                /* 换算成拍数后别把两个码压成一样 */
    {
        s_ccr1 = (s_ccr0 < (uint16_t)(s_arr - 1U)) ? (uint16_t)(s_ccr0 + 1U) : s_ccr0;
    }

    /* 现场调时序时, 上一帧很可能是"错帧"(这正是要调它的原因): 先复位再重发,
       免得灯珠停在半帧状态 / 一直保持上一次的错误锁存。 */
    s_need_preamble = 1U;
    keyled_show_locked();                /* 立即重发一帧, 现场马上能看到颜色变化 */
    (void)keyled_wait_frames(1U);

    bsp_mutex_unlock(&s_lock);
}

/*---------------------------------------------------------------------------*/
/** 取当前时序信息(ns / 实际用的拍数); 调试用, 见串口 "KEYLED DIAG" */
void keyled_get_timing_info(keyled_timing_t *info)
{
    uint32_t mhz = SystemCoreClock / 1000000U;
    uint16_t bit = (uint16_t)KEYLED_BIT_NS;

    if (info == NULL)
    {
        return;
    }
    if (mhz == 0U)
    {
        mhz = 72U;
    }

    info->bit_ns = bit;                                             /* 位周期(定时器定死) */
    info->t0h_ns = (uint16_t)(((uint32_t)s_ccr0 * 1000U) / mhz);     /* 高 = CCR4 换算回来 */
    info->t1h_ns = (uint16_t)(((uint32_t)s_ccr1 * 1000U) / mhz);
    info->t0l_ns = (uint16_t)(bit - info->t0h_ns);                   /* 低电平是推出来的 */
    info->t1l_ns = (uint16_t)(bit - info->t1h_ns);

    info->t0h_cyc = s_ccr0;                                          /* 写进 CCR4 的值 */
    info->t1h_cyc = s_ccr1;
    info->t0l_cyc = (uint32_t)(s_arr - s_ccr0);                      /* 整位剩下的都是低 */
    info->t1l_cyc = (uint32_t)(s_arr - s_ccr1);

    /* comp / meas 是给老的"软件翻转"方式留的位置: PWM 方式段长由硬件保证, 恒为 0 */
    info->comp_t0h = 0U;
    info->comp_t0l = 0U;
    info->comp_t1h = 0U;
    info->comp_t1l = 0U;
    info->meas_t0h = 0U;
    info->meas_t0l = 0U;
    info->meas_t1h = 0U;
    info->meas_t1l = 0U;
}
/*---------------------------------------------------------------------------*/
/** 时序是否已就绪(1 = 可以发帧; PWM 方式恒为 1 —— 段长由比较寄存器保证, 不用标定) */
uint8_t keyled_calib_ok(void)
{
    return s_calib_ok;
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  数据线极性: 1 = 反相(数据线上串了反相器 / 三极管时用), 立即生效并重发一帧
  * @note   直接用硬件改 CC4 的输出极性(TIM_OC4PolarityConfig): 整条波形(含空闲/复位
  *         电平)一起翻过来, 高/低电平的时间长度完全不变, 比在软件里描波形简单也准。
  */
void keyled_set_invert(uint8_t invert)
{
    uint8_t v = (invert != 0U) ? 1U : 0U;

    bsp_mutex_lock(&s_lock);

    if (v != s_invert)
    {
        keyled_wait_idle();

        s_invert = v;
        TIM_OC4PolarityConfig(KEYLED_TIM,
                              (s_invert != 0U) ? TIM_OCPolarity_Low : TIM_OCPolarity_High);

        /* 极性刚翻过来, 线上电平也跟着翻了: 先按新极性给一段复位电平(空闲电平), 再重发 */
        s_need_preamble = 1U;
        keyled_show_locked();
        (void)keyled_wait_frames(1U);
    }

    bsp_mutex_unlock(&s_lock);
}

/*---------------------------------------------------------------------------*/
uint8_t keyled_get_invert(void)
{
    return s_invert;
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  现场核实"数据线到底通不通": 先把引脚按低电平驱动、再按高电平驱动,
  *         各自读回引脚的真实电平(调试用, 见串口 "KEYLED DIAG")
  * @param  low_read  引脚被驱动为低时读回的电平(0/1)
  * @param  high_read 引脚被驱动为高时读回的电平(0/1)
  * @note   正常推挽输出读回和驱动一致(低->0, 高->1); 任一方向读回相反 = 这条线被
  *         外部强驱动 / 接错线 / 和别的信号短接。(单纯的"反相器 / 三极管在灯珠侧"
  *         读回来仍然是一致的, 那种要试 KEYLED INVERT 1。)
  *         本函数会往灯珠上推一段乱电平, 结束时自动"复位 + 重发当前一帧"。
  *         探测期间把 PC9 临时切成普通推挽输出(定时器的 CC4 就管不到它了), 探完再交回去。
  */
void keyled_probe_line(uint8_t *low_read, uint8_t *high_read)
{
    GPIO_InitTypeDef gpio;
    uint8_t          lo;
    uint8_t          hi;

    bsp_mutex_lock(&s_lock);

    keyled_wait_idle();                              /* 别撞上正在发的帧 */

    /* 先摆好 ODR 再切输出态: 否则从复用态切过来的一瞬间会按 ODR 里的旧值驱动一下 */
    GPIO_ResetBits(KEYLED_PORT, KEYLED_PIN);
    gpio.GPIO_Pin   = KEYLED_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(KEYLED_PORT, &gpio);

    GPIO_ResetBits(KEYLED_PORT, KEYLED_PIN);
    bsp_delay_us(10U);
    lo = (GPIO_ReadInputDataBit(KEYLED_PORT, KEYLED_PIN) != 0U) ? 1U : 0U;

    GPIO_SetBits(KEYLED_PORT, KEYLED_PIN);
    bsp_delay_us(10U);
    hi = (GPIO_ReadInputDataBit(KEYLED_PORT, KEYLED_PIN) != 0U) ? 1U : 0U;

    /* 交回给定时器(复用推挽), 再复位 + 重发当前一帧, 免得灯珠停在半帧上 */
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(KEYLED_PORT, &gpio);

    s_need_preamble = 1U;
    keyled_show_locked();
    (void)keyled_wait_frames(1U);

    bsp_mutex_unlock(&s_lock);

    if (low_read != NULL)
    {
        *low_read = lo;
    }
    if (high_read != NULL)
    {
        *high_read = hi;
    }
}

/*---------------------------------------------------------------------------*/
/** 取某路"设定值"(未经效果/亮度处理), 配合 keyled_get_output() 排查用 */
void keyled_get_set(uint8_t idx, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (idx >= KEYLED_COUNT)
    {
        return;
    }
    if (r != NULL)
    {
        *r = s_rgb[((uint32_t)idx * 3U) + 0U];
    }
    if (g != NULL)
    {
        *g = s_rgb[((uint32_t)idx * 3U) + 1U];
    }
    if (b != NULL)
    {
        *b = s_rgb[((uint32_t)idx * 3U) + 2U];
    }
}
/*---------------------------------------------------------------------------*/
static void keyled_show_locked(void)
{
    keyled_update_output();
    keyled_tx();
}

/*---------------------------------------------------------------------------*/
void keyled_show(void)
{
    bsp_mutex_lock(&s_lock);
    keyled_show_locked();
    /* 上面那次调用只是"把这一帧交给 DMA": 这里等它真发完(336 位 x 1.25us ≈ 420us)。
       命令回显 / 上线自检要的是"确实发完了"。 */
    (void)keyled_wait_frames(1U);
    bsp_mutex_unlock(&s_lock);
}

/*---------------------------------------------------------------------------*/
/** 已经真正发完的帧数(DIAG 用: 发一条颜色命令后再看, 涨了就说明发帧这条路是活的) */
uint32_t keyled_frames(void)
{
    return s_frames;
}

/*---------------------------------------------------------------------------*/
void keyled_process(void)
{
    uint8_t i;
    uint8_t changed = 0U;

    if (s_inited == 0U)
    {
        return;
    }

    if (bsp_elapsed_ms(s_last_refresh_ms) < KEYLED_REFRESH_MS)
    {
        return;
    }
    s_last_refresh_ms = bsp_time_ms();

    bsp_mutex_lock(&s_lock);

    keyled_update_output();

    for (i = 0U; i < (uint8_t)sizeof(s_out); i++)
    {
        if (s_out[i] != s_shown[i])
        {
            changed = 1U;
            break;
        }
    }

    if (changed != 0U)
    {
        keyled_tx();                 /* 内容变了才发, 空闲时不会反复刷同一帧 */
    }

    bsp_mutex_unlock(&s_lock);
}

/*---------------------------------------------------------------------------*/
void keyled_get_output(uint8_t idx, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (idx >= KEYLED_COUNT)
    {
        return;
    }
    if (r != NULL)
    {
        *r = s_out[((uint32_t)idx * 3U) + 0U];
    }
    if (g != NULL)
    {
        *g = s_out[((uint32_t)idx * 3U) + 1U];
    }
    if (b != NULL)
    {
        *b = s_out[((uint32_t)idx * 3U) + 2U];
    }
}

/*---------------------------------------------------------------------------*/
void keyled_set_buffer(const uint8_t *rgb, uint8_t count)
{
    uint8_t i;

    if (rgb == NULL)
    {
        return;
    }
    if (count > KEYLED_COUNT)
    {
        count = KEYLED_COUNT;
    }

    bsp_mutex_lock(&s_lock);

    for (i = 0U; i < count; i++)
    {
        s_fx[i].fx = (uint8_t)KEYLED_FX_STATIC;
        s_rgb[((uint32_t)i * 3U) + 0U] = rgb[((uint32_t)i * 3U) + 0U];
        s_rgb[((uint32_t)i * 3U) + 1U] = rgb[((uint32_t)i * 3U) + 1U];
        s_rgb[((uint32_t)i * 3U) + 2U] = rgb[((uint32_t)i * 3U) + 2U];
    }
    keyled_show_locked();

    bsp_mutex_unlock(&s_lock);
}







