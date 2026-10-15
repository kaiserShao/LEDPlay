/**
  ******************************************************************************
  * @file    drv_keyled.c
  * @brief   按键灯 WS2812 驱动 (PC4 单线, 4 颗, 软件时序)
  ******************************************************************************
  */
#include "drv_keyled.h"
#include "bsp_rtos.h"
#include "bsp_time.h"
#include "bsp_util.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"

#define KEYLED_PIN_HIGH()    GPIO_SetBits(KEYLED_PORT, KEYLED_PIN)
#define KEYLED_PIN_LOW()     GPIO_ResetBits(KEYLED_PORT, KEYLED_PIN)

/* "数据电平" = 灯珠 DIN 上看到的电平。KEYLED_INVERT = 1 时整条波形取反
   (数据线上串了反相器 / 三极管的反相电路时用), 连空闲/复位电平也一起反过来。
   注意: 反相只改变管脚电平的极性, 高低电平的时间长度完全不变。 */
#define KEYLED_D_HIGH()      ((s_invert != 0U) ? KEYLED_PIN_LOW() : KEYLED_PIN_HIGH())
#define KEYLED_D_LOW()       ((s_invert != 0U) ? KEYLED_PIN_HIGH() : KEYLED_PIN_LOW())

/** 每路效果状态 */
typedef struct
{
    uint8_t  fx;          /*!< keyled_fx_t */
    uint16_t period_ms;   /*!< 效果周期 */
    uint32_t base_ms;     /*!< 效果起始时间 */
} keyled_fx_state_t;

static uint8_t  s_rgb[KEYLED_COUNT * 3U];    /*!< 应用设定的颜色 */
static uint8_t  s_out[KEYLED_COUNT * 3U];    /*!< 加上效果/亮度后的输出颜色 */
static uint8_t  s_shown[KEYLED_COUNT * 3U];  /*!< 最近一次真正发送的颜色 */
static keyled_fx_state_t s_fx[KEYLED_COUNT];
static uint8_t  s_brightness = 255U;
static uint8_t  s_inited = 0U;
static uint32_t s_last_refresh_ms = 0U;

/* 多任务保护: 按键灯的颜色/效果可能被"命令任务"和"灯光任务"同时修改,
   而 s_rgb 与 s_out/s_shown 必须成套一致, 因此整段操作加锁。 */
static bsp_mutex_t s_lock;

/*---------------------------------------------------------------------------*/
/* 内部: 刷新输出(调用者必须已经持有 s_lock) */
static void keyled_show_locked(void);

/* 时序(单位: CPU 周期)。s_nom_* = 目标值(ns 换算而来), s_cyc_* = 真正使用的值
   (目标值扣掉延时函数/循环的固定开销, 由 keyled_calibrate() 上电实测得到)。
   每位分"高电平 + 低电平"两段, 0 码/1 码各自一套(T0H/T0L/T1H/T1L): 规格书只规定
   这四段各自的上下限, 没有"位周期必须 1.25us"的硬约束, 所以这里不设位周期 ——
   位周期就是 T?H + T?L, 由这两段自然决定。 */
#define KEYLED_CALIB_BITS     24U    /* 自标定取样位数(1 字节) */
#define KEYLED_CALIB_LOOP     3U     /* 自标定迭代次数(2~3 次就收敛) */

/* 串口实时改时序时允许的范围(ns): 只放开规格书量级, 免得参数打错把灯珠写成一片白 */
#define KEYLED_T0H_MIN_NS     80U
#define KEYLED_T0H_MAX_NS     600U
#define KEYLED_T0L_MIN_NS     200U
#define KEYLED_T0L_MAX_NS     2000U
#define KEYLED_T1H_MIN_NS     300U
#define KEYLED_T1H_MAX_NS     1600U
#define KEYLED_T1L_MIN_NS     150U
#define KEYLED_T1L_MAX_NS     1000U
#define KEYLED_T1H_GAP_NS     100U   /* 两个码的高电平至少要差这么多, 灯珠才分得开 */

static uint32_t s_cyc_t0h = 21U;
static uint32_t s_cyc_t0l = 61U;
static uint32_t s_cyc_t1h = 54U;
static uint32_t s_cyc_t1l = 23U;
static uint32_t s_nom_t0h = 21U;     /* 0 码高电平目标(周期) */
static uint32_t s_nom_t0l = 61U;     /* 0 码低电平目标(周期) */
static uint32_t s_nom_t1h = 54U;     /* 1 码高电平目标(周期) */
static uint32_t s_nom_t1l = 23U;     /* 1 码低电平目标(周期) */
#if (KEYLED_USE_TIM_IRQ == 0U)      /* 下面四个只有老的 DWT 忙等方式才用 */
static uint32_t s_meas_high = 0U;    /* 标定用: 最近这个位"高电平"实测长度(周期) */
static uint32_t s_meas_low  = 0U;    /* 标定用: 上一个位"低电平"实测长度(周期) */
static uint32_t s_last_low_cyc = 0U; /* 标定用: 上一个位"拉低"后的采样点(0 = 还没有) */
static uint8_t  s_use_dwt = 1U;
#endif
static uint8_t  s_invert  = KEYLED_INVERT;
static uint8_t  s_calib_ok = 0U;     /* 1 = 上电自标定跑过(时序是按实测校准的) */
static uint8_t  s_need_preamble = 1U; /*!< 1 = 下一帧前先给一段复位电平(上电第一帧用) */

#if (KEYLED_USE_TIM_IRQ != 0U)
/*===================== 定时器中断发帧(默认方式) ==========================
  为什么这么做: PC4 在 STM32F103 上**没有定时器通道**(PC4 只有 ADC12_IN14 一个复用
  功能), 所以"定时器直接吐波形"这条路不通; 但可以让一个没人用的基本定时器当**秒表**:
    * KEYLED_TIM(TIM6: 只有计数器 + 更新中断, 不占引脚 / 不占比较通道 / 不占 DMA)
      跑 72MHz(PSC = 0), 1 拍 = 13.89ns;
    * 要发帧时 keyled_tx() 只做三件事: 整帧拷进影子缓冲 s_txbuf、置 s_req、
      软件触发一次更新事件 -> 中断立刻进来(不用等下一个节拍);
    * 96 位时序全在中断里 keyled_send_frame() 跑: 每段"采样 CNT -> 等到够拍数",
      段长就是 s_*_cyc 那个拍数本身, 没有"空循环固定开销"这回事, 所以不用自标定;
    * 中断优先级 = 0(BSP_IRQ_PRIO_KEYLED_TIM), 同/低优先级都打断不了它, 所以发帧
      期间不用关总中断(老 DWT 方式得靠 KEYLED_LOCK_IRQ 关中断)。
  影子缓冲: 命令任务改颜色和中断发帧可能撞上, 所以请求时先在临界区里整帧拷一份,
  中断只读这一份, 保证"一帧里的颜色是一致的"。 */
static uint8_t  s_txbuf[KEYLED_COUNT * 3U];        /*!< 影子缓冲: 中断发帧只读它 */
static volatile uint8_t  s_req = 0U;               /*!< 1 = 有一帧等着发 */
static volatile uint32_t s_frames = 0U;            /*!< 中断真正发出去过多少帧(DIAG 验证用) */
#define KEYLED_MEAS_NONE   0xFFFFFFFFU             /*!< 本帧还没有这种码的样本 */
static uint32_t s_meas_t0h_cyc = KEYLED_MEAS_NONE; /*!< 本帧实测: 0 码高电平走掉的拍数(取最小) */
static uint32_t s_meas_t0l_cyc = KEYLED_MEAS_NONE; /*!< 本帧实测: 0 码低电平走掉的拍数 */
static uint32_t s_meas_t1h_cyc = KEYLED_MEAS_NONE; /*!< 本帧实测: 1 码高电平走掉的拍数 */
static uint32_t s_meas_t1l_cyc = KEYLED_MEAS_NONE; /*!< 本帧实测: 1 码低电平走掉的拍数 */
#endif


/*---------------------------------------------------------------------------*/
/** 等待到"从 start 起经过 target 个 CPU 周期" */
static void keyled_wait(uint32_t start, uint32_t target)
{
#if (KEYLED_USE_TIM_IRQ != 0U)
    /* 定时器方式: 直接数 KEYLED_TIM->CNT 的拍数(72MHz, 1 拍 = 13.89ns)。
       target 是"从 start 这个采样点起还要走过多少拍"; 用 16 位差值比较, 计数回绕也安全。
       每段都从本段起点重新采样, 所以中断入口晚多少、前面多干了多少活, 都不会加进段长。 */
    uint32_t cyc = target;

    if (cyc > (uint32_t)KEYLED_TIM_WAIT_COMP_CYC)
    {
        cyc -= (uint32_t)KEYLED_TIM_WAIT_COMP_CYC;   /* 抠掉循环收尾误差(见 bsp_config.h 第 5 节) */
    }
    if (cyc < 2U)
    {
        cyc = 2U;
    }
    while ((uint16_t)(KEYLED_TIM->CNT - (uint16_t)start) < (uint16_t)cyc)
    {
        /* 等够拍数 */
    }
#else
    if (s_use_dwt != 0U)
    {
        while ((bsp_cycles() - start) < target)
        {
            /* 空转 */
        }
    }
    else
    {
        /* DWT 不可用时的兜底: 空循环(未校准, 只保证不会卡在上面的 while 里) */
        volatile uint32_t n = target / 3U;

        while (n != 0U)
        {
            n--;
        }
    }
#endif  /* KEYLED_USE_TIM_IRQ */
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  发送 1 个位: 先拉高 T0H/T1H, 再拉低 T0L/T1L —— 两段各自独立计时
  * @note   两段的时间基准都取在"管脚刚翻转之后"(紧接着写 GPIO 寄存器就采样),
  *         所以写寄存器那十来拍的延迟不会算进灯珠看到的高/低电平里。
  *         (旧版把基准取在写寄存器之前、又用"固定 1.25us 位周期"去撑低电平: 灯珠看到的
  *         高电平比配置值短约 200ns, T0L 被撑到 1.1us 以上、T1L 被压到规格下限附近,
  *         四条相位里两条低电平都跑出手册范围。)
  *         顺手把两段的实测长度记进 s_meas_high / s_meas_low, 给上电自标定用
  *         (生产路径里没人读它, 每个位只多花两三拍)。
  */
static void keyled_bit(uint8_t one)
{
#if (KEYLED_USE_TIM_IRQ != 0U)
    /* 定时器方式: 两段的长度都交给"数 KEYLED_TIM->CNT 的拍数", 和编译器/优化等级无关。
       顺手把"实测拍了多少"记下来(单帧取最小), DIAG 里 meas 那一行就是它 —— 没有示波器
       也能看出每段比目标长了几拍。 */
    uint16_t t0;
    uint16_t t1;
    uint16_t t2;
    uint32_t cyc_hi = (one != 0U) ? s_cyc_t1h : s_cyc_t0h;
    uint32_t cyc_lo = (one != 0U) ? s_cyc_t1l : s_cyc_t0l;
    uint32_t d;

    KEYLED_D_HIGH();                                  /* 拉高 */
    t0 = (uint16_t)KEYLED_TIM->CNT;                    /* 高电平起点 */
    keyled_wait((uint32_t)t0, cyc_hi);

    KEYLED_D_LOW();                                    /* 拉低 */
    t1 = (uint16_t)KEYLED_TIM->CNT;                    /* 低电平起点 */
    keyled_wait((uint32_t)t1, cyc_lo);
    t2 = (uint16_t)KEYLED_TIM->CNT;

    /* 实测(取本帧最小值: 最小值最接近"纯"延时, 不受偶发多算影响) */
    d = (uint32_t)(uint16_t)(t1 - t0);
    if (one != 0U)
    {
        if (d < s_meas_t1h_cyc)
        {
            s_meas_t1h_cyc = d;
        }
    }
    else if (d < s_meas_t0h_cyc)
    {
        s_meas_t0h_cyc = d;
    }

    d = (uint32_t)(uint16_t)(t2 - t1);
    if (one != 0U)
    {
        if (d < s_meas_t1l_cyc)
        {
            s_meas_t1l_cyc = d;
        }
    }
    else if (d < s_meas_t0l_cyc)
    {
        s_meas_t0l_cyc = d;
    }
#else
    uint32_t t_hi;
    uint32_t t_lo;

    KEYLED_D_HIGH();                                  /* 拉高 */
    t_hi = bsp_cycles();                              /* 高电平起点 */
    if (s_last_low_cyc != 0U)
    {
        s_meas_low = t_hi - s_last_low_cyc;           /* 结算上一个位的低电平 */
    }
    keyled_wait(t_hi, (one != 0U) ? s_cyc_t1h : s_cyc_t0h);

    KEYLED_D_LOW();                                   /* 拉低 */
    t_lo = bsp_cycles();                              /* 低电平起点 */
    s_meas_high    = t_lo - t_hi;
    s_last_low_cyc = t_lo;

    keyled_wait(t_lo, (one != 0U) ? s_cyc_t1l : s_cyc_t0l);
#endif  /* KEYLED_USE_TIM_IRQ */
}

/*---------------------------------------------------------------------------*/
/** 发送 1 个字节(MSB 先出) */
static void keyled_send_byte(uint8_t v)
{
    uint8_t mask;

    for (mask = 0x80U; mask != 0U; mask = (uint8_t)(mask >> 1))
    {
        keyled_bit((v & mask) != 0U);
    }
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  实测某种码的"高电平 / 低电平"实际有多长(CPU 周期), 只在管脚不驱动灯带时调用
  * @param  one  0 = 0 码, 1 = 1 码
  * @param  n    取样位数(每个位高、低两段各出一组样本)
  * @param  hi   返回: 高电平最小值(周期), 不需要传 NULL
  * @param  lo   返回: 低电平最小值(周期), 不需要传 NULL
  * @note   取"最小值"而不是平均值: 中断/总线争用只会把某一次的采样拉长, 最小值最
  *         接近真实值(所以标定不必关中断)。低电平是"上一个位拉低 -> 本位拉高",
  *         所以每组的第一个位不参与低电平统计。
  */
#if (KEYLED_USE_TIM_IRQ == 0U)
static void keyled_measure(uint8_t one, uint16_t n, uint32_t *hi, uint32_t *lo)
{
    uint32_t min_hi = 0xFFFFFFFFU;
    uint32_t min_lo = 0xFFFFFFFFU;
    uint16_t i;

    s_last_low_cyc = 0U;              /* 这一组的第一个位没有"上一个位" */

    for (i = 0U; i < n; i++)
    {
        keyled_bit(one);

        if (s_meas_high < min_hi)
        {
            min_hi = s_meas_high;
        }
        if ((i != 0U) && (s_meas_low < min_lo))
        {
            min_lo = s_meas_low;
        }
    }

    if (hi != NULL)
    {
        *hi = (min_hi == 0xFFFFFFFFU) ? 0U : min_hi;
    }
    if (lo != NULL)
    {
        *lo = (min_lo == 0xFFFFFFFFU) ? 0U : min_lo;
    }
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  用一次实测结果修正延时常数
  * @param  cur      当前使用的周期数
  * @param  nominal  目标周期数
  * @param  meas     实测"高电平"周期数(含固定开销)
  * @param  min_cyc  允许的最小值
  * @return 修正后的周期数(收敛很快, 两三次就稳)
  */
static uint32_t keyled_refine(uint32_t cur, uint32_t nominal, uint32_t meas, uint32_t min_cyc)
{
    int32_t next = (int32_t)cur - ((int32_t)meas - (int32_t)nominal);

    if (next < (int32_t)min_cyc)
    {
        next = (int32_t)min_cyc;
    }
    if (next > (int32_t)(nominal + 16U))
    {
        next = (int32_t)(nominal + 16U);      /* 兜一下, 免得参数被推到离谱的值 */
    }

    return (uint32_t)next;
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  上电自标定: 实测四个相位(0/1 码的高、低电平)实际有多长, 把延时函数/
  *         循环的固定开销扣掉
  * @note   必须在管脚处于输入态(不驱动灯带)时调用 —— 否则标定用的乱时序
  *         会被推给灯珠。DWT 不可用时不做标定, 只保留 KEYLED_CYC_COMP 补偿。
  */
static void keyled_calibrate(void)
{
    uint8_t  loop;
    uint32_t m_hi;
    uint32_t m_lo;

    for (loop = 0U; loop < KEYLED_CALIB_LOOP; loop++)
    {
        keyled_measure(0U, KEYLED_CALIB_BITS, &m_hi, &m_lo);
        s_cyc_t0h = keyled_refine(s_cyc_t0h, s_nom_t0h, m_hi, 4U);
        s_cyc_t0l = keyled_refine(s_cyc_t0l, s_nom_t0l, m_lo, 8U);

        keyled_measure(1U, KEYLED_CALIB_BITS, &m_hi, &m_lo);
        s_cyc_t1h = keyled_refine(s_cyc_t1h, s_nom_t1h, m_hi, 8U);
        s_cyc_t1l = keyled_refine(s_cyc_t1l, s_nom_t1l, m_lo, 8U);
    }

    s_last_low_cyc = 0U;
    s_calib_ok = 1U;
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  把数据线摆到"复位(空闲)电平"并保持 KEYLED_RESET_US
  * @note   灯珠只有在 DIN 上看到足够长的低电平(规格书 >=280us)之后, 才会把刚收到的
  *         96 位真正锁存下来。所以上电第一帧之前必须先做这件事 —— 否则从"输入态"
  *         切到推挽输出的瞬间引脚电平是未知的, 灯珠可能把噪声当成一帧数据锁存成
  *         自己的"上电默认色"(有些批次上电就是全白), 之后无论发什么颜色都不理。
  *         本函数内部有延时, 必须在"开中断"的状态下调用(不要放进关中断段)。
  */
#endif  /* KEYLED_USE_TIM_IRQ == 0U: 上面四件套(实测/精修/自标定)只有老 DWT 方式才用 */

/*---------------------------------------------------------------------------*/
static void keyled_line_reset(void)
{
    KEYLED_D_LOW();
#if (KEYLED_USE_TIM_IRQ != 0U)
    {
        /* 复位电平也数定时器的拍(300us = 21600 拍, 远小于 16 位计数的回绕周期 910us):
           这样连"DWT 能不能用"都不再影响帧前/帧间复位长度 */
        uint16_t t0  = (uint16_t)KEYLED_TIM->CNT;
        uint32_t cyc = ((SystemCoreClock / 1000000U) * (uint32_t)KEYLED_RESET_US);

        while ((uint32_t)(uint16_t)(KEYLED_TIM->CNT - t0) < cyc)
        {
            /* 等够复位电平 */
        }
    }
#else
    bsp_delay_us(KEYLED_RESET_US);
#endif
}

/*---------------------------------------------------------------------------*/
/** 把 s_out 中的 4 颗灯按 WS2812 时序(G R B)发送出去 */
/** 一帧的全部位: 帧前复位(需要时) + 96 位(每颗 G R B) + 帧尾复位电平。
    @param rgb 要发的颜色(3 字节/颗); 两个调用者: 定时器中断发 s_txbuf(影子缓冲),
               老 DWT 方式直接发 s_out */
static void keyled_send_frame(const uint8_t *rgb)
{
    uint8_t i;

    if (s_need_preamble != 0U)
    {
        s_need_preamble = 0U;
        keyled_line_reset();         /* 帧前复位: 上电第一帧 / 现场改完参数后需要 */
    }

#if (KEYLED_USE_TIM_IRQ != 0U)
    /* 本帧的实测统计从"没有样本"开始(见 keyled_bit) */
    s_meas_t0h_cyc = KEYLED_MEAS_NONE;
    s_meas_t0l_cyc = KEYLED_MEAS_NONE;
    s_meas_t1h_cyc = KEYLED_MEAS_NONE;
    s_meas_t1l_cyc = KEYLED_MEAS_NONE;
#endif

#if (KEYLED_USE_TIM_IRQ == 0U)      /* 帧首清掉「上一位低电平」的采样点(只有 DWT 方式记它) */
    s_last_low_cyc = 0U;
#endif

    for (i = 0U; i < KEYLED_COUNT; i++)
    {
        keyled_send_byte(rgb[(uint32_t)i * 3U + 1U]);     /* G */
        keyled_send_byte(rgb[(uint32_t)i * 3U + 0U]);     /* R */
        keyled_send_byte(rgb[(uint32_t)i * 3U + 2U]);     /* B */
    }

    KEYLED_D_LOW();                                      /* 帧尾复位电平(不反相时是低) */

#if (KEYLED_USE_TIM_IRQ != 0U)
    s_frames++;
#endif
}

/*---------------------------------------------------------------------------*/
/** 请求发一帧(keyled_show_locked() / keyled_process() 调用)。
    定时器方式: 只把这一帧拷进影子缓冲并置标志, 真正的位输出在定时器中断里做;
    老 DWT 方式: 当场忙等把这一帧发完(输出期间关中断)。 */
static void keyled_tx(void)
{
#if (KEYLED_USE_TIM_IRQ != 0U)
    uint8_t  i;
    uint32_t pm = bsp_irq_save();        /* "拷影子缓冲 + 置标志"要对中断原子(只有 12 字节) */

    for (i = 0U; i < (uint8_t)sizeof(s_out); i++)
    {
        s_txbuf[i] = s_out[i];
        s_shown[i] = s_out[i];           /* s_shown = "已经认下来的颜色" */
    }
    s_req = 1U;

    /* 软件触发一次更新事件: 中断立刻进来, 不用等下一个节拍 */
    TIM_GenerateEvent(KEYLED_TIM, TIM_EventSource_Update);
    bsp_irq_restore(pm);
#else
    uint8_t i;
#if (KEYLED_LOCK_IRQ != 0U)
    uint32_t pm;
#endif

    if (s_need_preamble != 0U)
    {
        s_need_preamble = 0U;
        keyled_line_reset();         /* 帧前复位: 上电第一帧 / 现场改完参数后需要 */
    }

#if (KEYLED_LOCK_IRQ != 0U)
    pm = bsp_irq_save();
#endif

    /* 帧首要清掉「上一位低电平」的采样点: keyled_send_frame() 内部会做 */
    keyled_send_frame(s_out);     /* 与定时器方式共用同一段「96 位 + 帧尾」代码:
                                     帧前复位已在上面发过(s_need_preamble 已清),
                                     所以这里只发位流 */

#if (KEYLED_LOCK_IRQ != 0U)
    bsp_irq_restore(pm);                                 /* 波形发完了, 中断可以开了 */
#endif

    /* 复位时间不需要精确, 可以在中断打开的情况下等 */
    bsp_delay_us(KEYLED_RESET_US);

    for (i = 0U; i < (uint8_t)sizeof(s_shown); i++)
    {
        s_shown[i] = s_out[i];
    }
#endif  /* KEYLED_USE_TIM_IRQ */
}


#if (KEYLED_USE_TIM_IRQ != 0U)
/*---------------------------------------------------------------------------*/
/** 定时器(TIM6/TIM7)初始化: 72MHz 自由计数 + 每 KEYLED_TIM_TICK_US 一次更新中断。
    只用到"计数器 + 更新中断": 不占引脚、不占比较通道、不占 DMA。
    TIM6/TIM7 是 16 位计数器, PSC = 0 时一个计数周期最长只有 910us, 所以节拍不能更大。 */
static void keyled_tim_init(void)
{
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef        nvic;

    RCC_APB1PeriphClockCmd(KEYLED_TIM_CLK, ENABLE);

    tim.TIM_Prescaler         = 0U;      /* 72MHz: 1 拍 = 13.89ns —— 精度的根 */
    tim.TIM_CounterMode       = TIM_CounterMode_Up;
    tim.TIM_Period            = (uint16_t)(((SystemCoreClock / 1000000U) * (uint32_t)KEYLED_TIM_TICK_US) - 1U);
    tim.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(KEYLED_TIM, &tim);

    TIM_SetCounter(KEYLED_TIM, 0U);
    TIM_ClearITPendingBit(KEYLED_TIM, TIM_IT_Update);
    TIM_ITConfig(KEYLED_TIM, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel                   = KEYLED_TIM_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = BSP_IRQ_PRIO_KEYLED_TIM;
    nvic.NVIC_IRQChannelSubPriority        = 0U;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(KEYLED_TIM, ENABLE);
}

/*---------------------------------------------------------------------------*/
/** 定时器更新中断: 有帧要发就把 96 位一次发完(约 120us), 没有就立刻返回。
    本中断优先级最高(BSP_IRQ_PRIO_KEYLED_TIM = 0): 同/低优先级都打断不了它,
    所以中断里数 CNT 数出来的段长就是准的 —— 这也正是"不用关总中断"的原因。 */
void KEYLED_TIM_IRQHANDLER(void)
{
    if (TIM_GetITStatus(KEYLED_TIM, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(KEYLED_TIM, TIM_IT_Update);

        if (s_req != 0U)
        {
            s_req = 0U;
            keyled_send_frame(s_txbuf);      /* 只发影子缓冲里那一份, 一帧颜色是一致的 */
        }
    }
}

/*---------------------------------------------------------------------------*/
/** 等中断把刚请求的帧真的发出去(keyled_show() 用: 命令/自检要的是"确实发完了")。
    超时兜底: 万一中断没跑起来也不会死等。
    @return 1 = 等到了, 0 = 超时 */
static uint8_t keyled_wait_frames(uint32_t n)
{
    uint32_t start = bsp_time_ms();
    uint32_t want  = s_frames + n;
    uint32_t limit = ((n * (uint32_t)KEYLED_TIM_TICK_US) / 1000U) + 50U;

    while (s_frames < want)
    {
        if (bsp_elapsed_ms(start) > limit)
        {
            return 0U;
        }
    }

    return 1U;
}
#endif  /* KEYLED_USE_TIM_IRQ */

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

        r = s_rgb[(uint32_t)i * 3U + 0U];
        g = s_rgb[(uint32_t)i * 3U + 1U];
        b = s_rgb[(uint32_t)i * 3U + 2U];

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
        s_out[(uint32_t)i * 3U + 0U] = (uint8_t)(((uint16_t)r * scale) >> 8);
        s_out[(uint32_t)i * 3U + 1U] = (uint8_t)(((uint16_t)g * scale) >> 8);
        s_out[(uint32_t)i * 3U + 2U] = (uint8_t)(((uint16_t)b * scale) >> 8);
    }
}

/*===========================================================================
 *                              对外接口
 *=========================================================================*/
void keyled_init(void)
{
    GPIO_InitTypeDef gpio;
    uint32_t         mhz = SystemCoreClock / 1000000U;
    uint8_t          i;

    if (mhz == 0U)
    {
        mhz = 72U;
    }

    /* 纳秒 -> CPU 周期(目标值) */
    s_nom_t0h = ((uint32_t)KEYLED_T0H_NS * mhz) / 1000U;
    s_nom_t0l = ((uint32_t)KEYLED_T0L_NS * mhz) / 1000U;
    s_nom_t1h = ((uint32_t)KEYLED_T1H_NS * mhz) / 1000U;
    s_nom_t1l = ((uint32_t)KEYLED_T1L_NS * mhz) / 1000U;

    s_cyc_t0h = s_nom_t0h;
    s_cyc_t0l = s_nom_t0l;
    s_cyc_t1h = s_nom_t1h;
    s_cyc_t1l = s_nom_t1l;
#if (KEYLED_USE_TIM_IRQ == 0U)
    s_last_low_cyc = 0U;         /* 只有老的 DWT 方式(位测量)读它 */
#endif

    RCC_APB2PeriphClockCmd(KEYLED_GPIO_CLK, ENABLE);

#if (KEYLED_USE_TIM_IRQ != 0U)
    /* 定时器方式: 段长就是 KEYLED_TIM 的拍数本身(见 bsp_config.h 第 5 节), 不需要
       自标定, 也和 DWT 能不能用无关。这里顺便把定时器 + 发帧中断开起来 —— 此刻
       引脚还是复位后的输入态, 但 s_req = 0, 中断进来只会立刻返回, 不会乱发帧。 */
    /* 本方式不用 DWT 计时: s_use_dwt / s_last_low_cyc / s_meas_* 这些只属于老方式 */
    s_calib_ok = 1U;             /* calib=1 = "段长就是配置值本身, 精度由定时器保证" */
    keyled_tim_init();
#else
    /* DWT 周期计数在 bsp_time_init() 里已经探过 */
    s_use_dwt = bsp_time_dwt_ok();

    if ((s_use_dwt != 0U) && (KEYLED_CALIB != 0U))
    {
        /* 标定期间把管脚设成"输入 + 下拉": 不驱动灯带, 免得把标定用的乱时序
           推给灯珠(线被拉低 = 灯珠看到复位电平, 保持原来的显示状态)。 */
        gpio.GPIO_Pin   = KEYLED_PIN;
        gpio.GPIO_Speed = GPIO_Speed_50MHz;
        gpio.GPIO_Mode  = GPIO_Mode_IPD;
        GPIO_Init(KEYLED_PORT, &gpio);

        keyled_calibrate();
    }
    else
    {
        /* DWT 不可用(或关掉了 KEYLED_CALIB): 只能按经验扣一个固定开销,
           时序精度会差一些 */
        if (s_cyc_t0h > KEYLED_CYC_COMP)
        {
            s_cyc_t0h -= KEYLED_CYC_COMP;
        }
        if (s_cyc_t0l > KEYLED_CYC_COMP)
        {
            s_cyc_t0l -= KEYLED_CYC_COMP;
        }
        if (s_cyc_t1h > KEYLED_CYC_COMP)
        {
            s_cyc_t1h -= KEYLED_CYC_COMP;
        }
        if (s_cyc_t1l > KEYLED_CYC_COMP)
        {
            s_cyc_t1l -= KEYLED_CYC_COMP;
        }
    }
#endif  /* KEYLED_USE_TIM_IRQ */

    if (s_cyc_t0h < 4U)
    {
        s_cyc_t0h = 4U;
    }
    if (s_cyc_t0l < 8U)
    {
        s_cyc_t0l = 8U;
    }
    if (s_cyc_t1h < 8U)
    {
        s_cyc_t1h = 8U;
    }
    if (s_cyc_t1l < 8U)
    {
        s_cyc_t1l = 8U;
    }

    /* 正式输出: 推挽输出。
       切输出模式之前先把 ODR 写成"空闲(复位)电平": 否则从输入态切过来的瞬间,
       引脚会按上一次标定遗留在 ODR 里的值驱动一下(可能冒一个高电平脉冲)。 */
    if (s_invert != 0U)
    {
        KEYLED_PIN_HIGH();           /* 反相: 引脚高 = 灯珠侧数据线低 */
    }
    else
    {
        KEYLED_PIN_LOW();
    }
    gpio.GPIO_Pin   = KEYLED_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(KEYLED_PORT, &gpio);
    KEYLED_D_LOW();

    s_need_preamble = 0U;
    keyled_line_reset();             /* 第一帧之前先给灯珠一段复位电平(>=280us) */

    for (i = 0U; i < KEYLED_COUNT; i++)
    {
        s_fx[i].fx        = (uint8_t)KEYLED_FX_OFF;
        s_fx[i].period_ms = KEYLED_DEFAULT_PERIOD_MS;
        s_fx[i].base_ms   = 0U;
    }
    for (i = 0U; i < (uint8_t)sizeof(s_rgb); i++)
    {
        s_rgb[i]   = 0U;
        s_out[i]   = 0U;
        s_shown[i] = 0xFFU;      /* 初值不同, 保证首次 process 会刷新一次 */
    }

    s_inited = 1U;

    bsp_mutex_init(&s_lock);     /* 多任务保护(必须在 bsp_rtos_init() 之后) */

    keyled_off();                /* 上电先确保全灭 */

    /* 再连发几帧全零: 上电瞬间引脚是浮空的, 灯珠可能已经把噪声锁存成"上电默认色"
       (有些批次就是全白); 只发一帧的话, 万一这一帧被破坏, 灯就停在白色不动了。
       多补几帧(内容仍全零)把它压回"全灭"。帧数见 KEYLED_INIT_FRAMES。 */
    for (i = 1U; i < (uint8_t)KEYLED_INIT_FRAMES; i++)
    {
        keyled_show();
    }
}

/*---------------------------------------------------------------------------*/
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

    s_rgb[(uint32_t)idx * 3U + 0U] = r;
    s_rgb[(uint32_t)idx * 3U + 1U] = g;
    s_rgb[(uint32_t)idx * 3U + 2U] = b;

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
        s_rgb[(uint32_t)i * 3U + 0U] = 0U;
        s_rgb[(uint32_t)i * 3U + 1U] = 0U;
        s_rgb[(uint32_t)i * 3U + 2U] = 0U;
    }
    keyled_show_locked();

    bsp_mutex_unlock(&s_lock);
}

/*---------------------------------------------------------------------------*/
void keyled_set_brightness(uint8_t brightness)
{
    s_brightness = brightness;
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  重新设置位时序(单位 ns), 立即生效并重发一帧
  * @param  t0h_ns 0 码高电平(80~600ns),   传 0 = 用回 KEYLED_T0H_NS
  * @param  t0l_ns 0 码低电平(200~2000ns), 传 0 = 用回 KEYLED_T0L_NS
  * @param  t1h_ns 1 码高电平(300~1600ns), 传 0 = 用回 KEYLED_T1H_NS
  * @param  t1l_ns 1 码低电平(150~1000ns), 传 0 = 用回 KEYLED_T1L_NS
  * @note   四个相位各自独立(不设位周期), 自标定出来的"固定开销"会一起搬过去,
  *         不需要重新标定。串口 "KEYLED T0H/T0L/T1H/T1L <ns>" 走的就是这里。
  */
void keyled_set_timing(uint16_t t0h_ns, uint16_t t0l_ns, uint16_t t1h_ns, uint16_t t1l_ns)
{
    uint32_t mhz = SystemCoreClock / 1000000U;
    int32_t  k0h;
    int32_t  k0l;
    int32_t  k1h;
    int32_t  k1l;
    int32_t  c;

    if (mhz == 0U)
    {
        mhz = 72U;
    }

    if (t0h_ns == 0U)
    {
        t0h_ns = (uint16_t)KEYLED_T0H_NS;
    }
    if (t0l_ns == 0U)
    {
        t0l_ns = (uint16_t)KEYLED_T0L_NS;
    }
    if (t1h_ns == 0U)
    {
        t1h_ns = (uint16_t)KEYLED_T1H_NS;
    }
    if (t1l_ns == 0U)
    {
        t1l_ns = (uint16_t)KEYLED_T1L_NS;
    }

    /* 只允许规格书量级的取值, 免得参数打错把灯珠写成一片白 */
    if (t0h_ns < KEYLED_T0H_MIN_NS)
    {
        t0h_ns = (uint16_t)KEYLED_T0H_MIN_NS;
    }
    if (t0h_ns > KEYLED_T0H_MAX_NS)
    {
        t0h_ns = (uint16_t)KEYLED_T0H_MAX_NS;
    }
    if (t0l_ns < KEYLED_T0L_MIN_NS)
    {
        t0l_ns = (uint16_t)KEYLED_T0L_MIN_NS;
    }
    if (t0l_ns > KEYLED_T0L_MAX_NS)
    {
        t0l_ns = (uint16_t)KEYLED_T0L_MAX_NS;
    }
    if (t1h_ns < KEYLED_T1H_MIN_NS)
    {
        t1h_ns = (uint16_t)KEYLED_T1H_MIN_NS;
    }
    if (t1h_ns > KEYLED_T1H_MAX_NS)
    {
        t1h_ns = (uint16_t)KEYLED_T1H_MAX_NS;
    }
    if (t1l_ns < KEYLED_T1L_MIN_NS)
    {
        t1l_ns = (uint16_t)KEYLED_T1L_MIN_NS;
    }
    if (t1l_ns > KEYLED_T1L_MAX_NS)
    {
        t1l_ns = (uint16_t)KEYLED_T1L_MAX_NS;
    }
    if (t1h_ns <= (uint16_t)(t0h_ns + KEYLED_T1H_GAP_NS))   /* 两个码必须能区分 */
    {
        t1h_ns = (uint16_t)(t0h_ns + KEYLED_T1H_GAP_NS);
    }

    bsp_mutex_lock(&s_lock);

    k0h = (int32_t)s_nom_t0h - (int32_t)s_cyc_t0h;     /* 已经扣掉的开销 */
    k0l = (int32_t)s_nom_t0l - (int32_t)s_cyc_t0l;
    k1h = (int32_t)s_nom_t1h - (int32_t)s_cyc_t1h;
    k1l = (int32_t)s_nom_t1l - (int32_t)s_cyc_t1l;

    s_nom_t0h = ((uint32_t)t0h_ns * mhz) / 1000U;
    s_nom_t0l = ((uint32_t)t0l_ns * mhz) / 1000U;
    s_nom_t1h = ((uint32_t)t1h_ns * mhz) / 1000U;
    s_nom_t1l = ((uint32_t)t1l_ns * mhz) / 1000U;

    /* 目标值按当前测得的固定开销整体平移; 必须用有符号中间量再夹取 ——
       若先转 uint32_t 再夹, 负值会变成巨大的正数, 等待循环会卡死几十秒。 */
    c = (int32_t)s_nom_t0h - k0h;
    if (c < 4)
    {
        c = 4;
    }
    s_cyc_t0h = (uint32_t)c;

    c = (int32_t)s_nom_t0l - k0l;
    if (c < 8)
    {
        c = 8;
    }
    s_cyc_t0l = (uint32_t)c;

    c = (int32_t)s_nom_t1h - k1h;
    if (c < 8)
    {
        c = 8;
    }
    s_cyc_t1h = (uint32_t)c;

    c = (int32_t)s_nom_t1l - k1l;
    if (c < 8)
    {
        c = 8;
    }
    s_cyc_t1l = (uint32_t)c;

#if (KEYLED_USE_TIM_IRQ == 0U)
    s_last_low_cyc = 0U;         /* 只有老的 DWT 方式(位测量)读它 */
#endif

    /* 现场调时序时, 上一帧很可能是"错帧"(这正是要调它的原因): 先复位再重发,
       免得灯珠停在半帧状态 / 一直保持上一次的错误锁存。 */
    s_need_preamble = 1U;

    keyled_show_locked();        /* 立即重发一帧, 现场马上能看到颜色变化 */

    bsp_mutex_unlock(&s_lock);
}

/*---------------------------------------------------------------------------*/
/** 取当前时序信息(ns / 实际用的周期数 / 扣掉的开销); 调试用, 见 "KEYLED DIAG" */
void keyled_get_timing_info(keyled_timing_t *info)
{
    uint32_t mhz = SystemCoreClock / 1000000U;
    int32_t  k0h;
    int32_t  k0l;
    int32_t  k1h;
    int32_t  k1l;

    if (info == NULL)
    {
        return;
    }
    if (mhz == 0U)
    {
        mhz = 72U;
    }

    k0h = (int32_t)s_nom_t0h - (int32_t)s_cyc_t0h;
    k0l = (int32_t)s_nom_t0l - (int32_t)s_cyc_t0l;
    k1h = (int32_t)s_nom_t1h - (int32_t)s_cyc_t1h;
    k1l = (int32_t)s_nom_t1l - (int32_t)s_cyc_t1l;

    info->t0h_ns   = (uint16_t)((s_nom_t0h * 1000U) / mhz);
    info->t0l_ns   = (uint16_t)((s_nom_t0l * 1000U) / mhz);
    info->t1h_ns   = (uint16_t)((s_nom_t1h * 1000U) / mhz);
    info->t1l_ns   = (uint16_t)((s_nom_t1l * 1000U) / mhz);
    info->t0h_cyc  = s_cyc_t0h;
    info->t0l_cyc  = s_cyc_t0l;
    info->t1h_cyc  = s_cyc_t1h;
    info->t1l_cyc  = s_cyc_t1l;
    info->comp_t0h = (k0h > 0) ? (uint32_t)k0h : 0U;
    info->comp_t0l = (k0l > 0) ? (uint32_t)k0l : 0U;
    info->comp_t1h = (k1h > 0) ? (uint32_t)k1h : 0U;
    info->comp_t1l = (k1l > 0) ? (uint32_t)k1l : 0U;

#if (KEYLED_USE_TIM_IRQ != 0U)
    /* 实测 - 目标(拍): 最近一帧里"这种码实际走掉了多少拍"减掉目标拍数。
       没有样本(这一帧没出现这种码)或算出负数(刚改过时序)就报 0。
       把打印出来的数填进 KEYLED_TIM_WAIT_COMP_CYC, 四段长度就正好是配置值。 */
    info->meas_t0h = ((s_meas_t0h_cyc != KEYLED_MEAS_NONE) && (s_meas_t0h_cyc > s_nom_t0h))
                     ? (s_meas_t0h_cyc - s_nom_t0h) : 0U;
    info->meas_t0l = ((s_meas_t0l_cyc != KEYLED_MEAS_NONE) && (s_meas_t0l_cyc > s_nom_t0l))
                     ? (s_meas_t0l_cyc - s_nom_t0l) : 0U;
    info->meas_t1h = ((s_meas_t1h_cyc != KEYLED_MEAS_NONE) && (s_meas_t1h_cyc > s_nom_t1h))
                     ? (s_meas_t1h_cyc - s_nom_t1h) : 0U;
    info->meas_t1l = ((s_meas_t1l_cyc != KEYLED_MEAS_NONE) && (s_meas_t1l_cyc > s_nom_t1l))
                     ? (s_meas_t1l_cyc - s_nom_t1l) : 0U;
#else
    info->meas_t0h = 0U;    /* 老 DWT 方式: 偏差已经在自标定里从 s_cyc_* 扣掉了 */
    info->meas_t0l = 0U;
    info->meas_t1h = 0U;
    info->meas_t1l = 0U;
#endif
}

/*---------------------------------------------------------------------------*/
/** 上电自标定是否生效(1 = 当前时序是按实测校准过的) */
uint8_t keyled_calib_ok(void)
{
    return s_calib_ok;
}

/*---------------------------------------------------------------------------*/
/** 数据线极性: 1 = 反相(线上有反相器/三极管时用), 立即生效并重发一帧 */
void keyled_set_invert(uint8_t invert)
{
    bsp_mutex_lock(&s_lock);

    s_invert = (invert != 0U) ? 1U : 0U;

    KEYLED_D_LOW();                 /* 先摆到新极性下的空闲(复位)电平 */
    bsp_delay_us(KEYLED_RESET_US);
    s_need_preamble = 0U;           /* 上面已经复位过了 */

    keyled_show_locked();

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
  * @note   引脚是推挽输出, 正常情况下读回和驱动一致(低->0, 高->1);
  *         任一方向读回相反 = 这条线被外部强驱动 / 接错线 / 和别的信号短接,
  *         (单纯的"反相器/三极管在灯珠侧"读回来仍然是一致的, 那种要试 KEYLED INVERT 1)。
  *         本函数会往灯珠上推一段乱电平, 结束时自动"复位 + 重发当前一帧"。
  */
void keyled_probe_line(uint8_t *low_read, uint8_t *high_read)
{
    uint8_t lo;
    uint8_t hi;

    bsp_mutex_lock(&s_lock);

    KEYLED_PIN_LOW();
    bsp_delay_us(10U);
    lo = (GPIO_ReadInputDataBit(KEYLED_PORT, KEYLED_PIN) != 0U) ? 1U : 0U;

    KEYLED_PIN_HIGH();
    bsp_delay_us(10U);
    hi = (GPIO_ReadInputDataBit(KEYLED_PORT, KEYLED_PIN) != 0U) ? 1U : 0U;

    if (low_read != NULL)
    {
        *low_read = lo;
    }
    if (high_read != NULL)
    {
        *high_read = hi;
    }

    /* 收起乱电平: 复位 + 重发当前一帧, 免得灯珠停在半帧上 */
    s_need_preamble = 1U;
    keyled_show_locked();

    bsp_mutex_unlock(&s_lock);
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
        *r = s_rgb[(uint32_t)idx * 3U + 0U];
    }
    if (g != NULL)
    {
        *g = s_rgb[(uint32_t)idx * 3U + 1U];
    }
    if (b != NULL)
    {
        *b = s_rgb[(uint32_t)idx * 3U + 2U];
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
#if (KEYLED_USE_TIM_IRQ != 0U)
    /* 上面那次调用只是"请求": 这里等中断真把这一帧发出去(软件触发更新事件后
       中断大概几个微秒就进, 一帧约 120us)。命令回显/上线自检要的是"确实发完了"。 */
    (void)keyled_wait_frames(1U);
#endif
    bsp_mutex_unlock(&s_lock);
}

/*---------------------------------------------------------------------------*/
uint32_t keyled_frames(void)
{
#if (KEYLED_USE_TIM_IRQ != 0U)
    return s_frames;
#else
    return 0U;      /* 老 DWT 方式是"调用即发完", 没有中断计数可以报 */
#endif
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
        keyled_tx();
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
        *r = s_out[(uint32_t)idx * 3U + 0U];
    }
    if (g != NULL)
    {
        *g = s_out[(uint32_t)idx * 3U + 1U];
    }
    if (b != NULL)
    {
        *b = s_out[(uint32_t)idx * 3U + 2U];
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
        s_rgb[(uint32_t)i * 3U + 0U] = rgb[(uint32_t)i * 3U + 0U];
        s_rgb[(uint32_t)i * 3U + 1U] = rgb[(uint32_t)i * 3U + 1U];
        s_rgb[(uint32_t)i * 3U + 2U] = rgb[(uint32_t)i * 3U + 2U];
    }
    keyled_show_locked();

    bsp_mutex_unlock(&s_lock);
}
