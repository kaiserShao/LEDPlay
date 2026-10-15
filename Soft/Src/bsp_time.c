/**
  ******************************************************************************
  * @file    bsp_time.c
  * @brief   时间基准: DWT 微秒延时 + TIM2 1MHz 时间戳
  ******************************************************************************
  */
#include "bsp_time.h"
#include "bsp_config.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"

#define BSP_TIME_TIM       TIM2
#define BSP_TIME_TIM_CLK   RCC_APB1Periph_TIM2
#define BSP_TIME_TIM_IRQn  TIM2_IRQn

static uint8_t  s_dwt_ok = 0U;
static uint32_t s_cycles_per_us = 72U;   /* 复位后默认按 72MHz 估算, init 中修正 */

static volatile uint32_t s_us_tick = 0U; /*!< 由 1ms 中断累加出来的微秒时间戳 */
static volatile uint32_t s_ms_tick = 0U; /*!< 毫秒时间戳 */

/*---------------------------------------------------------------------------*/
void bsp_time_init(void)
{
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef        nvic;
    RCC_ClocksTypeDef       clocks;
    uint32_t                t0;
    uint32_t                i;

    RCC_GetClocksFreq(&clocks);
    if (clocks.HCLK_Frequency != 0U)
    {
        s_cycles_per_us = clocks.HCLK_Frequency / 1000000U;
    }
    if (s_cycles_per_us == 0U)
    {
        s_cycles_per_us = 1U;
    }

    /*--------------------- DWT 周期计数器(用于精确延时) -------------------*/
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    t0 = DWT->CYCCNT;
    for (i = 0U; i < 64U; i++)      /* 等一会儿看计数器有没有动 */
    {
        __NOP();
    }
    s_dwt_ok = (DWT->CYCCNT != t0) ? 1U : 0U;
    if (s_dwt_ok == 0U)
    {
        DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
    }

    /*------- TIM2: 1MHz 计数 + 1ms 周期中断(软件扩展为 32 位时间戳) --------*/
    RCC_APB1PeriphClockCmd(BSP_TIME_TIM_CLK, ENABLE);
    tim.TIM_Prescaler         = (uint16_t)(s_cycles_per_us - 1U); /* 72MHz/72 = 1MHz */
    tim.TIM_CounterMode       = TIM_CounterMode_Up;
    tim.TIM_Period            = 999U;      /* 1000 个 1us 计数 = 1ms */
    tim.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(BSP_TIME_TIM, &tim);
    TIM_SetCounter(BSP_TIME_TIM, 0U);
    TIM_ClearITPendingBit(BSP_TIME_TIM, TIM_IT_Update);
    TIM_ITConfig(BSP_TIME_TIM, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel                   = BSP_TIME_TIM_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = BSP_IRQ_PRIO_TIME;
    nvic.NVIC_IRQChannelSubPriority        = 0U;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    s_us_tick = 0U;
    s_ms_tick = 0U;
    TIM_Cmd(BSP_TIME_TIM, ENABLE);
}

/*---------------------------------------------------------------------------*/
/** TIM2 更新中断: 每 1ms 累加一次软件时间戳(极短, 不影响其它中断) */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(BSP_TIME_TIM, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(BSP_TIME_TIM, TIM_IT_Update);
        s_us_tick += 1000U;
        s_ms_tick++;
    }
}

/*---------------------------------------------------------------------------*/
uint32_t bsp_cycles(void)
{
    if (s_dwt_ok != 0U)
    {
        return DWT->CYCCNT;
    }
    return 0U;
}

/*---------------------------------------------------------------------------*/
uint32_t bsp_time_us(void)
{
    uint32_t base;
    uint32_t cnt;
    uint32_t cnt2;

    /* 读"毫秒累加值 + 当前 1us 计数", 若中途发生了 ms 更新或计数回绕则重读 */
    do
    {
        base = s_us_tick;
        cnt  = BSP_TIME_TIM->CNT;
        cnt2 = BSP_TIME_TIM->CNT;
    } while ((base != s_us_tick) || (cnt2 < cnt));

    return (base + cnt);
}

/*---------------------------------------------------------------------------*/
uint32_t bsp_time_ms(void)
{
    return s_ms_tick;
}

/*---------------------------------------------------------------------------*/
uint32_t bsp_elapsed_us(uint32_t start_us)
{
    return (bsp_time_us() - start_us);
}

/*---------------------------------------------------------------------------*/
uint32_t bsp_elapsed_ms(uint32_t start_ms)
{
    return (bsp_time_ms() - start_ms);
}

/*---------------------------------------------------------------------------*/
void bsp_delay_cycles(uint32_t cycles)
{
    uint32_t start;

    if (cycles == 0U)
    {
        return;
    }

    if (s_dwt_ok != 0U)
    {
        start = DWT->CYCCNT;
        while ((DWT->CYCCNT - start) < cycles)
        {
            /* 等待 */
        }
    }
    else
    {
        /* 退化模式: 约 3 个周期一条空循环(未校准, 仅保证"等够大致时间") */
        volatile uint32_t n = cycles / 3U;

        while (n != 0U)
        {
            n--;
        }
    }
}

/*---------------------------------------------------------------------------*/
void bsp_delay_us(uint32_t us)
{
    uint64_t cyc = (uint64_t)us * (uint64_t)s_cycles_per_us;

    while (cyc > 0xFFFFFFFFULL)
    {
        bsp_delay_cycles(0xFFFFFFFFU);
        cyc -= 0xFFFFFFFFULL;
    }
    bsp_delay_cycles((uint32_t)cyc);
}

/*---------------------------------------------------------------------------*/
void bsp_delay_ms(uint32_t ms)
{
    while (ms != 0U)
    {
        bsp_delay_us(1000U);
        ms--;
    }
}

/*---------------------------------------------------------------------------*/
uint8_t bsp_time_dwt_ok(void)
{
    return s_dwt_ok;
}
