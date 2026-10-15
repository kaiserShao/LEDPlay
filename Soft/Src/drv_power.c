/**
  ******************************************************************************
  * @file    drv_power.c
  * @brief   一键开关机实现
  ******************************************************************************
  */
#include "drv_power.h"
#include "bsp_exti.h"
#include "bsp_rtos.h"
#include "bsp_time.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

static uint8_t  s_inited      = 0U;
static uint8_t  s_key_level   = 0U;   /*!< 原始电平(1=按下) */
static uint8_t  s_stable      = 0U;   /*!< 消抖后状态 */
static uint32_t s_edge_ms     = 0U;
static uint32_t s_press_ms    = 0U;
static uint8_t  s_long_done   = 0U;
static uint8_t  s_shutdown    = 0U;
static uint32_t s_last_poll_ms = 0U;

static void (*s_shutdown_cb)(void) = NULL;
static void (*s_short_cb)(void)    = NULL;

static volatile uint8_t  s_isr_flag     = 0U;
static volatile uint32_t s_isr_stamp_ms = 0U;

/*---------------------------------------------------------------------------*/
static void power_hold_set(uint8_t on)
{
    uint8_t level = (on != 0U) ? POWER_HOLD_ON_LEVEL : POWER_HOLD_OFF_LEVEL;

    if (level != 0U)
    {
        GPIO_SetBits(POWER_HOLD_PORT, POWER_HOLD_PIN);
    }
    else
    {
        GPIO_ResetBits(POWER_HOLD_PORT, POWER_HOLD_PIN);
    }
}

/*---------------------------------------------------------------------------*/
static uint8_t power_key_pressed(void)
{
    return (GPIO_ReadInputDataBit(POWER_KEY_PORT, POWER_KEY_PIN) == Bit_RESET) ? 1U : 0U;
}

/*---------------------------------------------------------------------------*/
/** EXTI 回调: 只记录时间戳 + 置标志 */
static void power_key_exti_cb(uint16_t exti_line)
{
    (void)exti_line;
    s_isr_stamp_ms = bsp_time_ms();
    s_isr_flag     = 1U;
}

/*===========================================================================*/
void power_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA
                           | RCC_APB2Periph_AFIO, ENABLE);

    /* 时钟一开就立刻稳住电源(自锁), 否则松手就断电 */
    power_hold_set(1U);

    gpio.GPIO_Pin   = POWER_HOLD_PIN;         /* PC3 */
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(POWER_HOLD_PORT, &gpio);
    power_hold_set(1U);

    gpio.GPIO_Pin  = POWER_KEY_PIN;           /* PA7, 内部上拉 */
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(POWER_KEY_PORT, &gpio);

    s_key_level    = power_key_pressed();
    s_stable       = s_key_level;
    s_edge_ms      = bsp_time_ms();
    s_press_ms     = s_edge_ms;
    s_long_done    = (s_stable != 0U) ? 1U : 0U;   /* 上电时按键已按住则忽略本次 */
    s_shutdown     = 0U;
    s_last_poll_ms = s_edge_ms;
    s_isr_flag     = 0U;
    s_isr_stamp_ms = 0U;

    bsp_exti_config(POWER_KEY_EXTI_LINE, GPIO_PortSourceGPIOA, POWER_KEY_PIN_SOURCE,
                    (uint32_t)EXTI_Trigger_Falling, BSP_IRQ_PRIO_EXTI, power_key_exti_cb);

    s_inited = 1U;
}

/*===========================================================================
 *                            主循环状态机
 *=========================================================================*/
void power_process(void)
{
    uint32_t now;
    uint8_t  lvl;
    uint32_t dur;

    if (s_inited == 0U)
    {
        return;
    }
    if (s_shutdown != 0U)
    {
        return;
    }
    if (bsp_elapsed_ms(s_last_poll_ms) < POWER_KEY_POLL_MS)
    {
        return;
    }
    s_last_poll_ms = bsp_time_ms();
    now = s_last_poll_ms;

    lvl = power_key_pressed();

    if (lvl != s_key_level)
    {
        s_key_level = lvl;
        s_edge_ms   = now;
    }

    if (s_isr_flag != 0U)
    {
        s_edge_ms   = s_isr_stamp_ms;    /* 用中断记录的时间更准 */
        s_isr_flag  = 0U;
    }

    /* 消抖(与按键驱动一致的策略) */
    if ((s_key_level != s_stable) && (bsp_elapsed_ms(s_edge_ms) >= KEY_DEBOUNCE_MS))
    {
        s_stable = s_key_level;
        if (s_stable != 0U)
        {
            s_press_ms  = s_edge_ms;
            s_long_done = 0U;
        }
        else
        {
            dur = bsp_elapsed_ms(s_press_ms);
            if (dur < POWER_OFF_LONG_MS)
            {
                if (s_short_cb != NULL)
                {
                    s_short_cb();                /* 短按: 交给业务层决定 */
                }
            }
        }
    }

    /* 按住期间的关机判定 */
    if (s_stable != 0U)
    {
        dur = bsp_elapsed_ms(s_press_ms);

        if ((s_long_done == 0U) && (dur >= POWER_OFF_LONG_MS))
        {
            s_long_done = 1U;
            power_request_shutdown();            /* 正常关机流程(带提示音) */
        }
        else if (dur >= POWER_FORCE_OFF_MS)
        {
            power_shutdown_now();                /* 强力兜底: 立即断电 */
        }
        else
        {
            /* 继续等待 */
        }
    }
}

/*===========================================================================*/
uint8_t power_key_is_down(void)
{
    return s_stable;
}

/*---------------------------------------------------------------------------*/
uint32_t power_key_press_ms(void)
{
    if (s_stable == 0U)
    {
        return 0U;
    }
    return bsp_elapsed_ms(s_press_ms);
}

/*---------------------------------------------------------------------------*/
void power_shutdown_now(void)
{
    s_shutdown = 1U;

    power_hold_set(0U);          /* 拉掉自锁 -> 硬件断电 */

    /* 等电容放电 / 电源芯片关断(不需要精确, 让出 CPU 给别人) */
    bsp_sleep_ms(200U);

#if (POWER_HARD_CUT_LOOP != 0U)
    for (;;)
    {
        /* 正常情况下这里已经彻底断电了。
           在 USB/调试器供电时程序会停在这里 —— 这是预期的"关机"状态。
           用 sleep 而不是 __WFI 死循环, 这样其它任务(灯带/串口)仍然能正常运行。 */
        bsp_sleep_ms(1000U);
    }
#else
    /* 不进入死循环时: 保持断电电平并返回(业务层可自行决定后续动作) */
    power_hold_set(0U);
#endif
}

/*---------------------------------------------------------------------------*/
void power_request_shutdown(void)
{
    if (s_shutdown != 0U)
    {
        return;
    }
    s_shutdown = 1U;

    if (s_shutdown_cb != NULL)
    {
        s_shutdown_cb();         /* 例如: 播"关机"提示音、存参数、关灯 */
    }

    /* 留时间给提示音播完(不需要可以改小 POWER_SHUTDOWN_NOTICE_MS):
       这里只阻塞当前任务, 其它任务照常运行 */
    bsp_sleep_ms(POWER_SHUTDOWN_NOTICE_MS);

    power_hold_set(0U);
    bsp_sleep_ms(200U);

#if (POWER_HARD_CUT_LOOP != 0U)
    for (;;)
    {
        bsp_sleep_ms(1000U);
    }
#endif
}

/*---------------------------------------------------------------------------*/
void power_set_shutdown_handler(void (*handler)(void))
{
    s_shutdown_cb = handler;
}

/*---------------------------------------------------------------------------*/
void power_set_short_press_handler(void (*handler)(void))
{
    s_short_cb = handler;
}

/*---------------------------------------------------------------------------*/
uint8_t power_is_shutting_down(void)
{
    return s_shutdown;
}
