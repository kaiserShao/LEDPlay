/**
  ******************************************************************************
  * @file    drv_key.c
  * @brief   按键驱动实现 (中断只记时间戳, 判定在主循环完成)
  ******************************************************************************
  */
#include "drv_key.h"
#include "bsp_exti.h"
#include "bsp_rtos.h"
#include "bsp_time.h"
#include "bsp_util.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

/*---------------------------------------------------------------------------*/
typedef struct
{
    uint8_t  level;       /*!< 最近一次采样到的原始电平(1=按下) */
    uint8_t  stable;      /*!< 消抖后的稳定状态 */
    uint32_t edge_ms;     /*!< 原始电平最近一次跳变时间(优先取中断时间戳) */
    uint32_t press_ms;    /*!< 稳定按下开始时间 */
    uint8_t  long_sent;   /*!< 长按事件是否已上报 */
    uint32_t hold_ms;     /*!< 上次长按保持上报时间 */
} key_state_t;

static key_state_t      s_key[KEY_COUNT];
static uint8_t          s_inited = 0U;
static uint32_t         s_last_scan_ms = 0U;

/* 中断与主循环之间传递的最少信息 */
static volatile uint8_t  s_isr_flag[KEY_COUNT];    /*!< 1=中断记录过一个边沿 */
static volatile uint32_t s_isr_stamp_ms[KEY_COUNT];/*!< 边沿时间戳(ms) */

/* 事件队列 */
static key_msg_t        s_queue[KEY_EVENT_QUEUE_LEN];
static volatile uint8_t s_q_head = 0U;
static volatile uint8_t s_q_tail = 0U;
static key_callback_t   s_cb = NULL;

/* 事件标志: 中断里置位 -> drv_key_wait_event() 立刻返回(无需轮询) */
static bsp_signal_t     s_evt_sig;

/*---------------------------------------------------------------------------*/
/** 取按键对应的 EXTI 线 (PA0->EXTI0 ... PA6->EXTI6) */
#define KEY_EXTI_LINE(id)      ((uint16_t)(1UL << (id)))

/*---------------------------------------------------------------------------*/
static uint8_t key_pin_pressed(uint8_t id)
{
    /* 内部上拉, 按下 = 低电平 */
    return (GPIO_ReadInputDataBit(KEY_PORT, (uint16_t)(KEY_PINS & (1UL << (id)))) == Bit_RESET) ? 1U : 0U;
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  EXTI 回调 —— 必须"最轻量": 只记录时间戳 + 置标志
  * @note   由 bsp_exti 的中断服务程序调用, 只能做几百个周期内的事情
  */
static void key_exti_callback(uint16_t exti_line)
{
    uint32_t bit = (uint32_t)exti_line;   /* EXTI_Linex = (1 << x) */
    uint8_t  id  = 0U;

    while (((bit & 1UL) == 0UL) && (id < 16U))
    {
        bit >>= 1;
        id++;
    }

    if (id < KEY_COUNT)
    {
        s_isr_stamp_ms[id] = bsp_time_ms();   /* 一次寄存器读 + 一次写内存 */
        s_isr_flag[id]     = 1U;
        bsp_signal_set(&s_evt_sig);           /* 唤醒等待按键事件的任务 */
    }
}

/*---------------------------------------------------------------------------*/
static void key_push_event(const key_msg_t *msg)
{
    uint8_t next;

    BSP_CRITICAL_BEGIN();
    next = (uint8_t)((s_q_head + 1U) % KEY_EVENT_QUEUE_LEN);
    if (next != s_q_tail)
    {
        s_queue[s_q_head] = *msg;
        s_q_head = next;
    }
    BSP_CRITICAL_END();
}

/*---------------------------------------------------------------------------*/
static void key_report(uint8_t id, key_event_t ev, uint32_t duration_ms)
{
    key_msg_t msg;

    msg.id          = id;
    msg.event       = (uint8_t)ev;
    msg.duration_ms = (uint16_t)((duration_ms > 0xFFFFU) ? 0xFFFFU : duration_ms);

    if (s_cb != NULL)
    {
        s_cb(&msg);
    }
    key_push_event(&msg);
    bsp_signal_set(&s_evt_sig);      /* 有事件了, 唤醒等待者 */
}

/*---------------------------------------------------------------------------*/
void drv_key_init(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t          i;

    RCC_APB2PeriphClockCmd(KEY_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

    /* 无外部上拉电阻 -> 打开 MCU 内部上拉 */
    gpio.GPIO_Pin   = (uint16_t)KEY_PINS;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(KEY_PORT, &gpio);

    for (i = 0U; i < KEY_COUNT; i++)
    {
        s_key[i].level     = key_pin_pressed(i);
        s_key[i].stable    = s_key[i].level;
        s_key[i].edge_ms   = bsp_time_ms();
        s_key[i].press_ms  = s_key[i].edge_ms;
        s_key[i].long_sent = 1U;
        s_key[i].hold_ms   = 0U;
        s_isr_flag[i]      = 0U;
        s_isr_stamp_ms[i]  = 0U;

        /* 下降沿中断: PA0~PA6 -> EXTI0~EXTI6 */
        bsp_exti_config(KEY_EXTI_LINE(i), KEY_PORT_SOURCE, (uint8_t)(GPIO_PinSource0 + i),
                        (uint32_t)EXTI_Trigger_Falling, BSP_IRQ_PRIO_EXTI, key_exti_callback);
    }

    s_q_head = 0U;
    s_q_tail = 0U;
    bsp_signal_init(&s_evt_sig, BSP_SIGNAL_KEY_EVENT);
    s_inited = 1U;
}

/*---------------------------------------------------------------------------*/
static void key_scan_one(uint8_t id)
{
    key_state_t *k = &s_key[id];
    uint32_t     now;
    uint8_t      lvl;

    now = bsp_time_ms();
    lvl = key_pin_pressed(id);

    if (lvl != k->level)
    {
        k->level   = lvl;
        k->edge_ms = now;      /* 轮询也记录跳变时间, 防止漏掉中断 */
    }

    if (s_isr_flag[id] != 0U)
    {
        uint32_t ts = s_isr_stamp_ms[id];

        s_isr_flag[id] = 0U;

        /* 只在中断记录的时间"更新"时才采用(避免用旧时间戳推迟消抖) */
        if ((int32_t)(ts - k->edge_ms) > 0)
        {
            k->edge_ms = ts;
        }
    }

    /* 消抖: 电平稳定 KEY_DEBOUNCE_MS 之后才认账 */
    if ((k->level != k->stable) && (bsp_elapsed_ms(k->edge_ms) >= KEY_DEBOUNCE_MS))
    {
        k->stable = k->level;
        if (k->stable != 0U)
        {
            k->press_ms  = k->edge_ms;
            k->long_sent = 0U;
            k->hold_ms   = k->edge_ms;
            key_report(id, KEY_EV_DOWN, 0U);
        }
        else
        {
            uint32_t dur = bsp_elapsed_ms(k->press_ms);

            key_report(id, KEY_EV_UP, dur);
            if (dur < KEY_LONG_MS)
            {
                key_report(id, KEY_EV_SHORT, dur);   /* 短按: 松开时上报 */
            }
        }
    }

    /* 按住期间的长按判定 */
    if (k->stable != 0U)
    {
        uint32_t dur = bsp_elapsed_ms(k->press_ms);

        if ((k->long_sent == 0U) && (dur >= KEY_LONG_MS))
        {
            k->long_sent = 1U;
            k->hold_ms   = now;
            key_report(id, KEY_EV_LONG, dur);        /* 长按: 达到门限立即上报 */
        }
        else if ((k->long_sent != 0U) && (KEY_HOLD_REPEAT_MS > 0U)
                 && (bsp_elapsed_ms(k->hold_ms) >= KEY_HOLD_REPEAT_MS))
        {
            k->hold_ms = now;
            key_report(id, KEY_EV_HOLD, dur);        /* 长按保持: 周期上报 */
        }
        else
        {
            /* 无需处理 */
        }
    }
}

/*---------------------------------------------------------------------------*/
void drv_key_process(void)
{
    uint8_t i;

    if (s_inited == 0U)
    {
        return;
    }

    /* 主循环限速: 最快 KEY_SCAN_PERIOD_MS 扫描一次 */
    if (bsp_elapsed_ms(s_last_scan_ms) < KEY_SCAN_PERIOD_MS)
    {
        return;
    }
    s_last_scan_ms = bsp_time_ms();

    for (i = 0U; i < KEY_COUNT; i++)
    {
        key_scan_one(i);
    }
}

/*---------------------------------------------------------------------------*/
uint8_t drv_key_get_event(key_msg_t *msg)
{
    uint8_t ret = 0U;

    if (msg == NULL)
    {
        return 0U;
    }

    BSP_CRITICAL_BEGIN();
    if (s_q_tail != s_q_head)
    {
        *msg     = s_queue[s_q_tail];
        s_q_tail = (uint8_t)((s_q_tail + 1U) % KEY_EVENT_QUEUE_LEN);
        ret      = 1U;
    }
    BSP_CRITICAL_END();

    return ret;
}

/*---------------------------------------------------------------------------*/
uint8_t drv_key_wait_event(key_msg_t *msg, uint32_t timeout_ms)
{
    uint32_t t0      = bsp_time_ms();
    uint32_t left    = timeout_ms;
    uint8_t  forever = (timeout_ms == 0xFFFFFFFFU) ? 1U : 0U;

    if (msg == NULL)
    {
        return 0U;
    }

    for (;;)
    {
        if (drv_key_get_event(msg) != 0U)
        {
            return 1U;
        }
        if ((forever == 0U) && (left == 0U))
        {
            return 0U;                  /* 非阻塞模式: 队列空就直接返回 */
        }

        /* RTOS: 由按键中断唤醒(不占 CPU); 无 RTOS: 轮询等待 */
        if (bsp_signal_wait(&s_evt_sig, (forever != 0U) ? 0xFFFFFFFFU : left) == 0U)
        {
            return 0U;                  /* 超时 */
        }

        if (forever == 0U)
        {
            uint32_t used = bsp_elapsed_ms(t0);

            left = (used >= timeout_ms) ? 1U : (uint32_t)(timeout_ms - used);
        }
    }
}

/*---------------------------------------------------------------------------*/
void drv_key_flush_queue(void)
{
    BSP_CRITICAL_BEGIN();
    s_q_tail = s_q_head;
    BSP_CRITICAL_END();
}

/*---------------------------------------------------------------------------*/
void drv_key_set_callback(key_callback_t cb)
{
    s_cb = cb;
}

/*---------------------------------------------------------------------------*/
uint8_t drv_key_get_count(void)
{
    return KEY_COUNT;
}

/*---------------------------------------------------------------------------*/
uint8_t drv_key_is_down(uint8_t id)
{
    if (id >= KEY_COUNT)
    {
        return 0U;
    }
    return s_key[id].stable;
}

/*---------------------------------------------------------------------------*/
uint32_t drv_key_press_ms(uint8_t id)
{
    if ((id >= KEY_COUNT) || (s_key[id].stable == 0U))
    {
        return 0U;
    }
    return bsp_elapsed_ms(s_key[id].press_ms);
}

/*---------------------------------------------------------------------------*/
uint8_t drv_key_raw_pressed(uint8_t id)
{
    if (id >= KEY_COUNT)
    {
        return 0U;
    }
    return key_pin_pressed(id);
}

