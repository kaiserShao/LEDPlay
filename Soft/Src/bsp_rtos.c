/**
  ******************************************************************************
  * @file    bsp_rtos.c
  * @brief   CMSIS-RTOS(Keil RTX) 适配层实现
  *
  *  实现要点
  *  ---------------------------------------------------------------------------
  *  * 全部接口都是"双模式"的: BSP_USE_RTOS=1 用 RTX 原语, =0 退化为
  *    忙等 / 临界区 / 标志轮询。驱动代码因此不需要 #if。
  *  * RTX 的互斥锁/消息队列控制块由我们提供的存储区(uint32_t cb[4] /
  *    storage[4+N])充当, 不依赖堆, 也不会用光 RTX 的对象池。
  *  * bsp_signal_wait() 在非 RTOS 模式下用时间戳轮询, 超时精度 1ms。
  ******************************************************************************
  */
#include "bsp_rtos.h"
#include "bsp_time.h"
#include "bsp_util.h"

/*===========================================================================
 *                              内核控制
 *=========================================================================*/
void bsp_rtos_init(void)
{
#if (BSP_USE_RTOS != 0U)
    /* 初始化 RTX 内核对象池(线程栈池 / 控制块池)。
       必须在使用 osThreadCreate / osMutexCreate / osMessageCreate 之前调用,
       否则内核内部结构还没准备好。 */
    (void)osKernelInitialize();
#endif
}

/*---------------------------------------------------------------------------*/
void bsp_rtos_start(void)
{
#if (BSP_USE_RTOS != 0U)
    if (osKernelStart() != osOK)
    {
        /* 启动失败(几乎不会发生): 停在这里, 方便调试时定位 */
        for (;;)
        {
            __WFI();
        }
    }
#endif
    /* 未使能 RTOS: 直接返回, 由 main 的轮询循环接管 */
}

/*---------------------------------------------------------------------------*/
uint8_t bsp_rtos_running(void)
{
#if (BSP_USE_RTOS != 0U)
    return (osKernelRunning() != 0) ? 1U : 0U;
#else
    return 0U;
#endif
}

/*===========================================================================
 *                                延时
 *=========================================================================*/
void bsp_sleep_ms(uint32_t ms)
{
#if (BSP_USE_RTOS != 0U)
    /* 中断里不能阻塞: 自动退化为忙等 */
    if ((__get_IPSR() == 0U) && (osKernelRunning() != 0))
    {
        if (ms == 0U)
        {
            (void)osThreadYield();       /* 主动让出一次 CPU */
        }
        else
        {
            (void)osDelay(ms);           /* 只阻塞当前任务 */
        }
        return;
    }
#endif
    bsp_delay_ms(ms);
}

/*===========================================================================
 *                              互斥锁
 *=========================================================================*/
void bsp_mutex_init(bsp_mutex_t *m)
{
    if (m == NULL)
    {
        return;
    }

    m->inited  = 0U;
    m->primask = 0U;
#if (BSP_USE_RTOS != 0U)
    m->id    = NULL;
    m->cb[0] = 0U;
    m->cb[1] = 0U;
    m->cb[2] = 0U;
    m->cb[3] = 0U;

    if (__get_IPSR() == 0U)
    {
        osMutexDef_t def;

        def.mutex = (void *)m->cb;
        m->id     = osMutexCreate(&def);
    }
#endif
    m->inited = 1U;
}

/*---------------------------------------------------------------------------*/
void bsp_mutex_lock(bsp_mutex_t *m)
{
    if ((m == NULL) || (m->inited == 0U))
    {
        return;
    }

#if (BSP_USE_RTOS != 0U)
    if (m->id != NULL)
    {
        (void)osMutexWait(m->id, osWaitForever);
        return;
    }
#endif
    /* 退化: 关中断(本工程的用法都是"成对出现", 不做嵌套计数) */
    m->primask = bsp_irq_save();
}

/*---------------------------------------------------------------------------*/
void bsp_mutex_unlock(bsp_mutex_t *m)
{
    if ((m == NULL) || (m->inited == 0U))
    {
        return;
    }

#if (BSP_USE_RTOS != 0U)
    if (m->id != NULL)
    {
        (void)osMutexRelease(m->id);
        return;
    }
#endif
    bsp_irq_restore(m->primask);
}

/*===========================================================================
 *                            事件标志(信号)
 *=========================================================================*/
void bsp_signal_init(bsp_signal_t *s, uint8_t index)
{
    if (s == NULL)
    {
        return;
    }

    s->pending = 0U;
    s->inited  = 0U;
#if (BSP_USE_RTOS != 0U)
    s->owner = NULL;
    s->flag  = (int32_t)(1UL << (uint32_t)(index & 0x0FU));
#else
    (void)index;
#endif
    s->inited = 1U;
}

/*---------------------------------------------------------------------------*/
uint8_t bsp_signal_wait(bsp_signal_t *s, uint32_t timeout_ms)
{
    if ((s == NULL) || (s->inited == 0U))
    {
        return 0U;
    }

#if (BSP_USE_RTOS != 0U)
    if ((__get_IPSR() == 0U) && (osKernelRunning() != 0))
    {
        osEvent evt;

        s->owner = osThreadGetId();

        if (s->pending != 0U)        /* 置位时没有人在等 -> 直接消费掉 */
        {
            s->pending = 0U;
            return 1U;
        }

        evt = osSignalWait(s->flag, timeout_ms);
        if (evt.status == osEventSignal)
        {
            s->pending = 0U;
            return 1U;
        }
        return 0U;
    }
#endif

    /* 非 RTOS(或中断里): 用时间戳轮询 */
    {
        uint32_t t0 = bsp_time_ms();

        for (;;)
        {
            if (s->pending != 0U)
            {
                s->pending = 0U;
                return 1U;
            }
            if ((timeout_ms == 0U) || (bsp_elapsed_ms(t0) >= timeout_ms))
            {
                return 0U;
            }
            bsp_delay_us(200U);      /* 轮询间隔, 避免无意义的满速忙等 */
        }
    }
}

/*===========================================================================
 *                              消息队列
 *=========================================================================*/
void bsp_queue_init(bsp_queue_t *q)
{
    if (q == NULL)
    {
        return;
    }

    q->head   = 0U;
    q->tail   = 0U;
    q->inited = 0U;
#if (BSP_USE_RTOS != 0U)
    q->id = NULL;
    {
        uint32_t i;

        for (i = 0U; i < BSP_QUEUE_STORAGE_WORDS; i++)
        {
            q->storage[i] = 0U;      /* RTX 要求队列头(cb_type)==0, 必须清零 */
        }
    }
    if (__get_IPSR() == 0U)
    {
        osMessageQDef_t def;

        def.queue_sz = BSP_QUEUE_DEPTH;
        def.pool     = (void *)q->storage;
        q->id        = osMessageCreate(&def, NULL);
    }
#endif
    q->inited = 1U;
}

/*---------------------------------------------------------------------------*/
uint8_t bsp_queue_put(bsp_queue_t *q, uint32_t value)
{
    if ((q == NULL) || (q->inited == 0U))
    {
        return 0U;
    }

#if (BSP_USE_RTOS != 0U)
    if (q->id != NULL)
    {
        return (osMessagePut(q->id, value, 0U) == osOK) ? 1U : 0U;
    }
#endif

    /* 退化: 简单环形缓冲(单生产者/单消费者) */
    {
        uint16_t next = (uint16_t)((q->tail + 1U) % BSP_QUEUE_DEPTH);

        if (next == q->head)
        {
            return 0U;              /* 满 */
        }
        q->storage[q->tail] = value;
        q->tail             = next;
        return 1U;
    }
}

/*---------------------------------------------------------------------------*/
uint8_t bsp_queue_get(bsp_queue_t *q, uint32_t *value, uint32_t timeout_ms)
{
    uint32_t t0;

    if ((q == NULL) || (q->inited == 0U) || (value == NULL))
    {
        return 0U;
    }

    t0 = bsp_time_ms();

    for (;;)
    {
#if (BSP_USE_RTOS != 0U)
        if ((__get_IPSR() == 0U) && (osKernelRunning() != 0) && (q->id != NULL))
        {
            osEvent evt = osMessageGet(q->id, timeout_ms);

            if (evt.status == osEventMessage)
            {
                *value = evt.value.v;
                return 1U;
            }
            return 0U;               /* 超时 */
        }
#endif
        if (q->head != q->tail)
        {
            *value  = q->storage[q->head];
            q->head = (uint16_t)((q->head + 1U) % BSP_QUEUE_DEPTH);
            return 1U;
        }
        if ((timeout_ms == 0U) || (bsp_elapsed_ms(t0) >= timeout_ms))
        {
            return 0U;
        }
        bsp_delay_us(200U);          /* 非 RTOS 模式下的轮询间隔 */
    }
}

/*---------------------------------------------------------------------------*/
uint8_t bsp_queue_is_empty(const bsp_queue_t *q)
{
    if ((q == NULL) || (q->inited == 0U))
    {
        return 1U;
    }

#if (BSP_USE_RTOS != 0U)
    if (q->id != NULL)
    {
        osEvent evt = osMessageGet((osMessageQId)q->id, 0U);   /* 只看一眼, 不等待 */

        if (evt.status == osEventMessage)
        {
            /* 取出来了: 放回去(RTX 消息队列不能回退, 这是"非阻塞探测"的近似做法) */
            (void)osMessagePut((osMessageQId)q->id, evt.value.v, 0U);
            return 0U;
        }
        return 1U;
    }
#endif
    return (q->head == q->tail) ? 1U : 0U;
}

/*---------------------------------------------------------------------------*/
void bsp_queue_flush(bsp_queue_t *q)
{
    uint32_t dummy;

    if ((q == NULL) || (q->inited == 0U))
    {
        return;
    }

    while (bsp_queue_get(q, &dummy, 0U) != 0U)
    {
        /* 逐个取空 */
    }
}

/*===========================================================================
 *                                线程
 *=========================================================================*/
uint8_t bsp_thread_create(bsp_thread_fn_t fn, void *arg, int32_t priority)
{
#if (BSP_USE_RTOS != 0U)
    osThreadDef_t def;
    osThreadId    id;
    osPriority    prio = (osPriority)priority;

    if (fn == NULL)
    {
        return 0U;
    }
    if (prio < osPriorityIdle)
    {
        prio = osPriorityIdle;
    }
    if (prio > osPriorityRealtime)
    {
        prio = osPriorityRealtime;
    }

    def.pthread   = (os_pthread)fn;
    def.tpriority = prio;
    def.instances = 1U;
    def.stacksize = 0U;              /* 0 = 使用 OS_STKSIZE(见 RTX_Conf_CM.c, 默认 1KB) */

    id = osThreadCreate(&def, arg);
    return (id != NULL) ? 1U : 0U;
#else
    (void)fn;
    (void)arg;
    (void)priority;
    return 0U;
#endif
}


/*---------------------------------------------------------------------------*/
void bsp_signal_set(bsp_signal_t *s)
{
    if ((s == NULL) || (s->inited == 0U))
    {
        return;
    }

    s->pending = 1U;

#if (BSP_USE_RTOS != 0U)
    /* 中断上下文也安全: RTX 内部会自动走 ISR 分支 */
    if ((s->owner != NULL) && (osKernelRunning() != 0))
    {
        (void)osSignalSet(s->owner, s->flag);
    }
#endif
}

/*---------------------------------------------------------------------------*/
void bsp_signal_clear(bsp_signal_t *s)
{
    if ((s == NULL) || (s->inited == 0U))
    {
        return;
    }

    s->pending = 0U;
#if (BSP_USE_RTOS != 0U)
    if ((s->owner != NULL) && (__get_IPSR() == 0U) && (osKernelRunning() != 0))
    {
        (void)osSignalClear(s->owner, s->flag);
    }
#endif
}
