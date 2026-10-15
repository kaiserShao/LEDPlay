/**
  ******************************************************************************
  * @file    bsp_exti.c
  * @brief   外部中断分发器 (EXTI0~EXTI15)
  ******************************************************************************
  */
#include "bsp_exti.h"
#include "bsp_config.h"
#include "misc.h"

/* EXTI_Line0 对应的位号: LINE_n -> bit n */
#define EXTI_LINE_BIT(line)   ((uint8_t)(((line) == EXTI_Line0)  ? 0U  : \
                                         ((line) == EXTI_Line1)  ? 1U  : \
                                         ((line) == EXTI_Line2)  ? 2U  : \
                                         ((line) == EXTI_Line3)  ? 3U  : \
                                         ((line) == EXTI_Line4)  ? 4U  : \
                                         ((line) == EXTI_Line5)  ? 5U  : \
                                         ((line) == EXTI_Line6)  ? 6U  : \
                                         ((line) == EXTI_Line7)  ? 7U  : \
                                         ((line) == EXTI_Line8)  ? 8U  : \
                                         ((line) == EXTI_Line9)  ? 9U  : \
                                         ((line) == EXTI_Line10) ? 10U : \
                                         ((line) == EXTI_Line11) ? 11U : \
                                         ((line) == EXTI_Line12) ? 12U : \
                                         ((line) == EXTI_Line13) ? 13U : \
                                         ((line) == EXTI_Line14) ? 14U : 15U))

static bsp_exti_cb_t        s_exti_cb[16] = {0};
static volatile uint32_t    s_exti_count[16] = {0};
static uint8_t              s_irq_inited = 0U;   /* bit0:EXTI0_4, bit1:EXTI9_5, bit2:EXTI15_10 */

/*---------------------------------------------------------------------------*/
void bsp_exti_attach(uint16_t exti_line, bsp_exti_cb_t cb)
{
    s_exti_cb[EXTI_LINE_BIT(exti_line)] = cb;
}

/*---------------------------------------------------------------------------*/
void bsp_exti_clear(uint16_t exti_line)
{
    EXTI_ClearITPendingBit(exti_line);
}

/*---------------------------------------------------------------------------*/
void bsp_exti_config(uint16_t exti_line, uint8_t port_source, uint8_t pin_source,
                     uint32_t trigger, uint8_t preempt_prio, bsp_exti_cb_t cb)
{
    EXTI_InitTypeDef   exti;
    NVIC_InitTypeDef   nvic;
    IRQn_Type          irqn;

    /* 选择 EXTI 源: PA0 -> EXTI0 ... */
    GPIO_EXTILineConfig(port_source, pin_source);

    exti.EXTI_Line    = exti_line;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = (EXTITrigger_TypeDef)trigger;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    bsp_exti_attach(exti_line, cb);

    /* 映射中断向量: 0~4 独立, 5~9 共用, 10~15 共用 */
    if (exti_line <= EXTI_Line4)
    {
        irqn = (IRQn_Type)((uint32_t)EXTI0_IRQn + EXTI_LINE_BIT(exti_line));
        s_irq_inited |= 0x01U;
    }
    else if (exti_line <= EXTI_Line9)
    {
        irqn = EXTI9_5_IRQn;
        s_irq_inited |= 0x02U;
    }
    else
    {
        irqn = EXTI15_10_IRQn;
        s_irq_inited |= 0x04U;
    }

    nvic.NVIC_IRQChannel                   = irqn;
    nvic.NVIC_IRQChannelPreemptionPriority = preempt_prio;
    nvic.NVIC_IRQChannelSubPriority        = 0U;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

/*---------------------------------------------------------------------------*/
uint32_t bsp_exti_count(uint16_t exti_line)
{
    return s_exti_count[EXTI_LINE_BIT(exti_line)];
}

/*===========================================================================
 *                            中断服务程序
 *  要求: 只清标志 + 调用回调, 回调内只允许"记录时间戳 + 置标志"。
 *=========================================================================*/
#define BSP_EXTI_HANDLE(line, bit)                          \
    do {                                                    \
        if (EXTI_GetITStatus(line) != RESET)                \
        {                                                   \
            EXTI_ClearITPendingBit(line);                   \
            s_exti_count[bit]++;                            \
            if (s_exti_cb[bit] != NULL)                     \
            {                                               \
                s_exti_cb[bit](line);                       \
            }                                               \
        }                                                   \
    } while (0)

void EXTI0_IRQHandler(void)
{
    BSP_EXTI_HANDLE(EXTI_Line0, 0);
}

void EXTI1_IRQHandler(void)
{
    BSP_EXTI_HANDLE(EXTI_Line1, 1);
}

void EXTI2_IRQHandler(void)
{
    BSP_EXTI_HANDLE(EXTI_Line2, 2);
}

void EXTI3_IRQHandler(void)
{
    BSP_EXTI_HANDLE(EXTI_Line3, 3);
}

void EXTI4_IRQHandler(void)
{
    BSP_EXTI_HANDLE(EXTI_Line4, 4);
}

void EXTI9_5_IRQHandler(void)
{
    BSP_EXTI_HANDLE(EXTI_Line5, 5);
    BSP_EXTI_HANDLE(EXTI_Line6, 6);
    BSP_EXTI_HANDLE(EXTI_Line7, 7);
    BSP_EXTI_HANDLE(EXTI_Line8, 8);
    BSP_EXTI_HANDLE(EXTI_Line9, 9);
}

void EXTI15_10_IRQHandler(void)
{
    BSP_EXTI_HANDLE(EXTI_Line10, 10);
    BSP_EXTI_HANDLE(EXTI_Line11, 11);
    BSP_EXTI_HANDLE(EXTI_Line12, 12);
    BSP_EXTI_HANDLE(EXTI_Line13, 13);
    BSP_EXTI_HANDLE(EXTI_Line14, 14);
    BSP_EXTI_HANDLE(EXTI_Line15, 15);
}
