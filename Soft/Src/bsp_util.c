/**
  ******************************************************************************
  * @file    bsp_util.c
  * @brief   通用小工具: 临界区、字符串/数字转换
  ******************************************************************************
  */
#include "bsp_util.h"
#include "bsp_config.h"
#include "misc.h"

/*---------------------------------------------------------------------------*/
void bsp_util_init(void)
{
    /* 2 位抢占优先级 + 2 位子优先级, 便于给 DMA/串口/EXTI 分配不同优先级 */
    NVIC_PriorityGroupConfig(BSP_NVIC_PRIORITY_GROUP);
}

/*---------------------------------------------------------------------------*/
uint32_t bsp_irq_save(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/*---------------------------------------------------------------------------*/
void bsp_irq_restore(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
    else
    {
        __disable_irq();
    }
}

/*---------------------------------------------------------------------------*/
uint8_t bsp_utoa(uint32_t value, char *buf)
{
    char tmp[11];
    uint8_t i = 0U;
    uint8_t n = 0U;

    if (value == 0U)
    {
        buf[0] = '0';
        buf[1] = '\0';
        return 1U;
    }

    while ((value != 0U) && (i < sizeof(tmp)))
    {
        tmp[i++] = (char)('0' + (value % 10U));
        value /= 10U;
    }

    while (i > 0U)
    {
        buf[n++] = tmp[--i];
    }
    buf[n] = '\0';
    return n;
}

/*---------------------------------------------------------------------------*/
uint8_t bsp_itoa(int32_t value, char *buf)
{
    uint8_t n = 0U;

    if (value < 0)
    {
        buf[0] = '-';
        /* 避免 INT32_MIN 取反溢出 */
        n = (uint8_t)(1U + bsp_utoa((uint32_t)(-(value + 1)) + 1U, &buf[1]));
        return n;
    }
    return bsp_utoa((uint32_t)value, buf);
}

/*---------------------------------------------------------------------------*/
uint8_t bsp_u8toa(uint8_t value, char *buf)
{
    return bsp_utoa((uint32_t)value, buf);
}

/*---------------------------------------------------------------------------*/
uint8_t bsp_hex_to_str(uint32_t value, uint8_t digits, char *buf)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8_t i;

    if (digits == 0U)
    {
        buf[0] = '\0';
        return 0U;
    }
    if (digits > 8U)
    {
        digits = 8U;
    }
    for (i = 0U; i < digits; i++)
    {
        buf[i] = hex[(value >> ((digits - 1U - i) * 4U)) & 0x0FU];
    }
    buf[digits] = '\0';
    return digits;
}

/*---------------------------------------------------------------------------*/
uint32_t bsp_atoi(const char *s)
{
    uint32_t v = 0U;

    if (s == NULL)
    {
        return 0U;
    }
    while ((*s >= '0') && (*s <= '9'))
    {
        v = (v * 10U) + (uint32_t)(*s - '0');
        s++;
    }
    return v;
}

/*---------------------------------------------------------------------------*/
uint8_t bsp_stricmp_eq(const char *a, const char *b)
{
    if ((a == NULL) || (b == NULL))
    {
        return 0U;
    }
    while ((*a != '\0') && (*b != '\0'))
    {
        char ca = *a;
        char cb = *b;

        if ((ca >= 'a') && (ca <= 'z'))
        {
            ca = (char)(ca - 32);
        }
        if ((cb >= 'a') && (cb <= 'z'))
        {
            cb = (char)(cb - 32);
        }
        if (ca != cb)
        {
            return 0U;
        }
        a++;
        b++;
    }
    return ((*a == '\0') && (*b == '\0')) ? 1U : 0U;
}

/*---------------------------------------------------------------------------*/
void bsp_str_upper(char *s)
{
    if (s == NULL)
    {
        return;
    }
    while (*s != '\0')
    {
        if ((*s >= 'a') && (*s <= 'z'))
        {
            *s = (char)(*s - 32);
        }
        s++;
    }
}
