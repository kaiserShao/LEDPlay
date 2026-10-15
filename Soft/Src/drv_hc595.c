/**
  ******************************************************************************
  * @file    drv_hc595.c
  * @brief   4 位静态数码管驱动 (74HC595 x4)
  ******************************************************************************
  */
#include "drv_hc595.h"
#include "bsp_time.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

/* 逻辑位号 -> 移位顺序: hc595_digit_map[d] = 逻辑第 d 位"第几个被移出"。
   级联时"先移入的字节"最终停在最远端的 595 上, 所以数组里排在前面的位会落在
   离 MCU 最远的那片 595 上。
   本板实测: 用 {0,1,2,3} 时数码管是左右颠倒的(第 1 位显示在第 4 位上), 倒序成
   {3,2,1,0} 就正过来了 —— 用 HC595_DIGIT_REVERSE 切换(bsp_config.h 第 6 节)。 */
#if (HC595_DIGIT_REVERSE != 0U)
const uint8_t hc595_digit_map[HC595_DIGITS] = {3U, 2U, 1U, 0U};
#else
const uint8_t hc595_digit_map[HC595_DIGITS] = {0U, 1U, 2U, 3U};
#endif

/* 内部显示缓冲(已按 HC595_ACTIVE_LEVEL 处理好的"实际输出段码") */
static uint8_t s_seg[HC595_DIGITS];

/*---------------------------------------------------------------------------*/
#define HC595_SDI_HIGH()    GPIO_SetBits(HC595_SDI_PORT, HC595_SDI_PIN)
#define HC595_SDI_LOW()     GPIO_ResetBits(HC595_SDI_PORT, HC595_SDI_PIN)
#define HC595_SCLK_HIGH()   GPIO_SetBits(HC595_SCLK_PORT, HC595_SCLK_PIN)
#define HC595_SCLK_LOW()    GPIO_ResetBits(HC595_SCLK_PORT, HC595_SCLK_PIN)
#define HC595_LOAD_HIGH()   GPIO_SetBits(HC595_LOAD_PORT, HC595_LOAD_PIN)
#define HC595_LOAD_LOW()    GPIO_ResetBits(HC595_LOAD_PORT, HC595_LOAD_PIN)

/*---------------------------------------------------------------------------*/
static void hc595_clk_delay(void)
{
    volatile uint8_t i;

    for (i = 0U; i < HC595_CLK_DELAY_NOP; i++)
    {
        __NOP();
    }
}

/*---------------------------------------------------------------------------*/
/** 移位输出 1 字节(高位先出) */
static void hc595_shift_byte(uint8_t data)
{
    uint8_t mask;

    for (mask = 0x80U; mask != 0U; mask = (uint8_t)(mask >> 1))
    {
        if ((data & mask) != 0U)
        {
            HC595_SDI_HIGH();
        }
        else
        {
            HC595_SDI_LOW();
        }
        hc595_clk_delay();

        HC595_SCLK_HIGH();      /* 上升沿移入 */
        hc595_clk_delay();
        HC595_SCLK_LOW();
        hc595_clk_delay();
    }
}

/*---------------------------------------------------------------------------*/
/** 把缓冲内容全部移出并锁存 */
void hc595_flush(void)
{
    uint8_t i;

    HC595_LOAD_LOW();           /* 移位期间锁存保持低 */

    for (i = 0U; i < HC595_DIGITS; i++)
    {
        /* 按 map 找到"第 i 个被移出"的位 */
        uint8_t d;

        for (d = 0U; d < HC595_DIGITS; d++)
        {
            if (hc595_digit_map[d] == i)
            {
                break;
            }
        }
        if (d >= HC595_DIGITS)
        {
            d = i;
        }

        {
            uint8_t v = s_seg[d];

#if (HC595_ACTIVE_LEVEL == 0U)
            v = (uint8_t)(~v);      /* 共阳: 段码取反 */
#endif
            hc595_shift_byte(v);
        }
    }

    HC595_LOAD_HIGH();          /* 上升沿: 移位寄存器 -> 输出锁存器 */
    hc595_clk_delay();
    HC595_LOAD_LOW();
}

/*---------------------------------------------------------------------------*/
void hc595_init(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t          i;

    RCC_APB2PeriphClockCmd(HC595_GPIO_CLK, ENABLE);

    gpio.GPIO_Pin   = (uint16_t)(HC595_SDI_PIN | HC595_SCLK_PIN | HC595_LOAD_PIN);
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(HC595_SDI_PORT, &gpio);

    HC595_SCLK_LOW();
    HC595_LOAD_LOW();
    HC595_SDI_LOW();

    for (i = 0U; i < HC595_DIGITS; i++)
    {
        s_seg[i] = 0U;
    }
    hc595_flush();
}

/*---------------------------------------------------------------------------*/
uint8_t hc595_char_to_seg(char c)
{
    uint8_t seg;

    switch (c)
    {
        case '0': seg = 0x3FU; break;
        case '1': seg = 0x06U; break;
        case '2': seg = 0x5BU; break;
        case '3': seg = 0x4FU; break;
        case '4': seg = 0x66U; break;
        case '5': seg = 0x6DU; break;
        case '6': seg = 0x7DU; break;
        case '7': seg = 0x07U; break;
        case '8': seg = 0x7FU; break;
        case '9': seg = 0x6FU; break;
        case 'A': case 'a': seg = 0x77U; break;
        case 'B': case 'b': seg = 0x7CU; break;
        case 'C': case 'c': seg = 0x39U; break;
        case 'D': case 'd': seg = 0x5EU; break;
        case 'E': case 'e': seg = 0x79U; break;
        case 'F': case 'f': seg = 0x71U; break;
        case 'H': case 'h': seg = 0x76U; break;
        case 'L': case 'l': seg = 0x38U; break;
        case 'P': case 'p': seg = 0x73U; break;
        case 'R': case 'r': seg = 0x50U; break;
        case 'U': case 'u': seg = 0x3EU; break;
        case '-':           seg = 0x40U; break;
        case '_':           seg = 0x08U; break;
        case ' ':           seg = 0x00U; break;
        default:            seg = 0x00U; break;
    }
    return seg;
}

/*===========================================================================
 *                              对外接口
 *=========================================================================*/
void hc595_write_index(uint8_t digit, uint8_t seg)
{
    if (digit >= HC595_DIGITS)
    {
        return;
    }
    s_seg[digit] = seg;
}

/*---------------------------------------------------------------------------*/
uint8_t hc595_get_seg(uint8_t digit)
{
    if (digit >= HC595_DIGITS)
    {
        return 0U;
    }
    return s_seg[digit];
}

/*---------------------------------------------------------------------------*/
void hc595_write(const uint8_t seg[HC595_DIGITS])
{
    uint8_t i;

    if (seg == NULL)
    {
        return;
    }
    for (i = 0U; i < HC595_DIGITS; i++)
    {
        s_seg[i] = seg[i];
    }
    hc595_flush();
}

/*---------------------------------------------------------------------------*/
void hc595_clear(void)
{
    uint8_t i;

    for (i = 0U; i < HC595_DIGITS; i++)
    {
        s_seg[i] = 0U;
    }
    hc595_flush();
}

/*---------------------------------------------------------------------------*/
void hc595_show_digits(const char *text)
{
    char    packed[HC595_DIGITS + 1U];
    uint8_t dp[HC595_DIGITS];
    uint8_t n = 0U;
    uint8_t i;

    if (text == NULL)
    {
        return;
    }

    /* 先保存小数点状态(显示过程中保留原有小数点) */
    for (i = 0U; i < HC595_DIGITS; i++)
    {
        dp[i] = (uint8_t)(s_seg[i] & 0x80U);
    }

    /* "." 不占位: 归到它前面那一位的小数点上(方便显示 "1.234" 这种) */
    for (i = 0U; (text[i] != '\0') && (n < HC595_DIGITS); i++)
    {
        if (text[i] == '.')
        {
            if (n > 0U)
            {
                dp[n - 1U] = 0x80U;
            }
            continue;
        }
        packed[n] = text[i];
        n++;
    }

    for (i = 0U; i < HC595_DIGITS; i++)
    {
        uint8_t seg = (i < n) ? hc595_char_to_seg(packed[i]) : 0U;

        s_seg[i] = (uint8_t)(seg | dp[i]);
    }
    hc595_flush();
}

/*---------------------------------------------------------------------------*/
void hc595_show_int(int32_t value)
{
    char    buf[HC595_DIGITS + 1U];
    uint8_t i;

    for (i = 0U; i < HC595_DIGITS; i++)
    {
        buf[i] = ' ';
    }
    buf[HC595_DIGITS] = '\0';

    if (value > 9999L)
    {
        for (i = 0U; i < HC595_DIGITS; i++)
        {
            buf[i] = '-';
        }
    }
    else if (value < 0L)
    {
        int32_t v = (value == (-9999L)) ? 0L : -value;

        if (v > 999L)
        {
            v = 999L;
        }
        buf[0] = '-';
        buf[3] = (char)('0' + (v % 10));
        v /= 10;
        buf[2] = (char)('0' + (v % 10));
        v /= 10;
        buf[1] = (char)('0' + (v % 10));
    }
    else
    {
        buf[3] = (char)('0' + (value % 10));
        value /= 10;
        buf[2] = (char)('0' + (value % 10));
        value /= 10;
        buf[1] = (char)('0' + (value % 10));
        value /= 10;
        buf[0] = (char)('0' + (value % 10));
    }

    hc595_show_digits(buf);
}

/*---------------------------------------------------------------------------*/
void hc595_set_dp(uint8_t digit, uint8_t on)
{
    if (digit >= HC595_DIGITS)
    {
        return;
    }
    if (on != 0U)
    {
        s_seg[digit] |= 0x80U;
    }
    else
    {
        s_seg[digit] &= (uint8_t)(~0x80U);
    }
    hc595_flush();
}

/*---------------------------------------------------------------------------*/
void hc595_set_dp_mask(uint8_t mask)
{
    uint8_t i;

    for (i = 0U; i < HC595_DIGITS; i++)
    {
        if ((mask & (1U << i)) != 0U)
        {
            s_seg[i] |= 0x80U;
        }
        else
        {
            s_seg[i] &= (uint8_t)(~0x80U);
        }
    }
    hc595_flush();
}
