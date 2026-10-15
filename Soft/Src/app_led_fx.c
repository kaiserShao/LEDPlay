/**
  ******************************************************************************
  * @file    app_led_fx.c
  * @brief   主灯带效果层实现
  ******************************************************************************
  */
#include "app_led_fx.h"
#include "drv_ws2812.h"
#include "bsp_time.h"

#define LED_FX_MIN_PERIOD_MS   200U     /* 最慢 */
#define LED_FX_MAX_PERIOD_MS   8000U    /* 最快 */
#define LED_FX_TARGET_FPS      30U      /* 动画刷新率 */

static led_fx_t  s_fx       = LED_FX_OFF;
static uint8_t   s_r = 0U;
static uint8_t   s_g = 0U;
static uint8_t   s_b = 0U;
static uint16_t  s_period   = 1200U;
static uint32_t  s_base_ms  = 0U;
static uint32_t  s_last_ms  = 0U;
static uint8_t   s_inited   = 0U;

/*---------------------------------------------------------------------------*/
/** 色轮: hue(0~255) -> R G B */
static void hue_to_rgb(uint8_t hue, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint16_t h = hue;

    if (h < 85U)
    {
        *r = (uint8_t)(255U - (h * 3U));
        *g = (uint8_t)(h * 3U);
        *b = 0U;
    }
    else if (h < 170U)
    {
        h -= 85U;
        *r = 0U;
        *g = (uint8_t)(255U - (h * 3U));
        *b = (uint8_t)(h * 3U);
    }
    else
    {
        h -= 170U;
        *r = (uint8_t)(h * 3U);
        *g = 0U;
        *b = (uint8_t)(255U - (h * 3U));
    }
}

/*---------------------------------------------------------------------------*/
static uint8_t fx_update_interval(void)
{
    uint32_t iv = (uint32_t)s_period / LED_FX_TARGET_FPS;

    if (iv < 20U)
    {
        iv = 20U;
    }
    if (iv > 100U)
    {
        iv = 100U;
    }
    return (uint8_t)iv;
}

/*---------------------------------------------------------------------------*/
/** 画一帧 */
static void fx_render(uint32_t phase)
{
    uint16_t i;

    /* 与"命令任务"(app_cmd 的 FILL/LED 命令)互斥: 保证整帧颜色成套 */
    ws2812_lock();

    switch (s_fx)
    {
        case LED_FX_SOLID:
            ws2812_fill(s_r, s_g, s_b);
            break;

        case LED_FX_RAINBOW:
            for (i = 0U; i < WS2812_LED_NUM; i++)
            {
                uint8_t hue = (uint8_t)(((uint32_t)i * 256U / WS2812_LED_NUM) + phase);
                uint8_t r;
                uint8_t g;
                uint8_t b;

                hue_to_rgb(hue, &r, &g, &b);
                ws2812_set_pixel(i, r, g, b);
            }
            break;

        case LED_FX_COMET:
        {
            /* 头位置 + 拖尾(越靠后越暗) */
            uint16_t head = (uint16_t)(phase % WS2812_LED_NUM);
            uint16_t tail_len = (WS2812_LED_NUM / 4U);

            for (i = 0U; i < WS2812_LED_NUM; i++)
            {
                uint16_t dist = (uint16_t)((head + WS2812_LED_NUM - i) % WS2812_LED_NUM);

                if (dist < tail_len)
                {
                    uint16_t k = (uint16_t)(255U - ((uint32_t)dist * 255U / tail_len));
                    ws2812_set_pixel(i,
                                     (uint8_t)(((uint16_t)s_r * k) >> 8),
                                     (uint8_t)(((uint16_t)s_g * k) >> 8),
                                     (uint8_t)(((uint16_t)s_b * k) >> 8));
                }
                else
                {
                    ws2812_set_pixel(i, 0U, 0U, 0U);
                }
            }
            break;
        }

        case LED_FX_BREATHE:
        case LED_FX_BLINK:
        {
            uint8_t level;

            if (s_fx == LED_FX_BLINK)
            {
                level = (phase < 128U) ? 255U : 0U;
            }
            else
            {
                /* 呼吸: 0~255 三角波 */
                level = (uint8_t)((phase < 128U) ? (phase * 2U) : ((255U - phase) * 2U));
            }

            ws2812_fill((uint8_t)(((uint16_t)s_r * level) >> 8),
                        (uint8_t)(((uint16_t)s_g * level) >> 8),
                        (uint8_t)(((uint16_t)s_b * level) >> 8));
            break;
        }

        case LED_FX_CHASE:
        {
            uint16_t pos  = (uint16_t)((phase * WS2812_LED_NUM) / 256U);
            uint16_t span = (WS2812_LED_NUM / 12U);

            if (span == 0U)
            {
                span = 1U;
            }
            for (i = 0U; i < WS2812_LED_NUM; i++)
            {
                uint16_t d = (uint16_t)((i + WS2812_LED_NUM - pos) % WS2812_LED_NUM);

                if (d < span)
                {
                    ws2812_set_pixel(i, s_r, s_g, s_b);
                }
                else
                {
                    ws2812_set_pixel(i, 0U, 0U, 0U);
                }
            }
            break;
        }

        case LED_FX_OFF:
        default:
            ws2812_clear();
            break;
    }

    ws2812_show();
    ws2812_unlock();
}

/*===========================================================================
 *                              对外接口
 *=========================================================================*/
void led_fx_init(void)
{
    s_fx      = LED_FX_OFF;
    s_r       = 0U;
    s_g       = 0U;
    s_b       = 0U;
    s_period  = 1200U;
    s_base_ms = bsp_time_ms();
    s_last_ms = s_base_ms;
    s_inited  = 1U;

    fx_render(0U);      /* 上电先全灭 */
}

/*---------------------------------------------------------------------------*/
void led_fx_set(led_fx_t fx)
{
    if (fx >= LED_FX_COUNT)
    {
        fx = LED_FX_OFF;
    }
    s_fx      = fx;
    s_base_ms = bsp_time_ms();   /* 效果从 0 相位重新开始 */
    s_last_ms = s_base_ms;
    fx_render(0U);               /* 立刻出一帧, 切换无延迟 */
}

/*---------------------------------------------------------------------------*/
led_fx_t led_fx_get(void)
{
    return s_fx;
}

/*---------------------------------------------------------------------------*/
void led_fx_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    s_r = r;
    s_g = g;
    s_b = b;
}

/*---------------------------------------------------------------------------*/
void led_fx_get_color(uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (r != NULL)
    {
        *r = s_r;
    }
    if (g != NULL)
    {
        *g = s_g;
    }
    if (b != NULL)
    {
        *b = s_b;
    }
}

/*---------------------------------------------------------------------------*/
void led_fx_set_speed(uint16_t period_ms)
{
    if (period_ms < LED_FX_MIN_PERIOD_MS)
    {
        period_ms = LED_FX_MIN_PERIOD_MS;
    }
    if (period_ms > LED_FX_MAX_PERIOD_MS)
    {
        period_ms = LED_FX_MAX_PERIOD_MS;
    }
    s_period = period_ms;
}

/*---------------------------------------------------------------------------*/
uint16_t led_fx_get_speed(void)
{
    return s_period;
}

/*---------------------------------------------------------------------------*/
const char *led_fx_name(led_fx_t fx)
{
    switch (fx)
    {
        case LED_FX_OFF:     return "OFF";
        case LED_FX_SOLID:   return "SOLID";
        case LED_FX_RAINBOW: return "RAINBOW";
        case LED_FX_COMET:   return "COMET";
        case LED_FX_BREATHE: return "BREATHE";
        case LED_FX_BLINK:   return "BLINK";
        case LED_FX_CHASE:   return "CHASE";
        default:             return "?";
    }
}

/*---------------------------------------------------------------------------*/
void led_fx_process(void)
{
    uint32_t now;
    uint32_t elapsed;
    uint32_t phase;

    if (s_inited == 0U)
    {
        return;
    }
    if (s_fx == LED_FX_OFF)
    {
        return;
    }
    if (bsp_elapsed_ms(s_last_ms) < fx_update_interval())
    {
        return;
    }
    s_last_ms = bsp_time_ms();

    now     = s_last_ms;
    elapsed = (uint32_t)(now - s_base_ms);
    phase   = ((elapsed % (uint32_t)s_period) * 256U) / (uint32_t)s_period;

    fx_render(phase);
}
