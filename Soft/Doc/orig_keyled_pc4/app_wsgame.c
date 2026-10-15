/**
  ******************************************************************************
  * @file    app_wsgame.c
  * @brief   灯带消消乐(游戏层)实现: 运输带 + 同色子弹
  *
  *  数据模型(全部只在灯光任务里改 —— 单写者, 不需要加锁)
  *  ---------------------------------------------------------------------------
  *  s_track[]   : 整条灯带的"运输带", 每格 = 0(空) 或 颜色编号+1;
  *                每 WSGAME_TRACK_STEP_MS 整体朝索引 0 方向挪一格, 挪到索引 0
  *                还没被消掉 = 漏掉(只计数, 近端微亮提示后消失);
  *  s_shot_*[]  : 在飞的子弹 {颜色, 位置}; 每 WSGAME_SHOT_STEP_MS 朝索引
  *                WSGAME_MAX_LEDS-1 方向飞一格:
  *                  撞上同色格 -> 两颗一起消失(白闪 + 得分)
  *                  撞上不同色 -> 什么都不做, 子弹继续往外飞(可以打穿)
  *                  飞出远端   -> 收回;
  *  s_xxx_base_ms: 三个节拍的时间戳(运输带 / 远端口子 / 子弹)。到期就"基准 +=
  *                周期", 所以任务偶尔卡一拍也能自动补上, 不会越跑越慢。
  *
  *  渲染: 每 WSGAME_FRAME_MS 整条重画一次(运输带 -> 子弹 -> 命中白闪/漏掉提示),
  *        只在游戏运行时碰灯带; 停止时清屏并把灯带交还效果层。
  ******************************************************************************
  */
#include "app_wsgame.h"
#include "app_led_fx.h"
#include "bsp_time.h"
#include "drv_esp32.h"
#include "drv_hc595.h"
#include "drv_key.h"
#include "drv_keyled.h"
#include "drv_ws2812.h"
#include <stdarg.h>
#include <stdio.h>          /* vsnprintf */

/* 运输带格子的取值: 0 = 空, 1..WSGAME_COLOR_NUM = (颜色编号 + 1) */
#define WSGAME_CELL_EMPTY     0U

/* 命令任务 -> 灯光任务的单字请求(下一拍被消费) */
#define WSGAME_REQ_NONE       0U
#define WSGAME_REQ_START      1U
#define WSGAME_REQ_STOP       2U

#if (WSGAME_COLOR_NUM < 1U)
#error "WSGAME_COLOR_NUM 至少要是 1"
#endif
#if (WSGAME_MAX_SHOTS < 1U)
#error "WSGAME_MAX_SHOTS 至少要是 1"
#endif
#if (WSGAME_MAX_LEDS < 8U)
#error "WSGAME_MAX_LEDS 太小了, 运输带放不下"
#endif
#if (WSGAME_COLOR_NUM > KEYLED_COUNT)
#error "颜色键灯比按键灯还多, 请检查 KEYLED_NUM"
#endif

/*------------------------------- 内部状态 --------------------------------*/
static uint8_t  s_inited  = 0U;
static volatile uint8_t  s_running = 0U;            /* 灯光任务写, 其它任务只读 */
static volatile uint8_t  s_req     = WSGAME_REQ_NONE;
static volatile uint16_t s_track_step_ms = WSGAME_TRACK_STEP_MS;
static volatile uint16_t s_spawn_ms      = WSGAME_SPAWN_MS;
static volatile uint16_t s_shot_step_ms  = WSGAME_SHOT_STEP_MS;

static uint8_t  s_track[WSGAME_MAX_LEDS];           /* 运输带: 0 或 颜色编号+1 */
static uint16_t s_blocks = 0U;                      /* 场上还活着几颗灯 */

static uint8_t  s_shot_active[WSGAME_MAX_SHOTS];
static uint8_t  s_shot_color[WSGAME_MAX_SHOTS];
static uint16_t s_shot_pos[WSGAME_MAX_SHOTS];
static uint16_t s_shots = 0U;                       /* 在飞的子弹数 */

static uint16_t s_score   = 0U;
static uint16_t s_hits    = 0U;
static uint16_t s_misses  = 0U;
static uint16_t s_spawned = 0U;
static uint8_t  s_last_color = WSGAME_COLOR_NONE;   /* 上一次生成的色号(防连色) */

static uint32_t s_track_base_ms = 0U;               /* 运输带节拍基准 */
static uint32_t s_spawn_base_ms = 0U;               /* 远端补灯节拍基准 */
static uint32_t s_shot_base_ms  = 0U;               /* 子弹节拍基准 */
static uint32_t s_render_ms     = 0U;               /* 上一帧的时间 */
static uint8_t  s_need_render   = 1U;               /* 有"立刻要画"的变化 */

static uint8_t  s_hit_active = 0U;                  /* 命中白闪 */
static uint16_t s_hit_idx    = 0U;
static uint32_t s_hit_ms     = 0U;
static uint8_t  s_miss_active = 0U;                 /* 漏掉提示(近端微亮) */
static uint32_t s_miss_ms     = 0U;

static uint16_t s_keyled_flash = 0U;                /* 位图: 哪几颗按键灯正在闪 */
static uint32_t s_keyled_flash_ms = 0U;

static uint32_t s_rand = 0x1A2B3C4DU;               /* xorshift32: 不动标准库 rand() */

/*===========================================================================
 *                              小工具
 *=========================================================================*/
/** xorshift32: 便宜、够用的伪随机(不动标准库 rand(), 免得和其它模块抢种子) */
static uint32_t rnd_next(void)
{
    uint32_t x = s_rand;

    x ^= (x << 13);
    x ^= (x >> 17);
    x ^= (x << 5);
    s_rand = x;

    if (s_rand == 0U)
    {
        s_rand = 0x9E3779B9U;            /* 全 0 会锁死, 兜一下 */
    }

    return s_rand;
}

/** 颜色编号 -> RGB(取值见 bsp_config.h 第 10 节) */
static void color_rgb(uint8_t color, uint8_t *r, uint8_t *g, uint8_t *b)
{
    switch (color)
    {
        case WSGAME_COLOR_BLUE:
            *r = WSGAME_BLUE_R;   *g = WSGAME_BLUE_G;   *b = WSGAME_BLUE_B;
            break;

        case WSGAME_COLOR_GREEN:
            *r = WSGAME_GREEN_R;  *g = WSGAME_GREEN_G;  *b = WSGAME_GREEN_B;
            break;

        case WSGAME_COLOR_PURPLE:
            *r = WSGAME_PURPLE_R; *g = WSGAME_PURPLE_G; *b = WSGAME_PURPLE_B;
            break;

        default:
            *r = 0U; *g = 0U; *b = 0U;
            break;
    }
}

/**
  * @brief  逻辑位置(0 = 近端, WSGAME_MAX_LEDS-1 = 远端) -> 物理灯珠索引
  * @note   只在渲染时用; WSGAME_REVERSE_DIR = 1 时镜像一下, 逻辑完全不变。
  */
static uint16_t led_index(uint16_t logical)
{
    uint16_t last = (uint16_t)(WSGAME_MAX_LEDS - 1U);

    if (logical > last)
    {
        logical = last;
    }

#if (WSGAME_REVERSE_DIR != 0U)
    return (uint16_t)(last - logical);
#else
    return logical;
#endif
}

/** 调试行: 从 ESP32 串口打 "WSGAME ..."(WSGAME_TRACE = 0 时只剩一句空判断) */
static void trace_line(const char *fmt, ...)
{
#if (WSGAME_TRACE != 0U)
    char    buf[88];
    va_list ap;

    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    esp32_printf("WSGAME %s\r\n", buf);
#else
    (void)fmt;
#endif
}

/** 按键编号 -> 颜色编号(不是颜色键就返回 WSGAME_COLOR_NONE) */
static uint8_t key_color(uint8_t key_id)
{
    switch (key_id)
    {
        case WSGAME_KEY_BLUE:   return WSGAME_COLOR_BLUE;
        case WSGAME_KEY_GREEN:  return WSGAME_COLOR_GREEN;
        case WSGAME_KEY_PURPLE: return WSGAME_COLOR_PURPLE;
        default:                return WSGAME_COLOR_NONE;
    }
}

/*===========================================================================
 *                          按键灯 / 数码管
 *=========================================================================*/
/**
  * @brief  按"当前状态"刷一遍 4 颗按键灯
  * @note   只在状态变化时调用 —— keyled_set_fx() 会重置闪烁相位并立刻输出一次
  *         (关中断约 120us), 每拍都调的话闪烁就动不起来了。
  */
static void keyled_apply(void)
{
    uint8_t i;

    if (s_running == 0U)
    {
        keyled_off();
        return;
    }

    for (i = 0U; i < WSGAME_COLOR_NUM; i++)
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;

        color_rgb(i, &r, &g, &b);

        /* 三色统一打个折: 直接给 255 又刺眼, 又容易被看成一片白 */
        r = (uint8_t)(((uint16_t)r * WSGAME_KEYLED_LEVEL) / 255U);
        g = (uint8_t)(((uint16_t)g * WSGAME_KEYLED_LEVEL) / 255U);
        b = (uint8_t)(((uint16_t)b * WSGAME_KEYLED_LEVEL) / 255U);

        if ((s_keyled_flash & (uint16_t)(1U << i)) != 0U)
        {
            keyled_set_fx(i, KEYLED_FX_BLINK, r, g, b, WSGAME_KEYLED_BLINK_MS);
        }
        else
        {
            keyled_set_color(i, r, g, b);            /* 平时: 常亮本色 */
        }
    }

    /* 第 4 颗 = 运行指示(亮着就说明游戏在跑) */
    keyled_set_color(WSGAME_COLOR_NUM, WSGAME_LED3_R, WSGAME_LED3_G, WSGAME_LED3_B);
}

#if (WSGAME_SEG_SCORE != 0U)
/** 数码管显示得分(命中 +1, 开局清 0, 封顶 9999) */
static void seg_update(void)
{
    uint16_t v = s_score;

    if (v > 9999U)
    {
        v = 9999U;
    }

    hc595_show_int((int16_t)v);
}
#endif

/*===========================================================================
 *                            运输带(从远端往近端挪)
 *=========================================================================*/
/** 远端那边最靠外的一个空格 = 新灯放的位置; 整条满带返回 -1 */
static int16_t track_top_free(void)
{
    int16_t i;

    for (i = (int16_t)(WSGAME_MAX_LEDS - 1U); i >= 0; i--)
    {
        if (s_track[(uint16_t)i] == WSGAME_CELL_EMPTY)
        {
            return i;
        }
    }

    return -1;
}

/** 在远端补一颗新灯: 紧跟在上一次那颗后面(没空位就这一拍先不生成) */
static void block_spawn(void)
{
    int16_t pos = track_top_free();
    uint8_t c;

    if (pos < 0)
    {
        return;                          /* 整条都占满了: 等前面消掉/漏掉腾位置 */
    }

    c = (uint8_t)(rnd_next() % WSGAME_COLOR_NUM);

#if (WSGAME_AVOID_REPEAT != 0U)
    if (c == s_last_color)
    {
        /* 换一种(颜色数 >= 2 时一定换得掉) */
        c = (uint8_t)((c + 1U + (uint8_t)(rnd_next() % (WSGAME_COLOR_NUM - 1U))) % WSGAME_COLOR_NUM);
    }
#endif

    s_last_color = c;
    s_track[(uint16_t)pos] = (uint8_t)(c + 1U);
    s_blocks++;
    s_spawned++;
    s_need_render = 1U;

    trace_line("SPAWN color=%s idx=%u blocks=%u", wsgame_color_name(c), (unsigned)pos,
               (unsigned)s_blocks);
}

/**
  * @brief  结账一颗子弹当前所在格(子弹自己没动, 灯挪过来了也算)
  * @note   空格 -> 继续飞; 同色 -> 两颗一起消失 + 得分;
  *         异色 -> 子弹撞上就没了(色块留在原地, 不得分)。
  */
static void shot_check(uint8_t k)
{
    uint8_t cell;

    if (s_shot_active[k] == 0U)
    {
        return;
    }

    cell = s_track[s_shot_pos[k]];

    if (cell == WSGAME_CELL_EMPTY)
    {
        return;                                     /* 空格: 继续飞 */
    }

    if ((uint8_t)(cell - 1U) != s_shot_color[k])
    {
        /* 颜色不同: 子弹撞上就消失(收回, 不得分), 色块原地不动 */
        s_shot_active[k] = 0U;
        if (s_shots > 0U)
        {
            s_shots--;
        }
        s_need_render = 1U;

        trace_line("BLOCK my=%s cell=%s idx=%u shots=%u", wsgame_color_name(s_shot_color[k]),
                   wsgame_color_name((uint8_t)(cell - 1U)), (unsigned)s_shot_pos[k],
                   (unsigned)s_shots);
        return;
    }

    /* 同色: 同归于尽 */
    s_track[s_shot_pos[k]] = WSGAME_CELL_EMPTY;
    if (s_blocks > 0U)
    {
        s_blocks--;
    }
    s_shot_active[k] = 0U;
    if (s_shots > 0U)
    {
        s_shots--;
    }

    s_score++;
    s_hits++;
    s_hit_active  = 1U;
    s_hit_idx     = s_shot_pos[k];
    s_hit_ms      = bsp_time_ms();
    s_need_render = 1U;

    trace_line("HIT color=%s idx=%u score=%u left=%u", wsgame_color_name(s_shot_color[k]),
               (unsigned)s_shot_pos[k], (unsigned)s_score, (unsigned)s_blocks);

#if (WSGAME_SEG_SCORE != 0U)
    seg_update();
#endif
}

/** 运输带整体往近端挪一格(索引 0 = 最靠近按键的那颗) */
static void track_tick(void)
{
    uint16_t i;
    uint8_t  k;

    /* ① 已经挪到近端还没被消掉 = 漏掉(只计数不扣分, 近端微亮提示一下) */
    if (s_track[0] != WSGAME_CELL_EMPTY)
    {
        s_track[0] = WSGAME_CELL_EMPTY;
        if (s_blocks > 0U)
        {
            s_blocks--;
        }
        s_misses++;
        s_miss_active = 1U;
        s_miss_ms     = bsp_time_ms();

        trace_line("MISS near-end misses=%u left=%u", (unsigned)s_misses, (unsigned)s_blocks);
    }

    /* ② 整条往前挪一格, 远端空出一格 */
    for (i = 0U; i < (uint16_t)(WSGAME_MAX_LEDS - 1U); i++)
    {
        s_track[i] = s_track[i + 1U];
    }
    s_track[WSGAME_MAX_LEDS - 1U] = WSGAME_CELL_EMPTY;

    /* ③ 灯挪到子弹身上也算撞上 */
    for (k = 0U; k < WSGAME_MAX_SHOTS; k++)
    {
        shot_check(k);
    }

    s_need_render = 1U;
}

/*===========================================================================
 *                            子弹(从近端往远端飞)
 *=========================================================================*/
/** 发射一颗: 从近端(索引 0)出发, 朝远端飞; 天上的子弹满了就不响应 */
static void shot_fire(uint8_t color)
{
    uint8_t  k;
    uint32_t now = bsp_time_ms();

    for (k = 0U; k < WSGAME_MAX_SHOTS; k++)
    {
        if (s_shot_active[k] == 0U)
        {
            break;
        }
    }

    if (k >= WSGAME_MAX_SHOTS)
    {
        trace_line("SHOT full color=%s max=%u", wsgame_color_name(color),
                   (unsigned)WSGAME_MAX_SHOTS);
        return;
    }

    s_shot_active[k] = 1U;
    s_shot_color[k]  = color;
    s_shot_pos[k]    = 0U;
    s_shots++;
    s_need_render    = 1U;

    /* 对应的按键灯闪一下(WSGAME_KEYLED_FLASH_MS 之后自动回到常亮本色) */
    s_keyled_flash   |= (uint16_t)(1U << color);
    s_keyled_flash_ms = now;
    keyled_apply();

    /* 近端最外那颗上正好有灯的话, 这一发立刻就算撞上 */
    shot_check(k);

    trace_line("SHOT color=%s shots=%u", wsgame_color_name(color), (unsigned)s_shots);
}

/** 所有在飞的子弹整体往外飞一格(飞出远端就收回) */
static void shots_step(void)
{
    uint8_t k;

    for (k = 0U; k < WSGAME_MAX_SHOTS; k++)
    {
        if (s_shot_active[k] == 0U)
        {
            continue;
        }

        s_shot_pos[k]++;

        if (s_shot_pos[k] >= WSGAME_MAX_LEDS)
        {
            s_shot_active[k] = 0U;                  /* 飞出远端: 收回 */
            if (s_shots > 0U)
            {
                s_shots--;
            }
            continue;
        }

        shot_check(k);
    }

    s_need_render = 1U;
}

/*===========================================================================
 *                              渲染
 *=========================================================================*/
/** 整条重画一帧(只有游戏在跑时才调用) */
static void game_render(void)
{
    uint16_t i;
    uint8_t  k;
    uint8_t  r;
    uint8_t  g;
    uint8_t  b;

    ws2812_lock();                       /* 整帧成套: 不会出现半新半旧 */
    ws2812_clear();

    /* ① 运输带上的灯 */
    for (i = 0U; i < WSGAME_MAX_LEDS; i++)
    {
        if (s_track[i] != WSGAME_CELL_EMPTY)
        {
            color_rgb((uint8_t)(s_track[i] - 1U), &r, &g, &b);
            ws2812_set_pixel(led_index(i), r, g, b);
        }
    }

    /* ② 漏掉提示: 近端第 0 颗灰白微亮一下(告诉玩家"这颗跑掉了") */
    if (s_miss_active != 0U)
    {
        if (bsp_elapsed_ms(s_miss_ms) < WSGAME_MISS_MS)
        {
            ws2812_set_pixel(led_index(0U), 60U, 60U, 60U);
        }
        else
        {
            s_miss_active = 0U;
        }
    }

    /* ③ 在飞的子弹(画在灯上面: 一飞而过时看得见) */
    for (k = 0U; k < WSGAME_MAX_SHOTS; k++)
    {
        if (s_shot_active[k] != 0U)
        {
            color_rgb(s_shot_color[k], &r, &g, &b);
            ws2812_set_pixel(led_index(s_shot_pos[k]), r, g, b);
        }
    }

    /* ④ 命中白闪(画在最上面) */
    if (s_hit_active != 0U)
    {
        if (bsp_elapsed_ms(s_hit_ms) < WSGAME_HIT_MS)
        {
            ws2812_set_pixel(led_index(s_hit_idx), 255U, 255U, 255U);
        }
        else
        {
            s_hit_active = 0U;
        }
    }

    ws2812_show();
    ws2812_unlock();

    s_render_ms   = bsp_time_ms();
    s_need_render = 0U;
}

/*===========================================================================
 *                            开局 / 收局
 *=========================================================================*/
/** 上一局开跑之前效果层停在哪个效果上(收局时还回去) */
static uint8_t s_fx_back = (uint8_t)LED_FX_COMET;

/** 清空运输带 / 子弹 / 计数(不动 s_running, 也不碰灯带) */
static void game_reset(void)
{
    uint8_t i;

    for (i = 0U; i < WSGAME_MAX_LEDS; i++)
    {
        s_track[i] = WSGAME_CELL_EMPTY;
    }

    for (i = 0U; i < WSGAME_MAX_SHOTS; i++)
    {
        s_shot_active[i] = 0U;
        s_shot_color[i]  = 0U;
        s_shot_pos[i]    = 0U;
    }

    s_blocks          = 0U;
    s_shots           = 0U;
    s_score           = 0U;
    s_hits            = 0U;
    s_misses          = 0U;
    s_spawned         = 0U;
    s_last_color      = WSGAME_COLOR_NONE;
    s_hit_active      = 0U;
    s_miss_active     = 0U;
    s_keyled_flash    = 0U;
    s_keyled_flash_ms = 0U;
}

static void game_start(void)
{
    uint32_t now;

    game_reset();

    /* 效果层让位: 记住原来在跑什么, 收局时还回去 */
    s_fx_back = (uint8_t)led_fx_get();
    if (s_fx_back == (uint8_t)LED_FX_OFF)
    {
        s_fx_back = (uint8_t)LED_FX_COMET;       /* 上电就是 OFF 的话, 收局跑彗星 */
    }
    led_fx_set(LED_FX_OFF);                      /* 灯带交给游戏(顺便清屏) */

    now             = bsp_time_ms();
    s_track_base_ms = now;
    s_spawn_base_ms = now;
    s_shot_base_ms  = now;
    s_render_ms     = now;
    s_need_render   = 1U;

    s_running = 1U;

    block_spawn();                               /* 开局先放一颗, 别对着空灯带等半秒 */

    keyled_apply();
#if (WSGAME_SEG_SCORE != 0U)
    seg_update();
#endif

    trace_line("START step=%ums spawn=%ums shot=%ums maxshots=%u",
               (unsigned)s_track_step_ms, (unsigned)s_spawn_ms,
               (unsigned)s_shot_step_ms, (unsigned)WSGAME_MAX_SHOTS);
}

static void game_stop(void)
{
    uint16_t score  = s_score;
    uint16_t hits   = s_hits;
    uint16_t misses = s_misses;

    s_running = 0U;
    game_reset();
    s_need_render = 0U;

    keyled_off();                                /* 4 颗按键灯全灭(交还 app_cmd 的按下反馈) */

    ws2812_lock();                               /* 清屏 */
    ws2812_clear();
    ws2812_show();
    ws2812_unlock();

    led_fx_set((led_fx_t)s_fx_back);             /* 灯带交还效果层(下一拍它自己画) */

    trace_line("STOP score=%u hits=%u misses=%u", (unsigned)score, (unsigned)hits,
               (unsigned)misses);
}

/*===========================================================================
 *                              主流程
 *=========================================================================*/
/** 把一条按键事件喂给游戏(颜色键 = 发射; Key4 短按 = 开关) */
static void game_key(const key_msg_t *msg)
{
    uint8_t color;

    if (msg->event == (uint8_t)KEY_EV_SHORT)
    {
        if (msg->id == WSGAME_KEY_START)
        {
            (void)wsgame_toggle();               /* 短按 Key4: 开始 / 暂停 */
        }
        return;                                  /* 颜色键只认"按下", 松手不再补一发 */
    }

    if (msg->event != (uint8_t)KEY_EV_DOWN)
    {
        return;                                  /* 长按/保持不连发 */
    }

    if (s_running == 0U)
    {
        return;                                  /* 没开局: 颜色键什么都不做 */
    }

    color = key_color(msg->id);
    if (color == WSGAME_COLOR_NONE)
    {
        return;
    }

    shot_fire(color);
}

/*===========================================================================
 *                              对外接口
 *=========================================================================*/
void wsgame_init(void)
{
    if (s_inited != 0U)
    {
        return;
    }

    game_reset();
    s_running = 0U;
    s_req     = WSGAME_REQ_NONE;
    s_inited  = 1U;

#if (WSGAME_AUTOSTART != 0U)
    wsgame_start();                              /* 只是发个请求, 下一拍真正开局 */
#endif
}

/*---------------------------------------------------------------------------*/
void wsgame_start(void)
{
    s_req = WSGAME_REQ_START;                    /* 灯光任务下一拍消费(跨任务只递请求) */
}

/*---------------------------------------------------------------------------*/
void wsgame_stop(void)
{
    s_req = WSGAME_REQ_STOP;
}

/*---------------------------------------------------------------------------*/
uint8_t wsgame_is_running(void)
{
    return s_running;
}

/*---------------------------------------------------------------------------*/
uint8_t wsgame_toggle(void)
{
    if ((s_running != 0U) || (s_req == WSGAME_REQ_START))
    {
        s_req = WSGAME_REQ_STOP;
        return 0U;
    }

    s_req = WSGAME_REQ_START;
    return 1U;
}

/*---------------------------------------------------------------------------*/
void wsgame_set_step_ms(uint16_t step_ms)
{
    if (step_ms < WSGAME_TRACK_STEP_MIN)
    {
        step_ms = WSGAME_TRACK_STEP_MIN;
    }
    if (step_ms > WSGAME_TRACK_STEP_MAX)
    {
        step_ms = WSGAME_TRACK_STEP_MAX;
    }

    s_track_step_ms = step_ms;
}

/*---------------------------------------------------------------------------*/
uint16_t wsgame_get_step_ms(void)
{
    return s_track_step_ms;
}

/*---------------------------------------------------------------------------*/
void wsgame_set_spawn_ms(uint16_t spawn_ms)
{
    if (spawn_ms < WSGAME_SPAWN_MIN)
    {
        spawn_ms = WSGAME_SPAWN_MIN;
    }
    if (spawn_ms > WSGAME_SPAWN_MAX)
    {
        spawn_ms = WSGAME_SPAWN_MAX;
    }

    s_spawn_ms = spawn_ms;
}

/*---------------------------------------------------------------------------*/
uint16_t wsgame_get_spawn_ms(void)
{
    return s_spawn_ms;
}

/*---------------------------------------------------------------------------*/
void wsgame_set_shot_ms(uint16_t shot_step_ms)
{
    if (shot_step_ms < WSGAME_SHOT_STEP_MIN)
    {
        shot_step_ms = WSGAME_SHOT_STEP_MIN;
    }
    if (shot_step_ms > WSGAME_SHOT_STEP_MAX)
    {
        shot_step_ms = WSGAME_SHOT_STEP_MAX;
    }

    s_shot_step_ms = shot_step_ms;
}

/*---------------------------------------------------------------------------*/
uint16_t wsgame_get_shot_ms(void)
{
    return s_shot_step_ms;
}

/*---------------------------------------------------------------------------*/
void wsgame_get_info(wsgame_info_t *info)
{
    if (info == NULL)
    {
        return;
    }

    info->running       = s_running;
    info->state         = (s_running != 0U) ? (uint8_t)WSGAME_ST_RUN : (uint8_t)WSGAME_ST_OFF;
    info->blocks        = s_blocks;
    info->shots         = s_shots;
    info->track_step_ms = s_track_step_ms;
    info->spawn_ms      = s_spawn_ms;
    info->shot_step_ms  = s_shot_step_ms;
    info->score         = s_score;
    info->hits          = s_hits;
    info->misses        = s_misses;
    info->spawned       = s_spawned;
}

/*---------------------------------------------------------------------------*/
const char *wsgame_state_name(uint8_t state)
{
    switch (state)
    {
        case (uint8_t)WSGAME_ST_OFF: return "OFF";
        case (uint8_t)WSGAME_ST_RUN: return "RUN";
        default:                     return "?";
    }
}

/*---------------------------------------------------------------------------*/
const char *wsgame_color_name(uint8_t color)
{
    switch (color)
    {
        case WSGAME_COLOR_BLUE:   return "BLUE";
        case WSGAME_COLOR_GREEN:  return "GREEN";
        case WSGAME_COLOR_PURPLE: return "PURPLE";
        default:                  return "NONE";
    }
}

/*---------------------------------------------------------------------------*/
void wsgame_process(void)
{
    key_msg_t msg;
    uint8_t   guard;

    if (s_inited == 0U)
    {
        return;
    }

    /* ① 先消费命令任务递过来的请求 */
    if (s_req != WSGAME_REQ_NONE)
    {
        uint8_t req = s_req;

        s_req = WSGAME_REQ_NONE;              /* 先清再执行: 执行期间来的新请求不会丢 */
        if (req == WSGAME_REQ_START)
        {
            game_start();
        }
        else
        {
            game_stop();
        }
    }

    /* ② 按键事件: Key4 开关, 颜色键发射 */
    while (drv_key_get_event(&msg) != 0U)
    {
        game_key(&msg);
    }

    if (s_running == 0U)
    {
        return;                               /* 没在跑: 灯带归效果层, 本拍收工 */
    }

    /* ③ 按键灯的"发射闪烁"到点就恢复常亮 */
    if ((s_keyled_flash != 0U) &&
        (bsp_elapsed_ms(s_keyled_flash_ms) >= WSGAME_KEYLED_FLASH_MS))
    {
        s_keyled_flash = 0U;
        keyled_apply();
    }

    /* ④ 运输带: 每 s_track_step_ms 整体往近端挪一格
          (最多补 8 拍: 任务被卡住时能追上, 又不会一帧挪一大截) */
    guard = 0U;
    while ((bsp_elapsed_ms(s_track_base_ms) >= s_track_step_ms) && (guard < 8U))
    {
        s_track_base_ms += s_track_step_ms;
        track_tick();
        guard++;
    }

    /* ⑤ 远端口子: 每 s_spawn_ms 补一颗新灯(前一颗没消掉就紧跟着再来一颗) */
    guard = 0U;
    while ((bsp_elapsed_ms(s_spawn_base_ms) >= s_spawn_ms) && (guard < 4U))
    {
        s_spawn_base_ms += s_spawn_ms;
        block_spawn();
        guard++;
    }

    /* ⑥ 子弹: 每 s_shot_step_ms 整体往外飞一格 */
    guard = 0U;
    while ((bsp_elapsed_ms(s_shot_base_ms) >= s_shot_step_ms) && (guard < 8U))
    {
        s_shot_base_ms += s_shot_step_ms;
        shots_step();
        guard++;
    }

    /* ⑦ 画一帧: 有变化立刻画, 否则最多 WSGAME_FRAME_MS 一帧 */
    if ((s_need_render != 0U) || (bsp_elapsed_ms(s_render_ms) >= WSGAME_FRAME_MS))
    {
        game_render();
    }
}
