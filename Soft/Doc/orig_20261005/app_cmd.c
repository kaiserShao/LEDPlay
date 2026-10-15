/**
  ******************************************************************************
  * @file    app_cmd.c
  * @brief   业务层: 远程命令解析/应答 + 按键事件上报
  ******************************************************************************
  */
#include "app_cmd.h"
#include "drv_esp32.h"
#include "drv_key.h"
#include "drv_power.h"
#include "drv_ws2812.h"
#include "drv_keyled.h"
#include "drv_hc595.h"
#include "drv_wt2003h.h"
#include "app_led_fx.h"
#include "app_wsgame.h"
#include "bsp_time.h"
#include "bsp_util.h"
#include <stdio.h>
#include <stdarg.h>

#define CMD_TOKEN_MAX      8U

static uint16_t s_heartbeat_ms = 0U;    /* 0 = 不上报心跳 */
static uint32_t s_last_hb_ms   = 0U;
static uint32_t s_boot_ms      = 0U;

/*---------------------------------------------------------------------------*/
/** 以空格/制表符切分命令行(原地修改), 返回参数个数 */
static uint8_t cmd_tokenize(char *line, char *tok[], uint8_t max)
{
    uint8_t n = 0U;
    char   *p = line;

    while ((*p != '\0') && (n < max))
    {
        while ((*p == ' ') || (*p == '\t'))
        {
            p++;
        }
        if (*p == '\0')
        {
            break;
        }
        tok[n] = p;
        n++;

        while ((*p != '\0') && (*p != ' ') && (*p != '\t'))
        {
            p++;
        }
        if (*p != '\0')
        {
            *p = '\0';
            p++;
        }
    }
    return n;
}

/*---------------------------------------------------------------------------*/
static uint8_t arg_u8(const char *s)
{
    uint32_t v = bsp_atoi(s);

    return (uint8_t)((v > 255U) ? 255U : v);
}

/*---------------------------------------------------------------------------*/
static uint16_t arg_u16(const char *s, uint16_t max)
{
    uint32_t v = bsp_atoi(s);

    return (uint16_t)((v > (uint32_t)max) ? max : v);
}

/*---------------------------------------------------------------------------*/
/** 十六进制串 -> 字节流(追加式): 每两个十六进制字符合成一个字节,
 *  其余字符(空格 / "0x" 的 x / 逗号 / 冒号)当分隔符, 落单的半字节忽略。
 *  例: "7E0441" / "7E 04 41" / "0x7E,0x04" 都能解析。
 *  @return 追加后的总字节数(<= out_max) */
static uint8_t hex_append(const char *s, uint8_t *out, uint8_t out_max, uint8_t len)
{
    uint8_t nib = 0xFFU;        /* 高 4 位暂存; 0xFF = 还没有半个字节 */

    while ((*s != '\0') && (len < out_max))
    {
        char    c = *s++;
        uint8_t v;

        if ((c >= '0') && (c <= '9'))
        {
            v = (uint8_t)(c - '0');
        }
        else if ((c >= 'a') && (c <= 'f'))
        {
            v = (uint8_t)(c - 'a' + 10);
        }
        else if ((c >= 'A') && (c <= 'F'))
        {
            v = (uint8_t)(c - 'A' + 10);
        }
        else
        {
            nib = 0xFFU;        /* 分隔符: 断开重来 */
            continue;
        }

        if (nib == 0xFFU)
        {
            nib = v;
        }
        else
        {
            out[len] = (uint8_t)((uint8_t)(nib << 4) | v);
            len++;
            nib = 0xFFU;
        }
    }
    return len;
}

/*---------------------------------------------------------------------------*/
static const char s_hex_map[17] = "0123456789ABCDEF";

/** 字节流 -> 大写十六进制字符串(buf 至少 2*len+1 字节) */
static void hex_encode(const uint8_t *in, uint8_t len, char *buf)
{
    uint8_t i;

    for (i = 0U; i < len; i++)
    {
        buf[2U * i]        = s_hex_map[(in[i] >> 4) & 0x0FU];
        buf[(2U * i) + 1U] = s_hex_map[in[i] & 0x0FU];
    }
    buf[2U * len] = '\0';
}

/*---------------------------------------------------------------------------*/
static const char *key_event_name(uint8_t ev)
{
    switch (ev)
    {
        case (uint8_t)KEY_EV_DOWN:  return "DOWN";
        case (uint8_t)KEY_EV_UP:    return "UP";
        case (uint8_t)KEY_EV_SHORT: return "SHORT";
        case (uint8_t)KEY_EV_LONG:  return "LONG";
        case (uint8_t)KEY_EV_HOLD:  return "HOLD";
        default:                    return "?";
    }
}

/*---------------------------------------------------------------------------*/
/** 统一应答: 格式化 + 自动追加 "\r\n", 一次发送完成(防止多任务插队) */
static void cmd_reply(const char *fmt, ...)
{
    char    buf[132];
    va_list ap;
    int     n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf) - 2U, fmt, ap);   /* 预留 2 字节给 "\r\n" */
    va_end(ap);

    if (n < 0)
    {
        return;
    }
    if (n > (int)(sizeof(buf) - 3U))
    {
        n = (int)(sizeof(buf) - 3U);
    }

    buf[n]     = '\r';
    buf[n + 1] = '\n';
    n += 2;

    esp32_send((const uint8_t *)buf, (uint16_t)n);
}

/*---------------------------------------------------------------------------*/
void app_cmd_report_event(const char *name, int32_t value)
{
    if (name == NULL)
    {
        return;
    }
    cmd_reply("EVT %s %d", name, (int)value);
}

/*---------------------------------------------------------------------------*/
/** 按键事件: 本地响应 + 上报给 WiFi 端 */
static void app_key_callback(const key_msg_t *msg)
{
    if (msg == NULL)
    {
        return;
    }

    /* ---- 本地立即响应(可自由改成项目需要的交互) ---- */
    /* 灯带消消乐运行时, 按键灯由 app_wsgame 统一管理(蓝/绿/紫 + 发射闪烁),
       这里就不要再动它, 否则会被这个按下反馈覆盖掉。 */
    if (wsgame_is_running() == 0U)
    {
        if (msg->event == (uint8_t)KEY_EV_DOWN)
        {
            /* 按下时把对应按键灯点亮 */
            keyled_set_color(msg->id, 0U, 60U, 255U);
        }
        else if ((msg->event == (uint8_t)KEY_EV_UP) || (msg->event == (uint8_t)KEY_EV_SHORT))
        {
            keyled_set_color(msg->id, 0U, 0U, 0U);
        }
        else
        {
            /* 长按/保持交给业务层 */
        }
    }

    /* ---- 上报 ---- */
    cmd_reply("EVT KEY %u %s %u", (unsigned)msg->id, key_event_name(msg->event),
              (unsigned)msg->duration_ms);
}

/*---------------------------------------------------------------------------*/
/** 电源键短按 */
static void app_power_short_callback(void)
{
    cmd_reply("EVT POWER SHORT");
}

/*---------------------------------------------------------------------------*/
/** 关机前处理: 播提示音 + 关灯 */
static void app_shutdown_callback(void)
{
    cmd_reply("EVT POWER OFF");
    (void)wt2003_talk(10U);           /* 例: 编号 10 = "正在关机" */
    wsgame_stop();                    /* 灯带消消乐(如果正在跑)也收摊 */
    led_fx_set(LED_FX_OFF);
    keyled_off();
    hc595_clear();
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  效果层/调试命令要直接控制灯带时, 先把正在跑的"灯带消消乐"停掉
  * @retval 1 = 原来游戏在跑, 已请求停止(应答里会带一个提示); 0 = 游戏本来没跑
  * @note   游戏每 16ms 会重画整条灯带, 不停掉的话 FX/LED/FILL 的效果下一拍就没了。
  */
static uint8_t game_release_strip(void)
{
    if (wsgame_is_running() == 0U)
    {
        return 0U;
    }
    wsgame_stop();
    return 1U;
}

/*===========================================================================
 *                              命令分发
 *=========================================================================*/
static void app_cmd_dispatch(const char *line)
{
    static char buf[ESP32_LINE_MAX + 1U];
    char       *tok[CMD_TOKEN_MAX];
    uint8_t     argc;
    uint32_t    n;

    if (line == NULL)
    {
        return;
    }

    /* 复制到本地再切分(要在原地写 '\0' 并转大写) */
    n = 0U;
    while ((line[n] != '\0') && (n < ESP32_LINE_MAX))
    {
        buf[n] = line[n];
        n++;
    }
    buf[n] = '\0';

    argc = cmd_tokenize(buf, tok, CMD_TOKEN_MAX);
    if (argc == 0U)
    {
        return;
    }
    bsp_str_upper(tok[0]);

    /*------------------------- 连通性测试 -------------------------*/
    if (bsp_stricmp_eq(tok[0], "PING") != 0U)
    {
        cmd_reply("+PONG");
        return;
    }

    /*------------------------- 状态查询 ---------------------------*/
    if ((bsp_stricmp_eq(tok[0], "STATUS") != 0U) || (tok[0][0] == '?'))
    {
        uint8_t mask = 0U;
        uint8_t i;
        uint8_t r;
        uint8_t g;
        uint8_t b;

        for (i = 0U; i < drv_key_get_count(); i++)
        {
            if (drv_key_is_down(i) != 0U)
            {
                mask = (uint8_t)(mask | (1U << i));
            }
        }
        led_fx_get_color(&r, &g, &b);

        cmd_reply("STATUS fx=%s brt=%u spd=%u rgb=%u,%u,%u keys=0x%02X up=%u frame=%u rx=%u ovf=%u",
                  led_fx_name(led_fx_get()), (unsigned)ws2812_get_brightness(),
                  (unsigned)led_fx_get_speed(), (unsigned)r, (unsigned)g, (unsigned)b,
                  (unsigned)mask, (unsigned)(bsp_time_ms() - s_boot_ms),
                  (unsigned)ws2812_frame_count(), (unsigned)esp32_rx_total(),
                  (unsigned)esp32_rx_overflows());
        return;
    }

    /*------------------------- 灯带颜色 ---------------------------*/
    if (bsp_stricmp_eq(tok[0], "FILL") != 0U)
    {
        uint8_t game_on;

        if (argc < 4U)
        {
            cmd_reply("ERR FILL ARGS");
            return;
        }
        game_on = game_release_strip();
        led_fx_set_color(arg_u8(tok[1]), arg_u8(tok[2]), arg_u8(tok[3]));
        led_fx_set(LED_FX_SOLID);
        cmd_reply("OK FILL %u %u %u%s", (unsigned)arg_u8(tok[1]), (unsigned)arg_u8(tok[2]),
                  (unsigned)arg_u8(tok[3]), (game_on != 0U) ? " (game stopped)" : "");
        return;
    }

    if (bsp_stricmp_eq(tok[0], "LED") != 0U)
    {
        uint8_t game_on;

        if (argc < 6U)
        {
            cmd_reply("ERR LED ARGS");
            return;
        }
        game_on = game_release_strip();
        ws2812_lock();      /* 与灯光任务互斥, 避免改到一半被动画覆盖 */
        ws2812_fill_range(arg_u16(tok[1], WS2812_LED_NUM - 1U), arg_u16(tok[2], WS2812_LED_NUM),
                          arg_u8(tok[3]), arg_u8(tok[4]), arg_u8(tok[5]));
        ws2812_show();
        ws2812_unlock();
        cmd_reply("OK LED %u %u%s", (unsigned)bsp_atoi(tok[1]), (unsigned)bsp_atoi(tok[2]),
                  (game_on != 0U) ? " (game stopped)" : "");
        return;
    }

    if (bsp_stricmp_eq(tok[0], "FX") != 0U)
    {
        uint8_t game_on;

        if (argc < 2U)
        {
            cmd_reply("ERR FX ARGS");
            return;
        }
        game_on = game_release_strip();
        led_fx_set((led_fx_t)arg_u8(tok[1]));
        cmd_reply("OK FX %u %s%s", (unsigned)led_fx_get(), led_fx_name(led_fx_get()),
                  (game_on != 0U) ? " (game stopped)" : "");
        return;
    }

    if (bsp_stricmp_eq(tok[0], "BRT") != 0U)
    {
        if (argc < 2U)
        {
            cmd_reply("ERR BRT ARGS");
            return;
        }
        ws2812_set_brightness(arg_u8(tok[1]));
        if (wsgame_is_running() == 0U)
        {
            /* 重新渲染一帧; 游戏在跑时不必: 它 16ms 内就会用新亮度重画整条 */
            led_fx_set(led_fx_get());
        }
        cmd_reply("OK BRT %u", (unsigned)ws2812_get_brightness());
        return;
    }

    if (bsp_stricmp_eq(tok[0], "SPD") != 0U)
    {
        if (argc < 2U)
        {
            cmd_reply("ERR SPD ARGS");
            return;
        }
        led_fx_set_speed(arg_u16(tok[1], 8000U));
        cmd_reply("OK SPD %u", (unsigned)led_fx_get_speed());
        return;
    }

    /*------------------------- 灯带消消乐 -------------------------*/
    /* GAME / GAME STATUS : 查询(运行状态 / 场上灯数 / 在飞子弹 / 三个速度 / 得分)
       GAME START  : 开局(重开一局, 分数清零; 灯带从效果层交给游戏)
       GAME STOP   : 收局(清屏 + 按键灯灭, 灯带交还效果层)
       GAME SPD ms : 运输带速度 = 灯每挪一颗的时间(20~500ms, 越大越慢)
       GAME SPAWN ms : 远端口子 = 每多少毫秒冒一颗新灯(100~5000ms)
       GAME SHOT ms  : 子弹速度 = 每飞一颗灯的时间(5~100ms, 越小越快)
       注: 板子上 Key1/2/3 发射 蓝/绿/紫 子弹, Key4(PA3) 短按 = 开始/暂停。 */
    if (bsp_stricmp_eq(tok[0], "GAME") != 0U)
    {
        wsgame_info_t info;

        wsgame_get_info(&info);

        if ((argc < 2U) || (bsp_stricmp_eq(tok[1], "STATUS") != 0U))
        {
            cmd_reply("GAME %s state=%s blocks=%u shots=%u step=%ums spawn=%ums shot=%ums "
                      "score=%u hits=%u miss=%u spawned=%u",
                      (info.running != 0U) ? "RUN" : "STOP", wsgame_state_name(info.state),
                      (unsigned)info.blocks, (unsigned)info.shots,
                      (unsigned)info.track_step_ms, (unsigned)info.spawn_ms,
                      (unsigned)info.shot_step_ms, (unsigned)info.score, (unsigned)info.hits,
                      (unsigned)info.misses, (unsigned)info.spawned);
            return;
        }

        if (bsp_stricmp_eq(tok[1], "START") != 0U)
        {
            wsgame_start();
            cmd_reply("OK GAME START (step=%ums spawn=%ums shot=%ums maxshots=%u)",
                      (unsigned)wsgame_get_step_ms(), (unsigned)wsgame_get_spawn_ms(),
                      (unsigned)wsgame_get_shot_ms(), (unsigned)WSGAME_MAX_SHOTS);
            return;
        }

        if (bsp_stricmp_eq(tok[1], "STOP") != 0U)
        {
            wsgame_stop();
            cmd_reply("OK GAME STOP (strip back to fx)");
            return;
        }

        if (bsp_stricmp_eq(tok[1], "SPD") != 0U)
        {
            if (argc < 3U)
            {
                cmd_reply("ERR GAME SPD ARGS (ms %u..%u)", (unsigned)WSGAME_TRACK_STEP_MIN,
                          (unsigned)WSGAME_TRACK_STEP_MAX);
                return;
            }
            wsgame_set_step_ms(arg_u16(tok[2], 0xFFFFU));
            cmd_reply("OK GAME SPD %u", (unsigned)wsgame_get_step_ms());
            return;
        }

        if (bsp_stricmp_eq(tok[1], "SPAWN") != 0U)
        {
            if (argc < 3U)
            {
                cmd_reply("ERR GAME SPAWN ARGS (ms %u..%u)", (unsigned)WSGAME_SPAWN_MIN,
                          (unsigned)WSGAME_SPAWN_MAX);
                return;
            }
            wsgame_set_spawn_ms(arg_u16(tok[2], 0xFFFFU));
            cmd_reply("OK GAME SPAWN %u", (unsigned)wsgame_get_spawn_ms());
            return;
        }

        if (bsp_stricmp_eq(tok[1], "SHOT") != 0U)
        {
            if (argc < 3U)
            {
                cmd_reply("ERR GAME SHOT ARGS (ms %u..%u)", (unsigned)WSGAME_SHOT_STEP_MIN,
                          (unsigned)WSGAME_SHOT_STEP_MAX);
                return;
            }
            wsgame_set_shot_ms(arg_u16(tok[2], 0xFFFFU));
            cmd_reply("OK GAME SHOT %u", (unsigned)wsgame_get_shot_ms());
            return;
        }

        cmd_reply("ERR GAME ARGS (STATUS/START/STOP/SPD/SPAWN/SHOT)");
        return;
    }

    /*------------------------- 语音模块 ---------------------------*/
    if (bsp_stricmp_eq(tok[0], "VOICE") != 0U)
    {
        uint16_t id;

        if (argc < 2U)
        {
            cmd_reply("ERR VOICE ARGS");
            return;
        }
        id = arg_u16(tok[1], 9999U);
        if (wt2003_talk(id) == 0U)
        {
            cmd_reply("OK VOICE %u %s", (unsigned)id,
                      (wt2003_get_key_of(id) != 0xFFU) ? "KEY" : "UART");
        }
        else
        {
            cmd_reply("ERR VOICE %u", (unsigned)id);
        }
        return;
    }

    if (bsp_stricmp_eq(tok[0], "VKEY") != 0U)
    {
        if (argc < 2U)
        {
            cmd_reply("ERR VKEY ARGS");
            return;
        }
        /* 标准品(WT2003H4-16S)没开放 Key 口功能(需厂家定制) -> 明确回错,
           不要给"命令成功、但一点反应都没有"的假象 */
        if (wt2003_key_available() == 0U)
        {
            cmd_reply("ERR VKEY unavailable (WT2003_KEY_ENABLE=0, use VPLAY)");
            return;
        }
        wt2003_key_pulse((uint8_t)arg_u16(tok[1], 2U));
        cmd_reply("OK VKEY %u", (unsigned)bsp_atoi(tok[1]));
        return;
    }

    /* VPLAY6 <index> : 强制用 A6(指定 U 盘根目录索引播放) —— 手册 4.1.3.4:
       帧 7E 05 A6 <曲目高位> <曲目低位> <校验码> EF(校验码自动算),
       例 索引2 -> 7E 05 A6 00 02 AD EF。和 VPLAY0(A0, SPI Flash)对照,
       确认音频到底在哪块介质上(U 盘用 A0 播会回非 0)。 */
    if (bsp_stricmp_eq(tok[0], "VPLAY6") != 0U)
    {
        uint16_t idx;

        if (argc < 2U)
        {
            cmd_reply("ERR VPLAY6 ARGS (index 0..65535)");
            return;
        }
        idx = arg_u16(tok[1], 0xFFFFU);
        if (wt2003_play_udisk(idx) == 0U)
        {
            cmd_reply("OK VPLAY6 %u (A6 U-disk, see WTRX)", (unsigned)idx);
        }
        else
        {
            cmd_reply("ERR VPLAY6 %u", (unsigned)idx);
        }
        return;
    }

    /* VPLAY0 <index> : 强制用 A0(指定 SPI Flash 根目录索引播放) */
    if (bsp_stricmp_eq(tok[0], "VPLAY0") != 0U)
    {
        uint16_t idx;

        if (argc < 2U)
        {
            cmd_reply("ERR VPLAY0 ARGS (index 0..65535)");
            return;
        }
        idx = arg_u16(tok[1], 0xFFFFU);
        if (wt2003_play_flash(idx) == 0U)
        {
            cmd_reply("OK VPLAY0 %u (A0 flash, see WTRX)", (unsigned)idx);
        }
        else
        {
            cmd_reply("ERR VPLAY0 %u", (unsigned)idx);
        }
        return;
    }

    /* VPLAY <index> : 按索引播放(命令码由 bsp_config.h 的 WT2003_PLAY_USE_UDISK 选:
       1 = A6 播 U 盘根目录索引【默认】, 0 = A0 播 SPI Flash; 校验码自动算),
       方便"0,1,2,3… 逐个索引扫一遍"确认模块里文件的实际索引号
       (索引顺序是出厂/上位机设定的, 不一定等于你预期的编号)。
       模块回帧看 ESP32 串口的 WTRX/WTRXINFO 行: 数据字节 00 = 已接受, 非 0 = 没播
       -> 先查文件数/索引, 见 README 8.2。
       想显式指定介质: VPLAY6 <idx>(A6) / VPLAY0 <idx>(A0)。 */
    if (bsp_stricmp_eq(tok[0], "VPLAY") != 0U)
    {
        uint16_t idx;

        if (argc < 2U)
        {
            cmd_reply("ERR VPLAY ARGS (index 0..65535)");
            return;
        }
        idx = arg_u16(tok[1], 0xFFFFU);
        if (wt2003_play_index(idx) == 0U)
        {
#if (WT2003_PLAY_USE_UDISK != 0U)
            cmd_reply("OK VPLAY %u (A6 U-disk, see WTRX)", (unsigned)idx);
#else
            cmd_reply("OK VPLAY %u (A0 flash, see WTRX)", (unsigned)idx);
#endif
        }
        else
        {
            cmd_reply("ERR VPLAY %u", (unsigned)idx);
        }
        return;
    }

    if (bsp_stricmp_eq(tok[0], "VOL") != 0U)
    {
        if (argc < 2U)
        {
            cmd_reply("ERR VOL ARGS");
            return;
        }
        (void)wt2003_set_volume((uint8_t)arg_u16(tok[1], 31U));
        cmd_reply("OK VOL %u", (unsigned)wt2003_get_volume());
        return;
    }

    /* VOUT 0 = SPK(内部功放/喇叭), 1 = DAC 输出 (B6 命令) */
    if (bsp_stricmp_eq(tok[0], "VOUT") != 0U)
    {
        if (argc < 2U)
        {
            cmd_reply("ERR VOUT ARGS (0=SPK 1=DAC)");
            return;
        }
        if (arg_u16(tok[1], 1U) != 0U)
        {
            (void)wt2003_set_output(WT2003_OUT_DAC);
            cmd_reply("OK VOUT DAC");
        }
        else
        {
            (void)wt2003_set_output(WT2003_OUT_SPK);
            cmd_reply("OK VOUT SPK");
        }
        return;
    }

    /* VQRY VER|VOL|ST|FILES|CUR : 发查询命令(C0/C1/C2/C3/C9), 应答看 WTRX */
    if (bsp_stricmp_eq(tok[0], "VQRY") != 0U)
    {
        if (argc < 2U)
        {
            cmd_reply("ERR VQRY ARGS (VER/VOL/ST/FILES/CUR)");
            return;
        }
        if (bsp_stricmp_eq(tok[1], "VER") != 0U)
        {
            (void)wt2003_query_version();
        }
        else if (bsp_stricmp_eq(tok[1], "VOL") != 0U)
        {
            (void)wt2003_query_volume();
        }
        else if (bsp_stricmp_eq(tok[1], "ST") != 0U)
        {
            (void)wt2003_query_status();
        }
        else if (bsp_stricmp_eq(tok[1], "FILES") != 0U)
        {
            (void)wt2003_query_file_num();
        }
        else if (bsp_stricmp_eq(tok[1], "CUR") != 0U)
        {
            (void)wt2003_query_cur_file();
        }
        else
        {
            cmd_reply("ERR VQRY ARGS (VER/VOL/ST/FILES/CUR)");
            return;
        }
        cmd_reply("OK VQRY %s (see WTRX)", tok[1]);
        return;
    }

    /* VCHECK : 一键体检 —— 依次发 8 条查询命令(C0 版本 / C1 音量 / C2 状态 /
       C3 文件数 / CA 外设连接 / C7 U 盘文件数 / C9 当前文件 / CB 歌曲名),
       注: 手册写 C3 是 SPI-Flash 口径, 但实测本模块 C3 == C7 同为 1 字节,
           别拿 C3 判介质, 看音频在哪块盘用 A0/A6 试听或看 CA/C7(见 README 8.9)。
       每条间隔 120ms 等模块回帧。
       芯片返回的原始字节 + 解析会通过 USART1 直接打到 ESP32:
           WTRXRAW   模块回帧的原始字节(7E 长度 命令 数据 校验码 EF)
           WTRX      命令码 + 数据区
           WTRXINFO  人话解析: volume= / flash_files= / conn= / udisk_files= /
                     cur_file= / name= / status=
       本项目音频在 U 盘 -> 重点看 udisk_files=(是 0 就没认到盘)。注意:
       查询命令回的是"数据"不是返回码; 写命令的返回码是 00=OK / 01=FAIL / 02=EMP(无此文件),
       而本模块实测 A6/D2/C7 会回手册没定义的 0x05 —— 那几轮音频照样出声, 别当成"没播"(见 README 8.7)。 */
    if (bsp_stricmp_eq(tok[0], "VCHECK") != 0U)
    {
        (void)wt2003_query_version();
        bsp_delay_ms(120U);
        (void)wt2003_query_volume();
        bsp_delay_ms(120U);
        (void)wt2003_query_status();
        bsp_delay_ms(120U);
        (void)wt2003_query_file_num();          /* C3: SPI-Flash 文件数(本工程恒为 0) */
        bsp_delay_ms(120U);
        (void)wt2003_query_conn();              /* CA: 外设连接状态(重点看 U 盘位 bit2) */
        bsp_delay_ms(120U);
        (void)wt2003_query_udisk_num();         /* C7: U 盘内文件总数 <== 本工程看这条 */
        bsp_delay_ms(120U);
        (void)wt2003_query_cur_file();          /* C9: 当前播放文件 */
        bsp_delay_ms(120U);
        (void)wt2003_query_cur_name();          /* CB: 当前播放歌曲名 */
        cmd_reply("OK VCHECK (see WTRX/WTRXINFO)");
        return;
    }

    if ((bsp_stricmp_eq(tok[0], "VSTOP") != 0U) || (bsp_stricmp_eq(tok[0], "VPAUSE") != 0U)
        || (bsp_stricmp_eq(tok[0], "VNEXT") != 0U) || (bsp_stricmp_eq(tok[0], "VPREV") != 0U))
    {
        if (bsp_stricmp_eq(tok[0], "VSTOP") != 0U)
        {
            (void)wt2003_stop();
        }
        else if (bsp_stricmp_eq(tok[0], "VPAUSE") != 0U)
        {
            (void)wt2003_pause();
        }
        else if (bsp_stricmp_eq(tok[0], "VNEXT") != 0U)
        {
            (void)wt2003_play_next();
        }
        else
        {
            (void)wt2003_play_prev();
        }
        cmd_reply("OK %s", tok[0]);
        return;
    }

    /* VNAME : 按文件名播放/轮播(A7 播"U 盘根目录里 4 字符文件名"的音频)。
       为什么用它: 实测本模块 A6(指定 U 盘索引播放)收下命令却不出声(回 0x05),
       而 A7 按文件名播放能出声(见 README 8.9)。名字列表在 bsp_config.h 的
       WT2003_NAME_CYCLE_LIST, 例如 { "T001","T002" } 对应 U 盘上的 T001.mp3 / T002.mp3。
         VNAME            : 列出配置里的文件名(第几个 = 第 n 项)
         VNAME <n>        : 立刻播第 n 个(1..N), 数码管显示 n, 串口打 NAME n/N "T00n"
         VNAMEON/VNAMEOFF : 开/关"每 WT2003_NAME_CYCLE_MS 毫秒播下一个"的自动轮播 */
    if (bsp_stricmp_eq(tok[0], "VNAME") != 0U)
    {
        if (argc < 2U)
        {
            uint8_t i;
            uint8_t total = wt2003_name_count();

            cmd_reply("OK VNAME n=%u every=%ums (A7 by file name)", (unsigned)total,
                      (unsigned)WT2003_NAME_CYCLE_MS);
            for (i = 1U; i <= total; i++)
            {
                cmd_reply("NAME %u \"%s\"", (unsigned)i, wt2003_name_get(i));
            }
            return;
        }

        {
            uint16_t n = arg_u16(tok[1], 0U);

            if ((n == 0U) || (n > (uint16_t)wt2003_name_count())
                || (wt2003_name_fire((uint8_t)n) != 0U))
            {
                cmd_reply("ERR VNAME %u (1..%u)", (unsigned)n,
                          (unsigned)wt2003_name_count());
                return;
            }
            cmd_reply("OK VNAME %u = \"%s\" (A7 U-disk by name, see WTRX)",
                      (unsigned)n, wt2003_name_get((uint8_t)n));
        }
        return;
    }

    if ((bsp_stricmp_eq(tok[0], "VNAMEON") != 0U) || (bsp_stricmp_eq(tok[0], "VNAMEOFF") != 0U))
    {
        if (bsp_stricmp_eq(tok[0], "VNAMEON") != 0U)
        {
            wt2003_name_start();
            cmd_reply("OK VNAMEON %u files, %u ms each (A7 by file name)",
                      (unsigned)wt2003_name_count(), (unsigned)WT2003_NAME_CYCLE_MS);
        }
        else
        {
            wt2003_name_stop();
            cmd_reply("OK VNAMEOFF (VNAME <n> still works)");
        }
        return;
    }

    /*------------------------- 数码管 -----------------------------*/
    if (bsp_stricmp_eq(tok[0], "DISP") != 0U)
    {
        if (argc >= 2U)
        {
            hc595_show_digits(tok[1]);
            cmd_reply("OK DISP %s", tok[1]);
        }
        else
        {
            hc595_clear();
            cmd_reply("OK DISP CLEAR");
        }
        return;
    }

    /*------------------------- 语音模块调试 -----------------------*/
    /*免烧录反复试: 用这几条命令手工复现自检序列里的某一步。
       WTPROBE      : 不带参数 = 重新开始整轮(38 步)自检序列
       WTPROBE <n>  : 立刻单独发第 n 步 n=1..wt2003_probe_count()(提示文字见
                      wt2003_probe_cfg_hint(), 例: 6 = CA 查外设(有没有 U 盘), 7 = C7 查 U 盘文件数,
                      10 = A6 播 U 盘索引1, 25 = AC 下一曲, 29 = A6 播索引2, 31~38 = 返回码取证)
       WTSTOP       : 停止自动自检
       WTST         : 查询状态(模块回帧字节数 / 已发命令帧数 / 是否在自检) */
    /* WTRAW <hex...> : 把任意字节序列原样发给模块(不重烧就能试手册里的任意帧)。
       例: WTRAW 7E05A60001ACEF        (A6 播 U 盘索引 1；长度含自身 + 累加和)
           WTRAW 7E 05 A6 00 01 AC EF  (空格分隔, 也接受 0x7E / 逗号 / 冒号; A6 = 播 U 盘索引 1)
       发完看 ESP32 串口的 WTRX 行(WT2003_RX_TRACE=1 时)并听喇叭; 详见 README 第 8 节。 */
    if (bsp_stricmp_eq(tok[0], "WTRAW") != 0U)
    {
        uint8_t frame[WT2003_RX_FRAME_MAX];
        char    hex[(2U * WT2003_RX_FRAME_MAX) + 1U];
        uint8_t len = 0U;
        uint8_t i;

        for (i = 1U; (i < argc) && (len < (uint8_t)sizeof(frame)); i++)
        {
            len = hex_append(tok[i], frame, (uint8_t)sizeof(frame), len);
        }

        if (len == 0U)
        {
            cmd_reply("ERR WTRAW HEX (e.g. WTRAW 7E05A00001A6EF)");
            return;
        }

        hex_encode(frame, len, hex);
        if (wt2003_send_raw(frame, len) != 0U)
        {
            cmd_reply("ERR WTRAW TX %s", hex);
            return;
        }
        cmd_reply("OK WTRAW %s (%u bytes)", hex, (unsigned)len);
        return;
    }

    if (bsp_stricmp_eq(tok[0], "WTPROBE") != 0U)
    {
        if (argc < 2U)
        {
            wt2003_probe_start();
            cmd_reply("OK WTPROBE START n=%u", (unsigned)wt2003_probe_count());
            return;
        }

        if (wt2003_probe_fire(arg_u8(tok[1])) != 0U)
        {
            cmd_reply("ERR WTPROBE RANGE 1..%u", (unsigned)wt2003_probe_count());
            return;
        }
        cmd_reply("OK WTPROBE %u %s", (unsigned)arg_u8(tok[1]),
                  wt2003_probe_cfg_hint(arg_u8(tok[1])));
        return;
    }

    if (bsp_stricmp_eq(tok[0], "WTSTOP") != 0U)
    {
        wt2003_probe_stop();
        cmd_reply("OK WTSTOP");
        return;
    }

    if (bsp_stricmp_eq(tok[0], "WTST") != 0U)
    {
        cmd_reply("WTST probe=%u rx=%lu cmd=%lu", (unsigned)wt2003_probe_is_running(),
                  (unsigned long)wt2003_rx_total(), (unsigned long)wt2003_cmd_count());
        return;
    }

    /*------------------------- 按键灯 -----------------------------*/
    /* 排查"按键灯发白"用的子命令(不需要颜色参数):
         KEYLED DIAG            打印实测时序 + 每一路的设定色/输出色
         KEYLED T0H <ns>        改 0 码高电平(80~600ns), 立即生效
         KEYLED T0L <ns>        改 0 码低电平(200~2000ns), 立即生效
         KEYLED T1H <ns>        改 1 码高电平(300~1600ns), 立即生效
         KEYLED T1L <ns>        改 1 码低电平(150~1000ns), 立即生效
         KEYLED INVERT <0|1>    数据线是否反相(线上串了反相器/三极管时用 1)
         KEYLED DIAG 里还会带一行线路自检: line low/high 读回值(都该和驱动一致)
       其余用法不变: KEYLED <0..3|ALL> r g b [闪烁周期ms] */
    if (bsp_stricmp_eq(tok[0], "KEYLED") != 0U)
    {
        if ((argc >= 2U) && (bsp_stricmp_eq(tok[1], "DIAG") != 0U))
        {
            keyled_timing_t t;
            uint8_t         i;

            keyled_get_timing_info(&t);

            /* 每位 = 高电平 + 低电平两段(没有固定位周期); ns 是灯珠应该看到的时间,
               cyc 是驱动实际用的 CPU 周期数(目标值已扣掉自标定实测的固定开销) */
            cmd_reply("KEYLED 0code H%uns(%lucyc comp%lu) L%uns(%lucyc comp%lu)",
                      (unsigned)t.t0h_ns, (unsigned long)t.t0h_cyc, (unsigned long)t.comp_t0h,
                      (unsigned)t.t0l_ns, (unsigned long)t.t0l_cyc, (unsigned long)t.comp_t0l);
            cmd_reply("KEYLED 1code H%uns(%lucyc comp%lu) L%uns(%lucyc comp%lu)",
                      (unsigned)t.t1h_ns, (unsigned long)t.t1h_cyc, (unsigned long)t.comp_t1h,
                      (unsigned)t.t1l_ns, (unsigned long)t.t1l_cyc, (unsigned long)t.comp_t1l);
            cmd_reply("KEYLED invert=%u calib=%u fclk=%lu", (unsigned)keyled_get_invert(),
                      (unsigned)keyled_calib_ok(), (unsigned long)SystemCoreClock);

            {
                uint8_t lo;
                uint8_t hi;

                /* 线路自检: 驱动低/高时, 引脚读回应该分别是 0/1 */
                keyled_probe_line(&lo, &hi);
                cmd_reply("KEYLED line low=%u high=%u (want 0/1)", (unsigned)lo, (unsigned)hi);
            }

            for (i = 0U; i < KEYLED_COUNT; i++)
            {
                uint8_t sr;
                uint8_t sg;
                uint8_t sb;
                uint8_t ur;
                uint8_t ug;
                uint8_t ub;

                keyled_get_set(i, &sr, &sg, &sb);
                keyled_get_output(i, &ur, &ug, &ub);
                cmd_reply("KEYLED %u set=%u,%u,%u out=%u,%u,%u", (unsigned)i,
                          (unsigned)sr, (unsigned)sg, (unsigned)sb,
                          (unsigned)ur, (unsigned)ug, (unsigned)ub);
            }
            return;
        }

        if ((argc >= 3U) &&
            ((bsp_stricmp_eq(tok[1], "T0H") != 0U) || (bsp_stricmp_eq(tok[1], "T0L") != 0U) ||
             (bsp_stricmp_eq(tok[1], "T1H") != 0U) || (bsp_stricmp_eq(tok[1], "T1L") != 0U)))
        {
            keyled_timing_t t;
            uint16_t        v = arg_u16(tok[2], 0U);   /* 0 = 用回配置默认值 */

            keyled_get_timing_info(&t);                /* 只改指定的那一项, 其余保持现值 */
            if (bsp_stricmp_eq(tok[1], "T0H") != 0U)
            {
                t.t0h_ns = v;
            }
            else if (bsp_stricmp_eq(tok[1], "T0L") != 0U)
            {
                t.t0l_ns = v;
            }
            else if (bsp_stricmp_eq(tok[1], "T1H") != 0U)
            {
                t.t1h_ns = v;
            }
            else
            {
                t.t1l_ns = v;
            }

            keyled_set_timing(t.t0h_ns, t.t0l_ns, t.t1h_ns, t.t1l_ns);
            keyled_get_timing_info(&t);
            cmd_reply("OK KEYLED T0H %u T0L %u T1H %u T1L %u",
                      (unsigned)t.t0h_ns, (unsigned)t.t0l_ns,
                      (unsigned)t.t1h_ns, (unsigned)t.t1l_ns);
            return;
        }

        if ((argc >= 3U) && (bsp_stricmp_eq(tok[1], "INVERT") != 0U))
        {
            keyled_set_invert(arg_u8(tok[2]));
            cmd_reply("OK KEYLED INVERT %u", (unsigned)keyled_get_invert());
            return;
        }

        {
            uint16_t period = (argc >= 6U) ? arg_u16(tok[5], 5000U) : 0U;

            if (argc < 5U)
            {
                cmd_reply("ERR KEYLED ARGS");
                return;
            }

            if (bsp_stricmp_eq(tok[1], "ALL") != 0U)
            {
                uint8_t i;

                for (i = 0U; i < KEYLED_COUNT; i++)
                {
                    keyled_set_fx(i, (period == 0U) ? KEYLED_FX_STATIC : KEYLED_FX_BLINK,
                                  arg_u8(tok[2]), arg_u8(tok[3]), arg_u8(tok[4]), period);
                }
            }
            else
            {
                keyled_set_fx((uint8_t)arg_u16(tok[1], KEYLED_COUNT - 1U),
                              (period == 0U) ? KEYLED_FX_STATIC : KEYLED_FX_BLINK,
                              arg_u8(tok[2]), arg_u8(tok[3]), arg_u8(tok[4]), period);
            }
        }
        cmd_reply("OK KEYLED");
        return;
    }

    /*------------------------- 心跳周期 ---------------------------*/
    if (bsp_stricmp_eq(tok[0], "HEARTBEAT") != 0U)
    {
        s_heartbeat_ms = (argc >= 2U) ? arg_u16(tok[1], 60000U) : 0U;
        s_last_hb_ms   = bsp_time_ms();
        cmd_reply("OK HEARTBEAT %u", (unsigned)s_heartbeat_ms);
        return;
    }

    /*------------------------- 关机 -------------------------------*/
    if (bsp_stricmp_eq(tok[0], "POWER") != 0U)
    {
        if ((argc >= 2U) && (bsp_stricmp_eq(tok[1], "OFF") != 0U))
        {
            cmd_reply("OK POWER OFF");
            power_request_shutdown();        /* 走正常关机流程(会断电) */
            return;
        }
        cmd_reply("ERR POWER ARGS");
        return;
    }

    /*------------------------- 未知命令 ---------------------------*/
    cmd_reply("ERR UNKNOWN %s", tok[0]);
}

/*===========================================================================
 *                              初始化 / 周期
 *=========================================================================*/
static void app_esp32_line_handler(const char *line)
{
    app_cmd_dispatch(line);
}

/*---------------------------------------------------------------------------*/
void app_cmd_init(void)
{
    s_boot_ms      = bsp_time_ms();
    s_last_hb_ms   = s_boot_ms;
    s_heartbeat_ms = 0U;

    esp32_set_rx_handler(app_esp32_line_handler);
    drv_key_set_callback(app_key_callback);
    power_set_short_press_handler(app_power_short_callback);
    power_set_shutdown_handler(app_shutdown_callback);

    cmd_reply("READY LEDAnipop fw=1.0 led=%u", (unsigned)WS2812_LED_NUM);
}

/*---------------------------------------------------------------------------*/
void app_cmd_process(void)
{
    if (s_heartbeat_ms == 0U)
    {
        return;
    }
    if (bsp_elapsed_ms(s_last_hb_ms) < s_heartbeat_ms)
    {
        return;
    }
    s_last_hb_ms = bsp_time_ms();

    cmd_reply("HB %u %s %u", (unsigned)(s_last_hb_ms - s_boot_ms), led_fx_name(led_fx_get()),
              (unsigned)ws2812_get_brightness());
}
