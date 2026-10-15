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

/* 语音模块调试用的 hex 解析辅助函数(hex_append / hex_encode)随 WTRAW 命令一起退役, */
/* 留档见 Doc/WT2003_退役命令与自检备份.md */
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
    (void)wt2003_play_name(WT2003_FILE_POWER_OFF);   /* 关机提示音: 0005.mp3 */
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
    /* 只保留 4 条命令(命令表/帧格式见 bsp_config.h 第 8 节、drv_wt2003h.h):
         VOL <0..31>        设置音量                        (0xAE)
         VNUM               查询文件总数                    (0xC4)
         VPLAY <index>      按 U 盘根目录索引播放           (0x9D)
         VOICE <file4>      按文件名播放                    (0x9E)
         VNAME <file4>      同 VOICE(名字见 bsp_config.h 的 WT2003_FILE_xxx)
       想看发/收帧: bsp_config.h 的 WT2003_TRACE = 1(串口打 WTTX/WTRX/WTRXCOUNT)。 */
    if (bsp_stricmp_eq(tok[0], "VOL") != 0U)
    {
        if (argc < 2U)
        {
            cmd_reply("ERR VOL ARGS (0..%u)", (unsigned)WT2003_VOL_MAX);
            return;
        }
        (void)wt2003_set_volume((uint8_t)arg_u16(tok[1], (uint16_t)WT2003_VOL_MAX));
        cmd_reply("OK VOL %u (0xAE)", (unsigned)wt2003_get_volume());
        return;
    }

    /* VNUM: 发 0xC4 查文件总数 —— 结果几毫秒后由驱动解析, 看串口 WTRXCOUNT 行;
       这里顺手把"上一次查到的值"回给上位机(第一次查时是 0)。 */
    if (bsp_stricmp_eq(tok[0], "VNUM") != 0U)
    {
        (void)wt2003_query_count();
        cmd_reply("OK VNUM last=%u (0xC4, new answer on WTRXCOUNT)",
                  (unsigned)wt2003_get_count());
        return;
    }

    if (bsp_stricmp_eq(tok[0], "VPLAY") != 0U)
    {
        uint16_t idx;

        if (argc < 2U)
        {
            cmd_reply("ERR VPLAY ARGS (index 1..65535)");
            return;
        }
        idx = arg_u16(tok[1], 0xFFFFU);
        if (wt2003_play_index(idx) != 0U)
        {
            cmd_reply("ERR VPLAY %u", (unsigned)idx);
            return;
        }
        cmd_reply("OK VPLAY %u (0x9D by root index)", (unsigned)idx);
        return;
    }

    /* VOICE <file4> / VNAME <file4>: 按文件名播放(0x9E), 文件放 U 盘根目录、名字 4 个字符。
       不带参数 = 列出 bsp_config.h 里已经定好的名字, 方便和 U 盘里的文件核对。 */
    if ((bsp_stricmp_eq(tok[0], "VOICE") != 0U) || (bsp_stricmp_eq(tok[0], "VNAME") != 0U))
    {
        if (argc < 2U)
        {
            cmd_reply("VOICE <file4>   KEY=%s CLEAR=%s START=%s OVER=%s POWEROFF=%s BOOT=%s",
                      WT2003_FILE_KEY, WT2003_FILE_CLEAR, WT2003_FILE_START,
                      WT2003_FILE_OVER, WT2003_FILE_POWER_OFF, WT2003_BOOT_FILE);
            return;
        }
        if ((tok[1][0] == '\0') || (tok[1][1] == '\0') || (tok[1][2] == '\0')
            || (tok[1][3] == '\0') || (tok[1][4] != '\0'))
        {
            cmd_reply("ERR VOICE \"%s\" (name must be exactly 4 chars, e.g. 0001)", tok[1]);
            return;
        }
        if (wt2003_play_name(tok[1]) != 0U)
        {
            cmd_reply("ERR VOICE \"%s\" TX", tok[1]);
            return;
        }
        cmd_reply("OK VOICE \"%s\" (0x9E by file name)", tok[1]);
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

    /* 语音模块调试命令(WTRAW / WTPROBE / WTSTOP / WTST)已随自检一起退役:
       要看收发帧用 bsp_config.h 的 WT2003_TRACE = 1 打 WTTX/WTRX;
       想临时试手册里的任意帧, 可以在驱动里直接调 wt2003_send_cmd(cmd, data, len)。
       退役代码留档见 Doc/WT2003_退役命令与自检备份.md */

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
               cyc 是驱动实际用的计数拍数(定时器方式: 1 拍 = 13.89ns; 老 DWT 方式: 1 个 CPU 周期, 已扣标定开销) */
            cmd_reply("KEYLED 0code H%uns(%lucyc comp%lu) L%uns(%lucyc comp%lu)",
                      (unsigned)t.t0h_ns, (unsigned long)t.t0h_cyc, (unsigned long)t.comp_t0h,
                      (unsigned)t.t0l_ns, (unsigned long)t.t0l_cyc, (unsigned long)t.comp_t0l);
            cmd_reply("KEYLED 1code H%uns(%lucyc comp%lu) L%uns(%lucyc comp%lu)",
                      (unsigned)t.t1h_ns, (unsigned long)t.t1h_cyc, (unsigned long)t.comp_t1h,
                      (unsigned)t.t1l_ns, (unsigned long)t.t1l_cyc, (unsigned long)t.comp_t1l);
            /* meas = 最近一帧"实测 - 目标"(拍), 顺序 0H/0L/1H/1L: 定时器方式下它就是
               忙等循环的收尾误差, 填到 bsp_config.h 的 KEYLED_TIM_WAIT_COMP_CYC 就抠准了。 */
            cmd_reply("KEYLED meas +%lu/%lu/%lu/%lu cyc (0H,0L,1H,1L)",
                      (unsigned long)t.meas_t0h, (unsigned long)t.meas_t0l,
                      (unsigned long)t.meas_t1h, (unsigned long)t.meas_t1l);
            /* tim = 1 说明走的是定时器中断发帧; frames = 中断真正发出去的帧数 ——
               发一条颜色命令后再敲一次 DIAG, frames 有涨就说明"中断发帧"这条路是活的。 */
            cmd_reply("KEYLED tim=%u frames=%lu (0/0 = old DWT mode)",
                      (unsigned)KEYLED_USE_TIM_IRQ, (unsigned long)keyled_frames());

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

        /* KEYLED INIT [n]: 连发 n 帧"全灭"(默认 KEYLED_INIT_FRAMES 帧)。
           现场重来一次上电那几帧 —— 灯珠被噪声/上一个项目锁存成白色时, 用它压回全灭,
           再发颜色命令。 */
        if ((argc >= 2U) &&
            ((bsp_stricmp_eq(tok[1], "INIT") != 0U) ||
             (bsp_stricmp_eq(tok[1], "INIT_FRAMES") != 0U)))
        {
            uint16_t n = (argc >= 3U) ? arg_u16(tok[2], 20U) : (uint16_t)KEYLED_INIT_FRAMES;
            uint16_t k;

            if (n == 0U)
            {
                n = 1U;
            }

            keyled_off();                    /* 全灭 + 立即刷新一帧 */

            for (k = 1U; k < n; k++)         /* 再补 n-1 帧全零 */
            {
                keyled_show();
            }

            cmd_reply("OK KEYLED INIT %u frames (all off)", (unsigned)n);
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

    /*------------------------- 串口链路自检 -----------------------*/
    /* 排查"串口收不到命令 / 收到乱码 / 老是少字节"用:
         UART              打印计数 + 最近一条真正解析过的整行
         UART RAW <0|1>    1 = 每收到一个字节就打一行 "RX hh c"
         UART DUMP [n]     把"原始接收缓冲"打出来(最近 n 个字节, 默认 128, ALL = 全部)
       计数器(rx/lines/total 为累计值, 重启清零):
         rx    收到过多少字节(DMA 搬运时累加)   lines 拼出过多少条完整命令行
         ovf   环形缓冲溢出丢弃的字节           err   线上错误(噪声/帧错/被关中断挤掉)
         drop  被丢掉的字节(超长行 + 控制字符)  long  因超过 ESP32_LINE_MAX 整行作废次数
         cap   原始缓冲收到的累计字节(= g_esp32_rx_cap_total)
         lost  DMA 窗口被覆盖丢掉的字节(>0 = 任务被饿着了, 要查调度/别设大周期)
         wraps 原始缓冲已经覆盖掉的旧字节(只保留最近 ESP32_CAP_SIZE 个)
       "发一条命令看哪个计数涨" + "UART DUMP 和上位机发的字节逐字节对"就能定位问题。 */
    if (bsp_stricmp_eq(tok[0], "UART") != 0U)
    {
        if ((argc >= 2U) && (bsp_stricmp_eq(tok[1], "RAW") != 0U))
        {
            esp32_set_raw_trace((argc >= 3U) ? arg_u8(tok[2]) : 1U);
            cmd_reply("OK UART RAW %u", (unsigned)esp32_get_raw_trace());
            return;
        }

        if ((argc >= 2U) && (bsp_stricmp_eq(tok[1], "DUMP") != 0U))
        {
            uint32_t n = 128U;

            if ((argc >= 3U) && (bsp_stricmp_eq(tok[2], "ALL") != 0U))
            {
                n = 0U;                     /* 0 = 整个缓冲都打出来 */
            }
            else if (argc >= 3U)
            {
                n = (uint32_t)arg_u16(tok[2], (uint16_t)esp32_cap_size());
            }

            esp32_cap_dump(n);
            cmd_reply("OK UART DUMP");
            return;
        }

        cmd_reply("UART rx=%lu lines=%lu buf=%u pend=%u raw=%u",
                  (unsigned long)esp32_rx_total(), (unsigned long)esp32_rx_lines(),
                  (unsigned)esp32_rx_available(), (unsigned)esp32_line_pending(),
                  (unsigned)esp32_get_raw_trace());
        cmd_reply("UART ovf=%lu err=%lu drop=%lu long=%lu",
                  (unsigned long)esp32_rx_overflows(), (unsigned long)esp32_rx_errors(),
                  (unsigned long)esp32_rx_dropped(), (unsigned long)esp32_rx_long_lines());
        cmd_reply("UART cap=%lu lost=%lu wraps=%lu size=%lu",
                  (unsigned long)esp32_cap_total(), (unsigned long)esp32_cap_dma_lost(),
                  (unsigned long)esp32_cap_wraps(), (unsigned long)esp32_cap_size());
        cmd_reply("UART last=\"%s\"", esp32_last_line());
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
