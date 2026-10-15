/**
  ******************************************************************************
  * @file    main.c
  * @brief   灯带消消乐 —— 主程序 / 业务集成示例
  *
  *  硬件资源(详见 Inc/bsp_config.h):
  *      PA0~PA6  7 个按键(内部上拉 + EXTI, 短按/长按/长按保持)
  *      PA7      电源键(长按 3s 关机)
  *      PC3      电源自锁控制
  *      PB6      120 颗 WS2812 灯带 (TIM4_CH1 + DMA1_CH1, PWM+DMA, 不间断输出)
  *      PC4      4 路按键灯 WS2812 (软件时序, 无需 PWM)
  *      PC6~PC8  4 位静态数码管 (74HC595 x4)
  *      PA9/PA10 ESP32-C3 WiFi 转串口 (USART1, 远程命令)
  *      PB10/PB11 + PB0~PB2  WT2003H4-16S 语音模块(串口 + Key 口)
  *
  *  程序结构(CMSIS-RTOS / Keil RTX, 任务划分见 Inc/app_task.h):
  *      初始化全部外设 -> app_task_start() 创建 4 个任务并启动调度器(不返回):
  *          input   5ms        按键扫描/消抖 + 电源键状态机
  *          light   10ms       灯带消消乐(游戏) + 灯带动画 + 按键灯
  *          comm    事件唤醒    串口命令 + 心跳上报
  *          voice   事件唤醒    WT2003 播放请求 + 返回帧解析
  *      中断里只做"精确时序 + 记时间戳"(WS2812 的 DMA、按键灯位打、串口收发、
  *      TIM2 1ms 时间基准), 因此任务周期的抖动不会影响灯带/按键灯的波形。
  *
  *  把 bsp_config.h 的 BSP_USE_RTOS 改成 0: app_task_start() 会立刻返回, 本文件
  *  末尾的"裸机主循环"将轮询所有 *_process(), 功能完全一样(便于对比/移植)。
  ******************************************************************************
  */
#include "bsp_config.h"
#include "bsp_util.h"
#include "bsp_time.h"
#include "bsp_rtos.h"

#include "drv_key.h"
#include "drv_power.h"
#include "drv_ws2812.h"
#include "drv_keyled.h"
#include "drv_hc595.h"
#include "drv_esp32.h"
#include "drv_wt2003h.h"

#include "app_led_fx.h"
#include "app_cmd.h"
#include "app_task.h"
#include "app_wsgame.h"

/*---------------------------------------------------------------------------*/
/* 上电语音探测的进度显示(调试用):
   数码管前两位 00, 后两位 = 正在试第几步(01~30);
   探测结束时显示"模块回帧计数"的低 4 位 —— 全 0000 说明模块一句都没回, 那就是
   接线/波特率/模块模式的问题, 而不是帧格式问题。
   同一条进度还会从 ESP32 串口(USART1)打一行 ASCII, 数码管不正常时也能看。 */
#if (WT2003_PROBE_ON_BOOT != 0U)
static void wt2003_probe_show(uint8_t step, uint8_t total)
{
    char     buf[5];
    uint32_t rx;

    rx = wt2003_rx_total();

    if (step == 0xFFU)              /* 探测结束 */
    {
        buf[0] = (char)('0' + (char)((rx / 1000U) % 10U));
        buf[1] = (char)('0' + (char)((rx / 100U) % 10U));
        buf[2] = (char)('0' + (char)((rx / 10U) % 10U));
        buf[3] = (char)('0' + (char)(rx % 10U));
        buf[4] = '\0';
        hc595_show_digits(buf);
        /* 数码管万一不亮也能看: 同一句话再从 ESP32 串口打一行出去 */
        esp32_printf("PROBE DONE rx=%lu cmd=%lu %s\r\n", (unsigned long)rx,
                     (unsigned long)wt2003_cmd_count(),
                     (rx == 0UL) ? "module silent: check wiring/baud/mode"
                                 : "read WTRXINFO: CA=conn C7=udisk_files C3=flash_files C9=cur C2=status A6 ret (ret 05 still plays!)");
        return;
    }

    buf[0] = '0';
    buf[1] = '0';
    buf[2] = (char)('0' + (char)((step / 10U) % 10U));
    buf[3] = (char)('0' + (char)(step % 10U));
    buf[4] = '\0';
    hc595_show_digits(buf);         /* 0001 = 第 1 种帧, 0002 = 第 2 种 ... */
    esp32_printf("PROBE %u/%u %s rx=%lu\r\n", (unsigned)step, (unsigned)total,
                 wt2003_probe_cfg_hint(step), (unsigned long)rx);
}
#endif

/*---------------------------------------------------------------------------*/
/* 按文件名轮播的进度显示(WT2003_NAME_CYCLE_ON_BOOT = 1 时):
   数码管前两位 00, 后两位 = 正在播第几个文件名(01~); 串口同时打一行
   NAME n/N "T00n" play by file name (A7) —— 听到声音就对照这一行看是哪个文件名;
   播后驱动会自动发 C9/CB, 于是还会跟出 WTRXINFO cur_file= / name=,
   能直接确认"模块到底播的是不是这一个文件"。 */
#if (WT2003_NAME_CYCLE_ON_BOOT != 0U)
static void wt2003_name_show(const char *name, uint8_t n, uint8_t total)
{
    char buf[5];

    buf[0] = '0';
    buf[1] = '0';
    buf[2] = (char)('0' + (char)((n / 10U) % 10U));
    buf[3] = (char)('0' + (char)(n % 10U));
    buf[4] = '\0';
    hc595_show_digits(buf);

    esp32_printf("NAME %u/%u \"%s\" play by file name (A7), rx=%lu\r\n",
                 (unsigned)n, (unsigned)total, (name != NULL) ? name : "?",
                 (unsigned long)wt2003_rx_total());
}
#endif

/*---------------------------------------------------------------------------*/
/* 语音模块"发/收"帧的实时打印(排查用, 由 WT2003_RX_TRACE / WT2003_TX_TRACE 开关):
   以前必须挂调试器看驱动的 s_rx_buf 才知道"模块到底回了什么"(而且那里是环形缓冲的
   原貌, 还夹着上一帧的残留字节), 现在每次收发都从 ESP32 串口(USART1, 115200)打一行:
       WTTX 7E 05 A6 00 01 AC EF   主机"真正发出去"的原始字节(默认 A6 播 U 盘索引 1)
       WTRXRAW 7E 04 A6 00 AA EF   模块回帧的原始字节(含 7E/长度/校验码/EF)
       WTRX A6 00                  同一帧解析后的结果(命令码 + 数据区)
   判读要点(手册 4.1.3.4 指定 U 盘根目录索引播放):
       发 -> 7E 05 A6 00 02 AD EF   长度 05 = 1(自身) + 1(命令) + 2(曲目号) + 1(校验码)
       收 <- A6 00                   手册写的应答(命令 + 处理结果)
      返回码(手册 4.1.3.1"写操作指令返回码格式"): 00 = OK / 01 = FAIL(出错, 不执行) /
      02 = EMP(无此文件)。实测本模块 A6 / D2 / C7 都回 0x05(手册里没定义),
      而那几轮音频照样出声 -> 非 0 不等于"没播", 有没有声以耳朵为准(见 README 8.7)。
     实测模块回帧是"带 7E/长度/校验码/EF 的整帧"(如 A0 索引 1 实收 7E 04 A0 00 A4 EF),
     驱动对"整帧"和"裸命令+数据"两种口径都做了兼容解析。
     A0(SPI Flash)结构相同, 只是命令码不同: 发 7E 05 A0 00 01 A6 EF。 */

#if ((WT2003_RX_TRACE != 0U) || (WT2003_TX_TRACE != 0U))
/* 打一行 "<tag> XX XX ..." 到 ESP32 串口(tag 最长 7 字符, 内部已按最大帧长算过缓冲) */
static void wt2003_hex_line(const char *tag, const uint8_t *p, uint8_t len)
{
    static const char s_hex[] = "0123456789ABCDEF";
    char              buf[8U + 3U * (WT2003_RX_FRAME_MAX + 3U) + 2U];
    uint8_t           n = 0U;
    uint8_t           i;

    for (i = 0U; (tag[i] != '\0') && (i < 7U); i++)
    {
        buf[n] = tag[i];
        n++;
    }

    if (len > (uint8_t)(WT2003_RX_FRAME_MAX + 3U))   /* 防御: 超长直接截断 */
    {
        len = (uint8_t)(WT2003_RX_FRAME_MAX + 3U);
    }

    for (i = 0U; i < len; i++)
    {
        buf[n]      = ' ';
        buf[n + 1U] = s_hex[(uint8_t)((p[i] >> 4) & 0x0FU)];
        buf[n + 2U] = s_hex[(uint8_t)(p[i] & 0x0FU)];
        n           = (uint8_t)(n + 3U);
    }

    buf[n]      = '\r';
    buf[n + 1U] = '\n';
    n           = (uint8_t)(n + 2U);

    esp32_send((const uint8_t *)buf, (uint16_t)n);
}
#endif

#if (WT2003_RX_TRACE != 0U)
/* 写命令返回码口径(手册 4.1.3.1 "写操作指令返回码格式"): 模块回 <命令码> <1 字节操作码>。
     0x00 = OK   命令执行
     0x01 = FAIL 命令出错, 不执行
     0x02 = EMP  无此文件
   0x05 手册没定义, 但本模块对 D2 / A6 / C7 这三条"带 U 盘"的命令都回 0x05,
   而且同一轮里喇叭确实出声了 —— 所以 0x05 绝不能当成"没播"(见 README)。
   查询命令(C1/C2/C3/C7/C8/C9/CA/CB)回的是"数据", 不是返回码, 别混用这张表。 */
static const char *wt2003_ret_name(uint8_t r)
{
    switch (r)
    {
        case 0x00U: return "OK";
        case 0x01U: return "FAIL (command error, not executed)";
        case 0x02U: return "EMP (no such file)";
        case 0x05U: return "0x05 (undocumented; U-disk cmds answer it, audio still plays)";
        default:    return "unknown";
    }
}

static void wt2003_rx_show(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t        all[WT2003_RX_FRAME_MAX + 1U];
    const uint8_t *raw;
    uint8_t        raw_len = 0U;
    uint8_t        i;

    /* ① 原始回帧: 连 7E / 长度 / 校验码 / EF 一起打, 便于与手册示例逐字节对照 */
    raw = wt2003_last_rx_raw(&raw_len);
    if ((raw != NULL) && (raw_len != 0U))
    {
        wt2003_hex_line("WTRXRAW", raw, raw_len);
    }

    /* ② 解析结果: 命令码 + 数据区(驱动已剥掉校验码) -> "WTRX A0 00" */
    all[0] = cmd;
    for (i = 0U; (i < len) && (i < (uint8_t)WT2003_RX_FRAME_MAX); i++)
    {
        all[i + 1U] = data[i];
    }
    wt2003_hex_line("WTRX", all, (uint8_t)(i + 1U));

    /* ③ 解析结果 -> 翻成"人话"再打一行给 ESP32(WTRXINFO): 一眼看出模块里边有
          几个文件、音量多少、在播第几首、A0 到底有没有真的开始播放。
          这条打印链路就是 手机 -> ESP32 -> USART1 -> 本机, 所以 ESP32 侧能直接看到。 */
    switch (cmd)
    {
        case WT2003_CMD_GET_VERSION:     /* C0: 版本(实测回帧比手册写的长, 顺便翻成 ASCII 看是不是版本串) */
            if (len >= 1U)
            {
                char    txt[WT2003_RX_FRAME_MAX + 1U];
                uint8_t k;
                uint8_t m = (len < (uint8_t)WT2003_RX_FRAME_MAX) ? len : (uint8_t)WT2003_RX_FRAME_MAX;

                for (k = 0U; k < m; k++)
                {
                    txt[k] = ((data[k] >= 0x20U) && (data[k] < 0x7FU)) ? (char)data[k] : '.';
                }
                txt[m] = '\0';
                esp32_printf("WTRXINFO ver=%s (len=%u, hex on the WTRX line above)\r\n", txt, (unsigned)len);
            }
            break;

        case WT2003_CMD_GET_VOLUME:      /* C1: 1 字节音量 0~31(0 = 静音) */
            if (len >= 1U)
            {
                esp32_printf("WTRXINFO volume=%u (0=silent, 31=max)\r\n", (unsigned)data[0]);
            }
            break;

        case WT2003_CMD_GET_STATUS:      /* C2: 1 字节工作状态(含义见手册表 4; 实测恒为 0x02) */
            if (len >= 1U)
            {
                /* 状态有没有"变"是本轮最关键的信息: 一直不变 = 模块从头到尾都在同一个
                   状态(自认为在播/占线 或 根本没动), 会变 = 命令真的起作用了 */
                static uint8_t s_prev_st = 0xFFU;
                const char    *tag;

                if (s_prev_st == 0xFFU)
                {
                    tag = "(first read)";
                }
                else if (s_prev_st == data[0])
                {
                    tag = "(UNCHANGED)";
                }
                else
                {
                    tag = "(CHANGED!)";
                }

                /* 手册 4.1.4.2: 0x01 = 播放, 0x02 = 停止, 0x03 = 暂停。
                   注意: 本轮实测里 C2 从头到尾都是 02(停止), 而同轮 A6 回 0x05 + 喇叭有声
                   -> 本固件的 C2 并不完全跟着 U 盘播放走, 别只凭 C2 判断"有没有出声"。 */
                esp32_printf("WTRXINFO status=0x%02X %s %s\r\n", (unsigned)data[0], tag,
                             (data[0] == 0x01U) ? "[PLAYING]"
                                                : ((data[0] == 0x02U) ? "[STOPPED]"
                                                                      : ((data[0] == 0x03U) ? "[PAUSED]" : "[?]")));
                s_prev_st = data[0];
            }
            break;

        case WT2003_CMD_GET_FILE_NUM:    /* C3: 文件总数(手册 4.1.4.3 写 SPI-Flash; 实测本模块
                                            C3 与 C7 同为 1 字节口径, 不能用来判介质) */
            if (len >= 2U)
            {
                esp32_printf("WTRXINFO flash_files=%u (SPI-Flash; compare with C7 udisk_files)\r\n",
                             (unsigned)(((uint16_t)data[0] << 8) | (uint16_t)data[1]));
            }
            else if (len >= 1U)
            {
                esp32_printf("WTRXINFO flash_files=%u (SPI-Flash, 1-byte reply; compare with C7)\r\n",
                             (unsigned)data[0]);
            }
            break;

        case WT2003_CMD_GET_UDISK_NUM:   /* C7: 查询 U 盘内音乐文件总数(手册 4.1.4.4) <== 本工程看这条 */
        case WT2003_CMD_GET_UFOLDER_NUM: /* C8: 查询 U 盘指定文件夹内文件总数(手册 4.1.4.5) */
            if (len >= 2U)
            {
                uint16_t n = (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);

                esp32_printf("WTRXINFO udisk_files=%u %s\r\n", (unsigned)n,
                             (n == 0U) ? "[0 = module sees NO file on the U-disk!]"
                                       : "[ok: the module really sees files on the U-disk]");
            }
            else if (len >= 1U)
            {
                /* 本模块这条只回 1 字节(手册写 2 字节) —— 数量有限, 1 字节足够 */
                esp32_printf("WTRXINFO udisk_files=%u (U-disk, 1-byte reply) %s\r\n",
                             (unsigned)data[0],
                             (data[0] == 0U) ? "[0 = module sees NO file on the U-disk!]"
                                             : "[ok: the module really sees files on the U-disk]");
            }
            break;

        case WT2003_CMD_GET_CONN:        /* CA: 查询当前外设连接状态(手册 4.1.4.7) */
            if (len >= 1U)
            {
                /* bit3 = PC 连接, bit2 = U 盘, bit0 = SPI-Flash; 1 = 存在(手册 4.1.4.7)。
                   注意: 本模块实测出现过 CA=0x00(U 盘位 = 0) 但 C7 回 5 个 U 盘文件、
                   且 A6 能播出声音的情况 -> CA 这条查询在本固件上不完全可信,
                   "盘认没认到"要 CA + C7 两条一起看。 */
                esp32_printf("WTRXINFO conn=0x%02X PC=%u UDISK=%u FLASH=%u\r\n",
                             (unsigned)data[0],
                             (unsigned)((data[0] >> 3) & 0x01U),
                             (unsigned)((data[0] >> 2) & 0x01U),
                             (unsigned)(data[0] & 0x01U));
            }
            break;

        case WT2003_CMD_GET_CUR_NAME:    /* CB: 查询当前播放歌曲名(手册 4.1.4.8, 8 字节 ASCII) */
            if (len >= 1U)
            {
                char    nm[9];
                uint8_t k;
                uint8_t m = (len < 8U) ? len : 8U;

                for (k = 0U; k < m; k++)
                {
                    nm[k] = ((data[k] >= 0x20U) && (data[k] < 0x7FU)) ? (char)data[k] : '.';
                }
                nm[m] = '\0';
                esp32_printf("WTRXINFO name=\"%s\"\r\n", nm);
            }
            break;

        case WT2003_CMD_GET_CUR_FILE:    /* C9: 2 字节当前播放文件号(高字节在前) */
            if (len >= 2U)
            {
                /* 播放中/换曲时"当前文件"应该跟着动; 一直不动 = 引擎根本没换过曲目 */
                static uint16_t s_prev_cur = 0xFFFFU;
                uint16_t        cur;
                const char     *tag;

                cur = (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);

                if (s_prev_cur == 0xFFFFU)
                {
                    tag = "(first read)";
                }
                else if (s_prev_cur == cur)
                {
                    tag = "(UNCHANGED)";
                }
                else
                {
                    tag = "(CHANGED!)";
                }

                esp32_printf("WTRXINFO cur_file=%u %s\r\n", (unsigned)cur, tag);
                s_prev_cur = cur;
            }
            break;

        case WT2003_CMD_PLAY_INDEX:      /* A0: SPI-Flash 根目录索引播放的返回码(手册 4.1.3.2) */
            if (len >= 1U)
            {
                esp32_printf("WTRXINFO A0(SPI-Flash) ret=0x%02X = %s\r\n",
                             (unsigned)data[0], wt2003_ret_name(data[0]));
            }
            break;

        case WT2003_CMD_PLAY_UFLASH:     /* A6: U 盘根目录索引播放的返回码(手册 4.1.3.4) */
            if (len >= 1U)
            {
                esp32_printf("WTRXINFO A6(U-disk) ret=0x%02X = %s\r\n",
                             (unsigned)data[0], wt2003_ret_name(data[0]));
            }
            break;

        case WT2003_CMD_SET_DRIVE:       /* D2: 切换当前工作盘符(手册 4.1.3.16) */
            if (len >= 1U)
            {
                esp32_printf("WTRXINFO D2(drive switch) ret=0x%02X = %s\r\n",
                             (unsigned)data[0], wt2003_ret_name(data[0]));
            }
            break;

        case WT2003_CMD_PLAY_FLASHNAME:  /* A1: SPI-Flash 按文件名播放(手册 4.1.3.3) */
        case WT2003_CMD_PLAY_UNAME:      /* A7: U 盘按文件名播放(手册 4.1.3.5) */
        case WT2003_CMD_PLAY_UFOLDER:    /* A8: U 盘文件夹内索引播放(手册 4.1.3.6) */
        case WT2003_CMD_INSERT:          /* B1: 插播命令(手册 4.1.3.13) */
            if (len >= 1U)
            {
                esp32_printf("WTRXINFO cmd=0x%02X ret=0x%02X = %s\r\n",
                             (unsigned)cmd, (unsigned)data[0], wt2003_ret_name(data[0]));
            }
            break;

        default:
            break;                       /* 其余命令含义不明确的不猜, 看上面的 WTRX 原始行 */
    }
}

/* 结构不对的回帧(帧尾不是 EF / 帧长超缓冲)也原样打出来:
   以前这类字节会被驱动静默丢掉, 就会出现"字节数涨了却一行 WTRXRAW 都没有"的怪现象
   (实测 C0 版本查询的回帧就有这个问题, 见 README 8.4)。 */
static void wt2003_bad_show(const uint8_t *raw, uint8_t len)
{
    wt2003_hex_line("WTRXBAD", raw, len);
    esp32_printf("WTRXINFO bad frame (tail != EF or too long), bytes=%u\r\n", (unsigned)len);
}
#endif

#if (WT2003_TX_TRACE != 0U)
static void wt2003_tx_show(const uint8_t *frame, uint8_t len)
{
    wt2003_hex_line("WTTX", frame, len);
}
#endif

/*---------------------------------------------------------------------------*/
int main(void)
{
    /*----------------------- 1. 基础 -----------------------------*/
    bsp_util_init();          /* NVIC 优先级分组 */
    bsp_time_init();          /* DWT 微秒延时 + TIM2 1MHz 时间戳 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /*----------------------- 2. RTOS 内核对象池 ------------------*/
    /* 必须放在各模块 *_init() 之前: 它们会在里面创建互斥锁 / 消息队列 */
    bsp_rtos_init();

    /*----------------------- 3. 电源 -----------------------------*/
osDelay(500);
    power_init();             /* 尽早锁住电源(一键开关机) */
    /*----------------------- 4. 灯带 -----------------------------*/
    ws2812_init();            /* 初始化后按 WS2812_AUTO_START 自动连续输出 */
    led_fx_init();            /* 效果层(默认全灭) */

    /*----------------------- 5. 灯与人机界面 ---------------------*/
    keyled_init();            /* 4 路按键灯 */
    hc595_init();             /* 4 位数码管(内部已清屏, 平时不需要额外动作) */

#if (HC595_BOOT_SELFTEST != 0U)
    /* 可选上电自检(排查数码管用, 默认关; 打开后按 README 3.4 的对照表定位故障):
       ① 全段 8888 -> ② 全灭 -> ③ 1234, 三步都对才算极性 / 位序 / 接线都正确。 */
    hc595_show_digits("8888");              /* ① 全部段亮、小数点不亮 */
    bsp_delay_ms(HC595_SELFTEST_MS);
    hc595_clear();                          /* ② 应该完全黑屏 */
    bsp_delay_ms(HC595_SELFTEST_MS);
    hc595_show_digits("1234");              /* ③ 应该看到 1234 */
    bsp_delay_ms(HC595_SELFTEST_MS);
#endif

    /*----------------------- 6. 通信与语音 -----------------------*/
    esp32_init();             /* USART1 <-> ESP32-C3 */

#if (WS2812_BOOT_SELFTEST != 0U)
    /* 主灯带上电三步自检(排查"灯带从来不亮", 默认关, 见 bsp_config.h / README 3.2.1):
       此刻还没创建任务(灯光任务在 app_task_start() 之后才跑), 没有别的任务会改颜色,
       所以直接写颜色缓冲 + ws2812_show() 即可, 不需要 ws2812_lock()。
       放在 esp32_init() 之后: 每一步都从 ESP32 串口打一行, 可以"看灯带 + 看日志"对时间。 */
    /* 先把这一版固件"实际生效的时序参数"打出来(和 README 3.2.1 的判据表对照)。
       120 颗 x 24bit + 300us 复位段 = 3120 位 x 1.25us -> 帧周期 3.90ms -> ≈256fps。
       arr/ccr0/ccr1 = 驱动按实际时钟换算出的定时器拍数(72MHz 下: 一位 90 拍,
       0 码 29 拍 = 403ns, 1 码 58 拍 = 806ns), 可以和上面的 ns 值互相印证。 */
    {
        uint16_t arr;
        uint16_t ccr0;
        uint16_t ccr1;

        ws2812_get_timing(&arr, &ccr0, &ccr1);
        esp32_printf("WS2812ST cfg leds=%u data=%u buf=%u bit=%uns T0H=%uns T1H=%uns reset=%uus arr=%u ccr0=%u ccr1=%u\r\n",
                     (unsigned)WS2812_LED_NUM, (unsigned)WS2812_DATA_BITS, (unsigned)WS2812_BUF_LEN,
                     (unsigned)WS2812_BIT_NS, (unsigned)WS2812_T0H_NS, (unsigned)WS2812_T1H_NS,
                     (unsigned)WS2812_RESET_US, (unsigned)arr, (unsigned)ccr0, (unsigned)ccr1);
    }

    esp32_printf("WS2812ST 1/3 pixel0=red  ONLY first led should light\r\n");
    hc595_show_digits("0001");
    ws2812_clear();
    ws2812_set_pixel(0U, 255U, 0U, 0U);        /* ① 只有第 0 颗红 */
    ws2812_show();
    bsp_delay_ms(WS2812_SELFTEST_MS);

    esp32_printf("WS2812ST 2/3 all=dim green  (low current: tests 5V/GND)\r\n");
    hc595_show_digits("0002");
    ws2812_fill(0U, 32U, 0U);                  /* ② 整条暗绿(≈1/8 亮度) */
    ws2812_show();
    bsp_delay_ms(WS2812_SELFTEST_MS);

    esp32_printf("WS2812ST 3/3 all=white      (max current: tests power drop)\r\n");
    hc595_show_digits("0003");
    ws2812_fill(255U, 255U, 255U);             /* ③ 整条全白 */
    ws2812_show();
    bsp_delay_ms(WS2812_SELFTEST_MS);

    {
        /* 实测帧率 + 时钟交叉校验(用 PC 端串口工具的时间戳当裁判):
             A 行 -> bsp_delay_ms(1000) -> B 行
           * PC 上 A/B 两行的真实时间差 = 这个窗口到底有多久
           * B.frame - A.frame         = 窗口里 DMA 完成了几帧 -> 真实帧率
           * B.tim2  - A.tim2          = 同一窗口, 单片机 TIM2(1ms) 量出来多少毫秒
           本版参数下真实帧率应该 ≈256fps(帧周期 3.90ms = 3120 位 x 1.25us)。
           = 256  -> 定时器时钟/位宽/DMA/中断全对, 灯带还不亮就只剩硬件;
           若 PC 上看"1 秒"实际走了 1.2 秒左右, 而 frames x 3.90ms 刚好等于真实耗时
           -> 帧周期本身是对的, 只是 bsp_delay_ms 的 DWT 忙等偏慢(纯软件, 与灯带波形无关)。 */
        uint32_t f0;
        uint32_t f1;
        uint32_t m0;
        uint32_t m1;

        f0 = ws2812_frame_count();
        m0 = bsp_time_ms();
        esp32_printf("WS2812ST rate A frame=%lu tim2=%lums dwt=%u\r\n",
                     (unsigned long)f0, (unsigned long)m0, (unsigned)bsp_time_dwt_ok());

        bsp_delay_ms(1000U);

        f1 = ws2812_frame_count();
        m1 = bsp_time_ms();
        esp32_printf("WS2812ST rate B frame=%lu tim2=%lums frames=%lu tim2_dt=%lums (healthy=256@120led/3120bit)\r\n",
                     (unsigned long)f1, (unsigned long)m1,
                     (unsigned long)(f1 - f0), (unsigned long)(m1 - m0));
    }

    esp32_printf("WS2812ST done: frame=%lu running=%u brt=%u (see README 3.2.1)\r\n",
                 (unsigned long)ws2812_frame_count(),
                 (unsigned)ws2812_is_running(),
                 (unsigned)ws2812_get_brightness());
    hc595_show_digits("0000");
    ws2812_clear();
    ws2812_show();
#endif

    wt2003_init();            /* USART3 + K1~K3 */

    /*----------------------- 7. 按键 -----------------------------*/
    drv_key_init();           /* PA0~PA6: EXTI 只记时间戳, 判定在任务里 */

    /*----------------------- 8. 业务层 ---------------------------*/
    app_cmd_init();           /* 注册串口命令回调 + 按键事件上报 */

    /*----------------------- 9. 上电演示 -------------------------*/
    /* 上电语音有两条路, 由 WT2003_PROBE_ON_BOOT 选择:
       - 正式模式(WT2003_PROBE_ON_BOOT = 0): 先等 WT2003_BOOT_DELAY_MS(模块上电要做
         内部初始化, 命令发得太早会被忽略, 表现为开机没声音; 觉得等太久就调小), 再设
         音量, 最后固定走串口(USART3)播放 WT2003_BOOT_VOICE 号音频(命令码默认 0xA6
         播 U 盘根目录索引 + 2 字节曲目号; 用 A0 播 SPI Flash 由 WT2003_PLAY_USE_UDISK=0 选)。
       - 自检模式(= 1): 上电自动跑 38 步体检(见 drv_wt2003h.c 的 s_probe_tab):
             1~7   设置与查询: AB 停止 / B6=SPK / AE 音量31 / C1音量 / C3文件数 / C9当前文件 / C2状态
                   喇叭必须接在 SPK+ 与 SPK- 两端(BTL), 一端接 GND 则永远无声
             8     A6 播放索引 1(U 盘)                 <- 听! (第 1 次试播)
             9~10  C9/C2(有变化吗; 打印后面带 UNCHANGED / CHANGED!)
             11    AA 播放/暂停(把"自认为在播"的东西暂停掉)
             12    C2(暂停后的状态)
             13    A6 播放索引 1 再来(U 盘)       <- 听! 这步回 00 = 第 8 步的 02 是"忙碌拒绝"
             14~15 C9/C2
             16    AB 停止
             17    B6 输出 = DAC(试另一条输出通路)
             18    A6 播放索引 1(DAC 模式)             <- 听!
             19~20 AA 播放/暂停 / B6 输出 = SPK(切回)
             21    A6 播放索引 1(SPK 模式)             <- 听!
             22~23 C2 / AC 下一曲 + C9    24~26 AD 上一曲 + C9(cur_file 在 1 与 2 之间跳吗)
             27~28 A6 播放索引 2 / 3(扫 U 盘文件索引, 回 00 = 该索引有文件可播)
             29~30 AB 停止 + C2(最终状态)
         每一步的 WTTX/WTRX/WTRXINFO 都会从 ESP32 串口打出来, 数码管显示步号;
         排查完把 WT2003_PROBE_ON_BOOT 改回 0。
       注: 这里明确要"串口通路", 所以用 wt2003_talk_src() 指定 SRC_UART
       (Key 口关闭时 wt2003_talk() 本来也会走串口, 写上更直白)。 */
#if (WT2003_RX_TRACE != 0U)
    /* 模块返回帧 -> 从 ESP32 串口打一行 "WTRXRAW ..."/"WTRX ..." (免得为看回帧去挂调试器) */
    wt2003_set_frame_handler(wt2003_rx_show);
    /* 结构不对的回帧(帧尾非 EF / 超长)也要打出来, 否则"字节收了却看不见"没法判断 */
    wt2003_set_bad_frame_handler(wt2003_bad_show);
#endif
#if (WT2003_TX_TRACE != 0U)
    /* 主机发帧 -> 从 ESP32 串口打一行 "WTTX ..." (一眼确认"到底发出去几个字节") */
    wt2003_set_tx_handler(wt2003_tx_show);
#endif
    bsp_delay_ms(WT2003_BOOT_DELAY_MS);
    (void)wt2003_set_volume(WT2003_DEFAULT_VOLUME);
#if (WT2003_PROBE_ON_BOOT != 0U)
    /* 自检模式: 上电自动跑 38 步体检(AB 停止 -> D2 盘符=U 盘 -> B6=SPK / AE 音量 -> C1/CA/C7 查询基准
       -> A6 试播 -> AA 暂停后 A6 再播 -> DAC/SPK 各播一次 -> AC/AD 换曲
       -> C3 / 不存在的索引 / A6 索引 0 / A0 的"返回码取证"),
       数码管显示"正在试第几步" -> 听到声音就记下数字, 对照 README 第 8 节判读;
       排查完把 WT2003_PROBE_ON_BOOT 改回 0。 */
    wt2003_probe_set_handler(wt2003_probe_show);
    wt2003_probe_start();
#elif (WT2003_NAME_CYCLE_ON_BOOT != 0U)
    /* 按文件名轮播模式(现在用这条): 实测本模块 A6(指定索引播放)收下命令却不出声,
       改用 A7 按文件名播放 —— 每 WT2003_NAME_CYCLE_MS 毫秒播下一个名字
       (名字表见 bsp_config.h 的 WT2003_NAME_CYCLE_LIST, 文件名必须 4 字符且放在
       U 盘根目录), 播后自动查一次 C9/CB; 数码管 + 串口同步显示正在播第几个。
       排查完把 WT2003_NAME_CYCLE_ON_BOOT 改回 0。 */
    (void)wt2003_set_output(WT2003_OUT_SPK);   /* 与体检里"出声那次"一致: 喇叭输出 */
    wt2003_name_set_handler(wt2003_name_show);
    wt2003_name_start();
#elif (WT2003_BOOT_VOICE > 0)
    /* 上电提示音: 固定走串口播放 WT2003_BOOT_VOICE 号音频 */
    (void)wt2003_talk_src((uint16_t)WT2003_BOOT_VOICE, WT2003_SRC_UART);
#endif
    hc595_show_digits("0000");

#if (WSGAME_AUTOSTART == 0U)
    /* 没开自动开局时, 上电跑一段彗星灯表示"固件活着"; 要开局按 Key4 或发 GAME START */
    led_fx_set_color(0U, 128U, 255U);
    led_fx_set(LED_FX_COMET);
#endif

    /*----------------------- 9.1 灯带消消乐 ---------------------*/
    /* 游戏层(app_wsgame): 蓝/绿/紫 的单颗灯从远端冒出来, 整条缓慢往近端挪;
       Key1/Key2/Key3 = 朝远端发射同色子弹, 撞上同色灯 -> 两颗一起消失(得分),
       撞上不同色 -> 不反应(子弹打穿继续飞); 一直没消掉的灯走到近端算"漏掉"。
       必须放在 ws2812 / led_fx / keyled / hc595 / esp32 都初始化之后。
       WSGAME_AUTOSTART = 1(默认): 这里直接开局, 并接管灯带(效果层切 OFF);
                           = 0: 只准备好, 停在 OFF 状态等 Key4 / GAME START。
       布局不对(灯看着往远端跑)不用改接线: 把 bsp_config.h 的
       WSGAME_REVERSE_DIR 改成 1U 即可。 */
//    wsgame_init();

    /*----------------------- 10. 创建任务 + 启动调度器 -----------*/
    /* RTX 模式: 本函数不返回, 每个 *_process() 跑在各自的任务里;
       BSP_USE_RTOS = 0: 立刻返回, 继续执行下面的裸机主循环 */
    app_task_start();

    /*----------------------- 11. 裸机主循环(不使用 RTOS 时) ------*/
    for (;;)
    {
        drv_key_process();    /* 按键消抖 + 短按/长按判定(内部限速 1ms) */
        power_process();      /* 电源键长按关机 */
        esp32_process();      /* 串口收命令 -> app_cmd 回调 */
        app_cmd_process();    /* 心跳上报 */
        wt2003_process();     /* Key 口脉冲释放 + 模块返回帧解析 */
        keyled_process();     /* 按键灯闪烁/呼吸 */
//        wsgame_process();     /* 灯带消消乐(游戏运行时独占灯带) */
//        led_fx_process();     /* 灯带动画(自己控制刷新率) */
        ws2812_process();     /* 预留扩展点 */

        __WFI();              /* 空闲睡眠: 任意中断都能唤醒, 省电又实时 */
    }
}
