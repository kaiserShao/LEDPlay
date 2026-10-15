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
  *      PB10/PB11             WT2003HX 语音模块(只走串口, 无 Key 口)
  *
  *  程序结构(CMSIS-RTOS / Keil RTX, 任务划分见 Inc/app_task.h):
  *      初始化全部外设 -> app_task_start() 创建 4 个任务并启动调度器(不返回):
  *          input   5ms        按键扫描/消抖 + 电源键状态机
  *          light   10ms       灯带消消乐(游戏) + 灯带动画 + 按键灯
  *          comm    事件唤醒    串口命令 + 心跳上报
  *          voice   10ms        WT2003 返回帧解析(收应答)
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
/* 语音模块收发帧打印(WT2003_TRACE = 1 时): 从 ESP32 串口(USART1)打一行,
   不用挂调试器也能确认"到底发了什么 / 模块回了什么":
       WTTX 7E 05 9D 00 01 A3 EF   主机真正发出去的字节(含 7E/长度/校验/EF)
       WTRX 9D 00                  模块回帧(命令码 + 数据区)
       WTRXCOUNT 6                 0xC4 查到的文件总数(0 = 模块在盘上看不到文件)
   帧格式(手册): 7E 长度 命令 数据… 校验码 EF;  长度 = 自身 + 命令 + 数据 + 校验 */
#if (WT2003_TRACE != 0U)
static void wt2003_trace_line(const char *tag, const uint8_t *p, uint8_t len)
{
    char    buf[80];
    uint8_t n = 0U;
    uint8_t i;

    if ((tag == NULL) || (p == NULL))
    {
        return;
    }

    /* 整行拼好再一次发出去: 分多次发会被别的任务(命令应答 / 心跳 / 按键事件)插队,
       串口上就是"WTTX 7E 05 ERR UNKNOWN ..."这种两条信息焊在一行的乱象 */
    while ((tag[n] != '\0') && (n < 8U))
    {
        buf[n] = tag[n];
        n++;
    }

    for (i = 0U; (i < len) && ((uint16_t)n + 3U < (uint16_t)sizeof(buf)); i++)
    {
        char hx[3];

        bsp_hex_to_str((uint32_t)p[i], 2U, hx);
        buf[n++] = ' ';
        buf[n++] = hx[0];
        buf[n++] = hx[1];
    }

    buf[n++] = '\r';
    buf[n++] = '\n';

    esp32_send((const uint8_t *)buf, (uint16_t)n);
}

static void wt2003_trace_tx(const uint8_t *frame, uint8_t len)
{
    wt2003_trace_line("WTTX", frame, len);
}

static void wt2003_trace_rx(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t all[WT2003_RX_FRAME_MAX + 1U];
    uint8_t i;

    all[0] = cmd;
    for (i = 0U; (i < len) && (i < (uint8_t)WT2003_RX_FRAME_MAX); i++)
    {
        all[i + 1U] = data[i];
    }
    wt2003_trace_line("WTRX", all, (uint8_t)(i + 1U));

    if (cmd == (uint8_t)WT2003_CMD_QUERY_COUNT)
    {
        esp32_printf("WTRXCOUNT %u %s\r\n", (unsigned)wt2003_get_count(),
                     (wt2003_get_count() == 0U) ? "(0 = no file found in the module)"
                                                : "(file count read from the module)");
    }
}
#endif /* WT2003_TRACE */

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

    wt2003_init();            /* USART3: PB10 = TX / PB11 = RX */

    /*----------------------- 7. 按键 -----------------------------*/
    drv_key_init();           /* PA0~PA6: EXTI 只记时间戳, 判定在任务里 */

    /*----------------------- 8. 业务层 ---------------------------*/
    app_cmd_init();           /* 注册串口命令回调 + 按键事件上报 */

    /*----------------------- 9. 上电语音 -------------------------*/

#if (WT2003_TRACE != 0U)
    /* 收发帧打印(排查用): 发帧 -> WTTX, 回帧 -> WTRX / WTRXCOUNT */
    wt2003_set_tx_handler(wt2003_trace_tx);
    wt2003_set_frame_handler(wt2003_trace_rx);
#endif

    /* 上电语音: 等模块就绪 -> (可选)设音量 -> (可选)查文件总数 -> (可选)播开机提示音。
       顺序有讲究: 模块上电要做内部初始化, 命令发太早会被忽略(表现为开机没声音);
       注: bsp_delay_ms 是忙等 —— 这里还在初始化阶段(任务尚未创建), 没有别的模块受影响。 */
    bsp_delay_ms(WT2003_BOOT_DELAY_MS);
#if (WT2003_SET_VOLUME_ON_BOOT != 0U)
    (void)wt2003_set_volume((uint8_t)WT2003_DEFAULT_VOLUME);
#endif
#if (WT2003_QUERY_COUNT_ON_BOOT != 0U)
    (void)wt2003_query_count();          /* 应答会打 WTRXCOUNT 行(WT2003_TRACE = 1 时) */
#endif
#if (WT2003_BOOT_PLAY != 0U)
    (void)wt2003_play_name(WT2003_BOOT_FILE);   /* 按文件名播放: 0006.mp3 */
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
    wsgame_init();          /* ★ 游戏 + 声音的总闸, 必须留(之前排查音频时被注释掉过):
                                 不调它 -> s_inited = 0 -> wsgame_process() 第一行就 return,
                                 游戏既不开局也不收按键, bgm_start()/sfx_play() 一帧都发不出去。
                                 按 WSGAME_AUTOSTART 决定上电是否直接开局(默认 1 = 开局);
                                 想上电跑 COMET 灯: 把 bsp_config.h 的该宏改成 0U(本行仍要留) */

    /*----------------------- 10. 创建任务 + 启动调度器 -----------*/
    /* RTX 模式: 本函数不返回, 每个 *_process() 跑在各自的任务里;
       BSP_USE_RTOS = 0: 立刻返回, 继续执行下面的裸机主循环 */
    app_task_start();
//wsgame_stop();
    /*----------------------- 11. 裸机主循环(不使用 RTOS 时) ------*/
    for (;;)
    {
        drv_key_process();    /* 按键消抖 + 短按/长按判定(内部限速 1ms) */
        power_process();      /* 电源键长按关机 */
        esp32_process();      /* 串口收命令 -> app_cmd 回调 */
        app_cmd_process();    /* 心跳上报 */
        wt2003_process();     /* 解析语音模块回帧(查文件总数等应答) */

        /* 串口正在收命令、且收数靠 USART1 中断逐字节搬时先不发按键灯(发帧在定时器中断里
           要占着 120us, 会把正在收的字节挤掉); 与 app_task 里的灯光任务同一个道理 */
#if (ESP32_RX_USE_DMA == 0U)
        if ((esp32_line_pending() == 0U) && (esp32_rx_available() == 0U))
#endif
        {
            keyled_process(); /* 按键灯闪烁/呼吸 */
        }

        wsgame_process();     /* 灯带消消乐(游戏运行时独占灯带) */
        led_fx_process();     /* 灯带动画(自己控制刷新率) */
        ws2812_process();     /* 预留扩展点 */

        __WFI();              /* 空闲睡眠: 任意中断都能唤醒, 省电又实时 */
    }
}
