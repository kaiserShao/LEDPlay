/**
  ******************************************************************************
  * @file    bsp_config.h
  * @brief   全局硬件配置 / 可调参数 (灯带消消乐)
  *
  *  MCU      : STM32F103RCT6 (256KB Flash / 48KB RAM, SYSCLK = 72MHz)
  *  工程     : LEDAnipop (Keil MDK, StdPeriph Driver V3.6.0)
  *
  *  本文件集中了所有"硬件相关"的定义, 换板子 / 改引脚只改这里。
  *
  *  引脚分配总览
  *  ---------------------------------------------------------------------------
  *  功能                  引脚         说明
  *  ---------------------------------------------------------------------------
  *  按键 1~7              PA0~PA6      EXTI0~EXTI6, 内部上拉, 按下=低
  *  电源键                PA7          EXTI7, 内部上拉, 按下=低, 长按关机
  *  电源保持(自锁)         PC3          推挽输出, 高=保持供电, 低=断电
  *  主灯带 WS2812 (120灯)  PB6          TIM4_CH1 + DMA1_Channel1 (PWM+DMA)
  *  按键灯 WS2812 (4灯)    PC4          GPIO 软件延时模拟时序
  *  数码管 74HC595 x4      PC6/PC7/PC8  SDI / SCLK / LOAD
  *  ESP32-C3 (WiFi<->串口) PA9/PA10     USART1 TX/RX
  *  WT2003H4-16S 语音      PB10/PB11    USART3 TX/RX
  *  WT2003 Key 口          PB0/PB1/PB2  K1 / K2 / K3 (开漏, 低脉冲触发)
  *  ---------------------------------------------------------------------------
  ******************************************************************************
  */
#ifndef __BSP_CONFIG_H
#define __BSP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"
#include <stddef.h>     /* NULL */

/*==============================================================================
 * 0. 系统时钟
 *    注: RTE\Device\STM32F103RC\system_stm32f10x.c 中已使能 SYSCLK_FREQ_72MHz
 *        (要求 HSE = 8MHz 晶振)。驱动内所有时间计算运行时取 SystemCoreClock。
 *============================================================================*/
#define BSP_SYSCLK_HZ                72000000UL

/*==============================================================================
 * 1. NVIC 优先级分组与各中断优先级 (数值越小优先级越高)
 *============================================================================*/
#define BSP_NVIC_PRIORITY_GROUP      NVIC_PriorityGroup_2

#define BSP_IRQ_PRIO_WS2812_DMA      0U   /* 主灯带 DMA 半/全传输中断, 窗口 1.8ms */
#define BSP_IRQ_PRIO_ESP32_UART      1U   /* 与 WiFi 模块通信, 需要及时收数 */
#define BSP_IRQ_PRIO_WT2003_UART     2U   /* 语音模块串口 */
#define BSP_IRQ_PRIO_EXTI            3U   /* 按键 / 电源键外部中断 (只记时间戳) */
#define BSP_IRQ_PRIO_TIME            3U   /* TIM2 1ms 时基中断 */

/*==============================================================================
 * 2. 按键 PA0 ~ PA6
 *    无外部上拉电阻 -> 使用 MCU 内部上拉 (GPIO_Mode_IPU), 按下为低电平。
 *    中断里只做"记录时间戳 + 置标志", 全部判定放在 drv_key_process() 中完成。
 *============================================================================*/
#define KEY_PORT                     GPIOA
#define KEY_GPIO_CLK                 RCC_APB2Periph_GPIOA
#define KEY_PINS                     (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | \
                                      GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6)
#define KEY_COUNT                    7U
#define KEY_ID_BASE                  0U                     /* KEY_1 的索引 */
#define KEY_PORT_SOURCE              GPIO_PortSourceGPIOA   /* PA0~PA6 -> EXTI0~6 */
#define KEY_DEBOUNCE_MS              20U      /* 软件消抖时间 (ms) */
#define KEY_LONG_MS                  800U     /* 长按判定门限 (ms) */
#define KEY_HOLD_REPEAT_MS           200U     /* 长按保持时重复上报间隔, 0=不重复 */
#define KEY_EVENT_QUEUE_LEN          8U       /* 事件队列深度 (主循环消费) */
#define KEY_SCAN_PERIOD_MS           1U       /* 主循环扫描周期建议值 */

/*==============================================================================
 * 3. 一键开关机 (电源自锁)
 *============================================================================*/
#define POWER_HOLD_PORT              GPIOC
#define POWER_HOLD_PIN               GPIO_Pin_3
#define POWER_HOLD_ON_LEVEL          1U       /* 保持供电时 PC3 电平 (按实际硬件改) */
#define POWER_HOLD_OFF_LEVEL         0U       /* 断电时 PC3 电平 */

#define POWER_KEY_PORT               GPIOA
#define POWER_KEY_PIN                GPIO_Pin_7
#define POWER_KEY_EXTI_LINE          EXTI_Line7
#define POWER_KEY_PIN_SOURCE         GPIO_PinSource7
#define POWER_OFF_LONG_MS            1000U    /* 长按 3s -> 正常关机(走回调/提示音) */
#define POWER_FORCE_OFF_MS           6000U    /* 长按 6s -> 强制立即断电 */
#define POWER_KEY_POLL_MS            10U      /* 电源键扫描周期 */
#define POWER_SHUTDOWN_NOTICE_MS     1500U    /* 长按关机后, 留给提示音的延时(ms) */
#define POWER_HARD_CUT_LOOP          1U       /* 1 = 断电后停在死循环(等硬件掉电) */

/*==============================================================================
 * 4. 主灯带 WS2812 (120 颗)  PB6 = TIM4_CH1
 *    方案: TIM4 输出 800kHz PWM (周期 1.25us = 1 个数据位), DMA1_Channel1 循环把
 *          CCR1 值搬运到比较寄存器。用"半传输 + 传输完成"中断在后台交替刷新
 *          前/后半帧 (半帧 1440 位 = 1.8ms), 保证灯带数据永不断流。
 *============================================================================*/
#define WS2812_TIM                   TIM4
#define WS2812_TIM_CLK               RCC_APB1Periph_TIM4
#define WS2812_GPIO_PORT             GPIOB
#define WS2812_GPIO_PIN              GPIO_Pin_6
#define WS2812_GPIO_CLK              RCC_APB2Periph_GPIOB

#define WS2812_DMA_CLK               RCC_AHBPeriph_DMA1
#define WS2812_DMA_CHANNEL           DMA1_Channel1
#define WS2812_DMA_IRQn              DMA1_Channel1_IRQn
#define WS2812_DMA_IT_HT             DMA1_IT_HT1
#define WS2812_DMA_IT_TC             DMA1_IT_TC1

#define WS2812_LED_NUM               120U     /* 灯珠数量 */
#define WS2812_BIT_NS                1250U    /* 1 位 = 1.25us (800kHz) */
/* 0/1 码高电平按供应商(上架)给的 WS2812B 规格书取中心值:
   T0H = 0.4us +-150ns (250~550) / T1H = 0.8us +-150ns (650~950)。
   72MHz 下换算: 一位 90 拍; 0 码 29 拍 = 403ns, 1 码 58 拍 = 806ns,
   低电平随之 = 61 拍(847ns, 规格 T0L 700~1000) / 32 拍(444ns, 规格 T1L 300~600),
   四个窗口全落在规格书范围内。
   注: 原来的 300/600ns 总周期(1.25us)没错, 但 1 码高电平 600ns 低于规格书下限 650ns,
   有被灯珠当成 0 码的风险 -> 现象正是"发全红却一颗都不亮"。 */
#define WS2812_T0H_NS                400U     /* 0 码高电平 400ns */
#define WS2812_T1H_NS                800U     /* 1 码高电平 800ns */
/* 帧间复位低电平: 规格书表格写"大于 50us", 但同一份规格书的时序图/数据刷新周期
   标的是 >=280us; 实际 WS2812B(-V5) 都按 >=280us 做最稳 -> 取 300us。
   300us / 1.25us = 240 位, 帧周期 = 3120 位 x 1.25us = 3.90ms -> 约 256fps。 */
#define WS2812_RESET_US              300U
#define WS2812_MAX_BRIGHTNESS        255U     /* 默认亮度 0~255 */
#define WS2812_AUTO_START            1U       /* init 后自动启动 DMA 输出 */

/* 上电灯带三步自检(排查"灯带从来不亮"用, 默认 0 = 关闭):
   置 1 重新编译 -> 上电后灯带依次走三步(每步 WS2812_SELFTEST_MS), 同时
   ESP32 串口会打 WS2812ST 行、数码管显示步号 0001/0002/0003:
     ① 只有第 0 颗亮红      -> 验证"数据到底有没有发出去"
     ② 整条暗绿(≈1/8 亮度)  -> 验证"供电 / 共地"(电流小, 供电不够也能亮)
     ③ 整条全白(最大电流)   -> 验证"压降 / 线径 / 电源能力"
   三步的现象对照表见 README 3.2.1; 排查完改回 0(正常固件不需要这几秒)。
   主灯带"从来不亮"已定位: 根因是 3.3V 直驱(灯带 V_IH=0.7xVDD) + 时序没按规格书对齐
   (T1H 600ns < 下限 650ns; 帧间复位 60us < 280us), 现由 PB6 外加电平转换 +
   T0H/T1H/RESET 取规格书中心值解决(见 README 3.2.0)。正式固件保持 0U;
   换灯带 / 换板子排查"从来不亮"时再置 1。 */
#define WS2812_BOOT_SELFTEST         0U
#define WS2812_SELFTEST_MS           1200U    /* 自检每一步停留时间(ms) */

/*==============================================================================
 * 5. 按键灯 WS2812 (4 路, 单线串联在 PC4)
 *    不用 PWM: DWT 周期计数 + 管脚翻转, 输出 96 bit 约 110us, 期间关中断。
 *============================================================================*/
#define KEYLED_NUM                   4U
#define KEYLED_PORT                  GPIOC
#define KEYLED_PIN                   GPIO_Pin_4
#define KEYLED_GPIO_CLK              RCC_APB2Periph_GPIOC
/* 时序: 每位 = "高电平 + 低电平" 两段, 0 码和 1 码各自独立取值 —— 规格书就是这么
   定义的, 并没有"必须凑满 1.25us 位周期"这条硬约束, 所以这里不设位周期,
   只按 0 码/1 码各自的 H + L 时间走(低电平跑出范围同样是灯珠判错位的根因)。
   上电时驱动会用 DWT 实测"这四段实际有多长"(drv_keyled.c 的 keyled_calibrate),
   把延时函数/循环的固定开销扣掉 —— 所以这里填的就是灯珠真正看到的时间,
   不用再手工去猜 KEYLED_CYC_COMP。
   取值同时落在 WS2812B 与 WS2812B-V5 两份规格书的范围内:
     WS2812B:    T0H 250~550  T0L 700~1000  T1H 650~950   T1L 300~600
     WS2812B-V5: T0H 220~380  T0L 580~1000  T1H 580~1600  T1L 220~420
     两份手册的交集: T0H 250~380  T0L 700~1000  T1H 650~950  T1L 300~420
     本板取值(取交集偏下侧: 标定的"取最小值"只会欠补, 实际相位只会比目标偏长
     0~5 个周期, 不会偏短, 所以目标压在窗口下沿反而上下都安全):
                 T0H 300      T0L 850       T1H 750       T1L 320
   历史坑: 早先按"固定 1.25us 位周期"做, T1H 一调长 T1L 就被压到 300ns 以下,
   T0L 又被撑到 1.1us 以上 —— 四条相位里两条低电平都跑出手册范围, 几颗灯就一起发白。
   若仍发白/串色, 按顺序扫(串口都能实时改, 改完立即重发一帧, 不用重编):
     1) KEYLED T0H 250 / KEYLED T0H 350  0 码高电平偏长最容易被读成 1 码(偏白);
     2) KEYLED T1H 700 / KEYLED T1H 850  1 码高电平偏短会被读成 0 码(变暗/不亮);
     3) KEYLED T0L 750 / KEYLED T1L 400  低电平超范围也会整帧错位(T1L 上限 420 最紧);
     4) KEYLED INVERT 1                  数据线上串了反相器/三极管时用;
     5) KEYLED DIAG                      打印实测周期数, 发回来定位。 */
#define KEYLED_T0H_NS                300U     /* 0 码高电平 */
#define KEYLED_T0L_NS                850U     /* 0 码低电平 */
#define KEYLED_T1H_NS                750U     /* 1 码高电平 */
#define KEYLED_T1L_NS                320U     /* 1 码低电平 */
#define KEYLED_RESET_US              300U     /* 刷新后的复位低电平(>=280us, 同主灯带) */
#define KEYLED_CALIB                 1U       /* 1 = 上电用 DWT 自标定时序(推荐) */
#define KEYLED_CYC_COMP              5U       /* 只有 DWT 不可用(极端情况)时才用的固定补偿 */
#define KEYLED_LOCK_IRQ              1U       /* 1 = 输出期间关中断(推荐) */
#define KEYLED_INVERT                0U       /* 1 = 数据线取反(线上串了反相器/三极管;
                                                 也可远程 KEYLED INVERT 1 临时试) */

/*==============================================================================
 * 5.1 按键灯运行参数
 *============================================================================*/
#define KEYLED_REFRESH_MS            20U      /* 效果刷新周期(50Hz) */
#define KEYLED_DEFAULT_PERIOD_MS     500U     /* 默认闪烁/呼吸周期 */
#define KEYLED_INIT_FRAMES           3U       /* 上电连发几帧"全灭"(1 = 只发一帧):
                                                 上电瞬间引脚是浮空的, 灯珠可能把噪声
                                                 锁存成自己的"上电默认色"(有的批次就是
                                                 全白), 连发几帧全零能把它压回全灭 */

/*==============================================================================
 * 6. 数码管 (4 位静态显示, 4 片 74HC595 级联)
 *    位序: 先移入的字节最终落在"最远端"的 595 上, 用 HC595_DIGIT_MAP 做映射。
 *    实测本板要倒序(HC595_DIGIT_REVERSE = 1): 不改的话第 1 位会显示到第 4 位上。
 *============================================================================*/
#define HC595_DIGIT_REVERSE          1U       /* 1 = 数码管位序倒过来(见 drv_hc595.c) */
#define HC595_SDI_PORT               GPIOC
#define HC595_SDI_PIN                GPIO_Pin_6
#define HC595_SCLK_PORT              GPIOC
#define HC595_SCLK_PIN               GPIO_Pin_7
#define HC595_LOAD_PORT              GPIOC
#define HC595_LOAD_PIN               GPIO_Pin_8
#define HC595_GPIO_CLK               RCC_APB2Periph_GPIOC
#define HC595_DIGIT_NUM              4U
#define HC595_CLK_DELAY_NOP          2U       /* 移位时钟半周期 NOP 数 */
/* 段码有效电平: 1 = 共阴(段码高电平点亮, 不取反), 0 = 共阳(段码低电平点亮, 内部取反)。
   现象对照(上电/清屏后最常见的两种错法):
     * 全段连小数点一起亮 -> 板子是共阳, 但这里写成了 1 -> 改成 0U;
     * 该亮的地方全黑      -> 板子是共阴, 但这里写成了 0 -> 改成 1U。
   注: 74HC595 上电时输出寄存器全 0, 所以共阳板在数据送进来之前也表现为"全亮";
       若改成 0U 后仍然常亮/一直不变, 说明数据没移位进 595 —— 先量 PC6 是否接在
       595 的 DS(14 脚)上, 而不是 QH'(9 脚, 那是级联输出)。详见 README 3.4。 */
#define HC595_ACTIVE_LEVEL           0U       /* 本板 = 共阳(已实测确认), 保持 0U */

#define HC595_BOOT_SELFTEST          0U       /* 1 = 上电走 3 步自检 8888->全灭->1234(排查用, 见 README 3.4) */
#define HC595_SELFTEST_MS            500U     /* 自检每步停留时间(ms) */

/*==============================================================================
 * 7. ESP32-C3 WiFi 转串口 (USART1: PA9=TX, PA10=RX)
 *============================================================================*/
#define ESP32_UART                   USART1
#define ESP32_UART_CLK               RCC_APB2Periph_USART1
#define ESP32_UART_IRQn              USART1_IRQn
#define ESP32_TX_PORT                GPIOA
#define ESP32_TX_PIN                 GPIO_Pin_9
#define ESP32_RX_PORT                GPIOA
#define ESP32_RX_PIN                 GPIO_Pin_10
#define ESP32_GPIO_CLK               RCC_APB2Periph_GPIOA
#define ESP32_BAUD                   115200U
#define ESP32_RX_BUF_SIZE            512U     /* 接收环形缓冲(字节) */
#define ESP32_LINE_MAX               127U     /* 单条命令最大长度 */
#define ESP32_TX_TIMEOUT_MS          50U      /* 发送超时保护 */

/*==============================================================================
 * 8. WT2003H4-16S 语音模块 (USART3: PB10=TX, PB11=RX ; K1~K3: PB0~PB2)
 *============================================================================*/
#define WT2003_UART                  USART3
#define WT2003_UART_CLK              RCC_APB1Periph_USART3
#define WT2003_UART_IRQn             USART3_IRQn
#define WT2003_TX_PORT               GPIOB
#define WT2003_TX_PIN                GPIO_Pin_10
#define WT2003_RX_PORT               GPIOB
#define WT2003_RX_PIN                GPIO_Pin_11
#define WT2003_BAUD                  9600U
#define WT2003_RX_BUF_SIZE           128U
#define WT2003_USE_CHECKSUM          1U       /* 1 = 帧尾前带累加和校验(WT2003HX 手册固定带),
                                                 0 = 不带 */
#define WT2003_LEN_COUNT_SELF        1U       /* 1 = 长度字节把[长度字节自己]也算进去
                                                 (WT2003HX 手册固定如此: 长度 = 1 + 命令 +
                                                 数据 + 校验码),
                                                 0 = 长度 = 命令 + 数据(+校验码) */
/* "按索引播放"走哪条命令 —— 只影响 wt2003_play_index()/wt2003_talk() 的串口通路:
    1 = A6 指定 U 盘根目录索引播放(《WT2003H4-16S-A01》手册 4.1.3.4;
        帧 7E 05 A6 <曲目高位> <曲目低位> <校验码> EF, 例 索引2 -> 7E 05 A6 00 02 AD EF);
    0 = A0 指定 SPI Flash 根目录索引播放(模块内置/外挂 SPI Flash 上的文件)。

    !!! 实测(模块固件 WTS251016-70-A01V4.01, 复现日志见 README 8.7 / 结论见 8.9):
        A0 索引 1        -> 回 02 = EMP(无此文件), 与本工程音频放在 U 盘相符;
        A6 索引 0/1/2/99 -> 一律回 0x05(手册未定义), 而且自检里 AC/AD(下一曲/上一曲)
                            真的出声、A6 一次都没出声 => 本模块的 A6 是"收下命令但不播"。
        => 要播指定语音请改走"按文件名播放"(A7):
           文件放 U 盘根目录、文件名固定 4 字符(如 T001.mp3 -> 数据 "T001"),
           驱动接口 wt2003_play_udisk_name("T001"), 配置见下面的 WT2003_NAME_CYCLE_*,
           串口命令 VNAME <n> / VNAMEON / VNAMEOFF。
    不想重烧就想两种都试: 串口发 VPLAY6 <idx>(A6) / VPLAY0 <idx>(A0) 对照听。 */
#define WT2003_PLAY_USE_UDISK        1U

/* Key 口(K1~K3)总开关:
   0 = 关闭(默认, 本项目的标准品 WT2003H4-16S 未开放 Key 口功能, 需芯片厂家定制):
         - wt2003_init() 不初始化 PB0~PB2, 也不建立"语音编号 1/2/3 -> K1/K2/K3"绑定;
         - wt2003_talk()/VOICE 一律走串口 A0 -> 绝不会出现"编号 1/2/3 静默无反应";
         - VKEY 直接回 ERR(提示已关闭)。
   1 = 打开(仅当模块已定制 Key 口功能时才用)。 */
#define WT2003_KEY_ENABLE            0U
#define WT2003_KEY_COUNT             3U       /* K1/K2/K3 */
#define WT2003_KEY_PORT              GPIOB
#define WT2003_KEY_GPIO_CLK          RCC_APB2Periph_GPIOB
#define WT2003_KEY1_PIN              GPIO_Pin_0
#define WT2003_KEY2_PIN              GPIO_Pin_1
#define WT2003_KEY3_PIN              GPIO_Pin_2
#define WT2003_KEY_PULSE_MS          100U     /* Key 口触发脉冲宽度 (拉低时间) */
#define WT2003_KEY_ROUTE_NUM         8U       /* ID -> Key 口 绑定表容量 */
#define WT2003_DEFAULT_VOLUME        31U      /* 上电默认音量 0~31 (手册: 上电默认最大 1F) */
#define WT2003_BOOT_VOICE            1U       /* 上电提示音编号(0 = 开机不播放) */
#define WT2003_BOOT_DELAY_MS         300U     /* 上电先等模块就绪(ms)再发首条命令 */
#define WT2003_PROBE_ON_BOOT         0U       /* 0 = 关闭(当前值): 上电不跑体检, 只按下面的
                                                 提示音/按文件名轮播播音频(见 8.1 节);
                                                 1 = 上电自动跑"语音链路体检"38 步
                                                 (AB 停止 -> D2 盘符=U 盘 -> B6 切 SPK ->
                                                  AE 音量 31 -> CA 查连接 / C7 查 U 盘文件数
                                                  (C3/C7 实测都只回 1 字节, 别拿来判介质) ->
                                                  C9/C2 基准 -> A6 播 U 盘索引 1 ->
                                                  AA 暂停后再 A6(判断"忙碌拒绝") ->
                                                  DAC/SPK 各播一次 -> AC/AD 换曲看 C9 ->
                                                  A6 播索引 2 扫 U 盘文件 -> AB 停止),
                                                 数码管显示步号, 日志见
                                                  WTTX/WTRX/WTRXINFO; 排查时置 1 即可;
                                                  实测结论(本模块 A6 收下命令但不出声、
                                                  AC/AD 才出声, 故改用 A7 按文件名播放)
                                                  见 README 8.9 */
#define WT2003_PROBE_INTERVAL_MS     1500U    /* 每步默认间隔(ms); 单步可用表里的 gap_ms 覆盖 */
#define WT2003_PROBE_ROUNDS          0U       /* 整个序列跑几轮(体检用 1 轮, 免得反复播) */

/*==============================================================================
 * 8.1 按文件名轮播(A7 播 U 盘根目录已知文件名) —— 排查"没声音"用, 也可当正式播放
 *     为什么用它: 实测本模块 A6(指定 U 盘索引播放)收到命令却不出声(回 0x05),
 *     而 A7 按文件名播放能出声(见 README 8.9)。
 *     前提: 音频文件放在 U 盘根目录, 且文件名**固定 4 个字符**(扩展名另算),
 *           例如 T001.mp3 / T002.mp3 ... -> 命令数据就是 "T001" / "T002"。
 *     开: WT2003_NAME_CYCLE_ON_BOOT = 1(上电立刻开始, 每 WT2003_NAME_CYCLE_MS 毫秒
 *         播下一个并循环; 串口打 "NAME n/N \"T00n\"", 数码管显示第几个)。
 *============================================================================*/
#define WT2003_NAME_CYCLE_ON_BOOT    1U       /* 1 = 上电开始按文件名轮播(当前排查用);
                                                 0 = 关闭(正式模式交给业务层点播) */
#define WT2003_NAME_CYCLE_MS         4000U    /* 每个文件播放/间隔时间(ms)。不要小于单个
                                                 音频时长, 否则下一首会把上一首切掉 */
#define WT2003_NAME_CYCLE_QUERY_MS   800U     /* 播后多少毫秒自动查一次 C9(当前曲目号) +
                                                 CB(当前曲目名), 用来确认"到底播没播";
                                                 必须 < WT2003_NAME_CYCLE_MS */
/* 要轮播的 4 字符文件名列表(按 U 盘里的真实名字改; 个数不限, 至少 1 个) */
#define WT2003_NAME_CYCLE_LIST        { "T001", "T002", "T003", "T004", "T005" }
#define WT2003_TX_TIMEOUT_MS         50U      /* 串口发送超时保护 */
#define WT2003_RX_FRAME_MAX          32U      /* 接收帧最大长度(实测 C0 版本查询的回帧较长,
                                                  16 字节不够会被整帧丢弃, 见 README 8.4) */
#define WT2003_RX_TRACE              1U       /* 0 = 关闭(默认, 正式模式静默, 不占串口带宽);
                                                 1 = 把模块返回帧实时打印到 ESP32 串口
                                                 (形如 "WTRX A0 00", 排查用; 判读见 README 第 8 节) */
#define WT2003_TX_TRACE              1U       /* 0 = 关闭(默认, 正式模式静默);
                                                 1 = 把主机"真正发出去"的每一帧原样打印到
                                                 ESP32 串口, 形如:
                                                     WTTX 7E 05 A6 00 01 AC EF   (发给模块: A6 播 U 盘索引 1)
                                                     WTRXRAW 7E 04 A6 00 AA EF   (模块回帧, 原始字节, 形如)
                                                 注: 按索引播放默认走 A6(播 U 盘, 见 WT2003_PLAY_USE_UDISK);
                                                     若选 A0(播 SPI Flash)则首帧是 7E 05 A0 00 01 A6 EF。
                                                 排查"到底发了几个字节/模块回什么"用, 不用挂
                                                 调试器也不用量逻辑分析仪; 排查时置 1 即可 */

/*==============================================================================
 * 9. RTOS (CMSIS-RTOS / Keil RTX) 与任务划分
 *    组件来源: Keil 的 Manage Run-Time Environment -> CMSIS -> RTOS (Keil RTX)
 *    - BSP_USE_RTOS = 1 : 各 *_process() 跑在独立任务里(推荐)
 *    - BSP_USE_RTOS = 0 : 回到 main 的主循环轮询(不需要 RTOS 也能编译运行)
 *    - 任务栈大小在 RTE\CMSIS\RTX_Conf_CM.c 的 OS_STKSIZE(单位: 字, 4 字节)
 *    - RTX 的 OS_CLOCK 必须等于实际主频(BSP_SYSCLK_HZ), 否则 osDelay 不准
 *============================================================================*/
#define BSP_USE_RTOS                 1U

/* 任务优先级: 对应 CMSIS-RTOS osPriority (-1=BelowNormal 0=Normal 1=AboveNormal 2=High) */
#define APP_TASK_PRIO_INPUT          1     /* 按键/电源: 要求响应快 */
#define APP_TASK_PRIO_COMM           0     /* 串口命令: 普通 */
#define APP_TASK_PRIO_LIGHT          0     /* 灯带动画: 普通 */
#define APP_TASK_PRIO_VOICE          (-1)  /* 语音: 可以慢一点 */

/* 任务周期(ms): 只影响"最坏反应时间", 不影响正确性(判定都基于时间戳) */
#define APP_TASK_INPUT_PERIOD_MS     5U      /* 按键扫描 + 电源键状态机 */
#define APP_TASK_LIGHT_PERIOD_MS     10U     /* 灯带动画 + 按键灯 */
#define APP_TASK_COMM_PERIOD_MS      100U    /* 串口命令 + 心跳(有数据会被立刻唤醒) */
#define APP_TASK_VOICE_PERIOD_MS     10U     /* 语音请求 + 返回帧解析(有请求立刻唤醒) */

/* 事件标志序号(0~15, 只在本任务内有意义, 不同用途不要撞车) */
#define BSP_SIGNAL_ESP32_RX          0U      /* 通信任务: 串口收到一整行 */
#define BSP_SIGNAL_KEY_EVENT         0U      /* 输入任务: 按键/电源键有事件 */
#define BSP_SIGNAL_VOICE_DONE        0U      /* 语音任务: 串口收到返回帧 */

/*==============================================================================
 * 10. 灯带消消乐 (游戏层 app_wsgame)
 *     玩法: 灯带是一条"运输带" —— 随机 蓝/绿/紫 的单颗灯从"远端"(索引 119)出发,
 *           整条缓慢往"近端"(索引 0)挪; 每 WSGAME_SPAWN_MS 在远端再补一颗,
 *           前一颗没消掉, 后面就紧跟着再来一颗, 场上的灯越积越多(间距约
 *           WSGAME_SPAWN_MS / WSGAME_TRACK_STEP_MS 颗);
 *           近端按 Key1/Key2/Key3 = 朝远端**发射**一颗同色子弹(比灯快得多),
 *           子弹撞上同色灯 -> 两颗一起消失(白闪 + 得分, 数码管显示);
 *           撞上不同色   -> 子弹自己也消失(色块留在原地, 不得分; 也不会挡住后面子弹);
 *           一直没消掉的灯走到近端 = "漏掉"(只计数不扣分, 近端微亮提示), 消失。
 *     按键: Key1/Key2/Key3 (PA0/PA1/PA2) = 蓝 / 绿 / 紫(按一下发一颗)
 *           Key4 (PA3) 短按              = 开始 / 暂停
 *     按键灯: 第 1/2/3 颗 = 蓝/绿/紫(与按键一一对应, 发射时闪一下);
 *             第 4 颗 = 运行指示。
 *     说明: 游戏运行时效果层(app_led_fx)会被切成 LED_FX_OFF, 由游戏独占灯带;
 *           串口 GAME START/STOP/STATUS/SPD/SPAWN/SHOT 见 README 3.9 / 第 4 节。
 *============================================================================*/
#define WSGAME_AUTOSTART             1U      /* 1 = 上电直接开局; 0 = 上电跑效果层
                                                 (COMET), 用 GAME START 或 Key4 开局 */
#define WSGAME_MAX_LEDS              WS2812_LED_NUM   /* 游戏作用的灯珠数 */

/* ---- 三色(蓝 / 绿 / 紫): 色块 / 子弹 / 按键灯用同一组值, 想换色调只改这里 ---- */
#define WSGAME_BLUE_R                0U
#define WSGAME_BLUE_G                0U
#define WSGAME_BLUE_B                255U
#define WSGAME_GREEN_R               0U
#define WSGAME_GREEN_G               255U
#define WSGAME_GREEN_B               0U
#define WSGAME_PURPLE_R              255U    /* 紫(取更醒目的品红, 想偏蓝紫就改成
                                                 128U 左右, 与蓝色区分度会下降) */
#define WSGAME_PURPLE_G              0U
#define WSGAME_PURPLE_B              255U
#define WSGAME_KEYLED_LEVEL          160U    /* 按键灯亮度(0~255): 三色直接给 255 又刺眼又
                                                容易看成一片白, 打个折颜色更好分辨 */

/* ---- 运输带(色块) ---- */
#define WSGAME_TRACK_STEP_MS         100U    /* 色块整体每挪一颗灯的时间(ms): 越大越慢;
                                                120 颗 x 100ms = 12s 从远端走到近端。
                                                感觉还是快/慢就改这里, 也可远程
                                                "GAME SPD <ms>" 实时调 */
#define WSGAME_TRACK_STEP_MIN        20U     /* GAME SPD 允许的最快(ms) */
#define WSGAME_TRACK_STEP_MAX        500U    /* GAME SPD 允许的最慢(ms) */
#define WSGAME_SPAWN_MS              600U    /* 远端每补一颗新灯的间隔(ms);
                                                600/100 => 相邻两颗间距约 6 颗灯。
                                                远程 "GAME SPAWN <ms>" 可实时调 */
#define WSGAME_SPAWN_MIN             100U    /* GAME SPAWN 允许的最快(ms) */
#define WSGAME_SPAWN_MAX             5000U   /* GAME SPAWN 允许的最慢(ms) */
#define WSGAME_AVOID_REPEAT          1U      /* 1 = 相邻两颗不同色(免得一眼一片同色) */

/* ---- 子弹(近端发射) ---- */
#define WSGAME_SHOT_STEP_MS          15U     /* 子弹每飞一颗灯的时间(ms): 越小越快,
                                                15ms x 120 = 1.8s 飞到远端。
                                                远程 "GAME SHOT <ms>" 可实时调 */
#define WSGAME_SHOT_STEP_MIN         5U      /* GAME SHOT 允许的最快(ms) */
#define WSGAME_SHOT_STEP_MAX         100U    /* GAME SHOT 允许的最慢(ms) */
#define WSGAME_MAX_SHOTS             8U      /* 同时最多几颗子弹(按键连点的上限) */

/* ---- 反馈 / 显示 ---- */
#define WSGAME_HIT_MS                200U    /* 命中处白闪时长(ms) */
#define WSGAME_MISS_MS               200U    /* 漏掉时近端(第 0 颗)灰白提示时长(ms) */
#define WSGAME_KEYLED_FLASH_MS       300U    /* 发射时对应按键灯闪烁提示时长(ms) */
#define WSGAME_KEYLED_BLINK_MS       100U    /* 上面那段闪烁里"亮-灭"的周期(ms) */
#define WSGAME_FRAME_MS              16U     /* 画面刷新周期(ms, 约 60fps) */
#define WSGAME_REVERSE_DIR           0U      /* 1 = 灯带装反了(近端在 119 侧)时改成 1,
                                                 只改变渲染方向, 逻辑完全不变 */
#define WSGAME_SEG_SCORE             1U      /* 1 = 用 4 位数码管显示得分(命中 +1, 开局清 0) */
#define WSGAME_TRACE                 1U      /* 1 = 从 ESP32 串口打 WSGAME 调试行
                                                 (START/SPAWN/SHOT/HIT/MISS/BLOCK/STOP);
                                                 正式出货想彻底静默再改成 0 */
#define WSGAME_LED3_R                0U      /* 第 4 颗按键灯 = 运行指示(微亮深绿, 省电) */
#define WSGAME_LED3_G                40U
#define WSGAME_LED3_B                0U


#ifdef __cplusplus
}
#endif

#endif /* __BSP_CONFIG_H */
