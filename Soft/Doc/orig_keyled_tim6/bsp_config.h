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
  *  按键灯 WS2812 (4灯)    PC4          TIM6 更新中断里数拍翻转 GPIO(时序同主灯带, 第 5 节)
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

/* 注意优先级顺序(数值越小越高): 串口必须比主灯带 DMA 高!
   主灯带的 DMA 中断里要跑 ws2812_encode()(编 60 颗灯 = 1000+ 次循环, 每 1.8ms 一次),
   如果它比串口高, 这几百微秒里串口接收中断进不来; F103 的 USART 只有 1 个字节的
   接收寄存器, 这期间到的字节就直接丢了(ORE) —— 现象就是"命令老是少字母"。
   主灯带被串口中断(3us 级)打断完全没关系: 它手上还有 1.8ms 的余量。 */
#define BSP_IRQ_PRIO_WS2812_DMA      1U   /* 主灯带 DMA 半/全传输中断, 窗口 1.8ms */
#define BSP_IRQ_PRIO_ESP32_UART      0U   /* 与 WiFi 模块通信: 最高, 一个字节都不能丢 */
#define BSP_IRQ_PRIO_WT2003_UART     2U   /* 语音模块串口 */
#define BSP_IRQ_PRIO_EXTI            3U   /* 按键 / 电源键外部中断 (只记时间戳) */
#define BSP_IRQ_PRIO_TIME            3U   /* TIM2 1ms 时基中断 */
/* 按键灯发帧中断(第 5 节 KEYLED_TIM = TIM6): 必须是最高(0), 而且不能有比它更高的 ——
   优先级相同或更低的都无法打断它, 所以它一边数 KEYLED_TIM->CNT 一边翻转 PC4 的
   时候, 时序是"死"的: 这就是它不用关总中断、也不用自标定的原因。
   代价: 它发一帧(约 120us)期间, 同/低优先级的中断(EXTI/TIM2/主灯带 DMA/语音串口)
   都得排队等一等 —— 它们都按"能等"设计(见各驱动注释); ESP32 串口收数走 DMA(第 7 节),
   那是硬件搬字节, 不受任何中断占用影响。 */
#define BSP_IRQ_PRIO_KEYLED_TIM      0U

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
 * 5. 按键灯 WS2812 (KEYLED_NUM 颗)
 *    硬件: 按键灯带的 DIN 接 PC4(独立数据线, 与主灯带 PB6 各走一条)。
 *    驱动: PC4 在 STM32F103 上**没有定时器通道**(PC4 只有 ADC12_IN14 一个复用功能),
 *          所以输出只能"软件翻转 GPIO"; 但时间基准交给一个没人用的基本定时器
 *          (KEYLED_TIM = TIM6 —— 本工程 TIM2 = 时基, TIM4 = 主灯带, TIM1/3/5/6/7/8 都空着):
 *          需要发帧时先在任务里"请求", 96 位时序由该定时器的更新中断一次发完,
 *          每一位 = "拉高 -> 数 KEYLED_TIM->CNT 的拍数 -> 拉低 -> 再数拍"
 *          (72MHz, 1 拍 = 13.89ns)。段长就是下面那几个拍数本身, 不用 CPU 空循环估时间,
 *          也不需要老办法那套"上电自标定 + 固定补偿"。发一帧约 120us, 期间靠中断优先级
 *          (第 1 节 BSP_IRQ_PRIO_KEYLED_TIM = 0)保证不被打断, 所以**不用关总中断**。
 *          想回退成老的 DWT 忙等方式: 把 KEYLED_USE_TIM_IRQ 改成 0U(其余不用动)。
 *    时序: **直接抄主灯带那一套**(第 4 节 WS2812_T0H_NS/T1H_NS/BIT_NS/RESET_US) ——
 *          主灯带(PB6 + TIM4/DMA)时序已经实测好用, 按键灯就照同样的时间走, 两条支路
 *          不再各调一套:
 *              0 码 = T0H 400 + T0L 850 = 1250ns = WS2812_BIT_NS
 *              1 码 = T1H 800 + T1L 450 = 1250ns = WS2812_BIT_NS
 *          位周期与高/低电平和主灯带逐位相同, 灯珠型号一样, 判定结果自然一样。
 *          现场还能实时改/看: KEYLED T0H/T0L/T1H/T1L <ns>、KEYLED DIAG(见 README 第 4 节)。
 *============================================================================*/
#define KEYLED_NUM                   4U       /* 灯珠数量(第 4 颗留给"运行指示") */
#define KEYLED_PORT                  GPIOC
#define KEYLED_PIN                   GPIO_Pin_4
#define KEYLED_GPIO_CLK              RCC_APB2Periph_GPIOC

/* ---- 位时序(ns): 与主灯带同值, 高 + 低 = 主灯带位周期 1250ns ---- */
#define KEYLED_T0H_NS                WS2812_T0H_NS                   /* = 400ns  0 码高电平 */
#define KEYLED_T0L_NS                (WS2812_BIT_NS - WS2812_T0H_NS) /* = 850ns  0 码低电平 */
#define KEYLED_T1H_NS                WS2812_T1H_NS                   /* = 800ns  1 码高电平 */
#define KEYLED_T1L_NS                (WS2812_BIT_NS - WS2812_T1H_NS) /* = 450ns  1 码低电平 */
#define KEYLED_RESET_US              WS2812_RESET_US                 /* = 300us  帧间复位低电平 */
/* ---- 输出方式: 1 = 定时器中断发帧(推荐: 不用关中断 / 不用标定) ---- */
#define KEYLED_USE_TIM_IRQ           1U

/* 下面四个指的是同一个定时器, 换定时器时一起改。
   选基本定时器 TIM6/TIM7 的原因: 它们只有"计数器 + 更新中断", 没有引脚、没有比较
   通道、不占 DMA —— 本工程就是拿它当"秒表"用, 最省资源。TIM6/TIM7 挂在 APB1,
   但 APB1 预分频 != 1, 所以它们的计数时钟 = 72MHz(所以下面 PSC 填 0)。
   注: 只有高级/通用定时器(TIM1~TIM5/TIM8)才有输出通道, 而 PC4 又没有它们的复用功能,
   所以"定时器直接输出波形"这条路在本板走不通 —— 必须 GPIO 翻转 + 定时器数拍。 */
#define KEYLED_TIM                   TIM6
#define KEYLED_TIM_CLK               RCC_APB1Periph_TIM6
#define KEYLED_TIM_IRQn              TIM6_IRQn
#define KEYLED_TIM_IRQHANDLER        TIM6_IRQHandler

/* 更新中断节拍(us): 平时进中断没事干(立刻返回); 有帧要发时 keyled_tx() 会**软件触发
   一次更新事件**让它马上发, 所以这个值只是"兜底心跳"(也顺便给 DIAG 一个跑的迹象)。
   注意 TIM6/TIM7 是 16 位计数器, PSC = 0 时最长只能 65536/72MHz ≈ 910us。 */
#define KEYLED_TIM_TICK_US           500U

/* 忙等循环的收尾误差(拍): "等到够拍数"那个循环本身要几拍才退得出来, 所以每一段都会
   比目标长这么一点点(通常 3~6 拍 = 40~80ns, 在 WS200 系列的 ±150ns 容差内, 默认不补偿)。
   要抠准: 先跑一帧, 敲 KEYLED DIAG 看 "meas" 那一行(实测 - 目标, 单位拍), 把它填进来,
   四段长度就正好是配置值(单位拍, 1 拍 = 13.89ns)。 */
#define KEYLED_TIM_WAIT_COMP_CYC     0U

/* ---- 下面三个只有 KEYLED_USE_TIM_IRQ = 0(老的 DWT 忙等发帧)时才用 ---- */
#define KEYLED_CALIB                 1U       /* 1 = 上电用 DWT 自标定(实测这四段
                                                 实际多长, 把延时函数/循环的固定开销
                                                 扣掉 —— 所以上面填多少, 灯珠就真
                                                 看到多少, 不用手工猜 KEYLED_CYC_COMP) */
#define KEYLED_CYC_COMP              5U       /* 只有 DWT 不可用(极端情况)时才用的固定补偿 */
#define KEYLED_LOCK_IRQ              1U       /* 老方式: 1 = 输出期间关中断(推荐) */
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
#define ESP32_RX_BUF_SIZE            512U     /* 接收环形缓冲(字节, 给 App 层读) */

/* ---- 接收方式: DMA 环形接收(推荐) ----
 *   1 = USART1_RX 交给 DMA1_Channel5 循环搬进 s_dma_buf[]，CPU 完全不参与接收中断。
 *       好处: 任何"关中断 / 高优先级中断抢占"都不影响收数(按键灯发帧关中断 ~120us、
 *             主灯带 DMA 中断在编颜色, 都再也不会挤掉串口字节)。
 *       代价: 不能再靠中断唤醒任务, 所以通信任务周期要短一点(APP_TASK_COMM_PERIOD_MS),
 *             并且 esp32_process() 必须被周期性调用(内部把 DMA 里的字节搬进环形缓冲)。
 *   0 = 退回老的"每字节一次 RXNE 中断"方式(配合下面的优先级, 也够用)。
 *       DMA 环境有问题时用它兜底, 其它代码不用改。 */
#define ESP32_RX_USE_DMA             1U
#define ESP32_RX_DMA_CLK             RCC_AHBPeriph_DMA1
#define ESP32_RX_DMA_CHANNEL         DMA1_Channel5    /* USART1_RX = DMA1 通道 5(固定) */
#define ESP32_RX_DMA_SIZE            512U    /* 必须是 2 的幂! DMA 窗口: 超过这个字节数
                                                   还没被 esp32_process() 取走就会覆盖丢数 */
/* ---- 原始接收数据全局缓冲区(调试用, 一直开, 循环保存"最近多少个字节") ----
 *   收到的**每一个字节**(没经过任何过滤)按到达顺序存进 g_esp32_rx_cap[], 同一位置
 *   g_esp32_rx_flg[] 记这个字节的标记(见 drv_esp32.h 的 ESP32_CAP_FLAG_*):
 *   线上报错 / 是换行 / 与上一个字节隔了很久(一批数据的开头) / 被行过滤丢掉 /
 *   缓冲满了没存下 / DMA 窗口被覆盖(这里少了若干字节)。
 *   Keil 里可以直接看/watch 这两个数组 + g_esp32_rx_cap_idx, 串口侧用 UART DUMP 打印。 */
#define ESP32_CAP_SIZE               1024U   /* 必须是 2 的幂! 保存最近 N 个原始字节 */
#define ESP32_CAP_GAP_MS             5U      /* 两个字节之间静默超过这个值 => 标记"新一批"
                                                  (上位机换命令/桥上停顿会看到这个标记) */

#define ESP32_LINE_MAX               127U     /* 单条命令最大长度(超过 = 整行作废, 丢弃到下一个换行) */
#define ESP32_LINE_IDLE_MS           100U     /* "静默这么久还没等到换行"就把这段当成一整行:
                                                  有些上位机/工具发完命令不带 \r\n, 没这条
                                                  规则就永远等不到换行, 表现是"发什么都没
                                                  反应"; 值要明显大于"人工逐字符敲"的间隔,
                                                  手工敲太慢就把它设成 0(只认 \r\n) */
#define ESP32_LINE_KEEP_MAX          64U      /* 留一份"最近收到的整行"给串口命令 UART 回看 */
#define ESP32_TX_TIMEOUT_MS          50U      /* 发送超时保护 */

/*==============================================================================
 * 8. WT2003HX 语音模块 (USART3: PB10=TX, PB11=RX) —— 只用 4 条命令
 *
 *  串口帧格式(手册): [7E] [长度] [命令] [数据…] [校验码] [EF]
 *      长度   = 1(长度字节自身) + 1(命令) + N(数据) + 1(校验码)
 *      校验码 = 从"长度"字节起逐字节累加和 (mod 256)
 *
 *  本工程只保留 4 条实测可用的命令(其余命令/上电自检/Key 口全部退役,
 *  退役代码与旧命令表见 Doc/WT2003_退役命令与自检备份.md):
 *      设置音量         7E 04 AE <vol> <ck> EF        wt2003_set_volume()
 *      查询文件总数     7E 03 C4 C7 EF                wt2003_query_count()
 *      按根目录索引播放 7E 05 9D <高> <低> <ck> EF     wt2003_play_index()
 *      按文件名播放     7E 07 9E <4 字符> <ck> EF      wt2003_play_name()
 *  例: 索引 1 -> 7E 05 9D 00 01 A3 EF ;  "0001" -> 7E 07 9E 30 30 30 31 66 EF
 *============================================================================*/
#define WT2003_UART                  USART3
#define WT2003_UART_CLK              RCC_APB1Periph_USART3
#define WT2003_UART_IRQn             USART3_IRQn
#define WT2003_TX_PORT               GPIOB
#define WT2003_TX_PIN                GPIO_Pin_10
#define WT2003_RX_PORT               GPIOB
#define WT2003_RX_PIN                GPIO_Pin_11
#define WT2003_BAUD                  9600U
#define WT2003_RX_BUF_SIZE           64U      /* 接收环形缓冲(字节) */
#define WT2003_RX_FRAME_MAX          16U      /* 单帧最大长度(收帧解析用) */
#define WT2003_TX_TIMEOUT_MS         50U      /* 发送超时保护 */
#define WT2003_CMD_GAP_MS            30U      /* 两条命令之间的最小间隔: 发太密模块会丢命令 */

/* 命令码: 换固件/换手册只改这 4 行(驱动按长度自动算校验码) */
#define WT2003_CMD_SET_VOLUME        0xAEU    /* 设置音量: 数据 1 字节 0~31 */
#define WT2003_CMD_QUERY_COUNT       0xC4U    /* 查询文件总数: 无数据, 回 1 字节 */
#define WT2003_CMD_PLAY_INDEX        0x9DU    /* 按根目录索引播放: 数据 2 字节(高/低), 1 起 */
#define WT2003_CMD_PLAY_NAME         0x9EU    /* 按文件名播放: 数据 4 字节 ASCII, 如 "0001" */

/* 音量(0 = 静音, 31 = 最大)与上电动作 */
#define WT2003_VOL_MIN               0U
#define WT2003_VOL_MAX               31U
#define WT2003_DEFAULT_VOLUME        31U      /* 要设置的音量值(0~31) */
#define WT2003_SET_VOLUME_ON_BOOT    0U       /* 1 = 上电发一次"设置音量";
                                                 注: 本项默认 0 —— 先在串口用 VOL 命令确认
                                                 模块确实接受 0xAE 之后再置 1 */
#define WT2003_QUERY_COUNT_ON_BOOT   1U       /* 1 = 上电查一次文件总数(看 WTRXCOUNT 行) */
#define WT2003_BOOT_PLAY             1U       /* 1 = 上电播一次开机提示音(WT2003_BOOT_FILE) */
#define WT2003_BOOT_DELAY_MS         300U     /* 上电先等模块就绪(ms)再发第一条命令:
                                                 模块上电要做内部初始化, 命令发太早会被忽略 */
#define WT2003_TRACE                 1U       /* 1 = 收发帧从 ESP32 串口打一行:
                                                     WTTX 7E 05 9D 00 01 A3 EF   发出去的帧
                                                     WTRX 9D 00                  模块回帧
                                                     WTRXCOUNT 6                 查到的文件总数
                                                 0 = 静默(正式固件可关) */

/*==============================================================================
 * 8.1 语音文件表 —— 文件名固定 4 个字符, 放 U 盘根目录(扩展名 .mp3 另算)
 *     业务层点播:  wt2003_play_name(WT2003_FILE_KEY);      // 按键音
 *     加一条语音:  1) 把音频拷进 U 盘根目录, 名字按编号(0001.mp3 …);
 *                  2) 在下面加一行 #define(4 个字符)。
 *     注: 先把"编号 -> 用途"占位定下来, 以后换音频内容不用改代码。
 *     串口点播:    VOICE 0002 / VNAME 0002(见 README 第 4 节)
 *============================================================================*/
#define WT2003_FILE_KEY              "0001"   /* 按键音 */
#define WT2003_FILE_CLEAR            "0002"   /* 消除音效(消掉一颗灯) */
#define WT2003_FILE_START            "0003"   /* 开始游戏 */
#define WT2003_FILE_OVER             "0004"   /* 游戏结束 */
#define WT2003_FILE_POWER_OFF        "0005"   /* 关机提示音 */
#define WT2003_BOOT_FILE             "0006"   /* 开机提示音(WT2003_BOOT_PLAY = 1 时播) */

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
#define APP_TASK_COMM_PERIOD_MS      10U     /* 串口命令 + 心跳(有数据会被立刻唤醒)
                                                  注意: ESP32_RX_USE_DMA=1 时收数不再靠
                                                  中断唤醒, 这个值就是"命令最大响应延迟",
                                                  别设大(10ms 足够, 任务是空转极便宜) */
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
#define WSGAME_MAX_LEDS              WS2812_LED_NUM   /* 游戏作用的灯珠数(逻辑长度: 0 = 近端) */

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
