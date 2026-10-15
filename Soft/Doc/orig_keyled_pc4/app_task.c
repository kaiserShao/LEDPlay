/**
  ******************************************************************************
  * @file    app_task.c
  * @brief   任务创建与任务体实现(RTOS 模式)
  *
  *  这里只做"调度", 不做业务: 每个任务体就是"调用驱动的 *_process() + 让出
  *  CPU"。因此:
  *     * 想改周期/优先级 -> 改 bsp_config.h 第 9 节;
  *     * 想加任务        -> 在下面加一个 app_task_xxx() 并在 app_task_start()
  *                          里 bsp_thread_create(); 同时确认 RTX_Conf_CM.c 的
  *                          OS_TASKCNT 够用(本工程已设为 8, 用了其中的 4 个)。
  *
  *  任务之间共享资源(串口、灯带缓冲、按键灯)全部通过 bsp_mutex_t 保护,
  *  中断 -> 任务的通知通过 bsp_signal_t / bsp_queue_t 完成, 不使用全局标志轮询。
  ******************************************************************************
  */
#include "app_task.h"
#include "bsp_rtos.h"

#include "drv_key.h"
#include "drv_power.h"
#include "drv_ws2812.h"
#include "drv_keyled.h"
#include "drv_esp32.h"
#include "drv_wt2003h.h"

#include "app_led_fx.h"
#include "app_cmd.h"
#include "app_wsgame.h"

static uint8_t s_started = 0U;

/*===========================================================================
 *                          1. 输入任务(按键/电源)
 *=========================================================================*/
static void app_task_input(void const *arg)
{
    (void)arg;

    for (;;)
    {
        drv_key_process();      /* 消抖 + 短按/长按/长按保持判定(基于中断时间戳) */
        power_process();        /* 电源键长按关机 */

        /* 5ms 一次: 消抖 20ms 的判定误差 ≤5ms, 人感觉不到 */
        bsp_sleep_ms(APP_TASK_INPUT_PERIOD_MS);
    }
}

/*===========================================================================
 *                          2. 灯光任务(主灯带 + 按键灯)
 *=========================================================================*/
static void app_task_light(void const *arg)
{
    (void)arg;

    for (;;)
    {
        /* 顺序有讲究: 游戏先跑(它运行时独占灯带), 再跑效果层 —— 这样
           "GAME STOP 之后接 FX"能在同一拍里无缝接上, 不会闪一下黑 */
        wsgame_process();       /* 灯带消消乐: 取按键 + 运输带/子弹推进 + 渲染 */
        led_fx_process();       /* 算颜色 + ws2812_show(), 输出本身由 DMA 完成 */
        keyled_process();       /* 按键灯闪烁/呼吸(输出期间关中断约 120us) */
        ws2812_process();       /* 预留扩展点 */

        bsp_sleep_ms(APP_TASK_LIGHT_PERIOD_MS);
    }
}

/*===========================================================================
 *                          3. 通信任务(ESP32 / 命令)
 *=========================================================================*/
static void app_task_comm(void const *arg)
{
    (void)arg;

    for (;;)
    {
        /* 有完整命令行时被中断立刻唤醒, 最多等 APP_TASK_COMM_PERIOD_MS */
        if (esp32_wait_data(APP_TASK_COMM_PERIOD_MS) != 0U)
        {
            esp32_process();    /* 拆行 -> app_cmd 的命令回调(在任务上下文执行) */
        }

        app_cmd_process();      /* 心跳上报(内部按时间戳判断, 不需要精确周期) */
    }
}

/*===========================================================================
 *                          4. 语音任务(WT2003HX)
 *=========================================================================*/
static void app_task_voice(void const *arg)
{
    (void)arg;

    for (;;)
    {
        /* 收帧解析: 模块的应答(查文件总数等)在这里被拼出来。
           发命令由业务层直接调用驱动完成(驱动内部用互斥锁保护串口),
           所以本任务只负责"收", 顺便让出 CPU。 */
        wt2003_process();

        bsp_sleep_ms(APP_TASK_VOICE_PERIOD_MS);
    }
}

/*===========================================================================
 *                            创建 + 启动
 *=========================================================================*/
void app_task_start(void)
{
    uint8_t ok = 0U;

    if (bsp_rtos_running() != 0U)
    {
        return;                 /* 已经在跑了(重复调用保护) */
    }

    ok = (uint8_t)(ok + bsp_thread_create(app_task_input, NULL, APP_TASK_PRIO_INPUT));
    ok = (uint8_t)(ok + bsp_thread_create(app_task_light, NULL, APP_TASK_PRIO_LIGHT));
    ok = (uint8_t)(ok + bsp_thread_create(app_task_comm,  NULL, APP_TASK_PRIO_COMM));
    ok = (uint8_t)(ok + bsp_thread_create(app_task_voice, NULL, APP_TASK_PRIO_VOICE));

    s_started = (ok >= 4U) ? 1U : 0U;

    /* 启动调度器: RTX 模式下不返回; 未使能 RTOS 时立刻返回 */
    bsp_rtos_start();

    /* 走到这里说明没有在用 RTOS: 各 *_process() 交给 main 的主循环轮询,
       s_started 保持 0, 业务层可用 app_task_started() 判断当前模式。 */
    s_started = 0U;
}

/*---------------------------------------------------------------------------*/
uint8_t app_task_started(void)
{
    return s_started;
}
