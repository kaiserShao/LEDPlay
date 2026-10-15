/**
  ******************************************************************************
  * @file    app_wsgame.h
  * @brief   灯带消消乐(游戏层): 灯带当"运输带", 近端按同色键"发射"子弹打同色灯
  *
  *  玩法
  *  ---------------------------------------------------------------------------
  *  ① 运输带: 随机 蓝 / 绿 / 紫 的单颗灯从"远端"冒出来, 整条按 WSGAME_TRACK_STEP_MS
  *     缓慢往"近端"挪; 每 WSGAME_SPAWN_MS 在远端再补一颗 —— 前一颗没消掉, 后面就
  *     紧跟着再来一颗, 场上的灯越积越多, 一起朝近端挤;
  *  ② 发射: 近端按 Key1/Key2/Key3 = 朝远端"发射"一颗同色子弹(快得多,
  *     WSGAME_SHOT_STEP_MS 一颗灯), 最多同时 WSGAME_MAX_SHOTS 颗;
  *  ③ 判定: 子弹撞上色块 —— 颜色相同 -> 两颗一起消失(白闪 + 得分);
  *                          颜色不同 -> 子弹自己也消失(色块留在原地, 不得分);
  *  ④ 漏掉: 一直没被消掉的灯走到近端(索引 0) -> 只计"漏掉"数, 近端灰白微亮提示后消失。
  *
  *  远 / 近端的定义
  *  ---------------------------------------------------------------------------
  *  索引 0   = 数据入口(DIN)那一端 = 近端(按键在这头, 灯冲着你来, 子弹从这里发出);
  *  索引 119 = 灯带末端             = 远端(灯从这里冒出来)。
  *  实物如果装反了(近端在 119 侧, 看着像往外跑), 不用改接线: 把 bsp_config.h 里的
  *  WSGAME_REVERSE_DIR 改成 1 即可(只影响渲染, 逻辑完全不变)。
  *
  *  按键(PA0~PA3, 见 bsp_config.h 第 1 节)
  *  ---------------------------------------------------------------------------
  *      Key1(PA0) = 蓝      Key2(PA1) = 绿      Key3(PA2) = 紫(按一下发一颗子弹)
  *      Key4(PA3) = 开始 / 暂停(短按)     <- 不用串口也能开局
  *  改按键只需改下面的 WSGAME_KEY_xxx(取值见 drv_key.h 的 KEY_1~KEY_7)。
  *
  *  按键灯(drv_keyled 的 4 颗)
  *  ---------------------------------------------------------------------------
  *      第 1/2/3 颗 = 蓝 / 绿 / 紫(与按键一一对应, 颜色取同一组宏), 平时常亮本色;
  *      发射时对应那颗闪一下(WSGAME_KEYLED_FLASH_MS), 一眼知道"这一发出去了";
  *      第 4 颗 = 运行指示(运行中微亮, 停止后熄灭)。
  *  游戏停止后按键灯回到"平时常亮"本色(wsgame_keyled_idle(), app_cmd 上电也会调一次)。
  *
  *  声音(WT2003HX 语音模块, 宏见 bsp_config.h 第 8/8.1/10 节)
 *  ---------------------------------------------------------------------------
 *  背景音: 开局把模块设成"单曲循环"(AF 01, 帧 = 7E 04 AF 01 B4 EF)再点播
 *          WT2003_BGM_INDEX(按索引 0x9D 播放, 帧 = 7E 05 9D 00 01 A3 EF)——
 *          一首循环到底; 收局先取消循环(AF 00)再停止播放(AA)。
 *  音效:   发射 / 命中消除 / 未消除(漏掉) 三种, 发法由 WSGAME_SFX_MODE 选:
 *          0 = 插播(B1, 现用): 7E 06 B1 <标示字> <曲目高> <曲目低> <ck> EF, 叠在背景音上放,
 *              背景音不停、不重头; 标示字取 WT2003_INSERT_FLAG
 *              (00 = FLASH(外挂 SPI Flash 片)【现用】, 02 = U 盘)
 *          1 = 打断式: 音效走 9E 按文件名点播, 背景音被顶掉, WSGAME_SFX_LEN_MS
 *              毫秒后再 AF 01 + 重播背景音接上(只用 9E/AF, 只听到"音乐重头"一下)。
 *          曲目索引(外挂 Flash 上按放入顺序): 发射 = WSGAME_SFX_SHOT_INDEX(1) /
 *                    消除 = _CLEAR_INDEX(2) / 未消除 = _MISS_INDEX(3)。
 *  注: 撞上"异色"灯也算没消掉 —— 除了漏到近端, 异色撞击时也会补一声未消除音效。
 *  总开关: WSGAME_SOUND_ENABLE / WSGAME_BGM_ENABLE / WSGAME_SFX_ENABLE
 *          (串口单独试听: VINS 1 / VINS 2 / VINS 3)
 *
 *  与效果层(app_led_fx)的关系
  *  ---------------------------------------------------------------------------
  *  游戏运行时把效果层切成 LED_FX_OFF, 由本模块独占灯带颜色缓冲;
  *  wsgame_stop() 时按"开局前记录的那个效果"还原(上电本来就是 OFF 的话还原成 COMET),
  *  灯带又归效果层自己画。
  *
  *  线程模型
  *  ---------------------------------------------------------------------------
  *  游戏状态只在 wsgame_process() 里改(灯光任务, 10ms 一拍); 其它任务(命令任务)
  *  只用 s_req / s_xxx_ms 这样的单字请求打招呼, 下一拍被消费 —— 无需加锁。
  *
  *  用法
  *  ---------------------------------------------------------------------------
  *      wsgame_init();                      // 初始化(按 WSGAME_AUTOSTART 决定是否开局)
  *      灯光任务(10ms): wsgame_process();    // 取按键事件 + 推进运输带/子弹 + 渲染
  *  远程: GAME STATUS / GAME START / GAME STOP
  *        GAME SPD <ms> 运输带速度(越大越慢)   GAME SPAWN <ms> 远端口子
  *        GAME SHOT <ms> 子弹速度(越小越快)     (见 README 3.9 / 第 4 节)
  ******************************************************************************/
#ifndef __APP_WSGAME_H
#define __APP_WSGAME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_config.h"
#include "drv_key.h"            /* KEY_1 ~ KEY_7: 下面的按键映射用 */

/*--------------------------- 按键 -> 颜色 映射 ----------------------------*/
#define WSGAME_KEY_BLUE       KEY_1     /*!< 蓝色键 (PA0) */
#define WSGAME_KEY_GREEN      KEY_2     /*!< 绿色键 (PA1) */
#define WSGAME_KEY_PURPLE     KEY_3     /*!< 紫色键 (PA2) */
#define WSGAME_KEY_START      KEY_4     /*!< 开始/暂停键 (PA3, 短按) */

/*------------------------------- 颜色编号 --------------------------------*/
#define WSGAME_COLOR_BLUE     0U        /*!< 蓝 */
#define WSGAME_COLOR_GREEN    1U        /*!< 绿 */
#define WSGAME_COLOR_PURPLE   2U        /*!< 紫 */
#define WSGAME_COLOR_NUM      3U        /*!< 颜色种类数 */
#define WSGAME_COLOR_NONE     0xFFU     /*!< 没有颜色(内部用: 上一次生成的色号) */

/*------------------------------- 游戏状态 --------------------------------*/
typedef enum
{
    WSGAME_ST_OFF = 0,   /*!< 未运行(灯带已交还效果层) */
    WSGAME_ST_RUN = 1    /*!< 运行中(运输带 + 子弹) */
} wsgame_state_t;

/** 游戏快照(给 GAME STATUS 用) */
typedef struct
{
    uint8_t  running;        /*!< 1 = 运行中 */
    uint8_t  state;          /*!< wsgame_state_t */
    uint16_t blocks;         /*!< 场上还活着几颗灯 */
    uint16_t shots;          /*!< 在飞的子弹数 */
    uint16_t track_step_ms;  /*!< 运输带速度: 每挪一颗灯的时间 */
    uint16_t spawn_ms;       /*!< 远端补灯的口子: 每多少毫秒冒一颗 */
    uint16_t shot_step_ms;   /*!< 子弹速度: 每飞一颗灯的时间 */
    uint16_t score;          /*!< 得分(命中 +1) */
    uint16_t hits;           /*!< 累计命中次数 */
    uint16_t misses;         /*!< 累计漏掉次数 */
    uint16_t spawned;        /*!< 本局一共冒出过几颗灯 */
} wsgame_info_t;

/** 初始化(按 WSGAME_AUTOSTART 决定是否立刻开局); 在 keyled/hc595/ws2812 初始化之后调用 */
void wsgame_init(void);

/** 开始(重开)一局: 顺带把效果层切到全灭, 灯带交给游戏; 得分/计数清零 */
void wsgame_start(void);

/** 停止: 清屏 + 按键灯回到"平时常亮"本色 + 灯带交还效果层(得分保留在数码管上) */
void wsgame_stop(void);

/** 按键灯"平时常亮": 第 1/2/3 颗 = 蓝 / 绿 / 紫(与游戏同色), 第 4 颗灭。
    上电和游戏停止后调用(按键灯不随按键亮/灭); 用 keyled_xxx 的 API, 不依赖游戏是否初始化 */
void wsgame_keyled_idle(void);

/** 1 = 游戏运行中(此时按键灯由本模块管理) */
uint8_t wsgame_is_running(void);

/** 运行中就停, 停着就开始(Key4 短按走的就是它); 返回"期望"状态, 实际下一拍生效 */
uint8_t wsgame_toggle(void);

/** 运输带速度: 色块每挪一颗灯的时间(ms), 自动裁剪到 WSGAME_TRACK_STEP_MIN/MAX */
void wsgame_set_step_ms(uint16_t step_ms);
uint16_t wsgame_get_step_ms(void);

/** 远端口子: 每多少毫秒冒一颗新灯, 自动裁剪到 WSGAME_SPAWN_MIN/MAX */
void wsgame_set_spawn_ms(uint16_t spawn_ms);
uint16_t wsgame_get_spawn_ms(void);

/** 子弹速度: 每飞一颗灯的时间(ms), 自动裁剪到 WSGAME_SHOT_STEP_MIN/MAX */
void wsgame_set_shot_ms(uint16_t shot_step_ms);
uint16_t wsgame_get_shot_ms(void);

/** 取当前状态快照(命令任务用; 内部一次性读, 允许看到"正在变"的中间态) */
void wsgame_get_info(wsgame_info_t *info);

/** 状态名 / 颜色名(串口应答用) */
const char *wsgame_state_name(uint8_t state);
const char *wsgame_color_name(uint8_t color);

/** 灯光任务(10ms)调用: 取按键事件 + 推进运输带/子弹 + 渲染 + 按键灯 */
void wsgame_process(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_WSGAME_H */
