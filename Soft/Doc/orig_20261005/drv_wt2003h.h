/**
  ******************************************************************************
  * @file    drv_wt2003h.h
  * @brief   WT2003H4-16S 语音模块驱动 (USART3: PB10=TX/PB11=RX, Key: PB0~PB2)
  *
  *  两条控制通路, 统一入口:
  *      1) 串口命令(USART3, 默认 9600 8N1): 可指定任意曲目号, 信息量大。
  *      2) 硬件 Key 口(K1~K3, 低脉冲触发): 由模块自己播放事先用上位机工具
  *         配置好的文件, 从触发到出声最快, 不占用串口。
  *      业务层只需要调用一个函数:
  *          wt2003_talk(voice_id);   // 自动选路(查绑定表, 没绑定就走串口)
  *
  *  !!! 协议说明(对齐《WT2003HX 芯片》手册) !!!
  *      串口帧格式(手册 5.1.2 各命令通用):
  *          [0x7E] [长度] [命令] [数据...] [校验码] [0xEF]
  *          长度   = 1(长度字节自身) + 1(命令) + N(数据) + 1(校验码)
  *          校验码 = 从"长度"字节开始逐字节累加和(mod 256)
  *      例(手册原例): A0 SPI Flash 索引 1 -> 7E 05 A0 00 01 A6 EF
  *                    A6 U 盘索引 2     -> 7E 05 A6 00 02 AD EF
  *                    音量 = 31         -> 7E 04 AE 1F D1 EF
  *      对应 bsp_config.h 的 WT2003_USE_CHECKSUM / WT2003_LEN_COUNT_SELF 都必须为 1。
  *      !!! A0 只播 SPI Flash 上的文件, A6 才播 U 盘上的文件 —— 命令用错时模块会回非 0
  *          ("指定曲目不存在")而不出声。本项目音频放在 U 盘上, 所以用 bsp_config.h 的
  *          WT2003_PLAY_USE_UDISK=1 选 A6; 也可直接调 wt2003_play_udisk()/wt2003_play_flash()
  *          或用串口 VPLAY6/VPLAY0 显式指定。!!!
  *      若某个命令不生效, 优先检查: 命令码(下面的 WT2003_CMD_xxx)、
  *      波特率(WT2003_BAUD) 、以及模块的音频输出方式(B6: SPK/DAC)。
  ******************************************************************************
  */
#ifndef __DRV_WT2003H_H
#define __DRV_WT2003H_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_config.h"

/*---------------------------------------------------------------------------
 * 串口命令码 (WT2003HX 手册: 5.1.2 节 / 表3 通信控制指令 / 表4 通信查询命令)
 *-------------------------------------------------------------------------*/
/* ---- 控制命令(手册 4.1.2 "通信控制指令") ---- */
#define WT2003_CMD_PLAY_INDEX    0xA0U   /* A0 指定 SPI Flash 根目录索引播放, 数据 2 字节(曲目高位/低位) */
#define WT2003_CMD_PLAY_FLASHNAME 0xA1U  /* A1 指定 Flash 按文件名播放, 数据 = 文件名(4 字节 ASCII) */
#define WT2003_CMD_PLAY_UFLASH   0xA6U   /* A6 指定 U 盘根目录索引播放,   数据 2 字节(曲目高位/低位) */
#define WT2003_CMD_PLAY_UNAME    0xA7U   /* A7 指定 U 盘文件名播放,       数据 = 文件名(4 字节 ASCII) */
#define WT2003_CMD_PLAY_UFOLDER  0xA8U   /* A8 指定 U 盘文件夹内文件索引播放,
                                            数据 = 5 字节文件夹名(ASCII) + 2 字节索引(高/低) */
#define WT2003_CMD_PAUSE         0xAAU   /* AA 暂停放音(播放 <-> 暂停 切换) */
#define WT2003_CMD_STOP          0xABU   /* AB 停止命令 */
#define WT2003_CMD_PLAY_NEXT     0xACU   /* AC 下一曲命令 */
#define WT2003_CMD_PLAY_PREV     0xADU   /* AD 上一曲命令 */
#define WT2003_CMD_SET_VOLUME    0xAEU   /* AE 音量控制命令, 数据 1 字节 0~31 (0=静音, 31=最大) */
#define WT2003_CMD_SET_MODE      0xAFU   /* AF 指定播放模式, 数据 1 字节:
                                            00=单曲不循环(默认) 01=单曲循环 02=全部循环 03=随机 */
#define WT2003_CMD_PLAY_COMBO    0xB0U   /* B0 组合播放(本版手册未列出, 保留兼容) */
#define WT2003_CMD_INSERT        0xB1U   /* B1 插播, 数据 = 标示字(00=SPI Flash / 02=U 盘) + 2 字节索引 */
#define WT2003_CMD_SET_OUTPUT    0xB6U   /* B6 音频输出方式切换, 数据 1 字节: 0x00=SPK / 0x01=DAC
                                            (手册 4.1.4.14: 该命令示例"无返回码") */
#define WT2003_CMD_FLASH_PIN     0xB7U   /* B7 Flash 引脚加载和释放, 数据 1 字节: 00=释放 / 01=加载 */
#define WT2003_CMD_LOW_POWER     0xB8U   /* B8 进入低功耗(本版手册未列出, 保留兼容) */
#define WT2003_CMD_SET_DRIVE     0xD2U   /* D2 切换当前工作盘符, 数据 1 字节: 00=SPI-Flash / 02=U 盘 */

/* ---- 查询命令(手册 4.1.2 "通信查询命令") ---- */
#define WT2003_CMD_GET_VERSION   0xC0U   /* C0 查询当前软件版本(本版手册未列出, 实测有应答) */
#define WT2003_CMD_GET_VOLUME    0xC1U   /* C1 查询当前设置音量, 回 1 字节 00~1F */
#define WT2003_CMD_GET_STATUS    0xC2U   /* C2 读取当前工作状态, 回 1 字节: 01=播放 02=停止 03=暂停 */
#define WT2003_CMD_GET_FILE_NUM  0xC3U   /* C3 查询 SPI Flash 内音乐文件总数, 回 2 字节 */
#define WT2003_CMD_GET_UDISK_NUM  0xC7U  /* C7 查询 U 盘内音乐文件总数,   回 2 字节  <== 本项目看这条 */
#define WT2003_CMD_GET_UFOLDER_NUM 0xC8U /* C8 查询 U 盘指定文件夹内文件总数,
                                            发: 数据 = 5 字节文件夹名(ASCII);
                                            回: 2 字节(都 0 = 无此文件夹/无音频文件) */
#define WT2003_CMD_GET_CUR_FILE  0xC9U   /* C9 查询当前播放文件曲目, 回 2 字节(文件编号高/低) */
#define WT2003_CMD_GET_CONN      0xCAU   /* CA 查询当前外设连接状态, 回 1 字节:
                                            bit3=PC 连接 / bit2=U 盘 / bit0=SPI-Flash (1=存在);
                                            插拔 U 盘时模块也会主动上报这条命令 */
#define WT2003_CMD_GET_CUR_NAME  0xCBU   /* CB 查询当前播放歌曲名, 回 8 字节 ASCII(不足补 0x20) */

/* 音量范围(手册 5.1.2.7: 共 32 级, 0 = 静音, 31 = 最大, 掉电记忆) */
#define WT2003_VOL_MIN           0U
#define WT2003_VOL_MAX           31U

/** 音频输出方式(B6 命令的数据字节) */
typedef enum
{
    WT2003_OUT_SPK = 0,   /*!< 0x00: 切换到 SPK(内部功放 / 喇叭) */
    WT2003_OUT_DAC = 1    /*!< 0x01: 切换到 DAC 输出 */
} wt2003_out_t;

/** 当前工作盘符(D2 命令 / B1 插播"标示字"的数据字节; 手册 4.1.3.16 / 4.1.3.13) */
typedef enum
{
    WT2003_DRIVE_FLASH = 0x00,   /*!< 0x00: SPI-Flash */
    WT2003_DRIVE_UDISK = 0x02    /*!< 0x02: U 盘(本项目音频放这里) */
} wt2003_drive_t;

/** 语音来源 */
typedef enum
{
    WT2003_SRC_AUTO = 0,   /*!< 自动: 在绑定表中找到 Key 就发 Key 脉冲, 否则走串口 */
    WT2003_SRC_UART = 1,   /*!< 强制走串口(可指定任意曲目号) */
    WT2003_SRC_KEY  = 2    /*!< 强制走硬件 Key 口(需先用 wt2003_bind_key 绑定) */
} wt2003_src_t;

/** 收到模块返回帧时的回调(命令码 + 数据) */
typedef void (*wt2003_frame_cb_t)(uint8_t cmd, const uint8_t *data, uint8_t len);

/** 发出帧时的回调(原始字节, 含 7E / 长度 / 命令 / 数据 / 校验码 / EF) —— 排查用 */
typedef void (*wt2003_tx_cb_t)(const uint8_t *frame, uint8_t len);

/** 收到"结构不对"的帧时的回调(raw = 从帧头到出问题那个字节的原样字节) —— 排查用:
    帧尾不是 EF(长度字节口径与手册不同)、或帧长超过 WT2003_RX_FRAME_MAX 被丢弃时触发;
    以前这些字节会被静默丢掉, 现在能打出来看"模块到底回了什么"。 */
typedef void (*wt2003_bad_frame_cb_t)(const uint8_t *raw, uint8_t len);

/*------------------------------- 初始化 -----------------------------------*/
/** 初始化 USART3 + K1~K3 引脚(开漏, 空闲释放) */
void wt2003_init(void);

/** 主循环调用: 处理 Key 脉冲的结束、解析模块返回帧 */
void wt2003_process(void);

/*--------------------------- 统一语音播放接口 -----------------------------*/
/**
  * @brief  播放语音(自动选路) —— 业务层最常用就是这个
  * @param  voice_id 语音编号
  * @retval 0=成功, 其它=失败(未绑定 Key 且串口发送失败等)
  */
uint8_t wt2003_talk(uint16_t voice_id);

/** 指定通路播放 */
uint8_t wt2003_talk_src(uint16_t voice_id, wt2003_src_t src);

/**
  * @brief  绑定 voice_id -> Key 口 (0/1/2 对应 K1/K2/K3)
  * @param  key_idx 0~2 绑定; 0xFF 解除绑定
  * @retval 0=成功, 1=参数错误, 2=表满
  */
uint8_t wt2003_bind_key(uint16_t voice_id, uint8_t key_idx);

/** 查询 voice_id 对应的 Key 口, 未绑定返回 0xFF */
uint8_t wt2003_get_key_of(uint16_t voice_id);

/*------------------------------- Key 口控制 -------------------------------*/
/** 触发一次 Key 脉冲(非阻塞: 拉低后由 wt2003_process() 到点释放)
    @note 只有 WT2003_KEY_ENABLE = 1 时才真的动引脚; = 0 时是空函数 */
void wt2003_key_pulse(uint8_t key_idx);

/** Key 口是否可用(0 = 本硬件没有该功能, WT2003_KEY_ENABLE = 0) */
uint8_t wt2003_key_available(void);

/** Key 口脉冲是否还在进行中 */
uint8_t wt2003_key_is_busy(uint8_t key_idx);

/*------------------------ 多任务模式(语音任务) ----------------------------*/
/**
  * @brief  打开/关闭"队列模式"
  * @param  enable 1=打开: 之后 wt2003_talk() 只投递请求(非阻塞), 由语音任务
  *                       通过 wt2003_take_request()/wt2003_exec_request() 执行;
  *                0=关闭: wt2003_talk() 就地执行(单任务/无 RTOS 时的行为)。
  * @note   由 app_task.c 的语音任务在进入循环前调用一次。
  */
void wt2003_use_task(uint8_t enable);

/** 播放请求打包/解包: 低 16 位 = 语音编号, 高 8 位 = 通路(wt2003_src_t) */
#define WT2003_REQ_PACK(id, src)   ((uint32_t)((uint32_t)(id) | ((uint32_t)(src) << 16)))
#define WT2003_REQ_ID(req)         ((uint16_t)((uint32_t)(req) & 0xFFFFU))
#define WT2003_REQ_SRC(req)        ((uint8_t)(((uint32_t)(req) >> 16) & 0xFFU))

/**
  * @brief  取一个播放请求(语音任务里用, 阻塞等待, 不占 CPU)
  * @param  req        取到的请求(用 WT2003_REQ_ID/SRC 解包)
  * @param  timeout_ms 超时(ms)
  * @retval 1=取到 0=超时
  */
uint8_t wt2003_take_request(uint32_t *req, uint32_t timeout_ms);

/**
  * @brief  执行一个播放请求(语音任务里调用)
  * @note   Key 口通路会阻塞 WT2003_KEY_PULSE_MS(约 100ms) —— 只影响当前任务。
  * @retval 0=成功 其它=失败
  */
uint8_t wt2003_exec_request(uint32_t req);

/*------------------------------- 串口控制 ---------------------------------*/
uint8_t wt2003_play_index(uint16_t index);   /*!< 按索引播放曲目(数据 2 字节大端):
                                                  命令码由 bsp_config.h 的 WT2003_PLAY_USE_UDISK 选
                                                  (1 = A6 播 U 盘根目录索引 / 0 = A0 播 SPI Flash) */
uint8_t wt2003_play_udisk(uint16_t index);   /*!< A6: 强制用 U 盘根目录索引播放 */
uint8_t wt2003_play_flash(uint16_t index);   /*!< A0: 强制用 SPI Flash 根目录索引播放 */

/** A7: U 盘根目录按文件名播放 (name4 = 4 字节 ASCII 文件名, 不含扩展名;
    例 "T002" -> 发 7E 07 A7 54 30 30 32 94 EF, 播 T002.mp3;
    注: 手册 4.1.3.5 该示例印的校验码是 96, 按"长度起逐字节累加和"实算应为 94,
        手册示例疑似笔误。驱动是按公式现算的, 不受影响。
    !!! 实测: 本模块的 A6(按索引)播放收下命令却不出声, 指定语音请改用这条(A7),
        文件名固定 4 字符并放在 U 盘根目录, 见 README 8.9) */
uint8_t wt2003_play_udisk_name(const char *name4);

/** A8: U 盘指定文件夹内按文件索引播放
    (folder5 = 5 字节 ASCII 文件夹名, index = 文件索引, 手册示例 "MUSIC" 第 2 首 = 索引 1) */
uint8_t wt2003_play_udisk_folder(const char *folder5, uint16_t index);

/** D2: 切换当前工作盘符 (drive: wt2003_drive_t; 只影响 AC/AD/循环播放等"跟着当前盘走"的命令) */
uint8_t wt2003_select_drive(uint8_t drive);

uint8_t wt2003_play_next(void);              /*!< AC: 下一曲 */
uint8_t wt2003_play_prev(void);              /*!< AD: 上一曲 */
uint8_t wt2003_pause(void);                  /*!< AA: 播放暂停 */
uint8_t wt2003_stop(void);                   /*!< AB: 停止 */
uint8_t wt2003_set_volume(uint8_t vol);      /*!< AE: 音量 0~31 (0=静音, 31=最大) */
uint8_t wt2003_get_volume(void);             /*!< 最近一次设置的音量(本地缓存) */

/** AF: 指定播放模式 (数据 1 字节, 循环模式, 取值见手册) */
uint8_t wt2003_set_mode(uint8_t mode);

/** B0: 组合播放 (info = 曲目信息, 格式见手册) */
uint8_t wt2003_play_combo(const uint8_t *info, uint8_t len);

/** B1: 插播 (info = 曲目信息, 格式见手册) */
uint8_t wt2003_insert(const uint8_t *info, uint8_t len);

/** B6: 切换音频输出方式 (SPK / DAC) */
uint8_t wt2003_set_output(wt2003_out_t out);

/** B8: 进入低功耗 (mode = 1 字节参数, 取值见手册) */
uint8_t wt2003_low_power(uint8_t mode);

/** B7: Flash 引脚加载 / 释放 (一个命令两个功能, param: 0x00 = 释放引脚, 0x01 = 加载引脚;
    手册 4.1.4.15: 释放后 SPI Flash 引脚可挪作它用, 本项目没用到, 只在诊断时切一下) */
uint8_t wt2003_flash_pin(uint8_t load);

/*------------------------------- 查询命令 ---------------------------------*/
/* 查询命令只负责发帧, 模块的应答由 wt2003_set_frame_handler() 注册的回调上报:
     回调 cmd 即被查询的命令码(0xC0/0xC1/...), data 为应答数据(已去掉校验码字节)。 */
uint8_t wt2003_query_version(void);          /*!< C0: 查询软件版本(应答 4 字节) */
uint8_t wt2003_query_volume(void);           /*!< C1: 查询当前音量(应答 1 字节) */
uint8_t wt2003_query_status(void);           /*!< C2: 读取工作状态(应答 1 字节) */
uint8_t wt2003_query_file_num(void);         /*!< C3: 查询 SPI Flash 文件总数(应答 2 字节) */
uint8_t wt2003_query_udisk_num(void);        /*!< C7: 查询 U 盘文件总数(应答 2 字节) <== 本项目用这条 */
uint8_t wt2003_query_ufolder_num(const char *folder5);  /*!< C8: 查询 U 盘指定文件夹内文件总数,
                                                             folder5 = 5 字节 ASCII 文件夹名(应答 2 字节) */
uint8_t wt2003_query_cur_file(void);         /*!< C9: 查询当前播放文件(应答 2 字节) */
uint8_t wt2003_query_conn(void);             /*!< CA: 查询外设连接状态(应答 1 字节:
                                                  bit3=PC / bit2=U 盘 / bit0=SPI-Flash, 1=存在) */
uint8_t wt2003_query_cur_name(void);         /*!< CB: 查询当前播放歌曲名(应答 8 字节 ASCII) */

/** 发送一帧自定义命令, 返回 0=成功 */
uint8_t wt2003_send_cmd(uint8_t cmd, const uint8_t *data, uint8_t len);

/** 发送手工拼好的原始帧(用于调试/兼容特殊固件) */
uint8_t wt2003_send_raw(const uint8_t *frame, uint8_t len);

/*----------------------- 语音链路自检序列(排查用) -------------------------*/
/**
  * @brief  自检回调
  * @param  step  1..total: 刚发出第几步(各步含义见 wt2003_probe_cfg_hint());
  *               0xFF: 自检结束
  * @param  total 步骤总数(当前 38)
  * @note   典型用法: 在回调里把 step 显示到数码管。
  */
typedef void (*wt2003_probe_cb_t)(uint8_t step, uint8_t total);

/**
  * @brief  开始"语音链路自检"(由 wt2003_process() 按各步 gap_ms 走一步,
  *         共重复 WT2003_PROBE_ROUNDS 轮)
  * @note   全部按手册真实协议发帧, 定位"到底卡在哪一环"(含义见
  *         wt2003_probe_cfg_hint()):
  *           1    = AB 停止播放      (清掉"卡在播放/忙"的状态);
  *           2    = D2 盘符 = U 盘   (让 AC/AD/循环这些"跟着当前盘走"的命令也作用在 U 盘);
  *           3~4  = B6 输出=SPK / AE 音量 31
  *                  (喇叭必须接 SPK+ 与 SPK- 两端, 一端接 GND 则永远无声);
  *           5    = C1 查询当前音量;
  *           6    = CA 查询外设连接状态(bit2 = 1 才说明模块真的认到了 U 盘!);
  *           7    = C7 查询 U 盘内音乐文件总数(0 = 模块在 U 盘上看不到文件);
  *           8~9  = C9/C2 查询        (当前文件号 / 工作状态, 播放前的基准);
  *           10   = A6 播放 U 盘索引 1(第 1 次试播, 听声音);
  *           11~12= C9/C2 查询        (当前文件/状态有没有变);
  *           13   = AA 播放/暂停      (把"自认为在播"的东西暂停掉);
  *           14   = C2 查询状态;
  *           15   = A6 播放索引 1 再来(这步回 00 = 第 10 步的 02 是"忙碌拒绝" = 其实在播);
  *           16~17= C9/C2;
  *           18   = AB 停止;
  *           19~20= B6 输出=DAC 后 A6 再试播(试另一条输出通路, 听声音);
  *           21~22= AA / B6 输出=SPK(切回);
  *           23~24= A6 再试播(SPK 模式, 听声音) / C2;
  *           25~28= AC 下一曲 + C9 / AD 上一曲 + C9(cur_file 在 1 与 2 之间跳吗);
  *           29   = A6 播放 U 盘索引 2(扫索引: 回 00 = 该索引有文件可播);
  *           30   = AB 停止;
  *           31~38= 返回码取证(实测本模块 D2/A6/C7 都回 0x05 而不是 00, 用这几步钉死):
  *                  C3(SPI-Flash 文件数, 和第 7 步 C7 对比) / A6 索引 99(不存在, 该回 02 = 无此文件) /
  *                  C2 / A6 索引 0(索引可能从 0 开始) / C2 / A0(Flash 索引 1) / C9 / C2。
  *         (手册 4.1.2: A0/A1 = SPI Flash 播放, A6/A7/A8 = U 盘播放, C3 = SPI Flash 文件数,
  *          C7 = U 盘文件数, CA = 外设连接状态, D2 = 切换工作盘符;
  *          本工程音频在 U 盘, 故一律走 U 盘口径。)
  *         上电自动跑: 把 bsp_config.h 的 WT2003_PROBE_ON_BOOT 置 1。
  */
void    wt2003_probe_start(void);

/** 停止自检(未在自检时调用无副作用) */
void    wt2003_probe_stop(void);

/**
  * @brief  手工单独发一步(远程命令/调试用, 不改变自动自检的状态机)
  * @param  step 1..wt2003_probe_count()
  * @retval 0=已发出, 非 0=step 越界
  * @note   可在任意任务里调用(串口帧走带锁的发送)。
  */
uint8_t wt2003_probe_fire(uint8_t step);

/** 是否正在探测 */
uint8_t wt2003_probe_is_running(void);

/** 注册探测回调(可为 NULL) */
void    wt2003_probe_set_handler(wt2003_probe_cb_t cb);

/** 候选帧总数 */
uint8_t wt2003_probe_count(void);

/** 第 step(1..N) 种候选的文字说明(便于打印/记录), 越界返回 "?" */
const char *wt2003_probe_cfg_hint(uint8_t step);

/*----------------------- 按文件名轮播(U 盘 A7, 排查/播指定语音用) ----------*/
/* 背景(实测见 README 8.9): 本模块 A6(指定 U 盘索引播放)收下命令却不出声, 于是改用
   A7 按文件名播放: 文件放 U 盘根目录、文件名 4 字符, 名字列在 bsp_config.h 的
   WT2003_NAME_CYCLE_LIST; 上电每 WT2003_NAME_CYCLE_MS 毫秒播一个并循环, 播后
   WT2003_NAME_CYCLE_QUERY_MS 毫秒自动查一次 C9(当前曲目号)与 CB(当前曲目名),
   从串口一眼看出"模块有没有真的播这一首"。 */
/** 轮播回调: name = 刚发出的 4 字符文件名, n = 第几个(1..total), total = 列表长度 */
typedef void (*wt2003_name_cb_t)(const char *name, uint8_t n, uint8_t total);

uint8_t     wt2003_name_count(void);      /*!< 列表里的文件个数 */
const char *wt2003_name_get(uint8_t n);   /*!< 第 n 个文件名(1..count), 越界返回 NULL */
uint8_t     wt2003_name_fire(uint8_t n);  /*!< 立刻用 A7 播第 n 个(0=已发出, 非 0=失败) */
void        wt2003_name_start(void);      /*!< 开始自动轮播(由 wt2003_process() 驱动) */
void        wt2003_name_stop(void);       /*!< 停止自动轮播(手动 wt2003_name_fire 仍可用) */
uint8_t     wt2003_name_is_running(void); /*!< 是否在自动轮播 */
void        wt2003_name_set_handler(wt2003_name_cb_t cb);   /*!< 注册回调(可为 NULL) */

/*------------------------------- 状态统计 ---------------------------------*/
void     wt2003_set_frame_handler(wt2003_frame_cb_t cb);
void     wt2003_set_tx_handler(wt2003_tx_cb_t cb);      /*!< 每次发帧后回调(可为 NULL) */
void     wt2003_set_bad_frame_handler(wt2003_bad_frame_cb_t cb);  /*!< 结构不对的帧(排查用) */

/** 最近一帧收到的完整原始帧(含 7E/长度/校验码/EF), *len 返回字节数(无帧时为 0) */
const uint8_t *wt2003_last_rx_raw(uint8_t *len);

uint32_t wt2003_cmd_count(void);      /*!< 累计发送命令帧数 */
uint32_t wt2003_rx_total(void);       /*!< 累计收到字节数 */

#ifdef __cplusplus
}
#endif

#endif /* __DRV_WT2003H_H */
