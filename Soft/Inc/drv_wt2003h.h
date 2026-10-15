/**
 ******************************************************************************
 * @file    drv_wt2003h.h
 * @brief   WT2003HX 语音模块驱动 (USART3: PB10=TX, PB11=RX)
 *
 *  本驱动只保留实测可用的串口命令;
 *  其余命令(A0/A6/A7/C0..CB/B6/D2…)、Key 口、上电自检、轮播全部退役,
 *  退役代码与旧命令表见 Doc/WT2003_退役命令与自检备份.md。
 *
 *  ---------------------------------------------------------------------------
 *  命令                帧(实测确认)                      驱动接口
 *  ---------------------------------------------------------------------------
 *  设置音量            7E 04 AE <vol> <ck> EF            wt2003_set_volume(v)
 *  查询文件总数        7E 03 C4 C7 EF                    wt2003_query_count()
 *  按根目录索引播放    7E 05 9D <高> <低> <ck> EF        wt2003_play_index(n)
 *  按文件名播放        7E 07 9E <4 字符> <ck> EF         wt2003_play_name("0001")
 *  指定播放模式        7E 04 AF <mode> <ck> EF           wt2003_set_mode(m)
 *  停止播放            7E 03 AA AD EF                    wt2003_stop()
 *  插播(叠加播放)      7E 06 B1 <flg> <高> <低> <ck> EF  wt2003_insert_index(n)
 *
 *  例: 索引 1     -> 7E 05 9D 00 01 A3 EF
 *      "0001"    -> 7E 07 9E 30 30 30 31 66 EF
 *      单曲循环   -> 7E 04 AF 01 B4 EF   (背景音乐循环用这条: 先 AF 01 再点播)
 *      插播 3 号  -> 7E 06 B1 00 00 03 BA EF
 *                    (标示字 00 = FLASH(外挂 SPI Flash 片), 02 = U 盘;
 *                     背景音不受影响)
 *      查文件总数  -> 7E 03 C4 C7 EF   (模块回 1 字节 = 文件总数)
 *
 *  帧格式(手册): [7E] [长度] [命令] [数据…] [校验码] [EF]
 *      长度   = 1(长度字节自身) + 1(命令) + N(数据) + 1(校验码)
 *      校验码 = 从"长度"字节起逐字节累加和 (mod 256)
 *
 *  音频文件: 放 U 盘根目录, 文件名固定 4 个字符(如 0001.mp3);
 *  文件名宏在 bsp_config.h 第 8.1 节(WT2003_FILE_xxx), 业务层这样点播:
 *      wt2003_play_name(WT2003_FILE_KEY);      // 按键音
 ******************************************************************************
 */
#ifndef __DRV_WT2003H_H
#define __DRV_WT2003H_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_config.h"

/*------------------------------- 回调类型 ---------------------------------*/
/** 收到模块回帧时的回调(调试打印用): cmd = 命令码, data = 数据区(已去掉校验码) */
typedef void (*wt2003_frame_cb_t)(uint8_t cmd, const uint8_t *data, uint8_t len);

/** 发出帧时的回调(原始字节, 含 7E/长度/命令/数据/校验码/EF) —— 调试打印用 */
typedef void (*wt2003_tx_cb_t)(const uint8_t *frame, uint8_t len);

/*------------------------------- 初始化 -----------------------------------*/
/** 初始化 USART3(PB10/PB11, 9600 8N1) + 接收中断 + 环形缓冲 */
void wt2003_init(void);

/** 主循环/语音任务调用: 解析模块回帧(必须周期调用, 否则收不到查询结果) */
void wt2003_process(void);

/*------------------------------- 4 条命令 ---------------------------------*/
/**
  * @brief  设置音量
  * @param  vol 0~31 (0 = 静音, 31 = 最大; 超出范围自动夹回)
  * @retval 0 = 已发出, 非 0 = 参数/发送失败
  */
uint8_t wt2003_set_volume(uint8_t vol);

/** 最近一次成功设置的音量(本地缓存) */
uint8_t wt2003_get_volume(void);

/**
  * @brief  按 U 盘根目录索引播放
  * @param  index 曲目索引(与文件在盘上的顺序有关, 1 起)
  * @retval 0 = 已发出, 非 0 = 发送失败
  */
uint8_t wt2003_play_index(uint16_t index);

/**
  * @brief  按文件名播放
  * @param  name4 4 字符文件名(不含扩展名), 如 "0001" -> 播 0001.mp3
  * @retval 0 = 已发出, 非 0 = 参数错误(不是 4 字符)或发送失败
  */
uint8_t wt2003_play_name(const char *name4);

/*------------------------------- 播放模式 -----------------------------*/
/** 播放模式(WT2003_CMD_SET_MODE 的数据字节): 00 单曲不循环 / 01 单曲循环 /
    02 全部循环 / 03 随机。背景音乐"单曲循环" = 先 wt2003_set_mode(单曲循环),
    再 wt2003_play_index(WT2003_BGM_INDEX) 按索引点播(0x9D)。 */
#define WT2003_MODE_SINGLE           0U
#define WT2003_MODE_SINGLE_LOOP      1U
#define WT2003_MODE_ALL_LOOP         2U
#define WT2003_MODE_RANDOM           3U

/**
  * @brief  指定播放模式(影响随后播放的曲目是否循环)
  * @param  mode WT2003_MODE_xxx(超出范围自动夹回)
  * @retval 0 = 已发出, 非 0 = 发送失败
  */
uint8_t wt2003_set_mode(uint8_t mode);

/**
  * @brief  停止播放(想让背景音停下的地方调它)
  * @retval 0 = 已发出, 非 0 = 发送失败
  */
uint8_t wt2003_stop(void);

/**
  * @brief  插播一段音频(叠在正在播的背景音上, 背景音不停、不重头)
  * @param  index 曲目索引(0xB1 的曲目号, 与文件名数字对应: "0008" -> 8)
  * @note   帧 = 7E 06 B1 <标示字> <曲目高> <曲目低> <校验码> EF;
  *         标示字取 bsp_config.h 的 WT2003_INSERT_FLAG
  *         (00 = FLASH(外挂 SPI Flash 片), 02 = U 盘) —— 见手册 4.1.2.14;
  *         例 曲目 3 -> 7E 06 B1 00 00 03 BA EF
  * @retval 0 = 已发出, 非 0 = 发送失败
  */
uint8_t wt2003_insert_index(uint16_t index);

/**
  * @brief  插播一段音频, 并临时指定"标示字"(不写回配置)
  * @param  index 曲目索引(0xB1 的曲目号, 与文件名数字对应: "0008" -> 8)
  * @param  flag  数据第 1 字节(标示字): WT2003_INSERT_FLAG_FLASH(00) /
  *               WT2003_INSERT_FLAG_UDISK(02)
  * @note   串口排查用: `VINS 8 2` 就能试 U 盘, 不用重新编译;
  *         业务代码请用 wt2003_insert_index()(标示字取 WT2003_INSERT_FLAG)。
  * @retval 0 = 已发出, 非 0 = 发送失败
  */
uint8_t wt2003_insert_ex(uint16_t index, uint8_t flag);

/**
  * @brief  查询文件总数(应答由 wt2003_process() 解析后存起来)
  * @retval 0 = 已发出, 非 0 = 发送失败
  */
uint8_t wt2003_query_count(void);

/** 最近一次查到的文件总数(0 = 还没查到 / 模块在盘上看不到文件) */
uint16_t wt2003_get_count(void);

/*------------------------------- 通用接口/统计 ----------------------------*/
/**
  * @brief  发一条命令(校验码自动算) —— 以后加命令/临时试手册里的帧都能用
  * @param  cmd  命令码, data/len = 数据区(无数据传 NULL/0)
  * @retval 0 = 已发出, 非 0 = 失败(帧太长/参数错)
  */
uint8_t wt2003_send_cmd(uint8_t cmd, const uint8_t *data, uint8_t len);

/**
  * @brief  原样发一帧(字节自己拼好, 含 7E / 长度 / 校验码 / EF) —— 排查/试手册帧用
  * @param  frame 整帧字节; len = 字节数, 5..WT2003_RX_FRAME_MAX(16),
  *               首字节必须 7E、末字节必须 EF
  * @retval 0 = 已发出, 非 0 = 参数不对(空/太短/太长/头尾不对)
  * @note   走和 wt2003_send_cmd() 同一条路: 命令间隔 WT2003_CMD_GAP_MS、
  *         WT2003_TX_TIMEOUT_MS 超时保护、WT2003_TRACE=1 时打 WTTX 行;
  *         校验码**不代算**(原样发), 串口命令 `VRAW 7E03C4C7EF` 可以直接敲。
  */
uint8_t wt2003_send_raw(const uint8_t *frame, uint8_t len);

void wt2003_set_frame_handler(wt2003_frame_cb_t cb);   /*!< 可为 NULL */
void wt2003_set_tx_handler(wt2003_tx_cb_t cb);         /*!< 可为 NULL */

uint32_t wt2003_cmd_count(void);   /*!< 累计发出的命令帧数 */
uint32_t wt2003_rx_total(void);    /*!< 累计收到的字节数 */

#ifdef __cplusplus
}
#endif

#endif /* __DRV_WT2003H_H */
