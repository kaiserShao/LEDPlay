/**
  ******************************************************************************
  * @file    app_cmd.h
  * @brief   业务层: 远程命令协议 + 应答 + 按键事件上报 (与 ESP32-C3 通信)
  *
  *  链路: 手机/上位机 --WiFi--> ESP32-C3 --USART1--> STM32 (本工程)
  *
  *  命令格式(ASCII 文本行, 以 \n 或 \r\n 结束, 命令不区分大小写;
  *          发完不跟换行也行: 静默 ESP32_LINE_IDLE_MS 之后当作一整行, 见 bsp_config.h 第 7 节):
  *  -------------------------------------------------------------------------
  *   PING                                 -> +PONG
  *   STATUS | ?                           -> STATUS fx=..,brt=..,keys=..,up=..
  *   UART                                 -> UART rx=.. lines=.. ovf=.. err=.. (串口链路自检)
  *   UART RAW <0|1>                       -> OK UART RAW x  (1 = 每个收到的字节都打出来)
  *   UART DUMP [n|ALL]                    -> CAP/FLG 行 (原始接收缓冲: 十六进制 + ASCII + 标记)
  *   FILL <r> <g> <b>                     -> OK FILL
  *   LED <start> <count> <r> <g> <b>      -> OK LED
  *   FX <0..6>                            -> OK FX <name>
  *   BRT <0..255>                         -> OK BRT
  *   SPD <200..8000>                      -> OK SPD
  *   VOICE <file4> | VNAME <file4>        -> OK VOICE "0001"    (按文件名播放: 0x9E;
  *                                         file4 = U 盘根目录里的 4 字符文件名, 如 0001;
  *                                         名字宏见 bsp_config.h 的 WT2003_FILE_xxx;
  *                                         不带宽参时列出这套宏, 便于和 U 盘里的文件核对)
  *   VOL <0..31>                           -> OK VOL 20          (设置音量: 0xAE)
  *   VNUM                                  -> OK VNUM last=6     (发 0xC4 查文件总数,
  *                                         新结果看串口 WTRXCOUNT 行)
  *   VPLAY <index>                         -> OK VPLAY 1         (按根目录索引播放: 0x9D;
  *                                         走模块"当前盘符"的根目录)
  *   VINS <index> [flag]                   -> OK VINS 8          (插播, 叠在背景音上放: 0xB1;
  *                                         flag = B1 的"标示字",
  *                                         缺省 = WT2003_INSERT_FLAG,
  *                                         00 = FLASH(外挂 SPI Flash 片)【现用】/ 02 = U 盘;
  *                                         255 = 不带标示字的 2 字节形式)
  *                                         注: 三个音效在外挂 Flash 上, 用 flag=0
  *                                         (bsp_config.h 的 WSGAME_SFX_MODE = 0)
  *   VSCAN [from] [to] [gap_ms] [flag]     -> OK VSCAN 1..12 ... (挨个插播一遍, 用耳朵找出
  *                                         "哪个索引/标示字是什么声音"; 期间串口安静一阵)
  *   VMODE <0..3>                          -> OK VMODE 1          (指定播放模式: 0xAF;
  *                                         0=单曲 1=单曲循环 2=全部循环 3=随机)
  *   VRAW <hex...>                         -> OK VRAW 7E03C4C7EF (原样发任意帧: 5..16 字节,
  *                                         首字节须 7E、末字节须 EF; 校验码不代算,
  *                                         可空格/0x/逗号/冒号分隔; 见 README 3.6 节)
  *   DISP <最多4字符>                      -> OK DISP
  *   KEYLED <0..3|ALL> <r> <g> <b> [周期]  -> OK KEYLED
  *   KEYLED DIAG                          -> 多行 KEYLED ...  (四段时序/极性/每路颜色)
  *   KEYLED T0H|T0L|T1H|T1L <ns>          -> OK KEYLED T0H x T0L y T1H z T1L w
  *   KEYLED INVERT <0|1>                  -> OK KEYLED INVERT x
  *   KEYLED INIT [n]                      -> OK KEYLED INIT n frames (all off)  (连发 n 帧全零)
  *   HEARTBEAT <ms>  (0=关闭)              -> OK HEARTBEAT
  *   POWER OFF                            -> OK POWER (长按关机流程)
  *   其它                                  -> ERR UNKNOWN <cmd>
  *   注: "ERR UNKNOWN <很长的乱码>" = 收到的这一行本身是乱的(链路丢字节/上位机连发),
  *       先发 UART 看计数器与 last=(见 README 3.5)。
  *
  *  主动上报(STM32 -> WiFi):
  *   EVT KEY <id> <DOWN|UP|SHORT|LONG|HOLD> <ms>
  *   EVT POWER SHORT
  *   HB <uptime_ms> <fps> <fx> <brt>
  ******************************************************************************
  */
#ifndef __APP_CMD_H
#define __APP_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 初始化: 注册串口命令回调 + 按键事件回调 */
void app_cmd_init(void);

/** 主循环调用(处理心跳上报等) */
void app_cmd_process(void);

/** 主动上报一条事件(会被拼成 "EVT <name> <value>") */
void app_cmd_report_event(const char *name, int32_t value);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CMD_H */
