/**
  ******************************************************************************
  * @file    app_cmd.h
  * @brief   业务层: 远程命令协议 + 应答 + 按键事件上报 (与 ESP32-C3 通信)
  *
  *  链路: 手机/上位机 --WiFi--> ESP32-C3 --USART1--> STM32 (本工程)
  *
  *  命令格式(ASCII 文本行, 以 \n 或 \r\n 结束, 命令不区分大小写):
  *  -------------------------------------------------------------------------
  *   PING                                 -> +PONG
  *   STATUS | ?                           -> STATUS fx=..,brt=..,keys=..,up=..
  *   FILL <r> <g> <b>                     -> OK FILL
  *   LED <start> <count> <r> <g> <b>      -> OK LED
  *   FX <0..6>                            -> OK FX <name>
  *   BRT <0..255>                         -> OK BRT
  *   SPD <200..8000>                      -> OK SPD
  *   VOICE <id>                           -> OK VOICE
  *   VKEY <0..2>                          -> OK VKEY   (K1~K3 脉冲; 本硬件标准品
  *                                                    未开放 Key 口 -> ERR)
  *   VPLAY <index>                        -> OK VPLAY  (按默认介质播放: A6 U 盘【默认】/
  *                                                    A0 Flash, 由 WT2003_PLAY_USE_UDISK 选)
  *   VPLAY6 <index> | VPLAY0 <index>      -> OK VPLAY6 / OK VPLAY0  (强制 A6 U 盘 / A0 Flash)
  *   VCHECK                               -> OK VCHECK (一键体检: 连发
  *                                        C0/C1/C2/C3/CA/C7/C9/CB,
  *                                        回帧+解析见 WTRX/WTRXINFO;
  *                                        重点看 udisk_files=;
  *                                        写命令返回码 00=OK/01=FAIL/02=EMP,
  *                                        实测 A6 回 05 也照样出声, 别当失败)
  *   VOL <0..30>                          -> OK VOL
  *   VSTOP | VNEXT | VPREV | VPAUSE       -> OK ...
  *   VNAME [n]                            -> OK VNAME (按文件名播放 A7: 不带宽参 = 列出名字表;
  *                                        VNAME <n> = 立刻播第 n 个; 名字表见 bsp_config.h 的
  *                                        WT2003_NAME_CYCLE_LIST, 文件名必须 4 字符放在 U 盘根目录。
  *                                        实测本模块 A6 按索引播不出声, 故改用 A7 按文件名)
  *   VNAMEON | VNAMEOFF                   -> OK VNAMEON / OK VNAMEOFF (开/关每 4 秒播下一个的轮播)
  *   DISP <最多4字符>                      -> OK DISP
  *   KEYLED <0..3|ALL> <r> <g> <b> [周期]  -> OK KEYLED
  *   KEYLED DIAG                          -> 多行 KEYLED ...  (四段时序/极性/每路颜色)
  *   KEYLED T0H|T0L|T1H|T1L <ns>          -> OK KEYLED T0H x T0L y T1H z T1L w
  *   KEYLED INVERT <0|1>                  -> OK KEYLED INVERT x
  *   HEARTBEAT <ms>  (0=关闭)              -> OK HEARTBEAT
  *   POWER OFF                            -> OK POWER (长按关机流程)
  *   其它                                  -> ERR UNKNOWN <cmd>
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
