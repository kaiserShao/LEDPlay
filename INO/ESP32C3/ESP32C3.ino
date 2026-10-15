/*
 * ESP32-C3  USB <-> UART1 透传桥 (把电脑的串口命令转发给 STM32, 再把 STM32 的应答转回来)
 *
 * 接线:  UART1 TX = GPIO0  -> STM32 PA10 (RX)
 *        UART1 RX = GPIO1  <- STM32 PA9  (TX)
 *        USB      <-> 电脑 (串口工具)
 *
 * 【为什么改成"批量转发"】
 * 旧版是"一圈 loop() 只搬 1 个字节":
 *      if (TargetSerial.available()) Serial.write(TargetSerial.read());
 *      if (Serial.available())       TargetSerial.write(Serial.read());
 * 一圈一个字节时, 每个字节都要单独走一次 USB CDC 写 / UART 写; 电脑一发一整行
 * (十几~几十字节)、STM32 一回一整行时, 只要中间有一处缓冲没被及时取走就会丢/挤字节,
 * 现象就是上位机看到的命令缺字母、回显串行:
 *      ERR UNKNOWN KYLED_INIT_FRAM...
 * 现在改成"一次搬一整批(最多 256 字节)": 每圈把所有已经到达的字节一次性转走,
 * 并把两侧接收缓冲都调到 1024 字节, 给 USB 主机留足取数的时间。
 *
 * 【注意】改完要重新上传到 ESP32-C3(Arduino IDE: 板子选 ESP32C3 Dev Module,
 * 端口选 USB 那个串口)。上传后电脑端串口工具的波特率可以随便设(USB 侧与波特率
 * 无关), 但 STM32 侧固定 115200 8N1, 与 Inc/bsp_config.h 的 ESP32_BAUD 一致。
 * 另外 Arduino IDE 的 Tools 里 "USB CDC On Boot" 必须是 Enabled, 否则这里的
 * Serial 会指向 UART0(不是 USB), 电脑就看不到应答了。
 * 旧版本留档: Soft/Doc/orig_esp32c3_bridge.ino.bak
 */

#include <Arduino.h>

// 使用 UART1，并将其映射到 GPIO0 (TX) 和 GPIO1 (RX)
HardwareSerial TargetSerial(1);

static uint8_t xfer[256];        // 一次转发的暂存区

// 把 from 里"现在能拿到"的字节整批搬到 to(最多 256 字节), 不做任何等待
static void pump(Stream &from, Stream &to)
{
  size_t n = 0;

  while ((n < sizeof(xfer)) && (from.available() > 0))
  {
    int c = from.read();

    if (c < 0)
    {
      break;                     // 读失败就收手, 剩下的下一圈再说
    }
    xfer[n++] = (uint8_t)c;
  }

  if (n > 0)
  {
    to.write(xfer, n);           // 整批写出: USB CDC 会打包成少数几次传输
  }
}

void setup()
{
  // 接收缓冲必须在 begin() 之前设置
  Serial.setRxBufferSize(1024);       // 电脑 -> ESP32
  Serial.begin(115200);

  TargetSerial.setRxBufferSize(1024); // STM32 -> ESP32
  // 参数顺序：波特率, 配置, RX引脚, TX引脚
  TargetSerial.begin(115200, SERIAL_8N1, 1, 0);

  Serial.println("ESP32-C3 USB-UART bridge ready (UART1 115200 8N1, GPIO0=TX/GPIO1=RX)");
}

void loop()
{
  pump(TargetSerial, Serial);    // STM32 -> 电脑
  pump(Serial, TargetSerial);    // 电脑 -> STM32

  yield();                       // 让出 CPU, 别把核心其它任务饿死
}
