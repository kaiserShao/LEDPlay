#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// AP 配置
const char* ap_ssid = "ESP32_Host";    // 热点名称
const char* ap_password = "12345678";  // 密码（至少8位）

// 创建 WebSocket 服务器，端口 81（避免与HTTP的80冲突）
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// 收到消息时的回调
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("客户端 #%u 已连接\n", client->id());
      break;

    case WS_EVT_DISCONNECT:
      Serial.printf("客户端 #%u 已断开\n", client->id());
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo *info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len) {
        // 拼接收到的完整消息
        String msg = "";
        for (size_t i = 0; i < len; i++) {
          msg += (char)data[i];
        }
        Serial.printf("收到消息：%s\n", msg.c_str());

        // 回复一条确认消息给发送方
        client->text("ESP32已收到：" + msg);
      }
      break;
    }

    default:
      break;
  }
}

void setup() {
  delay(3000);  // 关键！等待 USB 串口重新连接电脑
  Serial.begin(115200);
  delay(3000);  // 关键！等待 USB 串口重新连接电脑

  // 启动 AP 模式
  WiFi.softAP(ap_ssid, ap_password);
  Serial.println("AP 已启动");
  Serial.print("ESP32 IP 地址：");
  Serial.println(WiFi.softAPIP());  // 默认 192.168.4.1

  // 注册 WebSocket 事件回调
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // 启动 HTTP 服务器（WebSocket 需要 HTTP 升级握手）
  server.begin();
  Serial.println("WebSocket 服务器已启动");
}

void loop() {
  // 清理断开的客户端
  ws.cleanupClients();
  delay(10);
}