Page({
  data: {
    statusText: '未连接',
    logText: '',
    connected: false,
    isSending: false
  },

  socketTask: null,

  connect() {
    const that = this;
    this.setData({ statusText: '连接中...', logText: '' });

    // ESP32 AP 模式默认 IP 是 192.168.4.1
    // 端口号必须与 ESP32 代码中 AsyncWebServer 的端口一致
    const wsUrl = 'ws://192.168.4.1:80/ws';

    this.socketTask = wx.connectSocket({
      url: wsUrl,
      fail(err) {
        console.error('连接失败', err);
        that.setData({ statusText: '连接失败', logText: JSON.stringify(err) });
      }
    });

    this.socketTask.onOpen(() => {
      console.log('WebSocket 已连接');
      that.setData({ statusText: '已连接 ESP32', connected: true });
      wx.showToast({ title: '连接成功', icon: 'success' });
    });

    this.socketTask.onMessage((res) => {
      console.log('收到 ESP32 消息：', res.data);
      that.setData({ logText: 'ESP32 回复：' + res.data });
    });

    this.socketTask.onError((err) => {
      console.error('WebSocket 错误', err);
      that.setData({ statusText: '连接错误', logText: JSON.stringify(err), connected: false });
    });

    this.socketTask.onClose(() => {
      console.log('连接已关闭');
      that.setData({ statusText: '已断开', connected: false });
      that.socketTask = null;
    });
  },

  sendMessage() {
    if (!this.socketTask) {
      wx.showToast({ title: '请先连接 ESP32', icon: 'none' });
      return;
    }

    this.setData({ isSending: true });
    this.socketTask.send({
      data: '你好，我是主机',
      success() {
        console.log('消息发送成功');
        wx.showToast({ title: '发送成功', icon: 'success' });
      },
      fail(err) {
        console.error('发送失败', err);
        wx.showToast({ title: '发送失败', icon: 'none' });
      },
      complete: () => {
        this.setData({ isSending: false });
      }
    });
  },

  disconnect() {
    if (this.socketTask) {
      this.socketTask.close({ code: 1000 });
      this.socketTask = null;
      this.setData({ statusText: '已断开', connected: false });
    }
  }
});