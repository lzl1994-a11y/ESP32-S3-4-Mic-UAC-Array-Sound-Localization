# Walle Ear S3 — USB 收音与播放

分支：`codex/single-i2s`。原生 USB 提供 48 kHz 单声道麦克风、48 kHz 双声道扬声器与一个 CDC 虚拟串口。

## 当前音频路径

- I2S1 以 48 kHz 同时采集两颗 INMP441，并向 PCM5102 输出双声道播放音频。
- 两颗麦克风共用 SD，分别占用左右时隙；每帧每通道 1440 点（30 ms）。
- USB 麦克风继续输出 Mic0（I2S 左时隙），使用原有 2 倍数字增益与限幅；Mic1 仍采集，但暂未送至 USB 麦克风。
- USB 播放保持双声道顺序、4 倍数字增益与限幅，以及 32 位 I2S 时隙转换。
- TX 先初始化为 Master，RX 共享时钟；TX 启动前预填静音，播放缓冲耗尽时清零，维持采集与 DAC 时钟。

声源定位、定位专用历史缓存、位置查询接口、周期定位上报及测试音代码已删除。保留启动信息和异常日志，移除采样值、周期播放日志及 CDC 状态轮询任务。

## 保留但停用的 VAD

`main/afe_processor.c` 的 `AFE_ENABLE_VAD` 默认为 `0`。它同时控制保留的 WebRTC VAD 和原有 48→16 kHz 的 3:1 抽取降采样；当前不创建 VAD 实例、不分配算法缓冲、不执行降采样或人声检测。

需要恢复时将该开关设为 `1` 并重新构建。VAD 只检测当前帧，语音状态改变时输出 DEBUG 日志；它不控制 USB 收音的通断。保留的抽取代码没有抗混叠低通滤波。

ESP-SR 依赖继续保留以支持可选 VAD。当前没有启用 AEC、降噪或自动增益。

## CDC 通信

保留原来的 USB VID/PID（CAFE:4033）与设备识别协议：

```text
发送：getname:WHO_ARE_YOU
回复：IAM:ESP_MIC
```

CDC 不再主动发送位置、置信度或 VAD 状态。后续可复用这个虚拟串口承载显示、眼睛与舵机控制命令；当前还未集成显示项目。

## 接线

| 信号 | ESP32-S3 GPIO | 连接 |
| --- | --- | --- |
| BCLK | GPIO4 | 两颗 INMP441 SCK；PCM5102 BCK |
| WS | GPIO5 | 两颗 INMP441 WS；PCM5102 LRCK |
| DIN | GPIO7 | 两颗麦克风共用 SD |
| DOUT | GPIO10 | PCM5102 DIN |
| MCLK | GPIO9 | PCM5102 SCK，256fs = 12.288 MHz |

两颗麦克风的 L/R 分别接 GND 和 3.3 V；Mic0 表示左时隙，不代表机器人上的物理左右位置。本次没有重新分配 GPIO。

## 构建与验证

使用 ESP-IDF 5.5.4；配置为 ESP32-S3、16 MB Flash 和八线 PSRAM。

```sh
idf.py build
python tests/test_audio_capture.py
python tests/test_i2s_playback.py
```

主机测试需要 GCC。收音测试编译实际处理代码，检查 Mic0 选择、增益限幅、USB 开关和默认停用的 VAD，并额外检查 VAD 开启后的保留路径。播放测试检查 PCM 声道顺序、时隙转换、限幅及写入错误处理。主机测试不能替代 USB/I2S 硬件时序与音质实测。
