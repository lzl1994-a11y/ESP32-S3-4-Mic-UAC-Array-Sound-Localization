# Ear S3 — Single-I2S Two-Microphone Azimuth

This branch captures two INMP441 microphones through I2S1 and estimates horizontal
sound direction in the front semicircle. Forward is 0°, left is +90°, right is -90°.
USB provides a 48 kHz mono microphone, a 48 kHz stereo speaker, and CDC coordinates.

## 单 I2S 双麦克风版本

分支：`codex/single-i2s`。

- 只初始化 I2S1：TX 先初始化为 Master，RX 使用同一外设的全双工共享时钟。
- TX 启动前预填静音，启动后持续提供时钟；播放缓冲耗尽时自动清零，避免重复旧音频。
- Mic0、Mic1 组成水平麦克风对，默认间距 70 mm。
- 采集为 48 kHz、双通道、16-bit PCM，每通道每帧 1440 点（30 ms）。
- 保留现有 3:1 抽取，得到两路 16 kHz 音频，用于 VAD 和 TDOA。
- Mic0 检测到人声后，通过两路互相关和峰值插值估计时间差。
- 只选择前半圆方向：正前方 0°，向左 0°～+90°，向右 0°～-90°。
- 不再计算仰角；为兼容现有串口格式，有效结果的 `ele` 固定为 0。
- USB 麦克风仍输出 Mic0 单声道音频（48 kHz，2 倍数字增益）。
- USB 双声道播放继续通过 I2S1 TX 输出到 PCM5102（4 倍数字增益并限幅）。

两颗麦克风必须水平放置；算法按声源处于水平面、远场的假设估计角度。
双麦克风无法区分前后，也无法测量高度或距离。后方声源会映射到对应的
前半圆角度，“只保留前面”表示选择前方解，不表示能够排除后方声音。
当前降采样仍为直接抽取，尚未增加抗混叠低通滤波。

## 接线

以设备朝向为准：Mic0 放左侧，Mic1 放右侧。两颗麦克风共用 SD 数据线，
通过各自的 L/R 选择引脚分别占用左右时隙。

| 信号 | ESP32-S3 GPIO | 连接 |
| --- | --- | --- |
| BCLK | GPIO4 | 两颗 INMP441 SCK；PCM5102 BCK |
| WS | GPIO5 | 两颗 INMP441 WS；PCM5102 LRCK |
| DIN | GPIO7 | Mic0、Mic1 共用 SD |
| DOUT | GPIO10 | PCM5102 DIN |
| MCLK | GPIO9 | PCM5102 SCK，256fs = 12.288 MHz |
| Mic0 L/R | GND | I2S 左时隙，物理左侧 |
| Mic1 L/R | 3V3 | I2S 右时隙，物理右侧 |

原 GPIO6 的 Mic2/Mic3 不再采集。麦克风间距请在
`main/mic_array_config.h` 的 `MIC_ARRAY_SPACING_MM` 中按实际距离修改。
若左右麦克风物理位置颠倒，方位角正负号也会颠倒。

## 串口输出

```text
azi:42,ele:0,conf:90,vad:1
```

- `azi`：-90°～+90°，左正右负。与原四麦版本的角度正负约定不同。
- `ele`：有效定位固定为 0，仅保留字段，不代表实际测得仰角。
- `conf`：互相关峰值换算为 0～100，并非经校准的定位正确概率。
- `vad`：当前人声检测状态，0 或 1。
- 无效定位输出 `azi:null,ele:null,conf:0`。

语音开始时从最近 6 帧选取能量峰值帧定位；持续语音每 5 帧（约 150 ms）
更新一次。CDC 仍约每秒上报一次；静音时保留最近方向，`vad` 变为 0。

CDC 同时支持身份查询 `getname:WHO_ARE_YOU`，响应 `IAM:ESP_MIC`。

## 构建与验证

使用已安装并导出环境的 ESP-IDF 5.5.4；现有配置面向 ESP32-S3、16 MB Flash
和八线 PSRAM。依赖 esp-sr、esp_tinyusb 与 TinyUSB，沿用项目现有组件。

```sh
idf.py build
```

主机算法测试需要 Python 3 和 PATH 中的主机 GCC：

```sh
python tests/test_azimuth.py
python tests/test_i2s_playback.py
```

测试直接提取并编译固件的 C 算法函数，验证合成双通道音频的左右方向、
前半圆范围、±90° 边界、不合理延迟、静音、缺失通道和 30 ms 帧关系。
该测试不替代实物录音、接线及角度标定。

## License

MIT License
