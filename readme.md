# Walle Ear S3 听觉子系统开发说明

## 1. 项目目标

本项目基于 ESP32-S3、ESP-IDF 5.x、INMP441 数字麦克风和 TinyUSB，实现一个“机器人听觉前端”。

当前推荐方案不是追求真实三维坐标，而是稳定输出声源方向：

- UAC：通过 ESP32-S3 原生 USB 向电脑传输 1 路单声道音频。
- CDC：通过同一个原生 USB 口向电脑传输声源方位数据。
- I2S：采集多路 raw mic PCM；UAC 只取其中 1 路或 AFE 输出，其他通道用于方向解算。
- 方向解算：使用 raw mic 的 TDOA/互相关计算声源方向，输出方位角、俯仰角和置信度。

输出重点：

```text
azimuth_deg      方位角
elevation_deg    俯仰角
confidence       置信度 0-100
```

不建议把结果描述为真实 `x/y/z` 坐标，因为小型麦克风阵列更适合估计方向，不适合稳定估计距离。

## 2. 关键设计结论

### 2.1 UAC 只需要 1 路音频

电脑端最终只需要拿到一条可用麦克风音频流，所以 UAC 保持单声道即可。

推荐数据流：

```text
Mic raw PCM -> 选择 Mic0 或 AFE 输出 -> UAC mono -> 电脑
Mic0/Mic1/Mic2 raw PCM -> TDOA -> azimuth/elevation/confidence -> CDC
```

这样可以避免把 3 路麦克风都塞进 USB 音频，也避免把定位逻辑和音频上传逻辑强耦合。

### 2.2 esp-sr AFE 不作为 3 麦定位核心

实测日志表明当前 esp-sr AFE 对 SR 路径最多可靠支持 2 个麦克风通道：

```text
AFE supports two microphone channels at the most. The first two channels will be selected.
```

因此：

- AFE 可以用于 1 路或 2 路降噪/增强。
- 3 麦方向解算应使用 raw PCM 自己做 TDOA。
- 不再要求 `esp-sr` 内部提供 3 麦 DOA。

### 2.3 TinyUSB CDC 不是 ESP_LOG 调试串口

ESP32-S3 开发板上常见两个 Type-C 口：

- 原生 USB 口：接 ESP32-S3 USB OTG，使用 `GPIO19/GPIO20`，可枚举 UAC 麦克风和 TinyUSB CDC。
- USB-UART 口：接 USB 转串口芯片，通常对应 `GPIO43/GPIO44`，用于烧录、monitor 和 `ESP_LOG` 输出。

所以文档中的 `usb_composite.c/.h` 指的是原生 USB 口上的业务复合设备：

```text
UAC: 传输音频
CDC: 传输 azimuth/elevation/confidence 等业务数据
```

它不是默认 `ESP_LOGI()` 打印口。开发阶段建议两个 Type-C 都接电脑：原生 USB 口测 UAC/CDC，USB-UART 口看日志。

## 3. 角度定义

方向解算内部使用单位方向向量：

```text
u = (ux, uy, uz)
```

### 3.1 方位角 azimuth

定义：

- `+Y` 方向为 `0°`。
- 绕原点朝 `+X` 方向旋转时角度增加。
- 朝 `-X` 方向为负角度。
- 范围：`-180° ~ +180°`。

计算公式：

```c
azimuth_deg = atan2f(ux, uy) * 180.0f / M_PI;
```

示例：

```text
+Y:   0°
+X: +90°
-X: -90°
-Y: ±180°
```

### 3.2 俯仰角 elevation

定义：

- `xy` 平面为 `0°`。
- `+Z` 方向为正，最大 `+90°`。
- `-Z` 方向为负，最小 `-90°`。

计算公式：

```c
elevation_deg = atan2f(uz, sqrtf(ux * ux + uy * uy)) * 180.0f / M_PI;
```

## 4. 麦克风阵列配置

### 4.1 当前 3 麦方案

计划使用 3 个 INMP441，位置单位建议使用 mm，内部换算为 m：

```text
Mic0 = (  0,  0,   0)
Mic1 = ( 50, 50, 100)
Mic2 = (-50, 50, 100)
```

其中 Mic1/Mic2 坐标应做成可配置项，方便后续根据结构件实测调整。

推荐新增或维护一个阵列配置头文件，例如：

```text
main/mic_array_config.h
```

配置内容示例：

```c
#define MIC_ARRAY_POS0_MM_X 0.0f
#define MIC_ARRAY_POS0_MM_Y 0.0f
#define MIC_ARRAY_POS0_MM_Z 0.0f

#define MIC_ARRAY_POS1_MM_X 50.0f
#define MIC_ARRAY_POS1_MM_Y 50.0f
#define MIC_ARRAY_POS1_MM_Z 100.0f

#define MIC_ARRAY_POS2_MM_X -50.0f
#define MIC_ARRAY_POS2_MM_Y 50.0f
#define MIC_ARRAY_POS2_MM_Z 100.0f
```

### 4.2 镜像解与约束

3 个麦克风必然共面，因此 3D 方向解算存在镜像解。当前布局的麦克风平面为：

```text
z = 2y
```

镜像不是简单的 `z 正/负`，而是绕这个倾斜平面翻转。

实际机器人使用场景可以使用先验约束降低误判：

- 人声大概率来自机器人外部。
- 人声大概率比三个麦克风高，即 `elevation > 0`。
- 人声大概率在机器人前方，即优先 `y > 0`。
- 限制合理角度范围，例如 `azimuth -150°~+150°`、`elevation 0°~70°`。
- 使用上一帧方向做连续性约束。
- 使用三路 raw RMS 做强度辅助评分，但不要单独依赖强度判断。

### 4.3 可选 4 麦扩展

如果后续硬件允许，非共面 4 麦可以显著改善 3D 方向稳定性并降低镜像歧义。

示例布局：

```text
Mic0 = ( 40,   0,  0)
Mic1 = (-40,   0,  0)
Mic2 = (  0,  40, 40)
Mic3 = (  0, -40, 40)
```

注意：如果使用 4 个 INMP441，通常需要 I2S0 采两路、I2S1 采两路，并且必须处理 I2S0/I2S1 样本级同步和固定延迟校准。

## 5. 硬件连接

### 5.1 2 麦 I2S0 调试模式

用于快速验证音频链路是否正常：

```text
GPIO4 -> 两个 INMP441 SCK/BCLK
GPIO5 -> 两个 INMP441 WS/LRCLK
GPIO6 -> 两个 INMP441 SD/DOUT
3V3   -> 两个 INMP441 VDD
GND   -> 两个 INMP441 GND

Mic0 L/R -> GND   左声道
Mic1 L/R -> 3V3   右声道
```

此模式只使用 I2S0，两个通道天然样本对齐，适合先排查 UAC、INMP441 和 I2S 数据格式。

### 5.2 3 麦目标模式

```text
GPIO4 -> 三个 INMP441 SCK/BCLK
GPIO5 -> 三个 INMP441 WS/LRCLK

Mic0 SD -> GPIO6
Mic0 L/R -> GND

Mic1 SD -> GPIO6
Mic1 L/R -> 3V3

Mic2 SD -> GPIO7
Mic2 L/R -> GND

3V3 -> 三个 INMP441 VDD
GND -> 三个 INMP441 GND
```

说明：

- GPIO6 是 I2S0 数据输入，承载 Mic0/Mic1 的 left/right slot。
- GPIO7 是 I2S1 数据输入，承载 Mic2 的 left slot。
- Mic2 的 `L/R` 必须接 GND，因为当前代码读取 I2S1 左声道。
- I2S1 必须使用 I2S0 的 BCLK/WS，且需要启动后 flush 和固定延迟校准。

### 5.3 USB 口说明

```text
ESP32-S3 原生 USB:
  D- = GPIO19
  D+ = GPIO20
  功能 = TinyUSB UAC + CDC 业务复合设备

USB-UART 调试口:
  常见为 GPIO43/GPIO44
  功能 = 烧录、monitor、ESP_LOG 调试输出
```

## 6. 软件模块结构

```text
walle_ear_s3/
├── CMakeLists.txt
├── partitions.csv
├── sdkconfig.defaults
├── readme.md
└── main/
    ├── CMakeLists.txt
    ├── main.c
    ├── i2s_mic_driver.c / .h      # I2S0/I2S1 raw mic 采集与通道交织
    ├── afe_processor.c / .h       # UAC 音频选择、TDOA 方向解算、CDC 输出
    ├── usb_composite.c / .h       # 原生 USB TinyUSB UAC + CDC 业务复合设备
    ├── tusb_config.h              # TinyUSB 编译期配置
    └── mic_array_config.h         # 建议新增：麦克风坐标和角度约束配置
```

## 7. 模块职责

### 7.1 i2s_mic_driver

职责：

- 使用 ESP-IDF 5.x `driver/i2s_std.h`。
- 采样率 `16000 Hz`。
- INMP441 物理输出使用 32-bit slot。
- 将 32-bit raw sample 转换为 16-bit PCM。
- 输出 interleaved PCM：

```text
2 麦模式: [Mic0, Mic1, Mic0, Mic1, ...]
3 麦模式: [Mic0, Mic1, Mic2, Mic0, Mic1, Mic2, ...]
```

注意：3 麦模式下，I2S0/I2S1 跨外设采集，需要验证样本级对齐。

### 7.2 afe_processor

职责：

- 从 `i2s_mic_driver` 获取 raw mic frame。
- UAC 只输出 1 路音频：默认 Mic0 raw PCM，后续可切换到 AFE 输出。
- TDOA 使用 raw Mic0/Mic1/Mic2 做互相关延迟估计。
- 输出 `azimuth/elevation/confidence` 到 CDC。
- AFE 仅作为可选 1/2 麦降噪增强模块，不作为 3 麦定位核心。

推荐 CDC 输出格式：

```text
az:-32,el:18,conf:76
```

其中：

- `az` 单位为度，范围 `-180~+180`。
- `el` 单位为度，范围 `-90~+90`。
- `conf` 范围 `0~100`。

### 7.3 usb_composite

职责：

- 在 ESP32-S3 原生 USB 口枚举复合设备。
- UAC：单声道、16kHz、16-bit PCM microphone。
- CDC：业务数据串口，用于发送方向数据。
- 不负责 ESP-IDF 默认日志输出。

## 8. 调试建议

### 8.1 推荐调试顺序

1. 只接 2 个麦克风，验证 I2S0 + UAC 音频清晰。
2. 切换 UAC raw mic index，分别确认 Mic0/Mic1 都正常。
3. 接入 Mic2，进入 3 麦 raw 测试模式，确认 I2S1 不 timeout。
4. 通过日志查看三路 min/max/rms，确认 Mic2 有有效波形。
5. 做 I2S0/I2S1 固定延迟校准。
6. 开启 TDOA，输出 azimuth/elevation/confidence。
7. 最后再决定是否启用 AFE 输出音频。

### 8.2 常见问题判断

- UAC 有声音但 CDC 没日志：CDC 只发业务数据，不是 `ESP_LOG`。
- 原生 USB 口能识别麦克风但看不到 boot log：正常，boot log 默认走 USB-UART。
- USB-UART 口能看日志但识别不到麦克风：正常，该口不是原生 USB OTG。
- 3 麦模式 I2S1 timeout：优先检查 GPIO7、Mic2 L/R、BCLK/WS 共享和 I2S1 slave 时钟输入。
- 3 麦定位飘：优先检查 I2S0/I2S1 固定采样偏移、麦克风坐标、阵列开孔遮挡和室内反射。

## 9. 构建要求

- ESP-IDF 5.x。
- 目标芯片：ESP32-S3。
- 建议开启 PSRAM，用于音频 buffer 和 esp-sr 内存。
- TinyUSB 开启 Audio + CDC。
- 开发日志默认保留在 USB-UART console。

典型命令：

```powershell
idf.py set-target esp32s3
idf.py build flash monitor
```
