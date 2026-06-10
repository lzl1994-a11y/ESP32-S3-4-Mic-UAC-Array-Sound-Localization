# Ear S3 (4-Mic 3D Sound Source Localization)

[![English](#-english-version)](#-english-version) [![中文](#-中文版)](#-中文版)

A 4-microphone 3D sound source localization implementation solution based on ESP32-S3.
Supports Native USB Composite Device (Driver-free UAC audio streaming + CDC serial coordinate output).

---

## 🇺🇸 English Version

This project provides a pragmatic approach to 3D sound source tracking for interactive devices.

### Architecture & Design Choices

#### 1. VAD Trigger (WebRTC VAD)
Instead of running continuous, heavy TDOA matrix calculations or neural network VADs (like ESP-SR's VADN), this module uses the standard WebRTC VAD.
*   **Why**: TDOA calculation on a 4-mic array is CPU-intensive. The WebRTC VAD acts as a lightweight trigger. The TDOA algorithm only runs when actual speech is detected, freeing up the ESP32-S3's dual-core CPU for other tasks like UI rendering.

#### 2. Dual-Cone Intersection Geometry
We abandoned traditional 4-channel synchronized beamforming and instead use a decoupled Dual-Cone Intersection geometric algorithm.
*   **The Pitfall We Avoided**: The ESP32-S3 requires two separate I2S peripherals (I2S0 and I2S1) to capture 4 channels. We found that the cross-correlation between microphones on *different* I2S paths (e.g., Mic 0 on I2S1 vs. Mic 2 on I2S0) is inherently too low and unreliable. This is due to a microsecond-level hardware startup drift between the two I2S controllers. 
*   **The Solution**: We physically isolate the calculation. I2S0 (elevation pair) and I2S1 (azimuth pair) calculate their time differences (TDOA) strictly within their own perfectly synchronized domains.
*   **Filtering**: The algorithm maps the two separate time delays to two intersecting cones, yielding multiple spatial solutions. By applying the physical constraints of the mic array's tilt/height, it filters out impossible angles (e.g., underground) to output a single valid `Azimuth` and `Elevation`.
*   **Tolerance**: This decoupled approach easily absorbs 1~5mm physical assembly errors without significantly degrading the tracking angle.

#### 3. Native USB Output
The ESP32-S3 acts as a USB Composite Device:
*   **UAC (USB Audio Class)**: Streams 16kHz raw PCM audio to the host.
*   **CDC (Communications Device Class)**: Outputs the localized coordinates in real-time via serial.

### Hardware Wiring

**I2S0 is configured as the Master (clock generator), and I2S1 is the Slave, sharing the clock.**

| Signal | ESP32-S3 GPIO | INMP441 Pin | Notes |
| :--- | :--- | :--- | :--- |
| **BCLK** | `GPIO 4` | SCK | Shared across all 4 mics |
| **WS (L/R)**| `GPIO 5` | WS / L/R | Shared across all 4 mics |
| **DIN 1 (I2S1)** | `GPIO 7` | SD (Mic 0 & 1) | **Azimuth pair**. Mic 0 L/R to GND, Mic 1 L/R to 3V3 |
| **DIN 0 (I2S0)** | `GPIO 6` | SD (Mic 2 & 3) | **Elevation pair**. Mic 2 L/R to GND, Mic 3 L/R to 3V3 |

*Adjust coordinates in `main/mic_array_config.h` to match your physical layout.*

### Serial Output Format

Baud rate: 115200. Example output:
```text
azi:42,ele:18,conf:90,vad:1
```
*   **`vad`**: Voice Activity (0 or 1). Useful for the host PC to determine when the user stops speaking.
*   **`conf`**: Confidence (0 ~ 100). Recommend filtering out `conf < 50` on the host side.
*   **`azi`**: Azimuth (-180° ~ +180°). `0°` is front.
*   **`ele`**: Elevation (-90° ~ +90°). `0°` is horizon.

---

## 🇨🇳 中文版

一种基于 ESP32-S3 的 4 麦克风 3D 声源定位实现方案。
支持原生 USB 复合设备输出（免驱的 UAC 音频流 + CDC 串口输出声源坐标）。

### 设计思路与踩坑经验

#### 1. VAD 触发器
模块没有使用占用大量资源的波束成形常驻计算，也没有使用算力较重的神经网络 VAD（如 ESP-SR 的 VADN），而是使用了极轻量的 **WebRTC VAD**。
*   **原因**：4 麦克风的 TDOA（到达时间差）空间解算非常消耗 CPU。WebRTC VAD 在这里仅作为一个“扳机”，只有在确实检测到人声时，才启动 TDOA 计算。这为 ESP32-S3 留出了足够的算力去处理屏幕 UI 渲染等其他任务。

#### 2. 双锥面相交的空间解算
我们**彻底摒弃了要求 4 个通道绝对时间对齐的传统波束矩阵算法**，改为使用独立的“双锥面相交”几何算法。
*   **我们踩过的坑**：ESP32-S3 需要同时使用两个 I2S 外设（I2S0 和 I2S1）才能采集 4 路麦克风。在实际工程测试中发现，尽管它们共用时钟管脚，但底层硬件启动时仍存在微秒级的异步漂移。这导致**非同一路（跨 I2S0 和 I2S1）的麦克风之间，相关性极低且不可靠**。如果强行把 4 个麦克风的数据放在同一个矩阵里算延迟，结果往往是失效的。
*   **破局方案**：我们将水平对（I2S1）和垂直对（I2S0）在算法上完全隔离。让它们各自在自己 100% 同步的硬件域内单独计算 TDOA。
*   **概率过滤**：水平对和垂直对算出的延迟，在几何上对应两个圆锥面。算法联立这两个锥面方程，会得出几个可能的 3D 坐标解。随后结合麦克风阵列对地面的物理安装倾角，直接舍弃掉指向地板等不合理的解，最终得出一个高置信度的 `Azimuth`（方位角）和 `Elevation`（高度角）。
*   **容错率**：因为解耦了运算，装配时 1~5mm 的物理误差，最终只会导致 1°~3° 的角度偏差，完全在实际应用的可接受范围内。

#### 3. USB 复合输出
使用 ESP32-S3 的原生 USB，实现一根 Type-C 线直连电脑：
*   **🎤 USB 麦克风 (UAC)**：上位机可直接录制 16kHz 原始音频。
*   **📊 虚拟串口 (CDC)**：实时输出定位坐标。

### 硬件接线

**采用 I2S0 作为 Master 输出时钟，I2S1 作为 Slave 共享时钟的接法。**

| 信号 | ESP32-S3 GPIO | INMP441 引脚 | 备注说明 |
| :--- | :--- | :--- | :--- |
| **BCLK** | `GPIO 4` | SCK | 4 颗麦克风共用 |
| **WS (左右声道)**| `GPIO 5` | WS / L/R | 4 颗麦克风共用 |
| **DIN 1 (I2S1)** | `GPIO 7` | SD (Mic 0 & 1) | **水平方位对**。Mic 0 的 L/R 接地(左)，Mic 1 的 L/R 接 3V3(右) |
| **DIN 0 (I2S0)** | `GPIO 6` | SD (Mic 2 & 3) | **高度仰角对**。Mic 2 的 L/R 接地(左)，Mic 3 的 L/R 接 3V3(右) |

*请在 `main/mic_array_config.h` 中根据实际的毫米(mm)距离配置坐标。*

### 上位机通信格式

波特率：115200。数据输出示例：
```text
azi:42,ele:18,conf:90,vad:1
```
*   **`vad`**：人声检测 (0 或 1)。上位机可借此判断用户是否说完话，作为大模型语音识别的断句依据。
*   **`conf`**：置信度 (0 ~ 100)。建议上位机过滤掉 `conf < 50` 的低质量数据（通常是风噪或回音）。
*   **`azi`**：水平方位角 (-180° ~ +180°)。`0°` 为正前方。
*   **`ele`**：俯仰高度角 (-90° ~ +90°)。`0°` 为水平面。

---

## 🛠️ 构建要求

* `ESP-IDF 5.x` 
* `esp-sr` component (用于 VAD 依赖)

**`sdkconfig.defaults` 配置须知**：
* 需开启 **PSRAM** 以提供音频缓冲。
* 需开启 **TinyUSB** 驱动及 I2S 的 **ISR IRAM-Safe**。

## License

MIT License