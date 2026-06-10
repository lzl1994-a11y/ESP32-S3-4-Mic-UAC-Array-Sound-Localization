# 🤖 Ear S3 (3D Spatial Audio & Voice Radar)

[![English](https://img.shields.io/badge/Language-English-blue)](#-english-version) [![中文](https://img.shields.io/badge/Language-%E4%B8%AD%E6%96%87-red)](#-中文版)

<div align="center">
  <img src="https://img.shields.io/badge/MCU-ESP32--S3-orange.svg" alt="ESP32-S3">
  <img src="https://img.shields.io/badge/Mic_Array-4%20Channels-blue.svg" alt="4 Channels">
  <img src="https://img.shields.io/badge/Algorithm-WebRTC_VAD%20%7C%20TDOA-green.svg" alt="Algorithms">
  <img src="https://img.shields.io/badge/Interface-Native_USB%20(UAC%2BCDC)-purple.svg" alt="USB">
</div>

> A universal **4-microphone 3D sound source localization front-end** for robotics, smart home devices, and interactive screens.
> Plug-and-play via Native USB Composite Device (Driver-free raw audio streaming + Real-time serial coordinate tracking).

---

## 🇺🇸 English Version

Ear S3 is designed as a lightning-fast "Auditory Radar" system for visual interaction. It provides real-time 3D sound source tracking to enable robots or screens to naturally "look" at the speaker.

### 🌟 Core Philosophy & Architecture

#### 1. Ultra-fast Trigger: WebRTC VAD
Instead of heavy neural networks (VADN) commonly used in smart speakers, this module uses the lightweight and lightning-fast **WebRTC VAD** (based on Gaussian Mixture Models and Energy Thresholds).
*   **Purpose**: Acts strictly as a radar **trigger**. The CPU-intensive TDOA matrix algorithm sleeps during silence. Once "Speech" is detected, the radar instantly wakes up to capture the 3D coordinates.

#### 2. Decoupled Spatial Calculation: Dual-Cone Intersection
To support irregular physical arrays (non-circular/non-linear), we discard traditional synchronized beamforming in favor of an elegant **Dual-Cone Intersection Algorithm**.
*   **Physical Isolation**: I2S0 (elevation) and I2S1 (azimuth) calculate their time differences independently. This **perfectly bypasses the microsecond startup drift between hardware I2S peripherals**.
*   **Probability Filtering**: By solving quadratic equations from two cone formulas, the algorithm yields multiple spatial coordinates. It immediately discards "physically impossible" solutions (e.g., pointing underground) utilizing the installation tilt, outputting a single, high-confidence `Azimuth` and `Elevation`.
*   **High Fault Tolerance**: Thanks to decoupled calculation, a 1~5mm physical assembly tolerance (causing merely a 1°~3° angular deviation) has zero impact on the visual tracking experience.

#### 3. Single-Cable Magic: Native USB Composite Device
Ditch the jumper wires. Connect a single Type-C cable to the PC/Host. It configures dual channels automatically:
*   **🎤 USB Microphone (UAC)**: Driver-free recording of 16kHz raw, lossless PCM audio.
*   **📊 Virtual Serial Port (CDC)**: Streams real-time 3D tracking telemetry in plain text (e.g., `azi:45,ele:10,conf:90,vad:1`).

### 🔧 Wiring Configuration

Utilizes dual I2S channels bridging 4 INMP441 MEMS microphones. **I2S0 is the Master (clock generator), I2S1 is the Slave (clock receiver), ensuring system-wide synchronization.**

| Signal | ESP32-S3 GPIO | INMP441 Pin | Notes |
| :--- | :--- | :--- | :--- |
| **BCLK** | `GPIO 4` | SCK | Shared across all 4 mics |
| **WS (L/R Clock)**| `GPIO 5` | WS / L/R | Shared across all 4 mics |
| **DIN 1 (I2S1)** | `GPIO 7` | SD (Mic 0 & 1) | **Horizontal Pair**. Mic 0 L/R to GND, Mic 1 L/R to 3V3 |
| **DIN 0 (I2S0)** | `GPIO 6` | SD (Mic 2 & 3) | **Elevation Pair**. Mic 2 L/R to GND, Mic 3 L/R to 3V3 |

### 💻 Host PC Interaction Guide

Open your serial monitor (baud rate 115200) on your PC. When the radar locks, you will see a text stream:

```text
azi:42,ele:18,conf:90,vad:1
```

| Field | Meaning | Range | Integration Tip |
| :--- | :--- | :--- | :--- |
| **`vad`** | Voice Activity | `0` or `1` | **Crucial!** A transition from `1` to `0` signifies the user has finished speaking. Trigger your LLM STT processing here. |
| **`conf`** | Confidence | `0 ~ 100` | Radar quality score. Filter out data with `conf < 50` to prevent jitter from wind or echoes. |
| **`azi`** | Azimuth | `-180° ~ +180°` | `0°` is front. Positive is right, negative is left. Drives horizontal panning. |
| **`ele`** | Elevation | `-90° ~ +90°` | `0°` is horizon. Positive is up, negative is down. Drives vertical tilting. |

---

## 🇨🇳 中文版

Ear S3 是一套专为 **“强视觉交互设备”**（如桌面机器人、带屏智能中控）设计的极速听觉雷达系统。它的核心设计理念在于**响应速度**和**三维指向性**，让设备能自然地“看”向说话的人。

### 🌟 核心理念与架构特色

#### 1. 极速“扳机”：摒弃重型神经网络，采用 WebRTC VAD
常规语音音箱使用基于神经网络的 VADN 来决定何时发送录音到云端。而本模块为了让视觉互动做到秒级响应，采用了算力极低且速度极快的传统 **WebRTC VAD**（基于高斯混合模型和能量阈值）。
*   **用途**：纯粹作为空间声源雷达的**触发器**。平时 TDOA 矩阵算法处于休眠状态以节省算力；一旦探测到确切的“人声 (Speech)”，雷达瞬间开启，抓取目标的三维坐标，并输出跟踪指令。

#### 2. 空间降维解算：独立的双组 TDOA 与双锥面相交
针对非规则的物理外观，我们摒弃了要求所有通道绝对对齐的传统波束矩阵算法，采用了极其精巧的 **双锥面相交几何算法 (Dual-Cone Intersection)**。
*   **物理隔离避开死穴**：I2S0 (负责高度) 和 I2S1 (负责水平) 分别独自计算自己的时差。这**完美绕过了跨 I2S 硬件启动时的微小同步漂移**。
*   **概率过滤不合理点**：算法通过两个锥面方程式联立解一元二次方程，得出多个空间坐标解。随后利用安装倾角和高度差，直接舍弃那些“物理不可能”的解（比如指向地板深处），从而得出唯一的高置信度 `Azimuth`（方位角）和 `Elevation`（高度角）。
*   **强悍的容错率**：因为使用了分离降维计算，装配时的 **1~5mm 公差（投射到角度仅有 1°~3° 偏差）完全不会影响最终的视觉跟随体验**。

#### 3. 一线通：原生 USB 复合设备 (Composite Device)
拔掉繁琐的杜邦线，仅需一根 Type-C 即可直连电脑上位机或树莓派。内部配置了 `UAC` + `CDC` 双通道：
*   **🎤 USB 麦克风 (UAC)**：上位机免驱录制 16kHz 无损原始音频。
*   **📊 虚拟串口 (CDC)**：实时（每秒多次）以明文形式输出雷达解析出的三维追踪指令。

### 🔧 硬件引脚配置

采用双 I2S 通道挂载 4 颗 INMP441 麦克风芯片。**I2S0 作为 Master 输出时钟，I2S1 作为 Slave 共享时钟，保证系统级强同步**。

| 信号 | ESP32-S3 GPIO | INMP441 引脚 | 备注说明 |
| :--- | :--- | :--- | :--- |
| **BCLK (时钟)** | `GPIO 4` | SCK | 4 颗麦克风共用 |
| **WS (左右声道选择)**| `GPIO 5` | WS / L/R | 4 颗麦克风共用 |
| **DIN 1 (I2S1 数据)** | `GPIO 7` | SD (Mic 0 & Mic 1) | **水平方位对**。Mic 0 的 L/R 接地(左)，Mic 1 的 L/R 接 3V3(右) |
| **DIN 0 (I2S0 数据)** | `GPIO 6` | SD (Mic 2 & Mic 3) | **高度仰角对**。Mic 2 的 L/R 接地(左)，Mic 3 的 L/R 接 3V3(右) |

*坐标排布请在 `main/mic_array_config.h` 中严格匹配物理安装的毫米(mm)坐标。*

### 💻 上位机交互指北

当模块通过 USB 接入 PC 后，打开电脑的串口助手（波特率 115200）。
当雷达锁定时，你将看到以下格式的纯文本流水：

```text
azi:42,ele:18,conf:90,vad:1
```

| 字段 | 含义 | 范围 | 上位机应用建议 |
| :--- | :--- | :--- | :--- |
| **`vad`** | 人声检测 (Voice Activity) | `0` 或 `1` | **关键！** 当连续输出 `1` 变 `0` 时，可用于通知大模型“用户已说完，开始断句并思考”。 |
| **`conf`** | 追踪置信度 (Confidence) | `0 ~ 100` | 雷达质量评分。建议上位机过滤掉 `conf < 50` 的数据，避免设备的眼睛因为风噪而乱转。 |
| **`azi`** | 水平方位角 (Azimuth) | `-180° ~ +180°` | `0°` 为正前方，正数表示偏右，负数表示偏左。用于驱动左右摇头。 |
| **`ele`** | 俯仰高度角 (Elevation) | `-90° ~ +90°` | `0°` 为水平面，正数表示抬头，负数表示低头。用于驱动上下点头。 |

---

## 🛠️ 构建与编译 (Build Instructions)

**环境要求 (Prerequisites)**：`ESP-IDF 5.x` with `esp-sr` component.

```bash
cd walle_ear_s3
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

**关键配置说明 (`sdkconfig.defaults`)**：
* 必须开启 **PSRAM**（推荐 Octal 80MHz）以提供 VAD 的大容量缓冲。
* 必须开启 **TinyUSB** 驱动及 I2S 的 **ISR IRAM-Safe** 模式。

---

## 📄 License

MIT License.