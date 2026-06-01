# Walle Ear S3

> 基于 ESP32-S3 的 4 麦克风阵列机器人听觉前端  
> USB 即插即用：UAC 音频流 + CDC 串口实时上报声源方位信息

---

## 特性

- **4 路同步采集** — 4 × INMP441 MEMS 麦克风，16kHz / 16bit
- **双轴独立 TDOA 定位** — 各 I2S 端口内部互相关，无需跨端口时钟域对齐
- **USB 复合设备** — 单根 USB 线同时提供 UAC 音频和 CDC 串口

---

## 硬件规格

| 项目 | 参数 |
|------|------|
| MCU | ESP32-S3 (160 MHz) |
| 麦克风 | 4 × INMP441 |
| 采样率 | 16000 Hz |
| 位深 | 16-bit signed PCM |
| USB | 2.0 Full Speed (GPIO19/GPIO20) |
| I2S | I2S0 × 2ch + I2S1 × 2ch，共享 BCLK/WS 时钟搭桥 |

---

## 角度定义

| 角度 | 符号 | 范围 | 零位 | 正向 |
|------|------|------|------|------|
| 方位角 Azimuth | θ | −180° ~ +180° | +Y 方向 = 0° | 绕 +Z 轴逆时针 |
| 仰角 Elevation | φ | −90° ~ +90° | XY 平面 = 0° | +Z 方向为正 |

---

## 麦克风阵列布局

| Mic | 端口 | X (mm) | Y (mm) | Z (mm) | L/R | 角色 |
|-----|------|--------|--------|--------|-----|------|
| 0   | I2S1 | 0      | 0      | 0      | GND | 与 Mic1 成对，约束 uy+uz |
| 1   | I2S1 | 0      | 5      | 5      | 3V3 | 与 Mic0 成对 |
| 2   | I2S0 | −5     | 0      | 5      | GND | 与 Mic3 成对，约束 ux |
| 3   | I2S0 | 5      | 0      | 5      | 3V3 | 与 Mic2 成对 |

坐标可在 `main/mic_array_config.h` 中自定义。

---

## 硬件接线

```
GPIO4  → BCLK → 4×INMP441 SCK
GPIO5  → WS   → 4×INMP441 WS/LRCLK
GPIO7  → DIN  → Mic0 SD + Mic1 SD        (I2S1)
GPIO6  → DIN  → Mic2 SD + Mic3 SD        (I2S0)

Mic0 L/R → GND    (左声道)
Mic1 L/R → 3V3    (右声道)
Mic2 L/R → GND    (左声道)
Mic3 L/R → 3V3    (右声道)
```

> **时钟搭桥**：I2S0 配置为 Master 输出 BCLK/WS，I2S1 配置为 Slave 输入同一组 BCLK/WS。

---

## 定位原理

### 核心思路

4 颗麦克风分为两个 I2S 端口各一对。每个端口内的两颗 mic 共享同一个 BCLK——互相关（cross-correlation）在同一时钟域内进行，完全可靠，避免了跨 I2S 端口对因时钟微量漂移和信号路径不对称导致的互相关峰值发散。

每个端口的 mic pair 独立约束方向向量的一个分量：

```
I2S0 对 (Mic2→Mic3)：方向向量 r₀ = (10, 0, 0) mm
  → 仅约束 ux

I2S1 对 (Mic0→Mic1)：方向向量 r₁ = (0, 5, 5) mm
  → 约束 uy + uz
```

加上单位向量约束 `ux² + uy² + uz² = 1`，三个方程正好解三个未知数。

### 数学推导

令声源方向单位向量为 `u = (ux, uy, uz)`，声速 `c`，采样率 `fs`。

**Step 1 — TDOA 互相关**

对每对 mic `(a, b)`，互相关找到延迟 `τ`（样本数），对应的到达距离差：

```
d_ab = −c · τ / fs
```

TDOA 方程：

```
(r_b − r_a) · u = d_ab
```

**Step 2 — 直接求解分量**

代入实际坐标：

**I2S0 对 (2,3)**：
```
10 · ux = d₀    →    ux = d₀ / 10
```

**I2S1 对 (0,1)**：
```
5 · uy + 5 · uz = d₁    →    uy + uz = d₁ / 5
```

令 `s = uy + uz`。

`ux` 和 `s` 均由 TDOA 唯一确定（带符号，无歧义）。

**Step 3 — 解二次方程**

从单位向量约束得：

```
ux² + uy² + (s − uy)² = 1
2·uy² − 2·s·uy + (s² + ux² − 1) = 0
```

判别式：

```
Δ = 4·s² − 8·(s² + ux² − 1) = 4·(2 − 2·ux² − s²) = 4·(uy − uz)²
```

Δ 恒 ≥ 0，始终有实数解。两根互为 yz 互换：

```
Candidate A:  uy = (s + δ)/2,   uz = (s − δ)/2      (uy ≥ uz)
Candidate B:  uy = (s − δ)/2,   uz = (s + δ)/2      (uy ≤ uz)
```

其中 `δ = |uy − uz| = √(2 − 2·ux² − s²)`。

**Step 4 — 消歧**

在机器人典型使用场景下，声源（人嘴）距离机器人 0.3m 高的麦克风阵列在 1.5m 以外，仰角不超过 30°。此时水平分量 uy 的绝对值远大于垂直分量 uz，候选 A 天然命中。

当 `δ > 0.3`（声源过近 / 仰角过高，两个候选差异显著），本帧拒识。当 `δ ≤ 0.3`，选择 `|uy| ≥ |uz|` 的候选。

**Step 5 — 角度输出**

```
方位角:  θ = atan2(ux, uy)
仰角:    φ = atan2(uz, √(ux² + uy²))
```

### 置信度

```
corr_score   = (corr_pair0 + corr_pair1) / 2
ambig_score  = 1 − δ
confidence   = corr_score × ambig_score × 100
```

置信度 < 15 时拒绝输出。

### 性能预期

| 距离 | 方位角误差 (az=30°) | 方位角误差 (az=60°) |
|------|-------------------|-------------------|
| 0.5 m | ~16° | ~24° |
| 1.0 m | ~7° | ~13° |
| 1.5 m | ~4° | ~8° |
| 2.0 m | ~2° | ~4° |
| 3.0 m | < 1° | ~2° |

> 近距离高仰角场景（< 1m）由 `δ > 0.3` 拒识门限保护，不输出错误方位角。

---

## 构建

**前置要求**：ESP-IDF 5.x

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

---

## 使用

USB 连接后，电脑识别为两个设备：

| 设备 | 功能 |
|------|------|
| **UAC 麦克风** | 16kHz / 16bit / Mono |
| **CDC 串口** | 业务数据端口，每秒输出一行 |

串口数据格式（有效声源）：

```
azi:42,ele:18,conf:76
```

无有效声源时：

```
azi:null,ele:null,conf:0
```

---

## 项目结构

```
walle_ear_s3/
├── CMakeLists.txt
├── sdkconfig.defaults
├── README.md
└── main/
    ├── CMakeLists.txt
    ├── main.c
    ├── i2s_mic_driver.c/.h       # I2S 双外设驱动
    ├── mic_array_config.h        # 麦克风坐标配置
    ├── afe_processor.c/.h        # 双轴 TDOA 定位 + UAC + CDC
    ├── usb_composite.c/.h        # TinyUSB UAC+CDC
    └── tusb_config.h
```

---

## 配置说明

| 配置文件 | 主要内容 |
|----------|----------|
| `main/mic_array_config.h` | 麦克风坐标 (X/Y/Z mm)、声速、mic pair 组合 |
| `main/i2s_mic_driver.h` | 采样率、位宽、每帧采样数、麦克风数量 |
| `main/afe_processor.c` | TDOA 门限、歧义阈值 δ、CDC 上报间隔、UAC 音源选择 |
