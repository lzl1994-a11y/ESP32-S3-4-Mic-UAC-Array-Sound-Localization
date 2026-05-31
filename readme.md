# Walle Ear S3

> 基于 ESP32-S3 的 4 麦克风阵列机器人听觉前端

USB 即插即用：UAC 音频流 + CDC 串口实时上报声源方位角与仰角。

---

## 特性

- **4 路同步采集** — 4 × INMP441 MEMS 麦克风，16kHz / 16bit
- **声源定位** — TDOA 互相关 + 3D 远场最小二乘，6 对麦克风联合求解
- **USB 复合设备** — 单根 USB 线同时提供 UAC 音频和 CDC 串口
- **非共面阵列** — 第 4 个麦克风不共面，消除镜面歧义，3D 方向解唯一
- **1 Hz 定位上报** — JSON 格式，每秒输出一次方位角与仰角

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

## 麦克风阵列布局

```
         +Y (前方 0°)
          |
     Mic2 (0, 50, 100)
          |
Mic0 -----+----- Mic1
(-30,0,0) |     (30,0,0)
          |
     Mic3 (0, -50, 100)
```

| Mic | X (mm) | Y (mm) | Z (mm) | 说明 |
|-----|--------|--------|--------|------|
| 0   | -30    | 0      | 0      | 左声道 (I2S0 L) |
| 1   | 30     | 0      | 0      | 右声道 (I2S0 R) |
| 2   | 0      | 50     | 100    | 前上方 (I2S1 L) |
| 3   | 0      | -50    | 100    | 后上方 (I2S1 R)，**与其他三个不共面，消除镜面歧义** |

坐标可在 `main/mic_array_config.h` 中自定义。

---

## 角度定义

| 角度 | 符号 | 范围 | 零位 | 正向 |
|------|------|------|------|------|
| 方位角 Azimuth | azi | -180° ~ +180° | +Y 方向 = 0° | 绕 +Z 轴逆时针 (右手定则) |
| 仰角 Elevation | ele | -90° ~ +90° | XY 平面 = 0° | +Z 方向为正 |

```
方位角：atan2(vx, vy) × 180/π
仰  角：atan2(vz, √(vx²+vy²)) × 180/π
```

---

## 硬件接线

```
GPIO4  → BCLK → 4×INMP441 SCK
GPIO5  → WS   → 4×INMP441 WS/LRCLK
GPIO6  → DIN  → Mic0 SD + Mic1 SD        (I2S0)
GPIO7  → DIN  → Mic2 SD + Mic3 SD        (I2S1)

Mic0 L/R → GND    (左声道)
Mic1 L/R → 3V3    (右声道)
Mic2 L/R → GND    (左声道)
Mic3 L/R → 3V3    (右声道)
```

> **时钟搭桥**：I2S0 配置为 Master 输出 BCLK/WS，I2S1 配置为 Slave 输入同一组 BCLK/WS。利用 ESP32-S3 的 GPIO Matrix 实现双向路由，不需要外部硬件时钟分配器。

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
| **UAC 麦克风** | 16kHz / 16bit / Mono，标准 USB Audio Class 1.0 |
| **CDC 串口** | 业务数据端口，每秒输出一行 JSON |

**串口数据格式**：

```json
{"azi":-42.3,"ele":18.7}
```

无有效声源时输出：

```json
{"azi":null,"ele":null}
```

Linux 下可用 `cat /dev/ttyACM0` 查看，Windows 用串口助手打开对应 COM 口。

---

## 项目结构

```
walle_ear_s3/
├── CMakeLists.txt
├── sdkconfig.defaults
├── README.md
└── main/
    ├── CMakeLists.txt
    ├── main.c                    # 系统初始化
    ├── i2s_mic_driver.c/.h       # I2S 双外设驱动，4 通道采集
    ├── mic_array_config.h        # 麦克风坐标与 mic pair 定义
    ├── afe_processor.c/.h        # TDOA 定位 + UAC 音频 + CDC 上报
    ├── usb_composite.c/.h        # TinyUSB UAC+CDC 复合设备
    └── tusb_config.h             # TinyUSB 编译期配置
```

---

## 配置说明

所有可配置参数集中在以下文件中：

| 配置文件 | 主要内容 |
|----------|----------|
| `main/mic_array_config.h` | 麦克风坐标 (X/Y/Z mm)、声速、mic pair 组合 |
| `main/i2s_mic_driver.h` | 采样率、位宽、每帧采样数、麦克风数量 |
| `main/afe_processor.c` | TDOA 门限、CDC 上报间隔、UAC 音源选择 |
| `main/tusb_config.h` | USB 描述符参数、UAC 端点大小 |

---

## 定位算法

### 数据流

```
4×INMP441 → I2S DMA → Frame Pool (512 samples/ch × 4ch)
                          │
            ┌─────────────┼─────────────┐
            ▼                           ▼
    TDOA 互相关 × 6 对              UAC 单路上传
            │                     (Mic0 raw PCM)
            ▼
   3×3 最小二乘求解 (vx, vy, vz)
            │
            ▼
     azimuth / elevation
            │
            ▼
      CDC 每秒 JSON 输出
```

### 算法要点

- **6 对互相关**：C(4,2) = 6 对麦克风全部参与计算，每对执行 33 个 lag 位置的归一化互相关 + 抛物线亚采样插值
- **3×3 正规方程**：6 个方程 → AᵀA 压缩为 3×3 对称正定矩阵，伴随矩阵法闭式求逆
- **远场平面波**：假设声源距离远大于阵列孔径，方向向量为归一化最小二乘解
- **非共面阵列**：Mic3 不在 Mic0/Mic1/Mic2 所在平面，3D 解唯一，无需镜面消歧先验
