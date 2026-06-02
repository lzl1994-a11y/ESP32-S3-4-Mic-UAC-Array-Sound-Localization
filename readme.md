# 🤖 Walle Ear S3

🎤 ESP32-S3 based 4-microphone array auditory front-end for robotics.  
🔌 Plug-and-play via USB: UAC audio stream + CDC serial real-time
sound source localization.

---

## ✨ Features

| Icon | Feature |
|---|---|
| 🎙️ | **4-channel synchronous capture** -- 4x INMP441 MEMS microphones at 16 kHz / 16-bit |
| 📡 | **Dual-axis independent TDOA** -- intra-port cross-correlation per I2S peripheral, no clock-domain bridging needed |
| 🗣️ | **WebRTC VAD voice trigger** -- TDOA runs only during speech, reducing false positives |
| 🧮 | **General TDOA solver** -- automatically adapts to any microphone positions; no hardcoded layout assumptions |
| 🔌 | **USB composite device** -- single USB cable provides UAC audio input and CDC serial telemetry |

---

## 🔧 Hardware Specifications

| Item | Specification |
|---|---|
| 🧠 MCU | ESP32-S3 (240 MHz, dual-core) |
| 🎤 Microphones | 4x INMP441 MEMS |
| 🎵 Sample Rate | 16000 Hz |
| 📏 Bit Depth | 16-bit signed PCM |
| 🔌 USB | 2.0 Full Speed (GPIO19/GPIO20) |
| 🎛️ I2S | I2S0 x 2ch + I2S1 x 2ch, shared BCLK/WS clock bridge |

---

## 📐 Angle Convention

| Angle | Symbol | Range | Zero Reference | Positive Direction |
|---|---|---|---|---|
| 🧭 Azimuth | θ | -180° ~ +180° | +Y axis = 0° | Counter-clockwise about +Z |
| 📈 Elevation | φ | -90° ~ +90° | XY plane = 0° | +Z direction |

---

## 🎙️ Microphone Array Layout

The default 4-mic layout forms two orthogonal baselines for independent
dual-axis TDOA.  Coordinates are set in `main/mic_array_config.h`.

| Mic | I2S Port | X (mm) | Y (mm) | Z (mm) | L/R Pin | Role |
|---|---|---|---|---|---|---|
| 🎤 0 | I2S1 | -35 | 0 | 35 | GND | Pair 0: X-axis baseline (70 mm) |
| 🎤 1 | I2S1 | +35 | 0 | 35 | 3V3 | Pair 0 |
| 🎤 2 | I2S0 | 0 | 0 | 0 | GND | Pair 1: YZ-plane baseline |
| 🎤 3 | I2S0 | 0 | 30 | 30 | 3V3 | Pair 1 |

- **Pair 0 (I2S1, Mic0/Mic1)**: pure X-axis separation constrains `ux`.
- **Pair 1 (I2S0, Mic2/Mic3)**: YZ-plane separation constrains `uy + uz`.

Together with the unit-vector constraint `ux² + uy² + uz² = 1`, three
equations solve three unknowns.  The general solver in `afe_processor.c`
reads these positions at runtime -- change the coordinates in
`mic_array_config.h` and the solver adapts automatically.

---

## 🔌 Hardware Wiring

```
GPIO4  → BCLK  → 4x INMP441 SCK
GPIO5  → WS    → 4x INMP441 WS (LRCLK)
GPIO7  → DIN   → Mic0 SD + Mic1 SD       (I2S1)
GPIO6  → DIN   → Mic2 SD + Mic3 SD       (I2S0)

Mic0 L/R → GND    (left  channel on I2S1)
Mic1 L/R → 3V3    (right channel on I2S1)
Mic2 L/R → GND    (left  channel on I2S0)
Mic3 L/R → 3V3    (right channel on I2S0)
```

⏱️ **Clock bridging**: I2S0 is configured as Master, generating BCLK and WS.
I2S1 is configured as Slave, sharing the same BCLK/WS pair.  This ensures
sample-level synchronization across all four microphones.

---

## 📡 Localization Principle

### 🧠 Core Strategy

Each I2S port carries two microphones sharing a single BCLK domain.
Cross-correlation within a port is immune to clock drift and propagation-delay
mismatch between ports.  Two ports provide two independent projection
constraints on the source direction unit vector **u** = (ux, uy, uz).

### 📊 TDOA Estimation

For each mic pair (a, b) on the same I2S port, the normalized
cross-correlation function is evaluated over a lag range
[-max_lag, +max_lag].  The max lag is computed dynamically from the
pair's physical separation:

```
max_lag = round(distance / c × fs) + 2
```

where `c = 343.2 m/s` and `fs = 16000 Hz`.  Quadratic interpolation
refines the peak to sub-sample resolution.

The resulting delay τ (in samples) gives the path-length difference:

```
d_ab = -c × τ / fs = (r_b - r_a) · u
```

### 🧮 Dual-Axis Solver

For pair p with direction vector **r_p** = pos[b] - pos[a] and
magnitude |**r_p**|, the projection equation is:

```
d_hat_p · u = delay_to_distance(τ_p) / |r_p|
```

This yields a 2×2 linear system in (ux, uy) parametrized by uz:

```
nx0 · ux + ny0 · uy = proj0 - nz0 · uz
nx1 · ux + ny1 · uy = proj1 - nz1 · uz
```

Substituting `ux = A + B·uz`, `uy = C + D·uz` into
`ux² + uy² + uz² = 1` produces a quadratic in uz.  The positive-Z
root is selected (source is above the microphone plane).

💡 **Noise handling**: when cross-correlation noise pushes the combined
projection norm beyond the unit sphere (`proj0² + proj1² > 1`), the
solver scales both projections back to the boundary and retries, yielding
a best-effort estimate rather than failing silently.

### 🗣️ Voice Activity Detection

A WebRTC VAD instance runs on Mic0 (30 ms frames, mode 0).  Each 32 ms
I2S capture frame is stored in a 20-frame ring buffer (~80 KB PSRAM).

- 🎯 **Onset**: when VAD detects speech after silence, the solver searches
  the look-back window (6 frames, ~192 ms) for the frame with the highest
  RMS energy and runs TDOA on that peak frame.
- 🔁 **Sustained speech**: TDOA runs every 5th frame (~160 ms) on the
  current capture frame, giving multiple opportunities per utterance.
- 🔇 **Silence**: no TDOA, frame counter resets.

### 📈 Confidence

```
confidence = clamp(0.5 × (corr_pair0 + corr_pair1), 0, 1) × 100
```

Minimum threshold for output: 1.  (Tunable via `AFE_TDOA_MIN_CONFIDENCE`.)

---

## ⚡ Performance Characteristics

The theoretical angular resolution is determined by the baseline-to-wavelength
ratio `d / λ`.  At 16 kHz and 343.2 m/s, `λ/2 = 10.7 mm`.

| Baseline | Half-wavelength ratio | Effective resolution |
|---|---|---|
| 70 mm (X-axis) | 6.5λ | ~3° at broadside |
| 42.4 mm (YZ-plane) | 4.0λ | ~5° at broadside |

Larger baselines improve resolution.  The 70 mm X-axis pair provides
the dominant contribution to azimuth accuracy.

---

## 🛠️ Build

**Prerequisites**: ESP-IDF 5.x with `esp-sr` component installed.

```bash
cd walle_ear_s3
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

The `sdkconfig.defaults` enables PSRAM (octal, 80 MHz), TinyUSB CDC,
ESP-SR VAD model (VADNet1 Medium), I2S ISR IRAM-safe mode, and a 1000 Hz
FreeRTOS tick for USB audio timing.

---

## 💻 Usage

Connect the ESP32-S3 board via USB.  The computer enumerates two devices:

| Interface | Function |
|---|---|
| 🎤 **UAC Microphone** | 16 kHz / 16-bit / Mono (raw Mic0 by default) |
| 📊 **CDC Serial Port** | Real-time localization telemetry at 1 Hz |

### 📊 CDC Output Format

When a valid source direction is detected:

```
azi:42,ele:18,conf:76,vad:1
```

When no valid direction is available:

```
azi:null,ele:null,conf:0,vad:0
```

| Field | Meaning | Range |
|---|---|---|
| 🧭 `azi` | Azimuth (degrees) | -180° ~ +180° |
| 📈 `ele` | Elevation (degrees) | -90° ~ +90° |
| 📊 `conf` | Confidence score | 0 ~ 100 |
| 🗣️ `vad` | Voice activity (1 = speech, 0 = silence) | 0 or 1 |

### 🐧 Reading CDC on Linux

```bash
# Find the CDC device (usually /dev/ttyACM0)
dmesg | grep ttyACM

# Read at 115200 baud
screen /dev/ttyACM0 115200
```

### 🎵 Recording UAC Audio

The UAC device appears as a standard USB microphone.  Use any recording
software or command-line tool:

```bash
arecord -f S16_LE -r 16000 -c 1 -D plughw:CARD=UAC1Gadget,DEV=0 output.wav
```

---

## ⚙️ Configuring Microphone Positions

Edit `main/mic_array_config.h` to match your physical layout.  The macros
are in millimeters.  For example, a compact square layout:

```c
/* Mic 0 -- origin */
#define MIC_ARRAY_POS0_MM_X    0.0f
#define MIC_ARRAY_POS0_MM_Y    0.0f
#define MIC_ARRAY_POS0_MM_Z    0.0f

/* Mic 1 -- +X direction */
#define MIC_ARRAY_POS1_MM_X   50.0f
#define MIC_ARRAY_POS1_MM_Y    0.0f
#define MIC_ARRAY_POS1_MM_Z    0.0f

/* Mic 2 -- +Y direction */
#define MIC_ARRAY_POS2_MM_X    0.0f
#define MIC_ARRAY_POS2_MM_Y   50.0f
#define MIC_ARRAY_POS2_MM_Z    0.0f

/* Mic 3 -- +Z direction */
#define MIC_ARRAY_POS3_MM_X    0.0f
#define MIC_ARRAY_POS3_MM_Y    0.0f
#define MIC_ARRAY_POS3_MM_Z   50.0f
```

Then update the pair definitions in `main/afe_processor.c`:

```c
static const int s_mic_pairs[AFE_NUM_MIC_PAIRS][2] = {
    {0, 1},    /* X-axis pair, constrains ux */
    {0, 2},    /* Y-axis pair, constrains uy (or choose {0,3} for uz) */
};
```

The general solver auto-adapts to any two non-parallel pair directions.

### ⚠️ Key Constraints

- Each mic pair must share the same I2S port (same BCLK).  Do not mix
  microphones across I2S0 and I2S1 in a single pair.
- Pair directions must not be parallel (determinant near zero).  Two
  roughly orthogonal directions work best.
- Larger baselines improve angular resolution.  Keep them at least 30 mm
  for usable TDOA at 16 kHz.
- Place all microphones in the same plane, or account for Z offsets in
  the coordinate macros.

---

## 🎛️ Tuning Thresholds

All thresholds are in `main/afe_processor.c` near the top of the file:

```c
#define AFE_TDOA_MIN_RMS          5.0f    /* frame RMS energy floor          */
#define AFE_TDOA_MIN_CORRELATION  0.04f   /* cross-correlation minimum       */
#define AFE_TDOA_MIN_CONFIDENCE   1       /* confidence floor (0..100)       */
#define AFE_VAD_TDOA_INTERVAL_FRAMES 5    /* TDOA every N frames during speech */
```

| Parameter | Increase if... | Decrease if... |
|---|---|---|
| 🔉 **RMS** | Noisy environment triggers false positives | Whispers are missed |
| 📊 **Correlation** | Echo/reverberation causes false positives | Distant sources or weak signals are missed |
| 🎯 **Confidence** | Want more reliable output (fewer hits) | Want more hits (less filtering) |
| 🔁 **TDOA frequency** | — (lower = fewer updates, less CPU) | Want more frequent updates during speech |

Debug logging is available at the `DEBUG` level.  Each TDOA hit prints:

```
I (10479) afe_processor: TDOA HIT: azi=-63 ele=24 conf=10 corr=[0.112,0.087] lag=[3.08,-1.33]
```

Failed attempts print the specific rejection reason (RMS, per-pair
correlation, solver failure, or confidence).

---

## 📁 Project Structure

```
walle_ear_s3/
├── CMakeLists.txt              # Project declaration
├── sdkconfig.defaults          # Kconfig defaults (PSRAM, TinyUSB, VAD model)
├── partitions.csv              # Custom flash partition table (ESP-SR model)
├── README.md
├── main/
│   ├── CMakeLists.txt          # Component build (I2S, ESP-SR, TinyUSB deps)
│   ├── main.c                  # Entry point
│   ├── i2s_mic_driver.h        # I2S dual-peripheral driver API
│   ├── i2s_mic_driver.c        # I2S0 master + I2S1 slave implementation
│   ├── mic_array_config.h      # Microphone positions and pair count
│   ├── afe_processor.h         # TDOA + UAC + CDC public API
│   ├── afe_processor.c         # Dual-axis solver, VAD, ring buffer, USB audio
│   ├── usb_composite.h         # TinyUSB UAC+CDC composite device API
│   ├── usb_composite.c         # USB descriptor, UAC streaming, CDC write
│   └── tusb_config.h           # TinyUSB configuration (UAC + CDC enabled)
```

---

## 📦 Dependencies

| Component | Version | Purpose |
|---|---|---|
| 🏗️ ESP-IDF | 5.x | Build system and framework |
| 🧠 ESP-SR | latest | WebRTC VAD (VADNet1 Medium) |
| 🔌 TinyUSB | via ESP-IDF | USB UAC 1.0 + CDC-ACM composite |
| 🎛️ esp_driver_i2s | via ESP-IDF | Dual I2S peripheral driver |

---

## 📄 License

MIT License.  See the source files for details.