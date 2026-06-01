#pragma once

/*
 * Microphone coordinates in millimeters.
 *
 * Coordinate convention:
 *   +Y is the 0-degree azimuth direction (forward).
 *   +X increases azimuth (right).
 *   +Z increases elevation (up).
 *
 * 4-mic dual-axis layout:
 *   I2S1 pair Mic0-Mic1 lies in the YZ plane (constrains uy+uz).
 *   I2S0 pair Mic2-Mic3 lies in the XZ plane (constrains ux).
 *   Each I2S port runs on the same BCLK — intra-port cross-correlation
 *   is reliable without clock-domain bridging artifacts.
 */

/* ---------- Mic positions (mm) ---------- */
#define MIC_ARRAY_POS2_MM_X   0.0f
#define MIC_ARRAY_POS2_MM_Y   0.0f
#define MIC_ARRAY_POS2_MM_Z   0.0f

#define MIC_ARRAY_POS3_MM_X   0.0f
#define MIC_ARRAY_POS3_MM_Y   3.0f
#define MIC_ARRAY_POS3_MM_Z   3.0f

#define MIC_ARRAY_POS0_MM_X  -3.5f
#define MIC_ARRAY_POS0_MM_Y   0.0f
#define MIC_ARRAY_POS0_MM_Z   3.5f

#define MIC_ARRAY_POS1_MM_X   3.5f
#define MIC_ARRAY_POS1_MM_Y   0.0f
#define MIC_ARRAY_POS1_MM_Z   3.5f

/* ---------- Speed of sound (mm/s) ---------- */
#define SOUND_SPEED_MM_PER_S 343000

/*
 * Intra-port mic pairs for dual-axis TDOA.
 * Pair 0 (I2S1): Mic0→Mic1, direction vector (0, 5, 5) mm.
 * Pair 1 (I2S0): Mic2→Mic3, direction vector (10, 0, 0) mm.
 */
#define MIC_PAIR_COUNT 2