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
 *   Pair 0 (I2S1): Mic0--Mic1, 70 mm X-axis span, constrains ux.
 *   Pair 1 (I2S0): Mic2--Mic3, YZ-plane span, constrains uy+uz.
 *   Each I2S port shares a single BCLK -- intra-port cross-correlation
 *   is reliable without clock-domain bridging.
 *
 * To adapt for your own hardware, edit the six macros below and
 * update the mic pair assignment in afe_processor.c if needed.
 */

/* ---------- Mic positions (mm) ---------- */
#define MIC_ARRAY_POS2_MM_X   0.0f
#define MIC_ARRAY_POS2_MM_Y   0.0f
#define MIC_ARRAY_POS2_MM_Z   0.0f

#define MIC_ARRAY_POS3_MM_X   0.0f
#define MIC_ARRAY_POS3_MM_Y   30.0f
#define MIC_ARRAY_POS3_MM_Z   30.0f

#define MIC_ARRAY_POS0_MM_X  -35.f
#define MIC_ARRAY_POS0_MM_Y   0.0f
#define MIC_ARRAY_POS0_MM_Z   35.f

#define MIC_ARRAY_POS1_MM_X   35.f
#define MIC_ARRAY_POS1_MM_Y   0.0f
#define MIC_ARRAY_POS1_MM_Z   35.f

/* ---------- Speed of sound (mm/s) ---------- */
#define SOUND_SPEED_MM_PER_S 343000

/*
 * Number of intra-port mic pairs for dual-axis TDOA.
 * Pair 0 (I2S1): Mic0 -> Mic1,  direction (70, 0, 0) mm,  constrains ux.
 * Pair 1 (I2S0): Mic2 -> Mic3,  direction (0, 30, 30) mm, constrains uy+uz.
 *
 * The direction vectors are computed automatically from the positions above
 * by the general solver in afe_processor.c.  Just keep MIC_PAIR_COUNT at 2.
 */
#define MIC_PAIR_COUNT 2