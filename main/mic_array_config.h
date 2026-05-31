#pragma once

/*
 * Microphone coordinates in millimeters.
 *
 * Coordinate convention:
 *   +Y is the 0-degree azimuth direction (forward).
 *   +X increases azimuth (right).
 *   +Z increases elevation (up).
 *
 * The 4-mic non-coplanar layout eliminates mirror ambiguity in 3D TDOA.
 * Mic3 is offset in Z so it does not lie in the plane of Mic0/Mic1/Mic2.
 */

/* ---------- Mic positions (mm) ---------- */
#define MIC_ARRAY_POS0_MM_X   0.0f
#define MIC_ARRAY_POS0_MM_Y   0.0f
#define MIC_ARRAY_POS0_MM_Z   0.0f

#define MIC_ARRAY_POS1_MM_X   0.0f
#define MIC_ARRAY_POS1_MM_Y   60.0f
#define MIC_ARRAY_POS1_MM_Z   0.0f

#define MIC_ARRAY_POS2_MM_X   55.0f
#define MIC_ARRAY_POS2_MM_Y  100.0f
#define MIC_ARRAY_POS2_MM_Z 50.0f

#define MIC_ARRAY_POS3_MM_X   -20.0f
#define MIC_ARRAY_POS3_MM_Y 100.0f
#define MIC_ARRAY_POS3_MM_Z 50.0f

/* ---------- Speed of sound (mm/s) ---------- */
#define SOUND_SPEED_MM_PER_S 343000

/*
 * Precomputed mic pairs for TDOA.
 * Each pair is (mic_a, mic_b) with direction vector from a -> b.
 * With 4 mics: C(4,2) = 6 pairs.
 */
#define MIC_PAIR_COUNT 6

/*
 * Mic pair direction vectors (mm):
 *   (0,1) -> (+60,   0,   0)    left-right baseline
 *   (0,2) -> (+30, +50, +100)
 *   (0,3) -> (+30, -50, +100)
 *   (1,2) -> (-30, +50, +100)
 *   (1,3) -> (-30, -50, +100)
 *   (2,3) -> (  0,-100,   0)    front-back baseline (non-coplanar with others)
 */
