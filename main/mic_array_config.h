#pragma once

/*
 * Microphone coordinates are in millimeters. The DSP code converts them to
 * meters before using them in the TDOA equations.
 *
 * Coordinate convention:
 *   +Y is the 0 degree azimuth direction.
 *   +X increases azimuth.
 *   +Z increases elevation.
 */
#define MIC_ARRAY_POS0_MM_X 0.0f
#define MIC_ARRAY_POS0_MM_Y 0.0f
#define MIC_ARRAY_POS0_MM_Z 0.0f

#define MIC_ARRAY_POS1_MM_X 50.0f
#define MIC_ARRAY_POS1_MM_Y 50.0f
#define MIC_ARRAY_POS1_MM_Z 100.0f

#define MIC_ARRAY_POS2_MM_X -50.0f
#define MIC_ARRAY_POS2_MM_Y 50.0f
#define MIC_ARRAY_POS2_MM_Z 100.0f

/*
 * Mic3 is on I2S1 right slot, positioned behind the origin (negative Y)
 * at ground level (Z=0). Together with Mic0/1/2 this forms a non-coplanar
 * tetrahedron that enables unambiguous 3D direction-of-arrival estimation.
 */
#define MIC_ARRAY_POS3_MM_X 0.0f
#define MIC_ARRAY_POS3_MM_Y -50.0f
#define MIC_ARRAY_POS3_MM_Z 0.0f

/* Direction priors for choosing between the two mirror solutions of a 3-mic array. */
#define MIC_ARRAY_PRIOR_MIN_AZIMUTH_DEG -150.0f
#define MIC_ARRAY_PRIOR_MAX_AZIMUTH_DEG 150.0f
#define MIC_ARRAY_PRIOR_MIN_ELEVATION_DEG 0.0f
#define MIC_ARRAY_PRIOR_MAX_ELEVATION_DEG 70.0f
#define MIC_ARRAY_PRIOR_PREFER_POSITIVE_Y 1
