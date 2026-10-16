#pragma once

/* Mic0 (I2S left slot) is physically left; Mic1 (right slot) is right.
 * Place both microphones on the same horizontal line, facing forward.
 * Azimuth: forward = 0 degrees, left = +90, right = -90.
 * A single pair cannot distinguish front from rear; always select the front solution.
 */
#define MIC_ARRAY_SPACING_MM 70.0f
#define SOUND_SPEED_MM_PER_S 343200.0f
