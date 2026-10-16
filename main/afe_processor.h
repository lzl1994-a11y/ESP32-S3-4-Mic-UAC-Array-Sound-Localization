#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t afe_processor_init(void);
esp_err_t afe_processor_start(void);
size_t afe_processor_get_feed_samples_per_channel(void);
int afe_processor_get_doa_angle(void);

typedef struct {
    bool valid;
    int x_milli;        // Horizontal unit vector x * 1000, +X right
    int y_milli;        // Horizontal unit vector y * 1000, +Y forward (>= 0)
    int z_milli;        // Always 0: elevation is not measured
    int azimuth_deg;    // -90..90, left positive; check valid before using
    int elevation_deg;  // 0 when valid (not measured), -1 when invalid
    int confidence;     // 0..100
    uint32_t sequence;
} afe_source_position_t;

bool afe_processor_get_source_position(afe_source_position_t *position);

#ifdef __cplusplus
}
#endif
