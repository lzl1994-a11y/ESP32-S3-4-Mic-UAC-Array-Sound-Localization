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
    int x_milli;        // Far-field unit vector x * 1000
    int y_milli;        // Far-field unit vector y * 1000
    int z_milli;        // Far-field unit vector z * 1000, positive side of mic plane
    int azimuth_deg;    // -180..180, -1 when invalid
    int elevation_deg;  // -90..90, -1 when invalid
    int confidence;     // 0..100
    uint32_t sequence;
} afe_source_position_t;

bool afe_processor_get_source_position(afe_source_position_t *position);

#ifdef __cplusplus
}
#endif
