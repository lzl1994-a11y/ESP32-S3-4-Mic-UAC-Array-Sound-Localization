#pragma once

#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t afe_processor_init(void);
esp_err_t afe_processor_start(void);
size_t afe_processor_get_feed_samples_per_channel(void);
#ifdef __cplusplus
}
#endif
