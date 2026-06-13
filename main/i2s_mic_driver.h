#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2S_MIC_SAMPLE_RATE_HZ 16000
#define I2S_MIC_BITS_PER_SAMPLE 16

#define SAMPLES_PER_CHANNEL 512
#define MIC_COUNT 4
#define I2S_FRAME_SAMPLES (SAMPLES_PER_CHANNEL * MIC_COUNT)

esp_err_t i2s_mic_driver_init(size_t samples_per_channel);
esp_err_t i2s_mic_driver_start(void);
esp_err_t i2s_mic_driver_read(int16_t *out_interleaved,
                              size_t samples_per_channel,
                              TickType_t ticks_to_wait);

esp_err_t i2s_mic_driver_write_tx(const int16_t *pcm_stereo, size_t num_samples);

size_t i2s_mic_driver_get_samples_per_channel(void);
esp_err_t i2s_mic_driver_deinit(void);

#ifdef __cplusplus
}
#endif
