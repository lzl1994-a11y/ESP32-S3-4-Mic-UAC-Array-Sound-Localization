#include "afe_processor.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_vad.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2s_mic_driver.h"
#include "mic_array_config.h"
#include "usb_composite.h"

#define AUDIO_FRAME_SAMPLES SAMPLES_PER_CHANNEL
#define AUDIO_TASK_STACK 8192
#define AUDIO_TASK_PRIORITY 9
#define AUDIO_TASK_CORE 1
#define AFE_SPEED_OF_SOUND_MPS (SOUND_SPEED_MM_PER_S * 1e-3f)
#define AFE_TDOA_MIN_RMS 5.0f
#define AFE_TDOA_MIN_CORRELATION 0.04f
#define AFE_TDOA_MIN_CONFIDENCE 1
#define AFE_TDOA_MAX_LAG_SAMPLES 16
#define AFE_VAD_RING_FRAMES 20
#define AFE_VAD_LOOKBACK_FRAMES 6
#define AFE_VAD_FRAME_MS 30
#define AFE_ALGO_SAMPLE_RATE_HZ 16000
#define AFE_VAD_FRAME_SAMPLES ((AFE_ALGO_SAMPLE_RATE_HZ * AFE_VAD_FRAME_MS) / 1000)
#define AFE_VAD_TDOA_INTERVAL_FRAMES 5   /* TDOA every 5th frame during speech */
#define AFE_UNIT_VECTOR_SCALE 1000.0f
#define AFE_UAC_WRITE_MARGIN_MS 80
#define AFE_UAC_WRITE_MAX_MS 500
#define AFE_CDC_WRITE_TIMEOUT_MS 20
#define AFE_CDC_REPORT_INTERVAL_MS 1000
#define AFE_CDC_TASK_STACK 4096
#define AFE_CDC_TASK_PRIORITY 5
#define AFE_CDC_TASK_CORE 1
#define AFE_UART_STATUS_INTERVAL_MS 2000
#define AFE_UAC_SOURCE_AFE 0
#define AFE_UAC_SOURCE_RAW_MIC 1
#define AFE_UAC_SOURCE_TEST_TONE 2
#define AFE_UAC_AUDIO_SOURCE AFE_UAC_SOURCE_RAW_MIC
#define AFE_UAC_RAW_MIC_INDEX 0
#define AFE_UAC_TEST_TONE_HZ 1000.0f
#define AFE_UAC_TEST_TONE_AMPLITUDE 8000.0f
#define AFE_UAC_GAIN_NUM 2    /* 2x digital gain for UAC audio → +6 dB */
#define AFE_UAC_GAIN_DEN 1

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if AFE_UAC_RAW_MIC_INDEX >= MIC_COUNT
#error "AFE_UAC_RAW_MIC_INDEX out of range"
#endif

static const char *TAG = "afe_processor";

_Static_assert(MIC_COUNT == 2, "Azimuth processing requires exactly two microphones");
_Static_assert(I2S_MIC_SAMPLE_RATE_HZ % AFE_ALGO_SAMPLE_RATE_HZ == 0,
               "Capture rate must be an integer multiple of the algorithm rate");
_Static_assert(SAMPLES_PER_CHANNEL * AFE_ALGO_SAMPLE_RATE_HZ ==
               AFE_VAD_FRAME_SAMPLES * I2S_MIC_SAMPLE_RATE_HZ,
               "Each capture frame must cover one complete VAD frame");

static TaskHandle_t s_audio_task;
static TaskHandle_t s_cdc_task;
static size_t s_algo_samples_per_channel;
static size_t s_capture_samples_per_channel;
static portMUX_TYPE s_position_lock = portMUX_INITIALIZER_UNLOCKED;
static afe_source_position_t s_latest_position = {
    .valid = false,
    .azimuth_deg = -1,
};
static uint32_t s_position_sequence;
static bool s_initialized;
static bool s_started;
static vad_handle_t s_vad_handle;
static int16_t **s_ring_buffer;
static int s_ring_write_idx;
static int s_ring_count;
static bool s_vad_speech_active;
static int s_speech_frame_count;          /* frames since speech onset, for periodic TDOA */

static void *audio_calloc(size_t count, size_t size)
{
    void *ptr = heap_caps_calloc(count, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ptr == NULL) {
        ptr = heap_caps_calloc(count, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    return ptr;
}

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int float_to_int_round(float value)
{
    return (int)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static void publish_source_position(afe_source_position_t position)
{
    portENTER_CRITICAL(&s_position_lock);
    position.sequence = ++s_position_sequence;
    s_latest_position = position;
    portEXIT_CRITICAL(&s_position_lock);
}

static void publish_invalid_position(void)
{
    afe_source_position_t position = {
        .valid = false,
        .azimuth_deg = -1,
        .elevation_deg = -1,
        .confidence = 0,
    };
    publish_source_position(position);
}

static int find_peak_ring_frame(int window_frames)
{
    int best_idx = -1;
    float best_rms = 0.0f;

    const int available = (s_ring_count < AFE_VAD_RING_FRAMES)
                          ? s_ring_count : AFE_VAD_RING_FRAMES;
    const int search_frames = (window_frames < available) ? window_frames : available;
    if (search_frames <= 0) {
        return -1;
    }

    for (int f = 0; f < search_frames; ++f) {
        const int idx = (s_ring_write_idx - 1 - f + AFE_VAD_RING_FRAMES) % AFE_VAD_RING_FRAMES;
        const size_t total = s_algo_samples_per_channel * MIC_COUNT;
        uint64_t sum_sq = 0;
        for (size_t i = 0; i < total; ++i) {
            int32_t s = s_ring_buffer[idx][i];
            sum_sq += (uint64_t)(s * s);
        }
        const float rms = sqrtf((float)sum_sq / (float)total);
        if (rms > best_rms) {
            best_rms = rms;
            best_idx = idx;
        }
    }
    return best_idx;
}

static void ring_buffer_push(const int16_t *frame)
{
    if (s_ring_write_idx >= AFE_VAD_RING_FRAMES) {
        s_ring_write_idx = 0;
    }
    memcpy(s_ring_buffer[s_ring_write_idx], frame,
           s_algo_samples_per_channel * MIC_COUNT * sizeof(int16_t));
    s_ring_write_idx = (s_ring_write_idx + 1) % AFE_VAD_RING_FRAMES;
    if (s_ring_count < AFE_VAD_RING_FRAMES) {
        ++s_ring_count;
    }
}

static int max_tdoa_lag_samples(void)
{
    const float distance_m = MIC_ARRAY_SPACING_MM * 1e-3f;
    int lag = (int)ceilf(distance_m * AFE_ALGO_SAMPLE_RATE_HZ / AFE_SPEED_OF_SOUND_MPS) + 1;
    if (lag < 1) {
        lag = 1;
    }
    if (lag > AFE_TDOA_MAX_LAG_SAMPLES) {
        lag = AFE_TDOA_MAX_LAG_SAMPLES;
    }
    return lag;
}

static float channel_mean(const int16_t *interleaved_frame, int mic)
{
    int64_t sum = 0;
    for (size_t i = 0; i < s_algo_samples_per_channel; ++i) {
        sum += interleaved_frame[i * MIC_COUNT + mic];
    }
    return (float)sum / (float)s_algo_samples_per_channel;
}

static float frame_rms(const int16_t *interleaved_frame)
{
    uint64_t sum_sq = 0;
    const size_t total_samples = s_algo_samples_per_channel * MIC_COUNT;
    for (size_t i = 0; i < total_samples; ++i) {
        int32_t sample = interleaved_frame[i];
        sum_sq += (uint64_t)(sample * sample);
    }
    return sqrtf((float)sum_sq / (float)total_samples);
}

static float normalized_correlation_at_lag(const int16_t *interleaved_frame,
                                           int mic_a,
                                           int mic_b,
                                           int lag,
                                           float mean_a,
                                           float mean_b)
{
    const int abs_lag = lag >= 0 ? lag : -lag;
    if ((size_t)abs_lag >= s_algo_samples_per_channel) {
        return 0.0f;
    }

    const size_t count = s_algo_samples_per_channel - (size_t)abs_lag;
    float sum_ab = 0.0f;
    float sum_aa = 0.0f;
    float sum_bb = 0.0f;

    for (size_t i = 0; i < count; ++i) {
        size_t idx_a = i;
        size_t idx_b = i;
        if (lag >= 0) {
            idx_b = i + (size_t)lag;
        } else {
            idx_a = i + (size_t)(-lag);
        }

        const float a = (float)interleaved_frame[idx_a * MIC_COUNT + mic_a] - mean_a;
        const float b = (float)interleaved_frame[idx_b * MIC_COUNT + mic_b] - mean_b;
        sum_ab += a * b;
        sum_aa += a * a;
        sum_bb += b * b;
    }

    if (sum_aa <= 1.0f || sum_bb <= 1.0f) {
        return 0.0f;
    }
    return sum_ab / sqrtf(sum_aa * sum_bb);
}

static bool estimate_pair_delay_samples(const int16_t *interleaved_frame,
                                        int mic_a,
                                        int mic_b,
                                        const float means[MIC_COUNT],
                                        float *delay_samples,
                                        float *peak_corr)
{
    const int max_lag = max_tdoa_lag_samples();
    float best_corr = -1.0f;
    int best_lag = 0;
    float corr_by_lag[(AFE_TDOA_MAX_LAG_SAMPLES * 2) + 1] = {0};
    const int corr_offset = AFE_TDOA_MAX_LAG_SAMPLES;

    for (int lag = -max_lag; lag <= max_lag; ++lag) {
        float corr = normalized_correlation_at_lag(interleaved_frame,
                                                   mic_a,
                                                   mic_b,
                                                   lag,
                                                   means[mic_a],
                                                   means[mic_b]);
        corr_by_lag[lag + corr_offset] = corr;
        if (corr > best_corr) {
            best_corr = corr;
            best_lag = lag;
        }
    }

    if (best_corr < AFE_TDOA_MIN_CORRELATION) {
        return false;
    }

    float refined_lag = (float)best_lag;
    if (best_lag > -max_lag && best_lag < max_lag) {
        const float y0 = corr_by_lag[best_lag + corr_offset];
        const float ym = corr_by_lag[best_lag - 1 + corr_offset];
        const float yp = corr_by_lag[best_lag + 1 + corr_offset];
        const float denom = ym - 2.0f * y0 + yp;
        if (fabsf(denom) > 1e-6f) {
            refined_lag += clampf(0.5f * (ym - yp) / denom, -0.5f, 0.5f);
        }
    }

    *delay_samples = refined_lag;
    *peak_corr = best_corr;
    return true;
}

static float delay_to_distance(float delay_samples)
{
    return -AFE_SPEED_OF_SOUND_MPS * delay_samples / (float)AFE_ALGO_SAMPLE_RATE_HZ;
}

/* Assume a horizontal source and select uy >= 0 (the front semicircle).
 * Positive lag means Mic1 arrives later than Mic0, so the source is to the left.
 * Keep Cartesian +X pointing right, but report left-positive azimuth.
 */
static bool solve_front_azimuth(float delay_samples, float *out_ux,
                                float *out_uy, float *out_azimuth)
{
    const float spacing_m = MIC_ARRAY_SPACING_MM * 1e-3f;
    if (!isfinite(spacing_m) || spacing_m <= 0.0f || !isfinite(delay_samples)) {
        return false;
    }
    const float max_delay = spacing_m * AFE_ALGO_SAMPLE_RATE_HZ / AFE_SPEED_OF_SOUND_MPS;
    /* Allow half a sample of estimation error at endfire, reject impossible peaks. */
    if (fabsf(delay_samples) > max_delay + 0.5f) {
        return false;
    }
    const float ux = clampf(delay_to_distance(delay_samples) / spacing_m, -1.0f, 1.0f);
    *out_ux = ux;
    *out_uy = sqrtf(fmaxf(0.0f, 1.0f - ux * ux));
    *out_azimuth = -asinf(ux) * (180.0f / (float)M_PI);
    return true;
}

static void update_source_position(const int16_t *interleaved_frame)
{
    if (interleaved_frame == NULL || s_algo_samples_per_channel == 0) {
        publish_invalid_position();
        return;
    }

    const float rms = frame_rms(interleaved_frame);
    if (rms < AFE_TDOA_MIN_RMS) {
        ESP_LOGD(TAG, "TDOA skip: rms=%.1f < %.1f", (double)rms, (double)AFE_TDOA_MIN_RMS);
        publish_invalid_position();
        return;
    }

    float means[MIC_COUNT] = {0};
    for (int mic = 0; mic < MIC_COUNT; ++mic) {
        means[mic] = channel_mean(interleaved_frame, mic);
    }

    float delay, corr;
    if (!estimate_pair_delay_samples(interleaved_frame, 0, 1, means, &delay, &corr)) {
        publish_invalid_position();
        return;
    }

    float ux, uy, azimuth;
    if (!solve_front_azimuth(delay, &ux, &uy, &azimuth)) {
        publish_invalid_position();
        return;
    }
    const float confidence = clampf(corr, 0.0f, 1.0f) * 100.0f;

    if (confidence < (float)AFE_TDOA_MIN_CONFIDENCE) {
        ESP_LOGD(TAG, "TDOA skip: conf=%.1f < %d",
                 (double)confidence, AFE_TDOA_MIN_CONFIDENCE);
        publish_invalid_position();
        return;
    }

    afe_source_position_t position = {
        .valid = true,
        .x_milli = float_to_int_round(ux * AFE_UNIT_VECTOR_SCALE),
        .y_milli = float_to_int_round(uy * AFE_UNIT_VECTOR_SCALE),
        .z_milli = 0,
        .azimuth_deg = float_to_int_round(azimuth),
        .elevation_deg = 0,
        .confidence = float_to_int_round(confidence),
    };
    ESP_LOGI(TAG, "TDOA HIT: azi=%d conf=%d corr=%.3f lag=%.2f",
             position.azimuth_deg, position.confidence, (double)corr, (double)delay);
    publish_source_position(position);
}

static TickType_t uac_write_timeout_ticks(size_t byte_len)
{
    const size_t bytes_per_second = I2S_MIC_SAMPLE_RATE_HZ * sizeof(int16_t);
    uint32_t audio_ms = (uint32_t)((byte_len * 1000 + bytes_per_second - 1) / bytes_per_second);
    uint32_t timeout_ms = audio_ms + AFE_UAC_WRITE_MARGIN_MS;
    if (timeout_ms > AFE_UAC_WRITE_MAX_MS) {
        timeout_ms = AFE_UAC_WRITE_MAX_MS;
    }
    return pdMS_TO_TICKS(timeout_ms);
}

static void write_uac_audio(const int16_t *data, size_t byte_len, const char *label)
{
    if (!usb_composite_uac_is_streaming()) {
        return;
    }

    esp_err_t ret = usb_composite_uac_write(data, byte_len, uac_write_timeout_ticks(byte_len));
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "UAC write %s failed: %s, total_bytes=%u",
                 label, esp_err_to_name(ret), (unsigned)byte_len);
    }
}

#if AFE_UAC_AUDIO_SOURCE == AFE_UAC_SOURCE_TEST_TONE
static void fill_test_tone(int16_t *buffer, size_t samples)
{
    static float phase;
    const float phase_step = 2.0f * (float)M_PI * AFE_UAC_TEST_TONE_HZ / (float)I2S_MIC_SAMPLE_RATE_HZ;

    for (size_t i = 0; i < samples; ++i) {
        buffer[i] = (int16_t)float_to_int_round(sinf(phase) * AFE_UAC_TEST_TONE_AMPLITUDE);
        phase += phase_step;
        if (phase >= 2.0f * (float)M_PI) {
            phase -= 2.0f * (float)M_PI;
        }
    }
}
#endif

static void audio_capture_task(void *arg)
{
    (void)arg;

    const size_t capture_samples = s_capture_samples_per_channel * MIC_COUNT;
    int16_t *capture_buffer = (int16_t *)audio_calloc(capture_samples, sizeof(int16_t));
    if (capture_buffer == NULL) {
        ESP_LOGE(TAG, "capture buffer alloc failed");
        vTaskDelete(NULL);
        return;
    }

    const size_t algo_samples = s_algo_samples_per_channel * MIC_COUNT;
    int16_t *algo_buffer = (int16_t *)audio_calloc(algo_samples, sizeof(int16_t));
    if (algo_buffer == NULL) {
        ESP_LOGE(TAG, "algo buffer alloc failed");
        free(capture_buffer);
        vTaskDelete(NULL);
        return;
    }

    int16_t *uac_buffer = (int16_t *)audio_calloc(s_capture_samples_per_channel, sizeof(int16_t));
    if (uac_buffer == NULL) {
        ESP_LOGE(TAG, "UAC buffer alloc failed");
        free(algo_buffer);
        free(capture_buffer);
        vTaskDelete(NULL);
        return;
    }

    /* One continuous 30 ms Mic0 frame for VAD. */
    int16_t *vad_buf = (int16_t *)audio_calloc(AFE_VAD_FRAME_SAMPLES, sizeof(int16_t));
    if (vad_buf == NULL) {
        ESP_LOGE(TAG, "VAD buffer alloc failed");
        free(uac_buffer);
        free(algo_buffer);
        free(capture_buffer);
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        esp_err_t ret = i2s_mic_driver_read(capture_buffer,
                                            s_capture_samples_per_channel,
                                            portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "read mic frame failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* 3:1 Decimation */
        const int decimation_factor = I2S_MIC_SAMPLE_RATE_HZ / AFE_ALGO_SAMPLE_RATE_HZ;
        for (size_t i = 0; i < s_algo_samples_per_channel; ++i) {
            for (size_t ch = 0; ch < MIC_COUNT; ++ch) {
                algo_buffer[i * MIC_COUNT + ch] = capture_buffer[(i * decimation_factor) * MIC_COUNT + ch];
            }
        }

        /* Store the stereo downsampled frame in the ring buffer. */
        ring_buffer_push(algo_buffer);

        /* Extract Mic0 for VAD (30 ms = 480 samples). */
        for (int i = 0; i < AFE_VAD_FRAME_SAMPLES; ++i) {
            vad_buf[i] = algo_buffer[i * MIC_COUNT + 0];
        }
        vad_state_t vad_state = vad_process(s_vad_handle,
                                            vad_buf,
                                            AFE_ALGO_SAMPLE_RATE_HZ,
                                            AFE_VAD_FRAME_MS);

        if (vad_state == VAD_SPEECH) {
            if (!s_vad_speech_active) {
                s_vad_speech_active = true;
                s_speech_frame_count = 0;
                /* Onset: search ring buffer for peak frame */
                const int peak_idx = find_peak_ring_frame(AFE_VAD_LOOKBACK_FRAMES);
                if (peak_idx >= 0) {
                    ESP_LOGD(TAG, "VAD onset, TDOA on peak frame");
                    update_source_position(s_ring_buffer[peak_idx]);
                }
            } else {
                /* Periodic TDOA during sustained speech */
                ++s_speech_frame_count;
                if (s_speech_frame_count % AFE_VAD_TDOA_INTERVAL_FRAMES == 0) {
                    update_source_position(algo_buffer);
                }
            }
        } else {
            if (s_vad_speech_active) {
                s_vad_speech_active = false;
                s_speech_frame_count = 0;
                ESP_LOGD(TAG, "VAD silence");
            }
        }

#if AFE_UAC_AUDIO_SOURCE == AFE_UAC_SOURCE_RAW_MIC
        for (size_t i = 0; i < s_capture_samples_per_channel; ++i) {
            int32_t sample = (int32_t)capture_buffer[i * MIC_COUNT + AFE_UAC_RAW_MIC_INDEX]
                             * AFE_UAC_GAIN_NUM / AFE_UAC_GAIN_DEN;
            if (sample > INT16_MAX) sample = INT16_MAX;
            if (sample < INT16_MIN) sample = INT16_MIN;
            uac_buffer[i] = (int16_t)sample;
        }
        write_uac_audio(uac_buffer,
                        s_capture_samples_per_channel * sizeof(int16_t),
                        "raw_mic");
#elif AFE_UAC_AUDIO_SOURCE == AFE_UAC_SOURCE_TEST_TONE
        fill_test_tone(uac_buffer, s_capture_samples_per_channel);
        write_uac_audio(uac_buffer,
                        s_capture_samples_per_channel * sizeof(int16_t),
                        "test_tone");
#endif
    }
}

static void cdc_report_task(void *arg)
{
    (void)arg;

    TickType_t last_uart_log_tick = 0;
    esp_err_t last_cdc_ret = ESP_OK;

    while (true) {
        char line[64];
        int len = 0;

        portENTER_CRITICAL(&s_position_lock);
        bool valid = s_latest_position.valid;
        int azi = s_latest_position.azimuth_deg;
        int ele = s_latest_position.elevation_deg;
        int conf = s_latest_position.confidence;
        bool vad_active = s_vad_speech_active;
        portEXIT_CRITICAL(&s_position_lock);

        if (valid) {
            len = snprintf(line, sizeof(line), "azi:%d,ele:%d,conf:%d,vad:%d\r\n", azi, ele, conf, vad_active ? 1 : 0);
        } else {
            len = snprintf(line, sizeof(line), "azi:null,ele:null,conf:0,vad:%d\r\n", vad_active ? 1 : 0);
        }

        esp_err_t cdc_ret = ESP_ERR_INVALID_SIZE;
        if (len > 0) {
            cdc_ret = usb_composite_cdc_write_str(line,
                                                  pdMS_TO_TICKS(AFE_CDC_WRITE_TIMEOUT_MS));
        }

        TickType_t now = xTaskGetTickCount();
        if (last_uart_log_tick == 0 ||
            now - last_uart_log_tick >= pdMS_TO_TICKS(AFE_UART_STATUS_INTERVAL_MS) ||
            cdc_ret != last_cdc_ret) {
            ESP_LOGI(TAG,
                     "status alive: usb_mounted=%d cdc_connected=%d line=0x%02x avail=%u cdc_write=%s",
                     usb_composite_is_mounted() ? 1 : 0,
                     usb_composite_cdc_is_connected() ? 1 : 0,
                     (unsigned)usb_composite_cdc_get_line_state(),
                     (unsigned)usb_composite_cdc_write_available(),
                     esp_err_to_name(cdc_ret));
            last_uart_log_tick = now;
            last_cdc_ret = cdc_ret;
        }

        vTaskDelay(pdMS_TO_TICKS(AFE_CDC_REPORT_INTERVAL_MS));
    }
}

esp_err_t afe_processor_init(void)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!isfinite(MIC_ARRAY_SPACING_MM) || MIC_ARRAY_SPACING_MM <= 0.0f ||
        MIC_ARRAY_SPACING_MM * 1e-3f * AFE_ALGO_SAMPLE_RATE_HZ / AFE_SPEED_OF_SOUND_MPS >
            AFE_TDOA_MAX_LAG_SAMPLES - 1) {
        return ESP_ERR_INVALID_ARG;
    }

    s_capture_samples_per_channel = AUDIO_FRAME_SAMPLES;
    s_algo_samples_per_channel = AUDIO_FRAME_SAMPLES / (I2S_MIC_SAMPLE_RATE_HZ / AFE_ALGO_SAMPLE_RATE_HZ);
    publish_invalid_position();

    /* Allocate ring buffer for VAD lookback */
    const size_t frame_bytes = s_algo_samples_per_channel * MIC_COUNT * sizeof(int16_t);
    s_ring_buffer = (int16_t **)audio_calloc(AFE_VAD_RING_FRAMES, sizeof(int16_t *));
    if (s_ring_buffer == NULL) {
        ESP_LOGE(TAG, "ring buffer ptr alloc failed");
        return ESP_ERR_NO_MEM;
    }
    for (int i = 0; i < AFE_VAD_RING_FRAMES; ++i) {
        s_ring_buffer[i] = (int16_t *)audio_calloc(1, frame_bytes);
        if (s_ring_buffer[i] == NULL) {
            ESP_LOGE(TAG, "ring buffer frame %d alloc failed", i);
            return ESP_ERR_NO_MEM;
        }
    }
    s_ring_write_idx = 0;
    s_ring_count = 0;

    /* Create WebRTC VAD: mode 0 (most sensitive), 16kHz, 30ms frame */
    s_vad_handle = vad_create_with_param(VAD_MODE_0,
                                         AFE_ALGO_SAMPLE_RATE_HZ,
                                         AFE_VAD_FRAME_MS,
                                         128,   /* min speech ms */
                                         500);  /* min noise ms */
    if (s_vad_handle == NULL) {
        ESP_LOGE(TAG, "VAD create failed");
        return ESP_ERR_NO_MEM;
    }
    s_vad_speech_active = false;

    s_initialized = true;

    ESP_LOGI(TAG,
             "VAD + ring buffer ready: ring=%d frames (%u KB), vad=%dms frames",
             AFE_VAD_RING_FRAMES,
             (unsigned)((AFE_VAD_RING_FRAMES * frame_bytes) / 1024),
             AFE_VAD_FRAME_MS);
    ESP_LOGI(TAG,
             "Raw UAC pipeline ready: frame=%u samples/ch, capture_channels=%d",
             (unsigned)s_capture_samples_per_channel,
             MIC_COUNT);
    ESP_LOGI(TAG, "Horizontal pair: spacing=%.1f mm, azimuth left +90 / front 0 / right -90",
             (double)MIC_ARRAY_SPACING_MM);
#if AFE_UAC_AUDIO_SOURCE == AFE_UAC_SOURCE_RAW_MIC
    ESP_LOGI(TAG, "UAC audio source: raw mic%d, no AFE/NS/AGC/VAD", AFE_UAC_RAW_MIC_INDEX + 1);
#elif AFE_UAC_AUDIO_SOURCE == AFE_UAC_SOURCE_TEST_TONE
    ESP_LOGI(TAG, "UAC audio source: %dHz test tone", float_to_int_round(AFE_UAC_TEST_TONE_HZ));
#else
    ESP_LOGW(TAG, "Unsupported UAC audio source selected; no audio will be sent");
#endif
    return ESP_OK;
}

esp_err_t afe_processor_start(void)
{
    if (!s_initialized || s_started) {
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(audio_capture_task,
                                            "audio_capture",
                                            AUDIO_TASK_STACK,
                                            NULL,
                                            AUDIO_TASK_PRIORITY,
                                            &s_audio_task,
                                            AUDIO_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(cdc_report_task,
                                 "cdc_report",
                                 AFE_CDC_TASK_STACK,
                                 NULL,
                                 AFE_CDC_TASK_PRIORITY,
                                 &s_cdc_task,
                                 AFE_CDC_TASK_CORE);
    if (ok != pdPASS) {
        vTaskDelete(s_audio_task);
        s_audio_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    ESP_LOGI(TAG, "Raw audio and CDC report tasks started");
    return ESP_OK;
}

size_t afe_processor_get_feed_samples_per_channel(void)
{
    return s_capture_samples_per_channel;
}

int afe_processor_get_doa_angle(void)
{
    afe_source_position_t position;
    if (!afe_processor_get_source_position(&position)) {
        return -1;
    }
    return position.azimuth_deg;
}

bool afe_processor_get_source_position(afe_source_position_t *position)
{
    if (position == NULL) {
        return false;
    }

    portENTER_CRITICAL(&s_position_lock);
    *position = s_latest_position;
    portEXIT_CRITICAL(&s_position_lock);
    return position->valid;
}