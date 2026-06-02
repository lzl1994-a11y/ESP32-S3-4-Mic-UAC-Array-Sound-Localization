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
#define AFE_SPEED_OF_SOUND_MPS 343.2f
#define AFE_TDOA_MIN_RMS 5.0f
#define AFE_TDOA_MIN_CORRELATION 0.04f
#define AFE_TDOA_MIN_CONFIDENCE 1
#define AFE_TDOA_MAX_LAG_SAMPLES 16
#define AFE_VAD_RING_FRAMES 20
#define AFE_VAD_LOOKBACK_FRAMES 6
#define AFE_VAD_FRAME_MS 30
#define AFE_VAD_FRAME_SAMPLES ((I2S_MIC_SAMPLE_RATE_HZ * AFE_VAD_FRAME_MS) / 1000)
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if AFE_UAC_RAW_MIC_INDEX >= MIC_COUNT
#error "AFE_UAC_RAW_MIC_INDEX out of range"
#endif

static const char *TAG = "afe_processor";

typedef struct {
    float x;
    float y;
    float z;
} mic_position_t;

static const mic_position_t s_mic_positions[MIC_COUNT] = {
    {MIC_ARRAY_POS0_MM_X, MIC_ARRAY_POS0_MM_Y, MIC_ARRAY_POS0_MM_Z},
    {MIC_ARRAY_POS1_MM_X, MIC_ARRAY_POS1_MM_Y, MIC_ARRAY_POS1_MM_Z},
    {MIC_ARRAY_POS2_MM_X, MIC_ARRAY_POS2_MM_Y, MIC_ARRAY_POS2_MM_Z},
    {MIC_ARRAY_POS3_MM_X, MIC_ARRAY_POS3_MM_Y, MIC_ARRAY_POS3_MM_Z},
};

#define AFE_NUM_MIC_PAIRS 2
static const int s_mic_pairs[AFE_NUM_MIC_PAIRS][2] = {
    {0, 1},    /* Mic0→Mic1  (I2S1), direction (70, 0, 0) mm  → constrains ux     */
    {2, 3},    /* Mic2→Mic3  (I2S0), direction (0, 30, 30) mm → constrains uy+uz  */
};

static TaskHandle_t s_audio_task;
static TaskHandle_t s_cdc_task;
static size_t s_feed_samples_per_channel;
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

static mic_position_t vec_sub(mic_position_t a, mic_position_t b)
{
    return (mic_position_t) {
        .x = a.x - b.x,
        .y = a.y - b.y,
        .z = a.z - b.z,
    };
}

static float vec_norm(mic_position_t v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static float direction_azimuth_deg(float ux, float uy)
{
    float azimuth = atan2f(ux, uy) * (180.0f / (float)M_PI);
    if (azimuth > 180.0f) {
        azimuth -= 360.0f;
    } else if (azimuth <= -180.0f) {
        azimuth += 360.0f;
    }
    return azimuth;
}

static float direction_elevation_deg(float ux, float uy, float uz)
{
    return atan2f(uz, sqrtf(ux * ux + uy * uy)) * (180.0f / (float)M_PI);
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
        const size_t total = s_feed_samples_per_channel * MIC_COUNT;
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
           s_feed_samples_per_channel * MIC_COUNT * sizeof(int16_t));
    s_ring_write_idx = (s_ring_write_idx + 1) % AFE_VAD_RING_FRAMES;
    if (s_ring_count < AFE_VAD_RING_FRAMES) {
        ++s_ring_count;
    }
}

static int max_tdoa_lag_samples(void)
{
    float max_distance = 0.0f;
    for (int p = 0; p < AFE_NUM_MIC_PAIRS; ++p) {
        const int a = s_mic_pairs[p][0];
        const int b = s_mic_pairs[p][1];
        const float distance_m = vec_norm(vec_sub(s_mic_positions[b], s_mic_positions[a])) * 1e-3f; /* mm→m */
        if (distance_m > max_distance) {
            max_distance = distance_m;
        }
    }

    int lag = (int)((max_distance * I2S_MIC_SAMPLE_RATE_HZ / AFE_SPEED_OF_SOUND_MPS) + 2.0f);
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
    for (size_t i = 0; i < s_feed_samples_per_channel; ++i) {
        sum += interleaved_frame[i * MIC_COUNT + mic];
    }
    return (float)sum / (float)s_feed_samples_per_channel;
}

static float frame_rms(const int16_t *interleaved_frame)
{
    uint64_t sum_sq = 0;
    const size_t total_samples = s_feed_samples_per_channel * MIC_COUNT;
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
    if ((size_t)abs_lag >= s_feed_samples_per_channel) {
        return 0.0f;
    }

    const size_t count = s_feed_samples_per_channel - (size_t)abs_lag;
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
    return -AFE_SPEED_OF_SOUND_MPS * delay_samples / (float)I2S_MIC_SAMPLE_RATE_HZ;
}

/*
 * General dual-axis TDOA solver.
 *
 * For each mic pair p with direction vector rₚ = pos[b] − pos[a]:
 *   d̂ₚ·u = delay_to_distance(τₚ) / |rₚ|
 *
 * Together with |u| = 1, solve for the source unit vector u = (ux, uy, uz).
 * No hardcoded layout assumptions — auto-adapts to s_mic_positions.
 *
 * With the current layout (Pair 0: X-axis 70mm, Pair 1: YZ-plane 42.4mm),
 * this gives two independent constraints:
 *   ux = delay_to_distance(τ₀) / 0.070
 *   0.707·uy + 0.707·uz = delay_to_distance(τ₁) / 0.0424
 */
static bool solve_dual_axis(const float delays[AFE_NUM_MIC_PAIRS],
                            const float corrs[AFE_NUM_MIC_PAIRS],
                            float *out_ux, float *out_uy, float *out_uz,
                            float *out_confidence)
{
    /* Compute normalized direction vectors from actual mic positions */
    float nx[AFE_NUM_MIC_PAIRS], ny[AFE_NUM_MIC_PAIRS], nz[AFE_NUM_MIC_PAIRS];
    float proj[AFE_NUM_MIC_PAIRS];

    for (int p = 0; p < AFE_NUM_MIC_PAIRS; ++p) {
        const int a = s_mic_pairs[p][0];
        const int b = s_mic_pairs[p][1];
        const float dx = (s_mic_positions[b].x - s_mic_positions[a].x) * 1e-3f; /* mm→m */
        const float dy = (s_mic_positions[b].y - s_mic_positions[a].y) * 1e-3f;
        const float dz = (s_mic_positions[b].z - s_mic_positions[a].z) * 1e-3f;
        const float mag = sqrtf(dx * dx + dy * dy + dz * dz);

        if (mag < 1e-6f) return false;
        nx[p] = dx / mag;
        ny[p] = dy / mag;
        nz[p] = dz / mag;
        proj[p] = delay_to_distance(delays[p]) / mag;
    }

    /*
     * Solve 2×2 linear system for (ux, uy) in terms of uz:
     *   nx₀·ux + ny₀·uy = proj₀ − nz₀·uz
     *   nx₁·ux + ny₁·uy = proj₁ − nz₁·uz
     */
    const float det = nx[0] * ny[1] - nx[1] * ny[0];
    if (fabsf(det) < 1e-4f) return false;

    /* ux = A + B·uz,  uy = C + D·uz */
    const float A = (proj[0] * ny[1] - proj[1] * ny[0]) / det;
    const float B = (nz[1] * ny[0] - nz[0] * ny[1]) / det;
    const float C = (nx[0] * proj[1] - nx[1] * proj[0]) / det;
    const float D = (nx[1] * nz[0] - nx[0] * nz[1]) / det;

    /* Quadratic: (B² + D² + 1)·uz² + 2(AB + CD)·uz + (A² + C² − 1) = 0 */
    const float qa = B * B + D * D + 1.0f;
    const float qb = 2.0f * (A * B + C * D);
    const float qc = A * A + C * C - 1.0f;

    const float disc = qb * qb - 4.0f * qa * qc;
    if (disc < 0.0f) {
        /*
         * Noisy delay estimates can push the combined projection norm
         * beyond the unit sphere (proj₀² + proj₁² > 1).  Scale both
         * projections back to the boundary and recompute.
         */
        const float proj_norm_sq = proj[0] * proj[0] + proj[1] * proj[1];
        if (proj_norm_sq <= 1.0f) {
            return false;
        }
        const float scale = 1.0f / sqrtf(proj_norm_sq);
        const float p0 = proj[0] * scale;
        const float p1 = proj[1] * scale;

        const float A2 = (p0 * ny[1] - p1 * ny[0]) / det;
        const float C2 = (nx[0] * p1 - nx[1] * p0) / det;
        const float qa2 = B * B + D * D + 1.0f;
        const float qb2 = 2.0f * (A2 * B + C2 * D);
        const float qc2 = A2 * A2 + C2 * C2 - 1.0f;
        const float disc2 = qb2 * qb2 - 4.0f * qa2 * qc2;
        if (disc2 < 0.0f) return false;

        const float sqrt_disc2 = sqrtf(disc2);
        const float uz1b = (-qb2 + sqrt_disc2) / (2.0f * qa2);
        const float uz2b = (-qb2 - sqrt_disc2) / (2.0f * qa2);
        const float uzb = (uz1b >= uz2b) ? uz1b : uz2b;
        const float uxb = A2 + B * uzb;
        const float uyb = C2 + D * uzb;

        const float normb = sqrtf(uxb * uxb + uyb * uyb + uzb * uzb);
        if (normb < 0.01f) return false;

        *out_ux = uxb / normb;
        *out_uy = uyb / normb;
        *out_uz = uzb / normb;
        *out_confidence = clampf(0.5f * (corrs[0] + corrs[1]), 0.0f, 1.0f) * 100.0f;
        return true;
    }

    const float sqrt_disc = sqrtf(disc);
    const float uz1 = (-qb + sqrt_disc) / (2.0f * qa);
    const float uz2 = (-qb - sqrt_disc) / (2.0f * qa);

    /* Pick positive-z root (source is above mic plane) */
    const float uz = (uz1 >= uz2) ? uz1 : uz2;
    const float ux = A + B * uz;
    const float uy = C + D * uz;

    /* Renormalize to unit length (compensates floating-point drift) */
    const float norm = sqrtf(ux * ux + uy * uy + uz * uz);
    if (norm < 0.01f) return false;

    *out_ux = ux / norm;
    *out_uy = uy / norm;
    *out_uz = uz / norm;
    *out_confidence = clampf(0.5f * (corrs[0] + corrs[1]), 0.0f, 1.0f) * 100.0f;
    return true;
}

static void update_source_position(const int16_t *interleaved_frame)
{
    if (interleaved_frame == NULL || s_feed_samples_per_channel == 0) {
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

    float delays[AFE_NUM_MIC_PAIRS] = {0};
    float corrs[AFE_NUM_MIC_PAIRS] = {0};

    for (int p = 0; p < AFE_NUM_MIC_PAIRS; ++p) {
        const int a = s_mic_pairs[p][0];
        const int b = s_mic_pairs[p][1];
        if (!estimate_pair_delay_samples(interleaved_frame, a, b, means,
                                         &delays[p], &corrs[p])) {
            ESP_LOGD(TAG, "TDOA skip: pair%d corr=%.3f < %.3f",
                     p, (double)corrs[p], (double)AFE_TDOA_MIN_CORRELATION);
            publish_invalid_position();
            return;
        }
    }

    ESP_LOGD(TAG, "TDOA: pair0 corr=%.3f lag=%.2f  pair1 corr=%.3f lag=%.2f  rms=%.1f",
             (double)corrs[0], (double)delays[0],
             (double)corrs[1], (double)delays[1],
             (double)rms);

    float ux, uy, uz, confidence;
    if (!solve_dual_axis(delays, corrs, &ux, &uy, &uz, &confidence)) {
        ESP_LOGD(TAG, "TDOA skip: solve_dual_axis failed");
        publish_invalid_position();
        return;
    }

    if (confidence < (float)AFE_TDOA_MIN_CONFIDENCE) {
        ESP_LOGD(TAG, "TDOA skip: conf=%.1f < %d",
                 (double)confidence, AFE_TDOA_MIN_CONFIDENCE);
        publish_invalid_position();
        return;
    }

    const float azimuth = direction_azimuth_deg(ux, uy);
    const float elevation = direction_elevation_deg(ux, uy, uz);

    afe_source_position_t position = {
        .valid = true,
        .x_milli = float_to_int_round(ux * AFE_UNIT_VECTOR_SCALE),
        .y_milli = float_to_int_round(uy * AFE_UNIT_VECTOR_SCALE),
        .z_milli = float_to_int_round(uz * AFE_UNIT_VECTOR_SCALE),
        .azimuth_deg = float_to_int_round(azimuth),
        .elevation_deg = float_to_int_round(elevation),
        .confidence = float_to_int_round(confidence),
    };
    ESP_LOGI(TAG, "TDOA HIT: azi=%d ele=%d conf=%d corr=[%.3f,%.3f] lag=[%.2f,%.2f]",
             position.azimuth_deg, position.elevation_deg, position.confidence,
             (double)corrs[0], (double)corrs[1],
             (double)delays[0], (double)delays[1]);
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

    const size_t capture_samples = s_feed_samples_per_channel * MIC_COUNT;
    int16_t *capture_buffer = (int16_t *)audio_calloc(capture_samples, sizeof(int16_t));
    if (capture_buffer == NULL) {
        ESP_LOGE(TAG, "capture buffer alloc failed");
        vTaskDelete(NULL);
        return;
    }

    int16_t *uac_buffer = (int16_t *)audio_calloc(s_feed_samples_per_channel, sizeof(int16_t));
    if (uac_buffer == NULL) {
        ESP_LOGE(TAG, "UAC buffer alloc failed");
        free(capture_buffer);
        vTaskDelete(NULL);
        return;
    }

    /* VAD sub-frame buffer: extract Mic0 first 480 samples per 512-sample frame */
    int16_t *vad_buf = (int16_t *)audio_calloc(AFE_VAD_FRAME_SAMPLES, sizeof(int16_t));
    if (vad_buf == NULL) {
        ESP_LOGE(TAG, "VAD buffer alloc failed");
        free(uac_buffer);
        free(capture_buffer);
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        esp_err_t ret = i2s_mic_driver_read(capture_buffer,
                                            s_feed_samples_per_channel,
                                            portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "read mic frame failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Store full 4ch frame in ring buffer */
        ring_buffer_push(capture_buffer);

        /* Extract Mic0 first N samples for VAD (30ms = 480 samples) */
        for (int i = 0; i < AFE_VAD_FRAME_SAMPLES; ++i) {
            vad_buf[i] = capture_buffer[i * MIC_COUNT + 0];
        }
        vad_state_t vad_state = vad_process(s_vad_handle,
                                            vad_buf,
                                            I2S_MIC_SAMPLE_RATE_HZ,
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
                    update_source_position(capture_buffer);
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
        for (size_t i = 0; i < s_feed_samples_per_channel; ++i) {
            uac_buffer[i] = capture_buffer[i * MIC_COUNT + AFE_UAC_RAW_MIC_INDEX];
        }
        write_uac_audio(uac_buffer,
                        s_feed_samples_per_channel * sizeof(int16_t),
                        "raw_mic");
#elif AFE_UAC_AUDIO_SOURCE == AFE_UAC_SOURCE_TEST_TONE
        fill_test_tone(uac_buffer, s_feed_samples_per_channel);
        write_uac_audio(uac_buffer,
                        s_feed_samples_per_channel * sizeof(int16_t),
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

    s_feed_samples_per_channel = AUDIO_FRAME_SAMPLES;
    publish_invalid_position();

    /* Allocate ring buffer for VAD lookback */
    const size_t frame_bytes = s_feed_samples_per_channel * MIC_COUNT * sizeof(int16_t);
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
                                         I2S_MIC_SAMPLE_RATE_HZ,
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
             "Raw UAC/TDOA pipeline ready: frame=%u samples/ch, capture_channels=%d",
             (unsigned)s_feed_samples_per_channel,
             MIC_COUNT);
    ESP_LOGI(TAG,
             "Mic positions mm: m0=(%d,%d,%d) m1=(%d,%d,%d) m2=(%d,%d,%d) m3=(%d,%d,%d)",
             float_to_int_round(MIC_ARRAY_POS0_MM_X),
             float_to_int_round(MIC_ARRAY_POS0_MM_Y),
             float_to_int_round(MIC_ARRAY_POS0_MM_Z),
             float_to_int_round(MIC_ARRAY_POS1_MM_X),
             float_to_int_round(MIC_ARRAY_POS1_MM_Y),
             float_to_int_round(MIC_ARRAY_POS1_MM_Z),
             float_to_int_round(MIC_ARRAY_POS2_MM_X),
             float_to_int_round(MIC_ARRAY_POS2_MM_Y),
             float_to_int_round(MIC_ARRAY_POS2_MM_Z),
             float_to_int_round(MIC_ARRAY_POS3_MM_X),
             float_to_int_round(MIC_ARRAY_POS3_MM_Y),
             float_to_int_round(MIC_ARRAY_POS3_MM_Z));
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
    return s_feed_samples_per_channel;
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