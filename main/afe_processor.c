#include "afe_processor.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
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
#define AFE_TDOA_MIN_RMS 20.0f
#define AFE_TDOA_MIN_CORRELATION 0.25f
#define AFE_TDOA_MIN_CONFIDENCE 15
#define AFE_TDOA_MAX_LAG_SAMPLES 16
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
    {MIC_ARRAY_POS0_MM_X * 0.001f, MIC_ARRAY_POS0_MM_Y * 0.001f, MIC_ARRAY_POS0_MM_Z * 0.001f},
    {MIC_ARRAY_POS1_MM_X * 0.001f, MIC_ARRAY_POS1_MM_Y * 0.001f, MIC_ARRAY_POS1_MM_Z * 0.001f},
    {MIC_ARRAY_POS2_MM_X * 0.001f, MIC_ARRAY_POS2_MM_Y * 0.001f, MIC_ARRAY_POS2_MM_Z * 0.001f},
    {MIC_ARRAY_POS3_MM_X * 0.001f, MIC_ARRAY_POS3_MM_Y * 0.001f, MIC_ARRAY_POS3_MM_Z * 0.001f},
};

/* All C(N,2) mic pairs for 4-mic TDOA estimation (6 pairs). */
#define AFE_NUM_MIC_PAIRS 6
static const int s_mic_pairs[AFE_NUM_MIC_PAIRS][2] = {
    {0, 1}, {0, 2}, {0, 3},
    {1, 2}, {1, 3}, {2, 3},
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
static float s_latest_azimuth;
static float s_latest_elevation;
static bool s_position_valid;
static bool s_initialized;
static bool s_started;

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

static mic_position_t vec_add(mic_position_t a, mic_position_t b)
{
    return (mic_position_t) {
        .x = a.x + b.x,
        .y = a.y + b.y,
        .z = a.z + b.z,
    };
}

static mic_position_t vec_scale(mic_position_t v, float scale)
{
    return (mic_position_t) {
        .x = v.x * scale,
        .y = v.y * scale,
        .z = v.z * scale,
    };
}

static float vec_dot(mic_position_t a, mic_position_t b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static mic_position_t vec_cross(mic_position_t a, mic_position_t b)
{
    return (mic_position_t) {
        .x = a.y * b.z - a.z * b.y,
        .y = a.z * b.x - a.x * b.z,
        .z = a.x * b.y - a.y * b.x,
    };
}

static float vec_norm_sq(mic_position_t v)
{
    return vec_dot(v, v);
}

static float vec_norm(mic_position_t v)
{
    return sqrtf(vec_norm_sq(v));
}

static bool vec_normalize(mic_position_t *v)
{
    float norm = vec_norm(*v);
    if (norm < 1e-6f) {
        return false;
    }
    *v = vec_scale(*v, 1.0f / norm);
    return true;
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

    portENTER_CRITICAL(&s_position_lock);
    s_position_valid = false;
    portEXIT_CRITICAL(&s_position_lock);
}

static int max_tdoa_lag_samples(void)
{
    float max_distance = 0.0f;
    for (int a = 0; a < MIC_COUNT; ++a) {
        for (int b = a + 1; b < MIC_COUNT; ++b) {
            float distance = vec_norm(vec_sub(s_mic_positions[b], s_mic_positions[a]));
            if (distance > max_distance) {
                max_distance = distance;
            }
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

static float tdoa_residual(mic_position_t direction, int mic_a, int mic_b, float delay_samples)
{
    mic_position_t row = vec_sub(s_mic_positions[mic_b], s_mic_positions[mic_a]);
    return vec_dot(row, direction) - delay_to_distance(delay_samples);
}

/*
 * 3×3 symmetric positive-definite linear system solver.
 * Solves A * x = b where A is given by its upper-triangular entries (a11,a12,a13,a22,a23,a33).
 * Uses closed-form adjugate matrix inversion — no LU decomposition overhead,
 * numerically adequate for the well-conditioned mic-array geometry.
 *
 * x = A⁻¹ * b, with A⁻¹ = adj(A) / det(A).
 */
static bool solve_3x3_symmetric(float a11, float a12, float a13,
                                float a22, float a23, float a33,
                                float b1, float b2, float b3,
                                float *x1, float *x2, float *x3)
{
    /* Compute cofactors of the symmetric matrix A:
     *      [a11 a12 a13]
     *  A = [a12 a22 a23]
     *      [a13 a23 a33]
     */
    const float c11 = a22 * a33 - a23 * a23;
    const float c12 = a13 * a23 - a12 * a33;  /* = c21 */
    const float c13 = a12 * a23 - a13 * a22;  /* = c31 */
    const float c22 = a11 * a33 - a13 * a13;
    const float c23 = a13 * a12 - a11 * a23;  /* = c32 */
    const float c33 = a11 * a22 - a12 * a12;

    /* det(A) = a11*c11 + a12*c12 + a13*c13 (expansion along first row) */
    const float det = a11 * c11 + a12 * c12 + a13 * c13;

    if (fabsf(det) < 1e-12f) {
        return false;
    }

    const float inv_det = 1.0f / det;

    /* x = (1/det) * adj(A) * b = (1/det) * C^T * b
     * Since C is symmetric (C^T = C):  */
    *x1 = (c11 * b1 + c12 * b2 + c13 * b3) * inv_det;
    *x2 = (c12 * b1 + c22 * b2 + c23 * b3) * inv_det;
    *x3 = (c13 * b1 + c23 * b2 + c33 * b3) * inv_det;

    return true;
}

/*
 * 3D far-field direction-of-arrival via over-determined least-squares.
 *
 * For each mic pair (a,b):
 *   (pos_b - pos_a) · direction = delay_ab * speed_of_sound / sample_rate
 *
 * This yields N (=6 for 4-mic) equations in 3 unknowns (direction x,y,z).
 * The normal equations AᵀA·d = Aᵀb form a 3×3 symmetric positive-definite
 * system solved in closed form.
 *
 * With non-coplanar mics the system has a unique solution — no mirror
 * ambiguity.  The unit-vector constraint ‖d‖=1 is enforced by normalisation
 * of the least-squares result, which is the maximum-likelihood estimate
 * under Gaussian i.i.d. TDOA noise.
 */
static bool solve_far_field_vector(const float delays[AFE_NUM_MIC_PAIRS],
                                   float *ux, float *uy, float *uz,
                                   float *residual_norm)
{
    /* Accumulate AᵀA (3×3 symmetric, stored as 6 floats) and Aᵀb (3×1). */
    float a11 = 0.0f, a12 = 0.0f, a13 = 0.0f;
    float a22 = 0.0f, a23 = 0.0f, a33 = 0.0f;
    float atb1 = 0.0f, atb2 = 0.0f, atb3 = 0.0f;

    for (int p = 0; p < AFE_NUM_MIC_PAIRS; ++p) {
        const int a = s_mic_pairs[p][0];
        const int b = s_mic_pairs[p][1];

        /* Row vector: r = pos_b - pos_a */
        const mic_position_t r = vec_sub(s_mic_positions[b], s_mic_positions[a]);
        const float target = delay_to_distance(delays[p]);

        a11 += r.x * r.x;
        a12 += r.x * r.y;
        a13 += r.x * r.z;
        a22 += r.y * r.y;
        a23 += r.y * r.z;
        a33 += r.z * r.z;

        atb1 += r.x * target;
        atb2 += r.y * target;
        atb3 += r.z * target;
    }

    float dx = 0.0f, dy = 0.0f, dz = 0.0f;
    if (!solve_3x3_symmetric(a11, a12, a13, a22, a23, a33,
                             atb1, atb2, atb3,
                             &dx, &dy, &dz)) {
        return false;
    }

    /* The algebraic solution's magnitude is a quality indicator:
     * if it substantially exceeds 1, TDOAs are inconsistent. */
    const float norm_sq = dx * dx + dy * dy + dz * dz;
    if (norm_sq > 2.25f) {
        return false;
    }

    mic_position_t direction = {dx, dy, dz};
    if (!vec_normalize(&direction)) {
        return false;
    }

    /* Compute normalised residual across all pairs. */
    float residual_sum_sq = 0.0f;
    for (int p = 0; p < AFE_NUM_MIC_PAIRS; ++p) {
        const int a = s_mic_pairs[p][0];
        const int b = s_mic_pairs[p][1];
        const float r = tdoa_residual(direction, a, b, delays[p]);
        residual_sum_sq += r * r;
    }

    float max_distance = 0.0f;
    for (int a = 0; a < MIC_COUNT; ++a) {
        for (int b = a + 1; b < MIC_COUNT; ++b) {
            float d = vec_norm(vec_sub(s_mic_positions[b], s_mic_positions[a]));
            if (d > max_distance) {
                max_distance = d;
            }
        }
    }
    if (max_distance < 1e-6f) {
        return false;
    }
    *residual_norm = sqrtf(residual_sum_sq / (float)AFE_NUM_MIC_PAIRS) / max_distance;

    *ux = direction.x;
    *uy = direction.y;
    *uz = direction.z;
    return true;
}

static void update_source_position(const int16_t *interleaved_frame)
{
    if (interleaved_frame == NULL || s_feed_samples_per_channel == 0) {
        publish_invalid_position();
        return;
    }

#if MIC_COUNT < 4
    publish_invalid_position();
    return;
#endif

    if (frame_rms(interleaved_frame) < AFE_TDOA_MIN_RMS) {
        publish_invalid_position();
        return;
    }

    float means[MIC_COUNT] = {0};
    for (int mic = 0; mic < MIC_COUNT; ++mic) {
        means[mic] = channel_mean(interleaved_frame, mic);
    }

    /* Compute TDOA for all C(4,2)=6 mic pairs via cross-correlation. */
    float delays[AFE_NUM_MIC_PAIRS] = {0};
    float corrs[AFE_NUM_MIC_PAIRS] = {0};

    for (int p = 0; p < AFE_NUM_MIC_PAIRS; ++p) {
        const int a = s_mic_pairs[p][0];
        const int b = s_mic_pairs[p][1];
        if (!estimate_pair_delay_samples(interleaved_frame, a, b, means,
                                         &delays[p], &corrs[p])) {
            publish_invalid_position();
            return;
        }
    }

    float ux = 0.0f;
    float uy = 0.0f;
    float uz = 0.0f;
    float residual = 0.0f;
    if (!solve_far_field_vector(delays, &ux, &uy, &uz, &residual)) {
        publish_invalid_position();
        return;
    }

    /* Average correlation across all pairs for confidence scoring. */
    float corr_sum = 0.0f;
    for (int p = 0; p < AFE_NUM_MIC_PAIRS; ++p) {
        corr_sum += corrs[p];
    }
    const float corr_score = clampf(corr_sum / (float)AFE_NUM_MIC_PAIRS, 0.0f, 1.0f);
    const float residual_score = clampf(1.0f - residual * 4.0f, 0.0f, 1.0f);
    const int confidence = float_to_int_round(corr_score * residual_score * 100.0f);
    if (confidence < AFE_TDOA_MIN_CONFIDENCE) {
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
        .confidence = confidence,
    };
    publish_source_position(position);

    /* Update float globals for CDC JSON reporting. */
    portENTER_CRITICAL(&s_position_lock);
    s_latest_azimuth = azimuth;
    s_latest_elevation = elevation;
    s_position_valid = true;
    portEXIT_CRITICAL(&s_position_lock);
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

    /*
     * Write the entire frame in one go.  usb_composite_uac_write() calls
     * tud_audio_write() internally and already handles pacing: it delays
     * 1 ms only when the SW FIFO is full, and retries until timeout.
     * Removing the outer per-chunk vTaskDelay avoids doubling the latency
     * and keeps the audio pipeline within the 32 ms real-time budget.
     */
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

    while (true) {
        esp_err_t ret = i2s_mic_driver_read(capture_buffer,
                                            s_feed_samples_per_channel,
                                            portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "read mic frame failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // TDOA direction estimation runs before UAC write so its CPU
        // cost is absorbed by the I2S DMA buffer fill period rather than
        // starving the USB SW FIFO.
        update_source_position(capture_buffer);

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
        bool valid = s_position_valid;
        float azi = s_latest_azimuth;
        float ele = s_latest_elevation;
        portEXIT_CRITICAL(&s_position_lock);

        if (valid) {
            len = snprintf(line,
                           sizeof(line),
                           "{\"azi\":%.1f,\"ele\":%.1f}\n",
                           (double)azi,
                           (double)ele);
        } else {
            len = snprintf(line, sizeof(line), "{\"azi\":null,\"ele\":null}\n");
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
    s_initialized = true;

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
