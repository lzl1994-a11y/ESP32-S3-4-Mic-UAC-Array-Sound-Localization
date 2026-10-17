#include "afe_processor.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2s_mic_driver.h"
#include "usb_composite.h"

/* VAD and its 48 -> 16 kHz input conversion are retained but disabled. */
#ifndef AFE_ENABLE_VAD
#define AFE_ENABLE_VAD 0
#endif

#if AFE_ENABLE_VAD
#include "esp_vad.h"
#define AFE_VAD_FRAME_MS 30
#define AFE_ALGO_SAMPLE_RATE_HZ 16000
#define AFE_VAD_FRAME_SAMPLES ((AFE_ALGO_SAMPLE_RATE_HZ * AFE_VAD_FRAME_MS) / 1000)

_Static_assert(I2S_MIC_SAMPLE_RATE_HZ % AFE_ALGO_SAMPLE_RATE_HZ == 0,
               "Capture rate must be an integer multiple of the algorithm rate");
_Static_assert(SAMPLES_PER_CHANNEL * AFE_ALGO_SAMPLE_RATE_HZ ==
               AFE_VAD_FRAME_SAMPLES * I2S_MIC_SAMPLE_RATE_HZ,
               "Each capture frame must cover one complete VAD frame");

static size_t s_algo_samples_per_channel;
static vad_handle_t s_vad_handle;
static bool s_vad_speech_active;
#endif

#define AUDIO_FRAME_SAMPLES SAMPLES_PER_CHANNEL
#define AUDIO_TASK_STACK 8192
#define AUDIO_TASK_PRIORITY 9
#define AUDIO_TASK_CORE 1
#define AFE_UAC_WRITE_MARGIN_MS 80
#define AFE_UAC_WRITE_MAX_MS 500
#define AFE_UAC_RAW_MIC_INDEX 0
#define AFE_UAC_GAIN_NUM 2
#define AFE_UAC_GAIN_DEN 1

#if AFE_UAC_RAW_MIC_INDEX < 0 || AFE_UAC_RAW_MIC_INDEX >= MIC_COUNT
#error "AFE_UAC_RAW_MIC_INDEX out of range"
#endif

static const char *TAG = "afe_processor";
static TaskHandle_t s_audio_task;
static size_t s_capture_samples_per_channel;
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

static void write_uac_audio(const int16_t *data, size_t byte_len)
{
    if (!usb_composite_uac_is_streaming()) {
        return;
    }

    esp_err_t ret = usb_composite_uac_write(data, byte_len, uac_write_timeout_ticks(byte_len));
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "UAC microphone write failed: %s, total_bytes=%u",
                 esp_err_to_name(ret), (unsigned)byte_len);
    }
}

static void audio_capture_task(void *arg)
{
    (void)arg;

    const size_t capture_samples = s_capture_samples_per_channel * MIC_COUNT;
    int16_t *capture_buffer = (int16_t *)audio_calloc(capture_samples, sizeof(int16_t));
    int16_t *uac_buffer = (int16_t *)audio_calloc(s_capture_samples_per_channel, sizeof(int16_t));
#if AFE_ENABLE_VAD
    const size_t algo_samples = s_algo_samples_per_channel * MIC_COUNT;
    int16_t *algo_buffer = (int16_t *)audio_calloc(algo_samples, sizeof(int16_t));
    int16_t *vad_buf = (int16_t *)audio_calloc(AFE_VAD_FRAME_SAMPLES, sizeof(int16_t));
    if (algo_buffer == NULL || vad_buf == NULL) {
        goto no_mem;
    }
#endif
    if (capture_buffer == NULL || uac_buffer == NULL) {
        goto no_mem;
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

#if AFE_ENABLE_VAD
        /* Retain the original 3:1 decimation solely for optional VAD. */
        const int decimation_factor = I2S_MIC_SAMPLE_RATE_HZ / AFE_ALGO_SAMPLE_RATE_HZ;
        for (size_t i = 0; i < s_algo_samples_per_channel; ++i) {
            for (size_t ch = 0; ch < MIC_COUNT; ++ch) {
                algo_buffer[i * MIC_COUNT + ch] =
                    capture_buffer[(i * decimation_factor) * MIC_COUNT + ch];
            }
        }
        for (int i = 0; i < AFE_VAD_FRAME_SAMPLES; ++i) {
            vad_buf[i] = algo_buffer[i * MIC_COUNT + AFE_UAC_RAW_MIC_INDEX];
        }
        const bool speech_active = vad_process(s_vad_handle, vad_buf,
                                               AFE_ALGO_SAMPLE_RATE_HZ,
                                               AFE_VAD_FRAME_MS) == VAD_SPEECH;
        if (speech_active != s_vad_speech_active) {
            s_vad_speech_active = speech_active;
            ESP_LOGD(TAG, "VAD speech=%d", speech_active ? 1 : 0);
        }
#endif

        for (size_t i = 0; i < s_capture_samples_per_channel; ++i) {
            int32_t sample = (int32_t)capture_buffer[i * MIC_COUNT + AFE_UAC_RAW_MIC_INDEX]
                             * AFE_UAC_GAIN_NUM / AFE_UAC_GAIN_DEN;
            if (sample > INT16_MAX) sample = INT16_MAX;
            if (sample < INT16_MIN) sample = INT16_MIN;
            uac_buffer[i] = (int16_t)sample;
        }
        write_uac_audio(uac_buffer, s_capture_samples_per_channel * sizeof(int16_t));
    }

no_mem:
    ESP_LOGE(TAG, "capture task buffer allocation failed");
#if AFE_ENABLE_VAD
    free(vad_buf);
    free(algo_buffer);
#endif
    free(uac_buffer);
    free(capture_buffer);
    vTaskDelete(NULL);
}

esp_err_t afe_processor_init(void)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_capture_samples_per_channel = AUDIO_FRAME_SAMPLES;
#if AFE_ENABLE_VAD
    s_algo_samples_per_channel =
        AUDIO_FRAME_SAMPLES / (I2S_MIC_SAMPLE_RATE_HZ / AFE_ALGO_SAMPLE_RATE_HZ);
    s_vad_handle = vad_create_with_param(VAD_MODE_0, AFE_ALGO_SAMPLE_RATE_HZ,
                                         AFE_VAD_FRAME_MS, 128, 500);
    if (s_vad_handle == NULL) {
        ESP_LOGE(TAG, "VAD create failed");
        return ESP_ERR_NO_MEM;
    }
    s_vad_speech_active = false;
#endif
    s_initialized = true;
    ESP_LOGI(TAG, "Raw UAC ready: %d Hz, capture_channels=%d, USB mic=Mic%d, VAD=%s",
             I2S_MIC_SAMPLE_RATE_HZ, MIC_COUNT, AFE_UAC_RAW_MIC_INDEX,
             AFE_ENABLE_VAD ? "on" : "off");
    return ESP_OK;
}

esp_err_t afe_processor_start(void)
{
    if (!s_initialized || s_started) {
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(audio_capture_task, "audio_capture",
                                            AUDIO_TASK_STACK, NULL,
                                            AUDIO_TASK_PRIORITY, &s_audio_task,
                                            AUDIO_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    return ESP_OK;
}

size_t afe_processor_get_feed_samples_per_channel(void)
{
    return s_capture_samples_per_channel;
}
