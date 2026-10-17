#include "i2s_mic_driver.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define I2S_BCLK_GPIO GPIO_NUM_4
#define I2S_WS_GPIO GPIO_NUM_5
#define I2S_DIN_GPIO GPIO_NUM_7       // Mic0 (L) and Mic1 (R)
#define I2S_DOUT_GPIO GPIO_NUM_10     // PCM5102 DIN
#define I2S_MCLK_GPIO GPIO_NUM_9
#define I2S_DMA_DESC_NUM 8
#define I2S_FRAME_POOL_BLOCKS 8
#define I2S_CAPTURE_TASK_STACK 4096
#define I2S_CAPTURE_TASK_PRIORITY 8
#define I2S_CAPTURE_TASK_CORE 0
#define I2S_READ_TIMEOUT_MS 200
#define I2S_STARTUP_SETTLE_MS 20
#define I2S_STARTUP_FLUSH_FRAMES 3
#define INMP441_32_TO_16_SHIFT 16
#define I2S_SPK_GAIN 4             // 4x digital gain for speaker output (PCM5102)

static const char *TAG = "i2s_mic";

static i2s_chan_handle_t s_i2s1_rx;
static i2s_chan_handle_t s_i2s1_tx;
static int32_t *s_i2s1_raw;
static int32_t *s_tx_raw_buf;
static int16_t *s_frame_pool[I2S_FRAME_POOL_BLOCKS];
static QueueHandle_t s_free_queue;
static QueueHandle_t s_filled_queue;
static TaskHandle_t s_capture_task;
static size_t s_samples_per_channel;
static size_t s_i2s_read_bytes;
static size_t s_frame_bytes;
static bool s_initialized;
static volatile bool s_started;
static volatile bool s_tx_enabled;

static void *audio_malloc(size_t size)
{
    void *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ptr == NULL) {
        ptr = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    return ptr;
}

static inline int16_t inmp441_32_to_16(int32_t sample)
{
    // INMP441 puts valid PCM bits in the high part of the 32-bit slot.
    int32_t v = sample >> INMP441_32_TO_16_SHIFT;
    
    if (v > INT16_MAX) v = INT16_MAX;
    if (v < INT16_MIN) v = INT16_MIN;
    
    return (int16_t)v;
}

static i2s_std_config_t make_std_rx_config(gpio_num_t din_gpio, gpio_num_t dout_gpio)
{
    i2s_std_config_t config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_MIC_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK_GPIO,
            .bclk = I2S_BCLK_GPIO,
            .ws = I2S_WS_GPIO,
            .dout = dout_gpio,
            .din = din_gpio,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    config.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;  // 256fs = 12.288MHz @ 48kHz
    config.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    config.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
    return config;
}

static esp_err_t init_i2s_channel(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = I2S_DMA_DESC_NUM;
    /* Send silence on underrun instead of replaying stale PCM. */
    chan_cfg.auto_clear_after_cb = true;
    /* Each 32-bit stereo DMA frame is 8 bytes; stay below the 4092-byte limit. */
    chan_cfg.dma_frame_num = 240;

    esp_err_t ret = i2s_new_channel(&chan_cfg, &s_i2s1_tx, &s_i2s1_rx);
    if (ret != ESP_OK) {
        return ret;
    }

    i2s_std_config_t std_cfg = make_std_rx_config(I2S_DIN_GPIO, I2S_DOUT_GPIO);
    /* The first initialized direction is the full-duplex clock master. */
    ret = i2s_channel_init_std_mode(s_i2s1_tx, &std_cfg);
    if (ret != ESP_OK) {
        return ret;
    }
    return i2s_channel_init_std_mode(s_i2s1_rx, &std_cfg);
}

static void recycle_frame(int16_t *frame)
{
    if (frame != NULL && s_free_queue != NULL) {
        (void)xQueueSend(s_free_queue, &frame, 0);
    }
}

static esp_err_t read_raw_i2s_frame(void)
{
    size_t bytes_read = 0;
    esp_err_t ret = i2s_channel_read(s_i2s1_rx, s_i2s1_raw,
                                    s_i2s_read_bytes, &bytes_read,
                                    I2S_READ_TIMEOUT_MS);
    if (ret != ESP_OK) {
        return ret;
    }
    return bytes_read == s_i2s_read_bytes ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

static esp_err_t flush_startup_frames(void)
{
    for (int i = 0; i < I2S_STARTUP_FLUSH_FRAMES; ++i) {
        esp_err_t ret = read_raw_i2s_frame();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2S startup flush failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    return ESP_OK;
}

static void i2s_capture_task(void *arg)
{
    (void)arg;

    while (s_started) {
        int16_t *frame = NULL;
        if (xQueueReceive(s_free_queue, &frame, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        esp_err_t ret = read_raw_i2s_frame();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "I2S read failed: %s", esp_err_to_name(ret));
            recycle_frame(frame);
            continue;
        }

        for (size_t i = 0; i < s_samples_per_channel * MIC_COUNT; ++i) {
            frame[i] = inmp441_32_to_16(s_i2s1_raw[i]);
        }

        if (xQueueSend(s_filled_queue, &frame, pdMS_TO_TICKS(10)) != pdTRUE) {
            ESP_LOGW(TAG, "mic frame queue full, dropping one frame");
            recycle_frame(frame);
        }
    }

    s_capture_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t i2s_mic_driver_init(size_t samples_per_channel)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (samples_per_channel == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    s_samples_per_channel = samples_per_channel;
    s_i2s_read_bytes = samples_per_channel * 2 * sizeof(int32_t);
    s_frame_bytes = samples_per_channel * MIC_COUNT * sizeof(int16_t);

    s_tx_raw_buf = (int32_t *)audio_malloc(s_i2s_read_bytes);
    s_i2s1_raw = (int32_t *)audio_malloc(s_i2s_read_bytes);
    if (s_i2s1_raw == NULL || s_tx_raw_buf == NULL) {
        i2s_mic_driver_deinit();
        return ESP_ERR_NO_MEM;
    }

    s_free_queue = xQueueCreate(I2S_FRAME_POOL_BLOCKS, sizeof(int16_t *));
    s_filled_queue = xQueueCreate(I2S_FRAME_POOL_BLOCKS, sizeof(int16_t *));
    if (s_free_queue == NULL || s_filled_queue == NULL) {
        i2s_mic_driver_deinit();
        return ESP_ERR_NO_MEM;
    }

    for (size_t i = 0; i < I2S_FRAME_POOL_BLOCKS; ++i) {
        s_frame_pool[i] = (int16_t *)audio_malloc(s_frame_bytes);
        if (s_frame_pool[i] == NULL) {
            i2s_mic_driver_deinit();
            return ESP_ERR_NO_MEM;
        }
        int16_t *frame = s_frame_pool[i];
        (void)xQueueSend(s_free_queue, &frame, 0);
    }

    esp_err_t ret = init_i2s_channel();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "init I2S1 failed: %s", esp_err_to_name(ret));
        i2s_mic_driver_deinit();
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "I2S1 master: %u samples/ch, BCLK=%d WS=%d DIN=%d DOUT=%d",
             (unsigned)s_samples_per_channel, I2S_BCLK_GPIO, I2S_WS_GPIO,
             I2S_DIN_GPIO, I2S_DOUT_GPIO);
    return ESP_OK;
}

esp_err_t i2s_mic_driver_start(void)
{
    if (!s_initialized || s_started) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Keep the TX master running even without USB playback so RX and the DAC
     * see continuous BCLK/WS. Preload silence before starting the DMA ring.
     */
    memset(s_tx_raw_buf, 0, s_i2s_read_bytes);
    size_t loaded = 0;
    esp_err_t ret = i2s_channel_preload_data(s_i2s1_tx, s_tx_raw_buf,
                                            s_i2s_read_bytes, &loaded);
    if (ret != ESP_OK || loaded == 0) {
        return ret != ESP_OK ? ret : ESP_ERR_INVALID_SIZE;
    }
    ret = i2s_channel_enable(s_i2s1_tx);
    if (ret != ESP_OK) {
        return ret;
    }
    s_tx_enabled = true;
    ret = i2s_channel_enable(s_i2s1_rx);
    if (ret != ESP_OK) {
        (void)i2s_channel_disable(s_i2s1_tx);
        s_tx_enabled = false;
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(I2S_STARTUP_SETTLE_MS));
    ret = flush_startup_frames();
    if (ret != ESP_OK) {
        (void)i2s_channel_disable(s_i2s1_rx);
        (void)i2s_channel_disable(s_i2s1_tx);
        s_tx_enabled = false;
        return ret;
    }

    s_started = true;
    BaseType_t ok = xTaskCreatePinnedToCore(i2s_capture_task,
                                            "i2s_capture",
                                            I2S_CAPTURE_TASK_STACK,
                                            NULL,
                                            I2S_CAPTURE_TASK_PRIORITY,
                                            &s_capture_task,
                                            I2S_CAPTURE_TASK_CORE);
    if (ok != pdPASS) {
        s_started = false;
        (void)i2s_channel_disable(s_i2s1_rx);
        (void)i2s_channel_disable(s_i2s1_tx);
        s_tx_enabled = false;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "I2S capture started");
    return ESP_OK;
}

esp_err_t i2s_mic_driver_read(int16_t *out_interleaved,
                              size_t samples_per_channel,
                              TickType_t ticks_to_wait)
{
    if (!s_started || s_filled_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (out_interleaved == NULL || samples_per_channel != s_samples_per_channel) {
        return ESP_ERR_INVALID_ARG;
    }

    int16_t *frame = NULL;
    if (xQueueReceive(s_filled_queue, &frame, ticks_to_wait) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    memcpy(out_interleaved, frame, s_frame_bytes);
    recycle_frame(frame);
    return ESP_OK;
}

size_t i2s_mic_driver_get_samples_per_channel(void)
{
    return s_samples_per_channel;
}

esp_err_t i2s_mic_driver_deinit(void)
{
    s_started = false;

    if (s_capture_task != NULL) {
        vTaskDelete(s_capture_task);
        s_capture_task = NULL;
    }

    if (s_i2s1_rx != NULL) {
        (void)i2s_channel_disable(s_i2s1_rx);
        (void)i2s_del_channel(s_i2s1_rx);
        s_i2s1_rx = NULL;
    }
    if (s_i2s1_tx != NULL) {
        if (s_tx_enabled) {
            (void)i2s_channel_disable(s_i2s1_tx);
            s_tx_enabled = false;
        }
        (void)i2s_del_channel(s_i2s1_tx);
        s_i2s1_tx = NULL;
    }

    if (s_free_queue != NULL) {
        vQueueDelete(s_free_queue);
        s_free_queue = NULL;
    }
    if (s_filled_queue != NULL) {
        vQueueDelete(s_filled_queue);
        s_filled_queue = NULL;
    }

    free(s_tx_raw_buf);
    free(s_i2s1_raw);
    s_tx_raw_buf = NULL;
    s_i2s1_raw = NULL;

    for (size_t i = 0; i < I2S_FRAME_POOL_BLOCKS; ++i) {
        free(s_frame_pool[i]);
        s_frame_pool[i] = NULL;
    }

    s_samples_per_channel = 0;
    s_i2s_read_bytes = 0;
    s_frame_bytes = 0;
    s_initialized = false;
    return ESP_OK;
}

esp_err_t i2s_mic_driver_write_tx(const int16_t *pcm_stereo, size_t num_samples)
{
    if (!s_started || s_i2s1_tx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (pcm_stereo == NULL || num_samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_tx_enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    if (num_samples > s_samples_per_channel) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Convert 16-bit PCM to 32-bit I2S slot: apply 4x gain, shift left 16 bits.
    // PCM5102 is 24-bit DAC; 16-bit data in high 16 bits of 32-bit slot is correct:
    //   0x7FFF << 16 = 0x7FFF0000, top 24 bits = 0x7FFF00, ~100% of full scale 0x7FFFFF
    for (size_t i = 0; i < num_samples * 2; ++i) {
        int32_t sample = (int32_t)pcm_stereo[i] * I2S_SPK_GAIN;
        if (sample > INT16_MAX) sample = INT16_MAX;
        if (sample < INT16_MIN) sample = INT16_MIN;
        s_tx_raw_buf[i] = sample * 65536;
    }

    size_t bytes_written = 0;
    esp_err_t ret = i2s_channel_write(s_i2s1_tx, s_tx_raw_buf,
                                      num_samples * 2 * sizeof(int32_t),
                                      &bytes_written, portMAX_DELAY);

    if (ret == ESP_OK && bytes_written != num_samples * 2 * sizeof(int32_t)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ret;
}
