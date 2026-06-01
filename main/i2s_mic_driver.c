#include "i2s_mic_driver.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "soc/i2s_periph.h"

#define I2S_BCLK_GPIO GPIO_NUM_4
#define I2S_WS_GPIO GPIO_NUM_5
#define I2S0_DIN_GPIO GPIO_NUM_6
#define I2S1_DIN_GPIO GPIO_NUM_7

//#define I2S_DMA_DESC_NUM 8
#define I2S_DMA_DESC_NUM 8
//#define I2S_FRAME_POOL_BLOCKS 4
#define I2S_FRAME_POOL_BLOCKS 8
#define I2S_CAPTURE_TASK_STACK 4096
#define I2S_CAPTURE_TASK_PRIORITY 8
#define I2S_CAPTURE_TASK_CORE 0
#define I2S_READ_TIMEOUT_MS 200
#define I2S_SECOND_PORT_READ_TIMEOUT_MS 2
#define I2S_STARTUP_SETTLE_MS 20
#define I2S_STARTUP_FLUSH_FRAMES 3
#define I2S_CAPTURE_LOG_FRAMES 4
#define I2S_USE_SECOND_PORT (MIC_COUNT > 2)
#define INMP441_32_TO_16_SHIFT 16

static const char *TAG = "i2s_mic";

static i2s_chan_handle_t s_i2s0_rx;
static i2s_chan_handle_t s_i2s1_rx;
static int32_t *s_i2s0_raw;
static int32_t *s_i2s1_raw;
static int16_t *s_frame_pool[I2S_FRAME_POOL_BLOCKS];
static QueueHandle_t s_free_queue;
static QueueHandle_t s_filled_queue;
static TaskHandle_t s_capture_task;
static size_t s_samples_per_channel;
static size_t s_i2s_read_bytes;
static size_t s_frame_bytes;
static uint32_t s_captured_frames;
static bool s_initialized;
static volatile bool s_started;

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

static i2s_std_config_t make_std_rx_config(gpio_num_t din_gpio)
{
    i2s_std_config_t config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_MIC_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_GPIO,
            .ws = I2S_WS_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din = din_gpio,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    config.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    config.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
    return config;
}

static esp_err_t init_i2s_channel(i2s_port_t port,
                                  gpio_num_t din_gpio,
                                  i2s_chan_handle_t *rx_handle)
{
    // 判断如果是 I2S0 就是 Master，如果是 I2S1 就是 Slave
    i2s_role_t role = (port == I2S_NUM_0) ? I2S_ROLE_MASTER : I2S_ROLE_SLAVE;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(port, role);
    
    
    chan_cfg.dma_desc_num = I2S_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = s_samples_per_channel;

    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, rx_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    i2s_std_config_t std_cfg = make_std_rx_config(din_gpio);
    ret = i2s_channel_init_std_mode(*rx_handle, &std_cfg);
    if (ret != ESP_OK) {
        i2s_del_channel(*rx_handle);
        *rx_handle = NULL;
    }
    return ret;
}

static void recycle_frame(int16_t *frame)
{
    if (frame != NULL && s_free_queue != NULL) {
        (void)xQueueSend(s_free_queue, &frame, 0);
    }
}

static void bridge_i2s_shared_clock(void)
{
#if I2S_USE_SECOND_PORT
    gpio_set_direction(I2S_BCLK_GPIO, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(I2S_WS_GPIO, GPIO_MODE_INPUT_OUTPUT);

    esp_rom_gpio_connect_out_signal(I2S_BCLK_GPIO,
                                    i2s_periph_signal[I2S_NUM_0].m_rx_bck_sig,
                                    false,
                                    false);
    esp_rom_gpio_connect_in_signal(I2S_BCLK_GPIO,
                                   i2s_periph_signal[I2S_NUM_1].s_rx_bck_sig,
                                   false);
    esp_rom_gpio_connect_out_signal(I2S_WS_GPIO,
                                    i2s_periph_signal[I2S_NUM_0].m_rx_ws_sig,
                                    false,
                                    false);
    esp_rom_gpio_connect_in_signal(I2S_WS_GPIO,
                                   i2s_periph_signal[I2S_NUM_1].s_rx_ws_sig,
                                   false);
#endif
}

static esp_err_t read_raw_i2s_frame(TickType_t ticks_to_wait,
                                    esp_err_t *ret0_out,
                                    esp_err_t *ret1_out,
                                    size_t *bytes_read0_out,
                                    size_t *bytes_read1_out)
{
/*     size_t bytes_read0 = 0;
    size_t bytes_read1 = 0;
    esp_err_t ret0 = i2s_channel_read(s_i2s0_rx,
                                      s_i2s0_raw,
                                      s_i2s_read_bytes,
                                      &bytes_read0,
                                      ticks_to_wait);
#if I2S_USE_SECOND_PORT
    TickType_t i2s1_wait = ticks_to_wait;
    if (ticks_to_wait != portMAX_DELAY) {
        i2s1_wait = pdMS_TO_TICKS(I2S_SECOND_PORT_READ_TIMEOUT_MS);
    }
    esp_err_t ret1 = i2s_channel_read(s_i2s1_rx,
                                      s_i2s1_raw,
                                      s_i2s_read_bytes,
                                      &bytes_read1,
                                      i2s1_wait);
#else**/

    size_t bytes_read0 = 0;
    size_t bytes_read1 = 0;
    esp_err_t ret0 = i2s_channel_read(s_i2s0_rx,
                                      s_i2s0_raw,
                                      s_i2s_read_bytes,
                                      &bytes_read0,
                                      ticks_to_wait);
#if I2S_USE_SECOND_PORT
    // 直接使用相同的 ticks_to_wait，确保不会因为轻微错位而被掐断
    esp_err_t ret1 = i2s_channel_read(s_i2s1_rx,
                                      s_i2s1_raw,
                                      s_i2s_read_bytes,
                                      &bytes_read1,
                                      ticks_to_wait);
#else
    esp_err_t ret1 = ESP_OK;
#endif

    if (ret0_out != NULL) {
        *ret0_out = ret0;
    }
    if (ret1_out != NULL) {
        *ret1_out = ret1;
    }
    if (bytes_read0_out != NULL) {
        *bytes_read0_out = bytes_read0;
    }
    if (bytes_read1_out != NULL) {
        *bytes_read1_out = bytes_read1;
    }

    if (ret0 != ESP_OK) {
        return ret0;
    }
    if (bytes_read0 != s_i2s_read_bytes) {
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

static esp_err_t flush_startup_frames(void)
{
    for (int i = 0; i < I2S_STARTUP_FLUSH_FRAMES; ++i) {
        esp_err_t ret0 = ESP_OK;
        esp_err_t ret1 = ESP_OK;
        size_t bytes_read0 = 0;
        size_t bytes_read1 = 0;
        esp_err_t ret = read_raw_i2s_frame(pdMS_TO_TICKS(I2S_READ_TIMEOUT_MS),
                                           &ret0,
                                           &ret1,
                                           &bytes_read0,
                                           &bytes_read1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG,
                     "I2S startup flush failed: ret0=%s ret1=%s bytes0=%u bytes1=%u",
                     esp_err_to_name(ret0),
                     esp_err_to_name(ret1),
                     (unsigned)bytes_read0,
                     (unsigned)bytes_read1);
            return ret;
        }
#if I2S_USE_SECOND_PORT
        if (ret1 != ESP_OK || bytes_read1 != s_i2s_read_bytes) {
            ESP_LOGW(TAG,
                     "I2S1 startup flush skipped: ret1=%s bytes1=%u",
                     esp_err_to_name(ret1),
                     (unsigned)bytes_read1);
        }
#endif
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

        esp_err_t ret0 = ESP_OK;
        esp_err_t ret1 = ESP_OK;
        size_t bytes_read0 = 0;
        size_t bytes_read1 = 0;
        esp_err_t ret = read_raw_i2s_frame(pdMS_TO_TICKS(I2S_READ_TIMEOUT_MS),
                                           &ret0,
                                           &ret1,
                                           &bytes_read0,
                                           &bytes_read1);

        if (ret != ESP_OK) {
            ESP_LOGW(TAG,
                     "I2S read failed: ret0=%s ret1=%s bytes0=%u bytes1=%u",
                     esp_err_to_name(ret0),
                     esp_err_to_name(ret1),
                     (unsigned)bytes_read0,
                     (unsigned)bytes_read1);
            recycle_frame(frame);
            continue;
        }

#if I2S_USE_SECOND_PORT
        const bool second_port_valid = (ret1 == ESP_OK && bytes_read1 == s_i2s_read_bytes);
        if (!second_port_valid && s_captured_frames < I2S_CAPTURE_LOG_FRAMES) {
            ESP_LOGW(TAG,
                     "I2S1 frame unavailable, keeping UAC audio alive: ret1=%s bytes1=%u",
                     esp_err_to_name(ret1),
                     (unsigned)bytes_read1);
        }
#endif

        for (size_t i = 0; i < s_samples_per_channel; ++i) {
            const size_t stereo_idx = i * 2;
            const size_t out_idx = i * MIC_COUNT;

            // I2S1 carries Mic0 (L) and Mic1 (R).
            // I2S0 carries Mic2 (L) and Mic3 (R).
            frame[out_idx + 0] = second_port_valid ? inmp441_32_to_16(s_i2s1_raw[stereo_idx + 0]) : 0;
            frame[out_idx + 1] = second_port_valid ? inmp441_32_to_16(s_i2s1_raw[stereo_idx + 1]) : 0;
            frame[out_idx + 2] = inmp441_32_to_16(s_i2s0_raw[stereo_idx + 0]);
            frame[out_idx + 3] = inmp441_32_to_16(s_i2s0_raw[stereo_idx + 1]);
        }

        if (s_captured_frames < I2S_CAPTURE_LOG_FRAMES) {
            int16_t min_v[4] = {INT16_MAX, INT16_MAX, INT16_MAX, INT16_MAX};
            int16_t max_v[4] = {INT16_MIN, INT16_MIN, INT16_MIN, INT16_MIN};
            for (size_t j = 0; j < s_samples_per_channel; ++j) {
                for (size_t ch = 0; ch < MIC_COUNT; ++ch) {
                    int16_t v = frame[j * MIC_COUNT + ch];
                    if (v < min_v[ch]) {
                        min_v[ch] = v;
                    }
                    if (v > max_v[ch]) {
                        max_v[ch] = v;
                    }
                }
            }
            ESP_LOGI(TAG,
#if I2S_USE_SECOND_PORT
                     "captured frame %u: mic0=%d[%d,%d] mic1=%d[%d,%d] mic2=%d[%d,%d] mic3=%d[%d,%d] raw=%08x/%08x/%08x/%08x",
                     (unsigned)(s_captured_frames + 1),
                     frame[0],
                     min_v[0],
                     max_v[0],
                     frame[1],
                     min_v[1],
                     max_v[1],
                     frame[2],
                     min_v[2],
                     max_v[2],
                     frame[3],
                     min_v[3],
                     max_v[3],
                     (unsigned int)(uint32_t)s_i2s0_raw[0],
                     (unsigned int)(uint32_t)s_i2s0_raw[1],
                     second_port_valid ? (unsigned int)(uint32_t)s_i2s1_raw[0] : 0U,
                     second_port_valid ? (unsigned int)(uint32_t)s_i2s1_raw[1] : 0U
#else
                     "captured frame %u: mic2=%d[%d,%d] mic3=%d[%d,%d] raw=%08x/%08x",
                     (unsigned)(s_captured_frames + 1),
                     frame[2],
                     min_v[2],
                     max_v[2],
                     frame[3],
                     min_v[3],
                     max_v[3],
                     (unsigned int)(uint32_t)s_i2s0_raw[0],
                     (unsigned int)(uint32_t)s_i2s0_raw[1]
#endif
                     );
        }
        s_captured_frames++;

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

    s_i2s0_raw = (int32_t *)audio_malloc(s_i2s_read_bytes);
#if I2S_USE_SECOND_PORT
    s_i2s1_raw = (int32_t *)audio_malloc(s_i2s_read_bytes);
#endif
    if (s_i2s0_raw == NULL
#if I2S_USE_SECOND_PORT
        || s_i2s1_raw == NULL
#endif
    ) {
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

    esp_err_t ret = ESP_OK;
#if I2S_USE_SECOND_PORT
    // 1. 【先】初始化 Slave (I2S1)。
    // 此时底层驱动会把 GPIO 4 和 5 乖乖地配置为“时钟输入”
    ret = init_i2s_channel(I2S_NUM_1, I2S1_DIN_GPIO, &s_i2s1_rx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "init I2S1 failed: %s", esp_err_to_name(ret));
        i2s_mic_driver_deinit();
        return ret;
    }
#endif

    // 2. 【后】初始化 Master (I2S0)。
    // 此时底层驱动会重新接管 GPIO 4 和 5，将其配置为“时钟输出”。
    // 妙就妙在，ESP32 的矩阵路由不会切断刚才 Slave 连好的“输入”线路！
    // 这样一进一出，完美搭桥！
    ret = init_i2s_channel(I2S_NUM_0, I2S0_DIN_GPIO, &s_i2s0_rx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "init I2S0 failed: %s", esp_err_to_name(ret));
        i2s_mic_driver_deinit();
        return ret;
    }

    /*
     * I2S1 slave 需要同一组 GPIO 的输入路径，I2S0 master 需要输出路径。
     * 显式设置 INPUT_OUTPUT，避免后续 GPIO 配置覆盖导致 Mic3 时钟输入丢失。
     */
    bridge_i2s_shared_clock();

    s_initialized = true;
    ESP_LOGI(TAG,
             "I2S mic driver ready: %u samples/ch, %u bytes/frame",
             (unsigned)s_samples_per_channel,
             (unsigned)s_frame_bytes);
#if I2S_USE_SECOND_PORT
    ESP_LOGI(TAG, "I2S0+I2S1 mode: BCLK=%d WS=%d DIN0=%d DIN1=%d",
             I2S_BCLK_GPIO,
             I2S_WS_GPIO,
             I2S0_DIN_GPIO,
             I2S1_DIN_GPIO);
#else
    ESP_LOGI(TAG, "I2S0 stereo test mode: BCLK=%d WS=%d DIN=%d, I2S1 disabled",
             I2S_BCLK_GPIO,
             I2S_WS_GPIO,
             I2S0_DIN_GPIO);
#endif
    return ESP_OK;
}

esp_err_t i2s_mic_driver_start(void)
{
    if (!s_initialized || s_started) {
        return ESP_ERR_INVALID_STATE;
    }

    bridge_i2s_shared_clock();

#if I2S_USE_SECOND_PORT
    // 【关键修复】：必须先开启 Slave，让它在起跑线上乖乖等待时钟
    esp_err_t ret = i2s_channel_enable(s_i2s1_rx);
    if (ret != ESP_OK) {
        return ret;
    }
#else
    esp_err_t ret = ESP_OK;
#endif

    // 然后再开启 Master，时钟一出，两个通道瞬间完美同步对齐！
    ret = i2s_channel_enable(s_i2s0_rx);
    if (ret != ESP_OK) {
#if I2S_USE_SECOND_PORT
        (void)i2s_channel_disable(s_i2s1_rx);
#endif
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(I2S_STARTUP_SETTLE_MS));
    ret = flush_startup_frames();
    if (ret != ESP_OK) {
        (void)i2s_channel_disable(s_i2s0_rx);
#if I2S_USE_SECOND_PORT
        (void)i2s_channel_disable(s_i2s1_rx);
#endif
        return ret;
    }

    s_captured_frames = 0;
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
        (void)i2s_channel_disable(s_i2s0_rx);
#if I2S_USE_SECOND_PORT
        (void)i2s_channel_disable(s_i2s1_rx);
#endif
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

    if (s_i2s0_rx != NULL) {
        (void)i2s_channel_disable(s_i2s0_rx);
        (void)i2s_del_channel(s_i2s0_rx);
        s_i2s0_rx = NULL;
    }
    if (s_i2s1_rx != NULL) {
        (void)i2s_channel_disable(s_i2s1_rx);
        (void)i2s_del_channel(s_i2s1_rx);
        s_i2s1_rx = NULL;
    }

    if (s_free_queue != NULL) {
        vQueueDelete(s_free_queue);
        s_free_queue = NULL;
    }
    if (s_filled_queue != NULL) {
        vQueueDelete(s_filled_queue);
        s_filled_queue = NULL;
    }

    free(s_i2s0_raw);
    free(s_i2s1_raw);
    s_i2s0_raw = NULL;
    s_i2s1_raw = NULL;

    for (size_t i = 0; i < I2S_FRAME_POOL_BLOCKS; ++i) {
        free(s_frame_pool[i]);
        s_frame_pool[i] = NULL;
    }

    s_samples_per_channel = 0;
    s_i2s_read_bytes = 0;
    s_frame_bytes = 0;
    s_captured_frames = 0;
    s_initialized = false;
    return ESP_OK;
}
