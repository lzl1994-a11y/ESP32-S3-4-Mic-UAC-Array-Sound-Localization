#include "usb_composite.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_default_config.h"
#include "tusb.h"
#include "walle_audio_desc.h"
#include "i2s_mic_driver.h"

#define USB_VID 0xCAFE
#define USB_PID 0x4031
#define USB_BCD 0x0100

#define EPNUM_AUDIO_IN 0x81
#define EPNUM_AUDIO_OUT 0x01
#define EPNUM_CDC_NOTIF 0x82
#define EPNUM_CDC_OUT 0x03
#define EPNUM_CDC_IN 0x83
#define CDC_NOTIF_EP_SIZE 8


// 1. 接口分配：必须把 CDC 提到最前面！（这是修复 Windows 识别的核心）
enum {
    ITF_NUM_CDC = 0,            // 串口控制排第 0 号
    ITF_NUM_CDC_DATA,           // 串口数据排第 1 号
    ITF_NUM_AUDIO_CONTROL,      // 音频控制退到第 2 号
    ITF_NUM_AUDIO_STREAMING_SPK,// 扬声器音频数据排第 3 号
    ITF_NUM_AUDIO_STREAMING_MIC,// 麦克风音频数据排第 4 号
    ITF_NUM_TOTAL,
};

// 2. 字符串 ID：保持原样（只是供系统查字典用的，顺序无所谓）
enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_AUDIO,
    STRID_CDC,
};

// 3. 音频拓扑图：保持原样（这是 UAC 麦克风标准的内部连线，从 1 开始）
enum {
    UAC_ENTITY_INPUT_TERMINAL = 1,
    UAC_ENTITY_FEATURE_UNIT = 2,
    UAC_ENTITY_OUTPUT_TERMINAL = 3,
    UAC_ENTITY_CLOCK = 4,
};

#define CONFIG_TOTAL_LEN \
    (TUD_CONFIG_DESC_LEN + WALLE_AUDIO_HEADSET_DESC_LEN + TUD_CDC_DESC_LEN)

static const char *TAG = "usb_composite";

static bool s_initialized;
static volatile bool s_uac_spk_streaming;
static volatile bool s_uac_mic_streaming;
static bool s_mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];
static int16_t s_volume_db[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];
static uint32_t s_sample_freq = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
static uint8_t s_clock_valid = 1;
static audio_control_range_4_n_t(1) s_sample_freq_range;
static audio_control_range_2_n_t(1) s_volume_range;

static const tusb_desc_device_t s_device_desc = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = USB_BCD,
    .iManufacturer = STRID_MANUFACTURER,
    .iProduct = STRID_PRODUCT,
    .iSerialNumber = STRID_SERIAL,
    .bNumConfigurations = 1,
};

static const uint8_t s_configuration_desc[] = {
    // 头部：告诉系统总共有几个接口
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // 第一块：先装载 CDC 串口描述符
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC,
                       STRID_CDC,
                       EPNUM_CDC_NOTIF,
                       CDC_NOTIF_EP_SIZE,
                       EPNUM_CDC_OUT,
                       EPNUM_CDC_IN,
                       CFG_TUD_CDC_EP_BUFSIZE),

    // 第二块：再装载 Audio Headset 描述符
    WALLE_AUDIO_HEADSET_DESCRIPTOR(ITF_NUM_AUDIO_CONTROL,
                                   STRID_AUDIO,
                                   CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX,
                                   CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX * 8,
                                   CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX,
                                   CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX * 8,
                                   EPNUM_AUDIO_OUT,
                                   CFG_TUD_AUDIO_EP_SZ_OUT,
                                   EPNUM_AUDIO_IN,
                                   CFG_TUD_AUDIO_EP_SZ_IN),
};

static const char *s_string_desc[] = {
    (const char[]){0x09, 0x04},
    "Walle",
    "Walle Ear S3",
    "000001",
    "Walle UAC Mic",
    "Walle DOA CDC",
};

static void init_audio_control_state(void)
{
    memset(s_mute, 0, sizeof(s_mute));
    memset(s_volume_db, 0, sizeof(s_volume_db));

    s_sample_freq = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
    s_clock_valid = 1;

    s_sample_freq_range.wNumSubRanges = 1;
    s_sample_freq_range.subrange[0].bMin = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
    s_sample_freq_range.subrange[0].bMax = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
    s_sample_freq_range.subrange[0].bRes = 0;

    s_volume_range.wNumSubRanges = 1;
    s_volume_range.subrange[0].bMin = -60;
    s_volume_range.subrange[0].bMax = 6;
    s_volume_range.subrange[0].bRes = 1;
}

static void tinyusb_event_callback(tinyusb_event_t *event, void *arg)
{
    (void)arg;

    if (event == NULL) {
        return;
    }

    if (event->id == TINYUSB_EVENT_ATTACHED || event->id == TINYUSB_EVENT_DETACHED) {
        s_uac_spk_streaming = false;
        s_uac_mic_streaming = false;
    }
}

static void audio_spk_task(void *arg)
{
    (void)arg;
    int16_t spk_buf[CFG_TUD_AUDIO_EP_SZ_OUT / 2]; // Max packet size

    while (1) {
        if (tud_mounted() && tud_audio_mounted() && s_uac_spk_streaming) {
            uint16_t available = tud_audio_available();
            
            // 关键修复：必须保证每次读取的字节数是 4 的整数倍（16-bit 立体声 = 4 bytes）
            // 否则如果有残留字节，会导致下一次读取时左右声道错位，发出极其刺耳的杂音！
            uint16_t to_read = available - (available % 4);
            uint16_t max_read = sizeof(spk_buf) - (sizeof(spk_buf) % 4);
            
            if (to_read > max_read) {
                to_read = max_read;
            }

            if (to_read > 0) {
                uint16_t bytes_read = tud_audio_read(spk_buf, to_read);
                if (bytes_read > 0) {
                    static int play_count = 0;
                    if (++play_count % 500 == 0) {
                        ESP_LOGI(TAG, "Speaker receiving data from PC... bytes: %d, sample: %d", bytes_read, spk_buf[0]);
                    }
                    // X3 Pi outputs 16kHz Stereo 16-bit PCM (2 channels)
                    // The I2S TX expects stereo frames, so num_samples = bytes_read / 4 (2 bytes * 2 channels)
                    i2s_mic_driver_write_tx(spk_buf, bytes_read / 4);
                }
            } else {
                vTaskDelay(1); // Wait for more data
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

#define CDC_CMD_BUF_SIZE 64

static char s_cdc_cmd_buf[CDC_CMD_BUF_SIZE];
static size_t s_cdc_cmd_len;

static void cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    if (event == NULL || event->type != CDC_EVENT_RX) {
        return;
    }

    size_t room = CDC_CMD_BUF_SIZE - s_cdc_cmd_len - 1;
    if (room == 0) {
        s_cdc_cmd_len = 0; /* overflow, reset */
    }
    size_t n = tud_cdc_n_read(itf,
                              (uint8_t *)&s_cdc_cmd_buf[s_cdc_cmd_len],
                              (uint32_t)room);
    s_cdc_cmd_len += n;
    s_cdc_cmd_buf[s_cdc_cmd_len] = '\0';

    if (strstr(s_cdc_cmd_buf, "getname:WHO_ARE_YOU") != NULL) {
        tud_cdc_n_write_str(itf, "IAM:ESP_MIC\r\n");
        tud_cdc_n_write_flush(itf);
        s_cdc_cmd_len = 0;
        s_cdc_cmd_buf[0] = '\0';
    } else if (s_cdc_cmd_len >= CDC_CMD_BUF_SIZE - 1) {
        s_cdc_cmd_len = 0;
        s_cdc_cmd_buf[0] = '\0';
    }
}

static void cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    if (event == NULL || event->type != CDC_EVENT_LINE_STATE_CHANGED) {
        return;
    }

    ESP_LOGI(TAG,
             "CDC line state: itf=%d dtr=%d rts=%d",
             itf,
             event->line_state_changed_data.dtr ? 1 : 0,
             event->line_state_changed_data.rts ? 1 : 0);
}

esp_err_t usb_composite_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    init_audio_control_state();

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG(tinyusb_event_callback);

    /*
     * Move TinyUSB task to Core 0 and raise priority to 6.
     * - Core 0 runs i2s_capture (prio 8) — TinyUSB at prio 6 won't starve it.
     * - Core 1 runs audio_capture (prio 9) — moving TinyUSB off Core 1
     *   means it no longer competes with the heavy TDOA + UAC write path.
     * - prio 6 > default 5 gives it an edge over cdc_report (5) and Tmr Svc (1).
     */
    tusb_cfg.task = TINYUSB_TASK_CUSTOM(4096, 6, 0);

    tusb_cfg.descriptor.device = &s_device_desc;
    tusb_cfg.descriptor.string = s_string_desc;
    tusb_cfg.descriptor.string_count = sizeof(s_string_desc) / sizeof(s_string_desc[0]);
    tusb_cfg.descriptor.full_speed_config = s_configuration_desc;
#if (TUD_OPT_HIGH_SPEED)
    tusb_cfg.descriptor.high_speed_config = s_configuration_desc;
#endif

    ESP_RETURN_ON_ERROR(tinyusb_driver_install(&tusb_cfg), TAG, "install TinyUSB failed");

    const tinyusb_config_cdcacm_t cdc_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = cdc_line_state_changed_callback,
        .callback_line_coding_changed = NULL,
    };
    ESP_RETURN_ON_ERROR(tinyusb_cdcacm_init(&cdc_cfg), TAG, "init CDC ACM failed");

    // Start Audio Speaker Task
    xTaskCreatePinnedToCore(audio_spk_task, "audio_spk_task", 4096, NULL, 5, NULL, 0);

    s_initialized = true;
    ESP_LOGI(TAG, "TinyUSB UAC + CDC composite device initialized");
    return ESP_OK;
}

bool usb_composite_is_mounted(void)
{
    return tud_mounted();
}

bool usb_composite_uac_is_streaming(void)
{
    return tud_mounted() && tud_audio_mounted() && s_uac_mic_streaming;
}

bool usb_composite_cdc_is_connected(void)
{
    return tud_cdc_n_connected(0);
}

uint8_t usb_composite_cdc_get_line_state(void)
{
    return tud_cdc_n_get_line_state(0);
}

uint32_t usb_composite_cdc_write_available(void)
{
    return tud_cdc_n_write_available(0);
}

esp_err_t usb_composite_uac_write(const int16_t *pcm, size_t byte_len, TickType_t ticks_to_wait)
{
    if (pcm == NULL || byte_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!usb_composite_uac_is_streaming()) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint8_t *cursor = (const uint8_t *)pcm;
    size_t written = 0;
    TickType_t start = xTaskGetTickCount();

    while (written < byte_len) {
        if (!usb_composite_uac_is_streaming()) {
            return ESP_ERR_INVALID_STATE;
        }

        size_t chunk_len = byte_len - written;
        if (chunk_len > UINT16_MAX) {
            chunk_len = UINT16_MAX;
        }

        uint16_t n = tud_audio_write(cursor + written, (uint16_t)chunk_len);
        if (n > 0) {
            written += n;
            continue;
        }

        if (ticks_to_wait == 0 ||
            (ticks_to_wait != portMAX_DELAY && xTaskGetTickCount() - start >= ticks_to_wait)) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return ESP_OK;
}

esp_err_t usb_composite_cdc_write_str(const char *text, TickType_t ticks_to_wait)
{
    if (text == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!tud_mounted()) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t len = strlen(text);
    size_t written = 0;
    TickType_t start = xTaskGetTickCount();

    while (written < len) {
        uint32_t available = tud_cdc_n_write_available(0);
        if (available > 0) {
            size_t chunk = len - written;
            if (chunk > available) {
                chunk = available;
            }
            written += tud_cdc_n_write(0, text + written, (uint32_t)chunk);
            (void)tud_cdc_n_write_flush(0);
            continue;
        }

        if (ticks_to_wait == 0 ||
            (ticks_to_wait != portMAX_DELAY && xTaskGetTickCount() - start >= ticks_to_wait)) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    (void)tud_cdc_n_write_flush(0);
    return ESP_OK;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *request)
{
    (void)rhport;

    uint8_t itf = TU_U16_LOW(request->wIndex);
    uint8_t alt = TU_U16_LOW(request->wValue);

    if (itf == ITF_NUM_AUDIO_STREAMING_SPK) {
        s_uac_spk_streaming = alt != 0;
        if (!s_uac_spk_streaming) {
            (void)tud_audio_clear_ep_out_ff();
        }
    } else if (itf == ITF_NUM_AUDIO_STREAMING_MIC) {
        s_uac_mic_streaming = alt != 0;
        if (!s_uac_mic_streaming) {
            (void)tud_audio_clear_ep_in_ff();
        }
    }

    return true;
}

bool tud_audio_set_req_ep_cb(uint8_t rhport,
                             tusb_control_request_t const *request,
                             uint8_t *buffer)
{
    (void)rhport;
    (void)request;
    (void)buffer;
    return false;
}

bool tud_audio_set_req_itf_cb(uint8_t rhport,
                              tusb_control_request_t const *request,
                              uint8_t *buffer)
{
    (void)rhport;
    (void)request;
    (void)buffer;
    return false;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport,
                                 tusb_control_request_t const *request,
                                 uint8_t *buffer)
{
    (void)rhport;

    uint8_t channel = TU_U16_LOW(request->wValue);
    uint8_t control = TU_U16_HIGH(request->wValue);
    uint8_t entity = TU_U16_HIGH(request->wIndex);

    if (request->bRequest != AUDIO_CS_REQ_CUR || entity != UAC_ENTITY_FEATURE_UNIT) {
        return false;
    }

    if (channel >= (CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1)) {
        return false;
    }

    switch (control) {
    case AUDIO_FU_CTRL_MUTE:
        if (request->wLength == sizeof(audio_control_cur_1_t)) {
            s_mute[channel] = ((audio_control_cur_1_t *)buffer)->bCur != 0;
            return true;
        }
        break;

    case AUDIO_FU_CTRL_VOLUME:
        if (request->wLength == sizeof(audio_control_cur_2_t)) {
            s_volume_db[channel] = ((audio_control_cur_2_t *)buffer)->bCur;
            return true;
        }
        break;

    default:
        break;
    }

    return false;
}

bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *request)
{
    (void)rhport;
    (void)request;
    return false;
}

bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const *request)
{
    (void)rhport;
    (void)request;
    return false;
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *request)
{
    uint8_t channel = TU_U16_LOW(request->wValue);
    uint8_t control = TU_U16_HIGH(request->wValue);
    uint8_t entity = TU_U16_HIGH(request->wIndex);

    if (channel >= (CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1)) {
        return false;
    }

    switch (entity) {
    case UAC_ENTITY_INPUT_TERMINAL:
        if (control == AUDIO_TE_CTRL_CONNECTOR &&
            request->bRequest == AUDIO_CS_REQ_CUR) {
            audio_desc_channel_cluster_t cluster = {
                .bNrChannels = CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX,
                .bmChannelConfig = (audio_channel_config_t)0,
                .iChannelNames = 0,
            };
            return tud_audio_buffer_and_schedule_control_xfer(rhport,
                                                              request,
                                                              &cluster,
                                                              sizeof(cluster));
        }
        break;

    case 5: // UAC_ENTITY_MIC_INPUT_TERMINAL
        if (control == AUDIO_TE_CTRL_CONNECTOR &&
            request->bRequest == AUDIO_CS_REQ_CUR) {
            audio_desc_channel_cluster_t cluster = {
                .bNrChannels = CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX,
                .bmChannelConfig = (audio_channel_config_t)0,
                .iChannelNames = 0,
            };
            return tud_audio_buffer_and_schedule_control_xfer(rhport,
                                                              request,
                                                              &cluster,
                                                              sizeof(cluster));
        }
        break;

    case UAC_ENTITY_FEATURE_UNIT:
        if (control == AUDIO_FU_CTRL_MUTE &&
            request->bRequest == AUDIO_CS_REQ_CUR) {
            return tud_audio_buffer_and_schedule_control_xfer(rhport,
                                                              request,
                                                              &s_mute[channel],
                                                              1);
        }

        if (control == AUDIO_FU_CTRL_VOLUME) {
            if (request->bRequest == AUDIO_CS_REQ_CUR) {
                return tud_audio_buffer_and_schedule_control_xfer(rhport,
                                                                  request,
                                                                  &s_volume_db[channel],
                                                                  sizeof(s_volume_db[channel]));
            }
            if (request->bRequest == AUDIO_CS_REQ_RANGE) {
                return tud_audio_buffer_and_schedule_control_xfer(rhport,
                                                                  request,
                                                                  &s_volume_range,
                                                                  sizeof(s_volume_range));
            }
        }
        break;

    case UAC_ENTITY_CLOCK:
        if (control == AUDIO_CS_CTRL_SAM_FREQ) {
            if (request->bRequest == AUDIO_CS_REQ_CUR) {
                return tud_audio_buffer_and_schedule_control_xfer(rhport,
                                                                  request,
                                                                  &s_sample_freq,
                                                                  sizeof(s_sample_freq));
            }
            if (request->bRequest == AUDIO_CS_REQ_RANGE) {
                return tud_audio_buffer_and_schedule_control_xfer(rhport,
                                                                  request,
                                                                  &s_sample_freq_range,
                                                                  sizeof(s_sample_freq_range));
            }
        }

        if (control == AUDIO_CS_CTRL_CLK_VALID &&
            request->bRequest == AUDIO_CS_REQ_CUR) {
            return tud_audio_buffer_and_schedule_control_xfer(rhport,
                                                              request,
                                                              &s_clock_valid,
                                                              sizeof(s_clock_valid));
        }
        break;

    default:
        break;
    }

    return false;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *request)
{
    (void)rhport;

    uint8_t itf = TU_U16_LOW(request->wIndex);
    if (itf == ITF_NUM_AUDIO_STREAMING_SPK) {
        s_uac_spk_streaming = false;
        (void)tud_audio_clear_ep_out_ff();
    } else if (itf == ITF_NUM_AUDIO_STREAMING_MIC) {
        s_uac_mic_streaming = false;
        (void)tud_audio_clear_ep_in_ff();
    }

    return true;
}
