#include "afe_processor.h"
#include "esp_err.h"
#include "esp_log.h"
#include "i2s_mic_driver.h"
#include "usb_composite.h"

static const char *TAG = "walle_ear_s3";

void app_main(void)
{
    ESP_LOGI(TAG, "Walle Ear S3 starting");

    ESP_ERROR_CHECK(usb_composite_init());
    ESP_ERROR_CHECK(afe_processor_init());

    /*
     * The processor owns the raw audio frame size. I2S outputs interleaved
     * mic frames; UAC takes one raw mic and TDOA uses all available mics.
     */
    ESP_ERROR_CHECK(i2s_mic_driver_init(afe_processor_get_feed_samples_per_channel()));
    esp_err_t ret = i2s_mic_driver_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S mic driver start failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_ERROR_CHECK(afe_processor_start());

    ESP_LOGI(TAG, "Walle Ear S3 audio pipeline is running");
}
