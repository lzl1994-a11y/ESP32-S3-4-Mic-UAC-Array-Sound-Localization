#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t usb_composite_init(void);
esp_err_t usb_composite_uac_write(const int16_t *pcm, size_t byte_len, TickType_t ticks_to_wait);
esp_err_t usb_composite_cdc_write_str(const char *text, TickType_t ticks_to_wait);
bool usb_composite_is_mounted(void);
bool usb_composite_uac_is_streaming(void);
bool usb_composite_cdc_is_connected(void);
uint8_t usb_composite_cdc_get_line_state(void);
uint32_t usb_composite_cdc_write_available(void);

#ifdef __cplusplus
}
#endif
