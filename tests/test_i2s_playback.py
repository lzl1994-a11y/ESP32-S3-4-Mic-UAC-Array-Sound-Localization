"""Check real I2S PCM conversion and write error handling using a host sink.

Run: python tests/test_i2s_playback.py (requires host GCC).
This does not simulate the ESP32 clock or DMA hardware.
"""

import re
import shutil
import subprocess
import tempfile
from pathlib import Path

from test_azimuth import ROOT, function


def main():
    compiler = shutil.which("gcc")
    if compiler is None:
        raise SystemExit("Host GCC is required on PATH")
    source = (ROOT / "main/i2s_mic_driver.c").read_text(encoding="utf-8")
    gain = re.search(r"^#define I2S_SPK_GAIN .+$", source, re.M).group()
    program = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
typedef int esp_err_t;
enum { ESP_OK, ESP_ERR_INVALID_STATE, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_SIZE,
       ESP_ERR_TIMEOUT };
#define portMAX_DELAY UINT32_MAX
static bool s_started = true, s_tx_enabled = true;
static void *s_i2s1_tx = (void *)1;
static size_t s_samples_per_channel = 4;
static int32_t raw[8], sink[8];
static int32_t *s_tx_raw_buf = raw;
static size_t sink_bytes;
static bool short_write;
static esp_err_t write_result;
static esp_err_t i2s_channel_write(void *handle, const void *src, size_t size,
                                  size_t *written, uint32_t timeout)
{
    assert(handle == s_i2s1_tx && timeout == portMAX_DELAY);
    assert(size <= sizeof(sink));
    sink_bytes = size;
    memcpy(sink, src, size);
    *written = short_write ? size / 2 : size;
    return write_result;
}
""" + gain + "\n" + function(source, "i2s_mic_driver_write_tx") + r"""
int main(void)
{
    int16_t pcm[] = {32767, -32768, 1, -1, 0, 0, 8191, -8192};
    const int32_t expected[] = {
        2147418112, INT32_MIN, 262144, -262144, 0, 0, 2147221504, INT32_MIN
    };
    assert(i2s_mic_driver_write_tx(pcm, 4) == ESP_OK);
    assert(sink_bytes == sizeof(expected));
    assert(memcmp(sink, expected, sizeof(expected)) == 0);
    assert(i2s_mic_driver_write_tx(pcm + 2, 1) == ESP_OK);
    assert(sink_bytes == 8 && sink[0] == 262144 && sink[1] == -262144);
    assert(i2s_mic_driver_write_tx(NULL, 1) == ESP_ERR_INVALID_ARG);
    assert(i2s_mic_driver_write_tx(pcm, 0) == ESP_ERR_INVALID_ARG);
    assert(i2s_mic_driver_write_tx(pcm, 5) == ESP_ERR_INVALID_SIZE);
    short_write = true;
    assert(i2s_mic_driver_write_tx(pcm, 1) == ESP_ERR_INVALID_SIZE);
    short_write = false;
    write_result = ESP_ERR_TIMEOUT;
    assert(i2s_mic_driver_write_tx(pcm, 1) == ESP_ERR_TIMEOUT);
    s_started = false;
    assert(i2s_mic_driver_write_tx(pcm, 1) == ESP_ERR_INVALID_STATE);
    s_started = true;
    s_tx_enabled = false;
    assert(i2s_mic_driver_write_tx(pcm, 1) == ESP_ERR_INVALID_STATE);
    puts("PASS: stereo PCM order, signed 32-bit slots, gain clipping, one-frame writes and errors");
}
"""
    with tempfile.TemporaryDirectory(prefix="i2s-playback-test-") as directory:
        test_source = Path(directory) / "playback_test.c"
        executable = Path(directory) / "playback_test.exe"
        test_source.write_text(program, encoding="utf-8")
        subprocess.run([
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2",
            str(test_source), "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
