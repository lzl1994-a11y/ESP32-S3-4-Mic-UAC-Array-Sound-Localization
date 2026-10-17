"""Exercise the real capture pipeline with host USB/I2S sinks.

Run: python tests/test_audio_capture.py (requires host GCC).
Checks the default VAD-off build and the retained VAD-on build, without
simulating hardware clocks, USB packet timing or acoustic quality.
"""

from pathlib import Path
import re
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def function(source, name):
    match = re.search(r"^(?:static )?[A-Za-z_][^\n]*\b" + name + r"\(", source, re.MULTILINE)
    if match is None:
        raise ValueError(f"Function not found: {name}")
    opening = source.index("{", match.start())
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


PRELUDE = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>

typedef int esp_err_t;
typedef int BaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
enum { ESP_OK, ESP_ERR_INVALID_STATE, ESP_ERR_NO_MEM, ESP_ERR_TIMEOUT };
enum { MALLOC_CAP_SPIRAM = 1, MALLOC_CAP_INTERNAL = 2, MALLOC_CAP_8BIT = 4 };
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define pdPASS 1
static void test_log(const char *tag, const char *format, ...)
{ (void)tag; (void)format; }
#define ESP_LOGE(...) test_log(__VA_ARGS__)
#define ESP_LOGW(...) test_log(__VA_ARGS__)
#define ESP_LOGI(...) test_log(__VA_ARGS__)
#define ESP_LOGD(...) test_log(__VA_ARGS__)
static const char *esp_err_to_name(esp_err_t error)
{ (void)error; return "mock"; }

static void *allocations[8];
static size_t allocation_count;
static void *heap_caps_calloc(size_t count, size_t size, unsigned caps)
{
    (void)caps;
    assert(allocation_count < 8);
    void *pointer = calloc(count, size);
    assert(pointer);
    allocations[allocation_count++] = pointer;
    return pointer;
}
static jmp_buf task_exit;
static void (*capture_task)(void *);
static bool task_create_fails, stopped_on_next_read, streaming;
static unsigned task_create_count, read_count, write_count;
static int16_t input_pcm[SAMPLES_PER_CHANNEL * MIC_COUNT];
static int16_t output_pcm[SAMPLES_PER_CHANNEL];

static void vTaskDelay(TickType_t ticks) { (void)ticks; }
static void vTaskDelete(TaskHandle_t task)
{ (void)task; longjmp(task_exit, 2); }
static BaseType_t xTaskCreatePinnedToCore(void (*entry)(void *), const char *name,
                                         uint32_t stack, void *argument,
                                         unsigned priority, TaskHandle_t *handle,
                                         BaseType_t core)
{
    assert(strcmp(name, "audio_capture") == 0);
    assert(stack > 0 && priority > 0 && argument == NULL && core == 1);
    ++task_create_count;
    if (task_create_fails) return 0;
    capture_task = entry;
    *handle = (void *)1;
    return pdPASS;
}
static esp_err_t i2s_mic_driver_read(int16_t *pcm, size_t samples, TickType_t timeout)
{
    assert(samples == SAMPLES_PER_CHANNEL && timeout == portMAX_DELAY);
    if (read_count++ == 0) {
        memcpy(pcm, input_pcm, sizeof(input_pcm));
        return ESP_OK;
    }
    stopped_on_next_read = true;
    longjmp(task_exit, 1);
}
static bool usb_composite_uac_is_streaming(void) { return streaming; }
static esp_err_t usb_composite_uac_write(const int16_t *pcm, size_t bytes,
                                         TickType_t timeout)
{
    assert(bytes == sizeof(output_pcm) && timeout == 110);
    memcpy(output_pcm, pcm, bytes);
    ++write_count;
    return ESP_OK;
}

#if TEST_VAD_ENABLED
typedef void *vad_handle_t;
typedef enum { VAD_SILENCE, VAD_SPEECH } vad_state_t;
enum { VAD_MODE_0 };
static int vad_instance;
static unsigned vad_create_count, vad_process_count;
static vad_handle_t vad_create_with_param(int mode, int rate, int ms,
                                          int min_speech, int min_silence)
{
    assert(mode == VAD_MODE_0 && rate == 16000 && ms == 30);
    assert(min_speech == 128 && min_silence == 500);
    ++vad_create_count;
    return &vad_instance;
}
static vad_state_t vad_process(vad_handle_t handle, int16_t *pcm, int rate, int ms)
{
    assert(handle == &vad_instance && rate == 16000 && ms == 30);
    for (int i = 0; i < 480; ++i) assert(pcm[i] == input_pcm[i * 3 * MIC_COUNT]);
    ++vad_process_count;
    return streaming ? VAD_SPEECH : VAD_SILENCE;
}
#endif
"""

ASSERTIONS = r"""
#if TEST_VAD_ENABLED != AFE_ENABLE_VAD
#error "The default VAD configuration changed"
#endif

static void run_capture(bool usb_streaming)
{
    streaming = usb_streaming;
    stopped_on_next_read = false;
    read_count = write_count = 0;
    memset(output_pcm, 0, sizeof(output_pcm));
    if (setjmp(task_exit) == 0) capture_task(NULL);
    assert(stopped_on_next_read);
    assert(write_count == (usb_streaming ? 1u : 0u));
    assert(allocation_count == (TEST_VAD_ENABLED ? 4u : 2u));
    if (usb_streaming) {
        const int16_t expected[] = {0, 2, -2, 32766, -32768, 32767, -32768, 32767, -32768};
        for (size_t i = 0; i < SAMPLES_PER_CHANNEL; ++i) {
            assert(output_pcm[i] == expected[i % 9]);
        }
    }
    for (size_t i = 0; i < allocation_count; ++i) free(allocations[i]);
    allocation_count = 0;
}
int main(void)
{
    assert(MIC_COUNT == 2 && I2S_MIC_SAMPLE_RATE_HZ == 48000);
    assert(afe_processor_start() == ESP_ERR_INVALID_STATE);
    assert(afe_processor_init() == ESP_OK);
    assert(afe_processor_init() == ESP_ERR_INVALID_STATE);
    assert(afe_processor_get_feed_samples_per_channel() == SAMPLES_PER_CHANNEL);
    task_create_fails = true;
    assert(afe_processor_start() == ESP_ERR_NO_MEM);
    task_create_fails = false;
    task_create_count = 0;
    assert(afe_processor_start() == ESP_OK && task_create_count == 1);
    assert(afe_processor_start() == ESP_ERR_INVALID_STATE);
    const int16_t mic0[] = {0, 1, -1, 16383, -16384, 16384, -16385, 32767, -32768};
    for (size_t i = 0; i < SAMPLES_PER_CHANNEL; ++i) {
        input_pcm[i * MIC_COUNT] = mic0[i % 9];
        input_pcm[i * MIC_COUNT + 1] = (int16_t)(1000 - (int)i);
    }
    run_capture(true);
    run_capture(false);
    assert(uac_write_timeout_ticks(48000 * sizeof(int16_t)) == 500);
#if TEST_VAD_ENABLED
    assert(vad_create_count == 1 && vad_process_count == 2);
    assert(!s_vad_speech_active);
#endif
    printf("PASS: raw Mic0 selection, 2x gain/clipping, USB gating, one task, VAD=%d\n",
           AFE_ENABLE_VAD);
}
"""


def main():
    compiler = shutil.which("gcc")
    if compiler is None:
        raise SystemExit("Host GCC is required on PATH")
    source = (ROOT / "main/afe_processor.c").read_text(encoding="utf-8")
    config = (ROOT / "main/i2s_mic_driver.h").read_text(encoding="utf-8")
    definitions = "\n".join(re.findall(r"^#define .+$", config, re.M))
    # Compile the production source; only platform headers/services are replaced.
    source = re.sub(r'^#include [<"].*[>"]\s*$', "", source, flags=re.M)
    program = definitions + "\n" + PRELUDE + "\n" + source + "\n" + ASSERTIONS
    with tempfile.TemporaryDirectory(prefix="audio-capture-test-") as directory:
        test_source = Path(directory) / "capture_test.c"
        test_source.write_text(program, encoding="utf-8")
        for enabled in (0, 1):
            executable = Path(directory) / f"capture_{enabled}.exe"
            arguments = [f"-DTEST_VAD_ENABLED={enabled}"]
            if enabled:
                arguments.append("-DAFE_ENABLE_VAD=1")
            subprocess.run([
                compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2",
                *arguments, str(test_source), "-o", str(executable),
            ], check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
