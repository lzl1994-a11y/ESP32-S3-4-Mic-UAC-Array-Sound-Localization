"""Compile and exercise the actual firmware TDOA functions on the host.

Run: python tests/test_azimuth.py (requires a host GCC on PATH).
Only hardware/task services are stubbed; no copy of the algorithm is maintained.
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


def main():
    compiler = shutil.which("gcc")
    if compiler is None:
        raise SystemExit("Host GCC is required on PATH")
    source = (ROOT / "main/afe_processor.c").read_text(encoding="utf-8")
    header = (ROOT / "main/afe_processor.h").read_text(encoding="utf-8")
    position_type = re.search(r"typedef struct \{.*?\} afe_source_position_t;", header, re.S).group()
    config = (ROOT / "main/i2s_mic_driver.h").read_text(encoding="utf-8")
    definitions = "\n".join(re.findall(r"^#define .+$", source + "\n" + config, re.M))
    names = [
        "clampf", "float_to_int_round", "publish_invalid_position",
        "max_tdoa_lag_samples", "channel_mean", "frame_rms",
        "normalized_correlation_at_lag", "estimate_pair_delay_samples",
        "delay_to_distance", "solve_front_azimuth", "update_source_position",
    ]
    program = "\n".join([
        "#include <assert.h>\n#include <math.h>\n#include <stdbool.h>\n"
        "#include <stdint.h>\n#include <stddef.h>\n#include <stdio.h>",
        '#include "mic_array_config.h"',
        definitions,
        "#define ESP_LOGD(...) ((void)0)\n#define ESP_LOGI(...) ((void)0)",
        position_type,
        "static size_t s_algo_samples_per_channel = AFE_VAD_FRAME_SAMPLES;\n"
        "static afe_source_position_t result;\n"
        "static void publish_source_position(afe_source_position_t p) { result = p; }",
        *(function(source, name) for name in names),
        r"""
static float waveform(float t)
{
    return 6000.0f * sinf(t * 0.071f) + 4000.0f * sinf(t * 0.197f)
         + 3000.0f * sinf(t * 0.431f) + 2000.0f * sinf(t * 0.817f);
}

static void check_frame(float angle)
{
    int16_t frame[AFE_VAD_FRAME_SAMPLES * MIC_COUNT];
    float delay = sinf(angle * (float)M_PI / 180.0f) *
                  MIC_ARRAY_SPACING_MM / SOUND_SPEED_MM_PER_S * AFE_ALGO_SAMPLE_RATE_HZ;
    for (int i = 0; i < AFE_VAD_FRAME_SAMPLES; ++i) {
        frame[i * MIC_COUNT] = (int16_t)(waveform((float)i) + 500.0f);
        frame[i * MIC_COUNT + 1] = (int16_t)(waveform((float)i - delay) - 300.0f);
    }
    update_source_position(frame);
    assert(result.valid);
    assert(abs(result.azimuth_deg - (int)angle) <= 4);
    assert(result.azimuth_deg >= -90 && result.azimuth_deg <= 90);
    assert(result.y_milli >= 0);
    assert(result.z_milli == 0 && result.elevation_deg == 0);
    printf("source %+.0f deg -> azimuth %+d deg, confidence %d\n",
           (double)angle, result.azimuth_deg, result.confidence);
}

int main(void)
{
    assert(MIC_COUNT == 2);
    assert(SAMPLES_PER_CHANNEL * AFE_ALGO_SAMPLE_RATE_HZ ==
           AFE_VAD_FRAME_SAMPLES * I2S_MIC_SAMPLE_RATE_HZ);
    for (int angle = -75; angle <= 75; angle += 15) {
        check_frame((float)angle);
    }
    float ux, uy, angle;
    float limit = MIC_ARRAY_SPACING_MM / SOUND_SPEED_MM_PER_S * AFE_ALGO_SAMPLE_RATE_HZ;
    assert(solve_front_azimuth(limit, &ux, &uy, &angle));
    assert(fabsf(angle - 90.0f) < 0.01f && uy < 0.001f);
    assert(solve_front_azimuth(-limit, &ux, &uy, &angle));
    assert(fabsf(angle + 90.0f) < 0.01f && uy < 0.001f);
    assert(solve_front_azimuth(limit + 0.25f, &ux, &uy, &angle));
    assert(fabsf(angle - 90.0f) < 0.01f);
    assert(!solve_front_azimuth(limit + 1.0f, &ux, &uy, &angle));
    assert(!solve_front_azimuth(-limit - 1.0f, &ux, &uy, &angle));
    assert(!solve_front_azimuth(NAN, &ux, &uy, &angle));
    assert(!solve_front_azimuth(INFINITY, &ux, &uy, &angle));

    int16_t silence[AFE_VAD_FRAME_SAMPLES * MIC_COUNT] = {0};
    update_source_position(silence);
    assert(!result.valid);
    update_source_position(NULL);
    assert(!result.valid);
    for (int i = 0; i < AFE_VAD_FRAME_SAMPLES; ++i) {
        silence[i * MIC_COUNT] = (int16_t)waveform((float)i);
    }
    update_source_position(silence); /* Missing right microphone. */
    assert(!result.valid);
    puts("PASS: stereo TDOA direction, front-only range, endfire, invalid inputs and frame timing");
    return 0;
}
""",
    ])
    # abs() is used only by the test assertions.
    program = "#include <stdlib.h>\n" + program
    with tempfile.TemporaryDirectory(prefix="azimuth-test-") as directory:
        test_source = Path(directory) / "azimuth_test.c"
        executable = Path(directory) / "azimuth_test.exe"
        test_source.write_text(program, encoding="utf-8")
        subprocess.run([
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2",
            "-I", str(ROOT / "main"), str(test_source), "-o", str(executable), "-lm",
        ], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
