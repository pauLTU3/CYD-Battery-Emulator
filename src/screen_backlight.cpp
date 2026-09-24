// pauLTU3
// Backlight dim/off handling for the CYD screen.
// Testing build, not validated on a real system.

#include "screen_backlight.h"

#include <Arduino.h>

namespace {

constexpr uint32_t DIM_AFTER_MS = 600000U;
constexpr uint32_t OFF_AFTER_MS = 1200000U;

constexpr uint8_t FULL_BRIGHTNESS = 100U;
constexpr uint8_t DIM_BRIGHTNESS = 50U;
constexpr uint8_t OFF_BRIGHTNESS = 0U;

uint32_t g_last_touch_ms = 0U;

uint8_t g_last_brightness = 255U;

uint8_t percent_to_pwm(uint8_t percent) {
    if (percent >= 100U) {
        return 255U;
    }
    return static_cast<uint8_t>((static_cast<uint16_t>(percent) * 255U) / 100U);
}

void apply_backlight_percent(uint8_t percent) {
    if (percent == g_last_brightness) {
        return;
    }

    g_last_brightness = percent;

#if TFT_BACKLIGHT_ON == HIGH
    analogWrite(TFT_BL, percent_to_pwm(percent));
#else
    analogWrite(TFT_BL, 255U - percent_to_pwm(percent));
#endif
}

}  // namespace

void screen_backlight_init() {
    pinMode(TFT_BL, OUTPUT);
    g_last_touch_ms = millis();
    g_last_brightness = 255U;
    apply_backlight_percent(FULL_BRIGHTNESS);
}

void screen_backlight_note_touch() {
    g_last_touch_ms = millis();
    apply_backlight_percent(FULL_BRIGHTNESS);
}

void screen_backlight_update() {
    const uint32_t idle_ms = millis() - g_last_touch_ms;
    if (idle_ms >= OFF_AFTER_MS) {
        apply_backlight_percent(OFF_BRIGHTNESS);
        return;
    }
    if (idle_ms >= DIM_AFTER_MS) {
        apply_backlight_percent(DIM_BRIGHTNESS);
        return;
    }
    apply_backlight_percent(FULL_BRIGHTNESS);
}
