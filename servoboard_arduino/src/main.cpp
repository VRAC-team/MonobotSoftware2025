#include <Arduino.h>
#include <FastLED.h>
#include <Servo.h>
#include <Wire.h>

#define GPIO_LED1_SERVO17 4
#define GPIO_LED2_SERVO18 3
#define GPIO_LED3_SERVO19 5
#define GPIO_LED4_SERVO20 6

#define SERVO_US_MIN 500
#define SERVO_US_MAX 2500
Servo g_servos[4];

#define LEDS_REFRESH_PERIOD_MS 30
#define LEDS_COUNT 38
#define LEDS_BRIGHTNESS 80
enum LED_PATTERN : uint8_t {
    RAINBOW = 0, // FastLED's built-in rainbow generator
    RAINBOW_WITH_GLITTER = 1, // built-in FastLED rainbow, plus some random sparkly glitter
    CONFETTI = 2, // random colored speckles that blink in and fade smoothly
    SINELON = 3, // a colored dot sweeping back and forth, with fading trails
    BPM = 4, // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
    JUGGLE = 5, // eight colored dots, weaving in and out of sync with each other

    NONE = 255 // any invalid pattern is equivalent to NONE
};
enum LED_PATTERN g_led_pattern[4] = { NONE, NONE, NONE, NONE };
CRGB g_leds[4][LEDS_COUNT];
uint8_t g_leds_hue = 0;

unsigned long g_last_time_blink = 0;

void i2c_flush()
{
    while (Wire.available()) {
        Wire.read();
    }
}

void i2c_write_callback(int length)
{
    uint8_t cmd = Wire.read();

    // CMD SET_LED_PATTERN 0x1
    if (cmd == 0x1) {
        if (length != 3) {
            i2c_flush();
            return;
        }

        uint8_t led_id = Wire.read();
        if (led_id > 3) {
            i2c_flush();
            return;
        }

        uint8_t led_pattern = Wire.read();

        g_led_pattern[led_id] = (enum LED_PATTERN)led_pattern;
    }
    // CMD WRITE_SERVO_US 0x2
    else if (cmd == 0x2) {
        if (length != 4) {
            i2c_flush();
            return;
        }

        uint8_t servo_id = Wire.read();

        if (servo_id > 3) {
            i2c_flush();
            return;
        }

        uint8_t data1 = Wire.read();
        uint8_t data2 = Wire.read();
        uint16_t us = (data1 << 8) | (data2);
        if (us < SERVO_US_MIN || us > SERVO_US_MAX) {
            i2c_flush();
            return;
        }

        g_servos[servo_id].writeMicroseconds(us);
    }
}

void setup()
{
    // setup a slave i2c peripheral on addr 0x8
    Wire.setClock(400000);
    Wire.begin(0x8);
    Wire.onReceive(i2c_write_callback);

    // For a given gpio, either setup as a led OR a servo depending of the needs

    // setup leds
    FastLED.addLeds<WS2812, GPIO_LED1_SERVO17, GRB>(g_leds[0], LEDS_COUNT).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<WS2812, GPIO_LED2_SERVO18, GRB>(g_leds[1], LEDS_COUNT).setCorrection(TypicalLEDStrip);
    // FastLED.addLeds<WS2812, GPIO_LED3_SERVO19, GRB>(g_leds[2], LEDS_COUNT).setCorrection(TypicalLEDStrip);
    // FastLED.addLeds<WS2812, GPIO_LED4_SERVO20, GRB>(g_leds[3], LEDS_COUNT).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(LEDS_BRIGHTNESS);

    // setup servos
    // g_servos[0].attach(GPIO_LED1_SERVO17, SERVO_US_MIN, SERVO_US_MAX);
    // g_servos[1].attach(GPIO_LED2_SERVO18, SERVO_US_MIN, SERVO_US_MAX);
    // g_servos[2].attach(GPIO_LED3_SERVO19, SERVO_US_MIN, SERVO_US_MAX);
    // g_servos[3].attach(GPIO_LED4_SERVO20, SERVO_US_MIN, SERVO_US_MAX);

    // Serial.begin(115200);
    // Serial.print("starting servoboard_arduino build:");
    // Serial.print(__DATE__);
    // Serial.print(" time:");
    // Serial.println(__TIME__);
}

void update_leds_pattern(CRGB* leds, uint16_t leds_count, enum LED_PATTERN pattern)
{
    switch (pattern) {
    case RAINBOW: {
        fill_rainbow(leds, leds_count, g_leds_hue, 7);
        break;
    }

    case RAINBOW_WITH_GLITTER: {
        fill_rainbow(leds, leds_count, g_leds_hue, 7);
        if (random8() < 80) {
            leds[random16(leds_count)] += CRGB::White;
        }
        break;
    }

    case CONFETTI: {
        fadeToBlackBy(leds, leds_count, 10);
        int pos = random16(leds_count);
        leds[pos] += CHSV(g_leds_hue + random8(64), 200, 255);
        break;
    }

    case SINELON: {
        fadeToBlackBy(leds, leds_count, 20);
        int pos = beatsin16(13, 0, leds_count - 1);
        leds[pos] += CHSV(g_leds_hue, 255, 192);
        break;
    }

    case BPM: {
        uint8_t BeatsPerMinute = 62;
        CRGBPalette16 palette = PartyColors_p;
        uint8_t beat = beatsin8(BeatsPerMinute, 64, 255);
        for (uint16_t i = 0; i < leds_count; i++) { // 9948
            leds[i] = ColorFromPalette(palette, g_leds_hue + (i * 2), beat - g_leds_hue + (i * 10));
        }
        break;
    }

    case JUGGLE: {
        fadeToBlackBy(leds, leds_count, 20);
        uint8_t dothue = 0;
        for (int i = 0; i < 8; i++) {
            leds[beatsin16(i + 7, 0, leds_count - 1)] |= CHSV(dothue, 200, 255);
            dothue += 32;
        }
        break;
    }

    default: {
        fill_solid(leds, leds_count, CRGB::Black);
        break;
    }
    }
}

void loop()
{
    if (millis() - g_last_time_blink > 50) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        g_last_time_blink = millis();
    }

    if (millis() - g_last_time_blink > LEDS_REFRESH_PERIOD_MS) {
        update_leds_pattern(g_leds[0], LEDS_COUNT, g_led_pattern[0]);
        update_leds_pattern(g_leds[1], LEDS_COUNT, g_led_pattern[1]);
        // update_leds_pattern(g_leds[2], LEDS_COUNT, g_led_pattern[2]);
        // update_leds_pattern(g_leds[3], LEDS_COUNT, g_led_pattern[3]);
        FastLED.show();
    }
    EVERY_N_MILLISECONDS(20) { g_leds_hue++; }
}
