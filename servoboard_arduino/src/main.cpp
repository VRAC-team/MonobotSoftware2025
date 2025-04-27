#include <Arduino.h>
#include <FastLED.h>
#include <Servo.h>
#include <Wire.h>

#define GPIO_LED1 4
#define GPIO_LED2 3
#define GPIO_LED3 5
#define GPIO_LED4 6

#define SERVO_US_MIN 500
#define SERVO_US_MAX 2500
Servo g_servos[4];

const uint32_t LEDS_REFRESH_PERIOD_MS = 10;
#define LEDS_COUNT 35
#define LEDS_BRIGHTNESS 96
enum LED_PATTERN {
    NONE = 0,
    RAINBOW = 1, // FastLED's built-in rainbow generator
    RAINBOW_WITH_GLITTER = 2, // built-in FastLED rainbow, plus some random sparkly glitter
    CONFETTI = 3, // random colored speckles that blink in and fade smoothly
    SINELON = 4, // a colored dot sweeping back and forth, with fading trails
    BPM = 5, // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
    JUGGLE = 6, // eight colored dots, weaving in and out of sync with each other
    LED_PATTERN_COUNT
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
    uint8_t id = Wire.read();

    // Serial.print("recv i2c write id:");
    // Serial.print(id);
    // Serial.print(" length:");
    // Serial.print(length);

    if (id >= 0 && id <= 3) {
        if (length != 2) {
            i2c_flush();
            return;
        }

        uint8_t led_pattern = Wire.read();

        // check if the pattern is valid
        if (led_pattern >= LED_PATTERN_COUNT) {
            led_pattern = NONE;
        }

        g_led_pattern[id] = (enum LED_PATTERN)led_pattern;
    }

    // else if (id >= 17 || id <= 20) {
    //   if (length != 3) {
    //     i2c_flush();
    //     return;
    //   }

    //   uint8_t data1 = Wire.read();
    //   uint8_t data2 = Wire.read();
    //   uint16_t angle = (data1 << 8) | (data2);

    //   if (angle > 270) {
    //     angle = 270;
    //   }

    //   uint16_t angle_us = map(angle, 0, 270, SERVO_US_MIN, SERVO_US_MAX);
    //   g_servos[id-17].writeMicroseconds(angle_us);
    // }
}

void setup()
{
    // setup a slave i2c peripheral on addr 0x8
    Wire.setClock(400000);
    Wire.begin(0x8);
    Wire.onReceive(i2c_write_callback);

    // setup leds
    FastLED.addLeds<WS2812, GPIO_LED1, GRB>(g_leds[0], LEDS_COUNT).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<WS2812, GPIO_LED2, GRB>(g_leds[1], LEDS_COUNT).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<WS2812, GPIO_LED3, GRB>(g_leds[2], LEDS_COUNT).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<WS2812, GPIO_LED4, GRB>(g_leds[3], LEDS_COUNT).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(LEDS_BRIGHTNESS);

    // if the robot need more than 16 servos, re can repurpose leds as servo
    // servo setup
    // g_servos[0].attach(GPIO_LED1, SERVO_US_MIN, SERVO_US_MAX);
    // g_servos[1].attach(GPIO_LED2, SERVO_US_MIN, SERVO_US_MAX);
    // setvo default positions
    // g_servos[0].writeMicroseconds( map(80, 0, 270, SERVO_US_MIN, SERVO_US_MAX));
    // g_servos[1].writeMicroseconds( map(1, 0, 270, SERVO_US_MIN, SERVO_US_MAX));

    Serial.begin(115200);
    Serial.print("starting servoboard_arduino build:");
    Serial.print(__DATE__);
    Serial.print(" time:");
    Serial.println(__TIME__);
}

void update_leds_pattern(CRGB* leds, uint16_t leds_count, enum LED_PATTERN pattern)
{
    switch (pattern) {
    case NONE: {
        fill_solid(leds, leds_count, CRGB::Black);
        break;
    }

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
        for (int i = 0; i < leds_count; i++) { // 9948
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
    if (millis() - g_last_time_blink > 500) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        g_last_time_blink = millis();
    }

    if (millis() - g_last_time_blink > LEDS_REFRESH_PERIOD_MS) {
        update_leds_pattern(g_leds[0], LEDS_COUNT, g_led_pattern[0]);
        update_leds_pattern(g_leds[1], LEDS_COUNT, g_led_pattern[1]);
        update_leds_pattern(g_leds[2], LEDS_COUNT, g_led_pattern[2]);
        update_leds_pattern(g_leds[3], LEDS_COUNT, g_led_pattern[3]);
        FastLED.show();
    }
    EVERY_N_MILLISECONDS(20) { g_leds_hue++; }

    // EVERY_N_SECONDS(10) {
    //   g_led_pattern = (enum LED_PATTERN)(g_led_pattern + 1);
    //   if (g_led_pattern == LED_PATTERN_COUNT) {
    //     g_led_pattern = RAINBOW;
    //   }
    // }
}
