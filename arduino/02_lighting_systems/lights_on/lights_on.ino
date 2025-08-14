/*
 * Simple LED On – keeps all LEDs on continuously
 * Adjust NUM_LEDS and LED_PIN to match your setup.
 */

#include <FastLED.h>

#define LED_PIN 3
#define NUM_LEDS 120
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

void setup() {
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(255);

  // Turn all LEDs on (pure white). Change color if needed.
  fill_solid(leds, NUM_LEDS, CRGB::White);
  FastLED.show();
}

void loop() {
  // Do nothing – LEDs remain on
}

