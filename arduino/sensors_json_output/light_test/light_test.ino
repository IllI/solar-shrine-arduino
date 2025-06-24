/*
 * Dual Ultrasonic Sensor Reader with JSON Output and FastLED Control
 * Optimized for WS2815 LED strips - 83 LEDs in Human Hand Configuration
 * 6 strips mapped to fingers and palm
 */

#include <ArduinoJson.h>
#include <FastLED.h>

// Sensor pins
const int trigPin1 = 9;
const int echoPin1 = 10;
const int trigPin2 = 5;
const int echoPin2 = 6;

// LED strip configuration for WS2815 - Human Hand Layout
#define LED_PIN 3
#define NUM_LEDS 83       // Total LEDs across all 6 strips
#define LED_TYPE WS2815   // WS2815 LED type (12V addressable LEDs)
#define COLOR_ORDER GRB   // WS2815 strips typically use GRB order

CRGB leds[NUM_LEDS];

// Hand LED Mapping - 6 strips across human hand
// Adjust these ranges based on your actual strip layout
struct HandMapping {
  int thumb_start = 0;     int thumb_end = 12;      // Strip 1: Thumb (13 LEDs)
  int index_start = 13;    int index_end = 27;      // Strip 2: Index finger (15 LEDs)  
  int middle_start = 28;   int middle_end = 43;     // Strip 3: Middle finger (16 LEDs)
  int ring_start = 44;     int ring_end = 57;       // Strip 4: Ring finger (14 LEDs)
  int pinky_start = 58;    int pinky_end = 69;      // Strip 5: Pinky (12 LEDs)
  int palm_start = 70;     int palm_end = 82;       // Strip 6: Palm (13 LEDs)
} hand;

// Range constants (in cm)
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

// Easing parameter for smooth transitions
float current_brightness = 0.0; // 0.0 = off, 1.0 = fully on
const float EASING = 0.08;      // Easing speed (smaller = slower fade)

void setup() {
  Serial.begin(9600);
  delay(2000);
  // Sensor setup
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  // FastLED setup optimized for WS2815
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(120);  // Reduced for 83 LEDs
  FastLED.setMaxPowerInVoltsAndMilliamps(12, 2000);  // Increased power for more LEDs
  // --- This is where setting the LEDs to a solid color works reliably ---
  // fill_solid(leds, NUM_LEDS, CRGB::Blue);
  // FastLED.show();
}

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  float duration = pulseIn(echoPin, HIGH);
  return (duration * 0.0343) / 2;
}

void fillHandSection(int start, int end, CRGB color) {
  for(int i = start; i <= end; i++) {
    leds[i] = color;
  }
}

void createHandWaveEffect(float leftDistance, float rightDistance, bool handsDetected) {
  if (!handsDetected) {
    // Turn all LEDs off when no hands detected
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    return;
  }
  
  // Calculate which hand is closer and effect intensity
  float difference = leftDistance - rightDistance;  // Positive = left closer, Negative = right closer
  float intensity = (leftDistance + rightDistance) / (2 * MAX_RANGE);  // Overall proximity
  intensity = 1.0 - constrain(intensity, 0.0, 1.0);  // Invert: closer = higher intensity
  
  // Base colors
  CRGB leftColor = CRGB::Red;
  CRGB rightColor = CRGB::Blue;
  CRGB palmColor = CRGB::Purple;
  
  // Apply intensity to colors
  leftColor.fadeToBlackBy(255 * (1.0 - intensity));
  rightColor.fadeToBlackBy(255 * (1.0 - intensity));
  palmColor.fadeToBlackBy(255 * (1.0 - intensity));
  
  if (abs(difference) < 3) {
    // Hands roughly equal distance - light up entire hand in purple
    fill_solid(leds, NUM_LEDS, palmColor);
    
  } else if (difference > 0) {
    // Left hand closer - emphasize left side fingers
    fillHandSection(hand.thumb_start, hand.thumb_end, leftColor);
    fillHandSection(hand.index_start, hand.index_end, leftColor);
    fillHandSection(hand.middle_start, hand.middle_end, blend(leftColor, palmColor, 128));
    fillHandSection(hand.ring_start, hand.ring_end, blend(palmColor, rightColor, 192));
    fillHandSection(hand.pinky_start, hand.pinky_end, rightColor.fadeToBlackBy(128));
    fillHandSection(hand.palm_start, hand.palm_end, palmColor);
    
  } else {
    // Right hand closer - emphasize right side fingers  
    fillHandSection(hand.thumb_start, hand.thumb_end, leftColor.fadeToBlackBy(128));
    fillHandSection(hand.index_start, hand.index_end, blend(palmColor, leftColor, 192));
    fillHandSection(hand.middle_start, hand.middle_end, blend(rightColor, palmColor, 128));
    fillHandSection(hand.ring_start, hand.ring_end, rightColor);
    fillHandSection(hand.pinky_start, hand.pinky_end, rightColor);
    fillHandSection(hand.palm_start, hand.palm_end, palmColor);
  }
}

void createFingerWaveEffect(float leftDistance, float rightDistance, bool handsDetected) {
  if (!handsDetected) {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    return;
  }
  
  // Create traveling wave effect across fingers based on sensor difference
  float wavePosition = map(leftDistance - rightDistance, -MAX_RANGE, MAX_RANGE, 0, 5);
  wavePosition = constrain(wavePosition, 0, 5);
  
  // Clear all LEDs first
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  
  // Light up fingers based on wave position
  CRGB waveColor = CHSV(map(leftDistance + rightDistance, 0, 2 * MAX_RANGE, 0, 255), 255, 255);
  
  int fingerIndex = (int)wavePosition;
  switch(fingerIndex) {
    case 0: fillHandSection(hand.thumb_start, hand.thumb_end, waveColor); break;
    case 1: fillHandSection(hand.index_start, hand.index_end, waveColor); break;
    case 2: fillHandSection(hand.middle_start, hand.middle_end, waveColor); break;
    case 3: fillHandSection(hand.ring_start, hand.ring_end, waveColor); break;
    case 4: fillHandSection(hand.pinky_start, hand.pinky_end, waveColor); break;
    case 5: fillHandSection(hand.palm_start, hand.palm_end, waveColor); break;
  }
  
  // Add fading effect to adjacent fingers
  float fadeAmount = (wavePosition - fingerIndex) * 255;
  CRGB fadeColor = waveColor;
  fadeColor.fadeToBlackBy(255 - fadeAmount);
  
  if (fingerIndex > 0) {
    switch(fingerIndex - 1) {
      case 0: fillHandSection(hand.thumb_start, hand.thumb_end, fadeColor); break;
      case 1: fillHandSection(hand.index_start, hand.index_end, fadeColor); break;
      case 2: fillHandSection(hand.middle_start, hand.middle_end, fadeColor); break;
      case 3: fillHandSection(hand.ring_start, hand.ring_end, fadeColor); break;
      case 4: fillHandSection(hand.pinky_start, hand.pinky_end, fadeColor); break;
    }
  }
}

void loop() {
  // Read sensors
  float distance1 = readDistance(trigPin1, echoPin1); // Left
  float distance2 = readDistance(trigPin2, echoPin2); // Right

  // Check if hands are detected (within range)
  bool inRange1 = (distance1 >= MIN_RANGE && distance1 <= MAX_RANGE);
  bool inRange2 = (distance2 >= MIN_RANGE && distance2 <= MAX_RANGE);
  bool hands_detected = inRange1 && inRange2;

  // Easing for brightness
  float target_brightness = hands_detected ? 1.0 : 0.0;
  current_brightness += (target_brightness - current_brightness) * EASING;

  // Calculate color: interpolate from red (left closer) to green (right closer)
  float total = distance1 + distance2;
  float ratio = 0.5; // Default to middle (yellow) if something goes wrong
  if (total > 0) {
    ratio = distance2 / total; // 0 = all left (red), 1 = all right (green)
  }
  CRGB color = blend(CRGB::Red, CRGB::Green, (uint8_t)(ratio * 255));

  // Apply brightness (fade in/out)
  CRGB fadedColor = color;
  fadedColor.nscale8_video((uint8_t)(current_brightness * 255));
  fill_solid(leds, NUM_LEDS, fadedColor);
  FastLED.show();
  delay(10);
} 
