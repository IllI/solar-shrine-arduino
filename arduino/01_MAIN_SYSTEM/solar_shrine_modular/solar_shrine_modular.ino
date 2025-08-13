/*
 * Solar Shrine Modular Theremin
 * 
 * A modular theremin system using ultrasonic sensors for gesture control.
 * This version provides independent pitch and volume control through hand proximity.
 * 
 * Hardware:
 * - Arduino Uno/Nano
 * - 2x HC-SR04 ultrasonic sensors (pitch and volume control)
 * - Audio output via Mozzi library
 * 
 * Features:
 * - Real-time gesture-based audio synthesis
 * - Independent pitch and volume control
 * - Smooth audio transitions with rolling averages
 * - Complete silence when no hands detected
 * 
 * Detection System:
 * - Uses distance-based detection (5-50cm range) like solar_shrine_theremin.ino
 * - Converts sensor duration to actual distance in centimeters
 * - Only detects hands within MIN_RANGE to MAX_RANGE
 * - Handles sensor timeout and very close readings properly
 * 
 * Recent fixes:
 * - Replaced duration-based thresholds with distance-based detection
 * - Fixed false hand detection by using proper range checking
 * - Ensure complete silence when no hands are detected
 * - Hardware-level audio cutoff via pin control
 * 
 * Author: AI Assistant
 * Date: 2024
 */

#define FASTLED_FORCE_BITBANG
#define FASTLED_ALLOW_INTERRUPTS 1
#include <FastLED.h>
#include <ArduinoJson.h>
#include "dj_scratch.h"
#include "theremin.h"
#include "alien.h"
#include "robots.h"
#include "led_mapping.h"
// NewPing removed to avoid timer conflicts with NewTone

// =============================================================================
// HARDWARE CONFIGURATION
// =============================================================================

// Pin assignments for HC-SR04 sensors
const int trigPin1 = 10; // Left sensor
const int echoPin1 = 11; // Keep shield wiring: left echo on 11
const int trigPin2 = 5;  // Right sensor
const int echoPin2 = 6; // Corrected from 12 to match hardware wiring and avoid audio conflict
// Distance-based detection constants (in cm)
const float MIN_RANGE = 5.0;                  // MinimTheremin effect initializedum detection range in cm
const float MAX_RANGE = 50.0;                 // Maximum detection range in cm

const unsigned long SENSOR_TIMEOUT = 12000;   // Shorter timeout to avoid Mozzi underruns

// LED strip configuration
#define LED_PIN 3
#define NUM_LEDS 120
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

// NewPing setup
// NewPing objects removed - using manual sensor reading

// Effect types
enum EffectType {
  DJ_SCRATCH,
  ALIEN,
  ROBOTS,
  MINITHEREMIN
};

EffectType currentEffect = DJ_SCRATCH; // Test: start on Alien for quick verification

const unsigned long INTERACTIVE_TIMEOUT = 10000; // 10s no hands -> back to attract
const unsigned long EFFECT_SWITCH_TIMEOUT = 5000; // 5s no hands -> rotate effect
// Effect rotation settings
const bool DISABLE_EFFECT_ROTATION = false; // Enable rotation after timeout when no hands
// Testing flags
const bool TEST_ALIEN_ONLY = false; // Allow all effects; no forced ALIEN-only mode


// Mode constants
enum LightMode {
  ATTRACT_MODE,
  INTERACTIVE_MODE
};

// State variables
LightMode currentMode = ATTRACT_MODE;
unsigned long lastHandDetectedTime = 0;
unsigned long lastEffectRotationTime = 0;

// Attract mode variables
const float ATTRACT_PERIOD = 5000.0;  // 5 seconds in milliseconds
float attractPhaseOffset = 0.0;
bool usePhaseOffset = false;

// Hand detection averaging
const int SAMPLES = 5;
float distance1Samples[SAMPLES];
float distance2Samples[SAMPLES];
int sampleIndex = 0;
bool samplesInitialized = false;

// Last known interactive colors for smooth transition
CRGB lastLeftColor = CRGB::Yellow;
CRGB lastRightColor = CRGB::Yellow;

// =============================================================================
// SETUP AND MAIN LOOP
// =============================================================================



void setup(){
  Serial.begin(9600);
  
  // Sensor pin setup (CRITICAL for manual reading)
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  // FastLED setup
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(150);
  
  // Initialize all LEDs to off
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  Serial.begin(9600);
  // Initialize sample arrays
  for (int i = 0; i < SAMPLES; i++) {
    distance1Samples[i] = MAX_RANGE + 10;
    distance2Samples[i] = MAX_RANGE + 10;
  }

  // Initialize timers for idle state and start first effect
  unsigned long now = millis();
  lastHandDetectedTime = now;
  lastEffectRotationTime = now;
  if (TEST_ALIEN_ONLY) {
    currentEffect = ALIEN;
    // Initialize Mozzi once in alien_setup to avoid start/stop instability
    effect_setup(currentEffect);
  }
}

CRGB getInteractiveColor(float distance) {
  if (distance < MIN_RANGE || distance > MAX_RANGE) {
    return CRGB::Black;
  }
  
  // Map distance to color: far = red, close = yellow
  float ratio = (distance - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
  ratio = constrain(ratio, 0.0, 1.0);
  
  // Red to Yellow interpolation through Orange
  int red = 255;
  int green = (int)(255 * (1.0 - ratio));
  int blue = 0;
  
  return CRGB(red, green, blue);
}

CRGB getAttractColor(unsigned long currentTime) {
  float phase;
  
  if (usePhaseOffset) {
    phase = attractPhaseOffset + (2.0 * PI * currentTime / ATTRACT_PERIOD);
    usePhaseOffset = false;
  } else {
    phase = 2.0 * PI * currentTime / ATTRACT_PERIOD;
  }
  
  // Sinusoidal fade from yellow to red
  float sineValue = (sin(phase) + 1.0) / 2.0;  // Normalize to 0-1
  
  int red = 255;
  int green = (int)(255 * sineValue);
  int blue = 0;
  
  return CRGB(red, green, blue);
}

// =============================================================================
// HANDS DETECTION (sacrosanct) – identical logic to solar_shrine_theremin.ino
// =============================================================================
static void updateHandsDetectionAndMode(
  float avgDistance1,
  float avgDistance2,
  bool &inRange1,
  bool &inRange2,
  bool &handsDetected,
  unsigned long currentTime
) {
  // Strict in-range check
  inRange1 = (avgDistance1 >= MIN_RANGE && avgDistance1 <= MAX_RANGE);
  inRange2 = (avgDistance2 >= MIN_RANGE && avgDistance2 <= MAX_RANGE);
  handsDetected = inRange1 || inRange2;

  // Additional validation in ATTRACT mode: require 3 consecutive detections
  static int falseDetectionCount = 0;
  if (currentMode == ATTRACT_MODE && handsDetected) {
    falseDetectionCount++;
    if (falseDetectionCount < 3) {
      handsDetected = false;
      inRange1 = false;
      inRange2 = false;
    }
  } else {
    falseDetectionCount = 0;
  }

  // Mode state machine and last-hand timestamp
  if (handsDetected) {
    if (currentMode == ATTRACT_MODE) {
      currentMode = INTERACTIVE_MODE;
    }
    lastHandDetectedTime = currentTime;
  } else {
    if (currentMode == INTERACTIVE_MODE &&
        (currentTime - lastHandDetectedTime) >= INTERACTIVE_TIMEOUT) {
      // Smooth transition back to attract mode (match theremin logic)
      CRGB transitionColor = (lastLeftColor.r + lastLeftColor.g > lastRightColor.r + lastRightColor.g)
                               ? lastLeftColor : lastRightColor;
      calculatePhaseOffset(transitionColor);
      currentMode = ATTRACT_MODE;
    }
  }
}

void calculatePhaseOffset(CRGB currentColor) {
  float greenRatio = currentColor.green / 255.0;
  float sineValue = 2.0 * greenRatio - 1.0;
  sineValue = constrain(sineValue, -1.0, 1.0);
  
  attractPhaseOffset = asin(sineValue);
  usePhaseOffset = true;
}

// =============================================================================
// EFFECT MANAGEMENT
// =============================================================================

void effect_disable(EffectType effect) {
  switch (effect) {
    case DJ_SCRATCH: dj_scratch_disable(); break;
    case ALIEN: alien_disable(); break;
    case ROBOTS: robots_disable(); break;
    case MINITHEREMIN: theremin_disable(); break;
  }
}

void effect_setup(EffectType effect) {
  switch (effect) {
    case DJ_SCRATCH:
      dj_scratch_setup();
      break;
    case ALIEN: alien_setup(); break;
    case ROBOTS: robots_setup(); break;
    case MINITHEREMIN: theremin_setup(); break;
  }
}

void effect_update(EffectType effect, float dLeft, float dRight) {
  switch (effect) {
    case DJ_SCRATCH: dj_scratch_update(dLeft, dRight); break;
    case ALIEN: alien_update(dLeft, dRight); break;
    case ROBOTS: robots_update(dLeft, dRight); break;
    case MINITHEREMIN: theremin_update(dLeft, dRight); break;
  }
}

EffectType nextEffect(EffectType effect) {
  switch (effect) {
    case DJ_SCRATCH: return ALIEN;
    case ALIEN: return ROBOTS;
    case ROBOTS: return MINITHEREMIN;
    case MINITHEREMIN: return DJ_SCRATCH;
  }
  return DJ_SCRATCH;
}

const char* effect_name(EffectType effect) {
  switch (effect) {
    case DJ_SCRATCH: return "dj_scratch";
    case ALIEN: return "alien";
    case ROBOTS: return "robots";
    case MINITHEREMIN: return "mini_theremin";
  }
  return "dj_scratch";
}

// Hard kill audio output on pin 12 regardless of current effect
void audio_all_off() {
  // Physically mute by tri-stating pin 12 so no carrier reaches the amplifier
  pinMode(12, INPUT);

  // If not running the Mozzi-based ALIEN effect, it's safe to fully stop Timer1
  if (currentEffect != ALIEN) {
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 &= ~_BV(OCIE1B);
  }
}

void updateLEDs(float avgDistance1, float avgDistance2, bool inRange1, bool inRange2, unsigned long currentTime) {
  if (currentMode == ATTRACT_MODE) {
    // Both sides show the same attract color
    CRGB attractColor = getAttractColor(currentTime);
    fill_solid(leds, NUM_LEDS, attractColor);
    
  } else {  // INTERACTIVE_MODE
    CRGB leftColor = getInteractiveColor(avgDistance1);
    CRGB rightColor = getInteractiveColor(avgDistance2);

    // Update last known colors for smooth transition back to attract mode, only if a hand is in range
    if (inRange1) lastLeftColor = leftColor;
    if (inRange2) lastRightColor = rightColor;
    
    // Apply colors to respective halves
    int halfPoint = NUM_LEDS / 2;
    for (int i = 0; i < halfPoint; i++) {
      leds[i] = leftColor;
    }
    for (int i = halfPoint; i < NUM_LEDS; i++) {
      leds[i] = rightColor;
    }
  }
  
  FastLED.show();
}

// =============================================
// DJ SCRATCH – sound-wave ripple across both hands
// Colors strictly blend: dark purple -> neon blue
// =============================================
static void updateDJLedVisual(float dLeft, float dRight) {
  // Map right-hand distance to wave speed (closer = faster)
  // Use 5..50 cm range
  uint8_t speed = 60; // default BPM-ish
  if (dRight >= 5 && dRight <= 50) {
    speed = (uint8_t)map((int)dRight, 5, 50, 160, 40); // close->fast
  }

  // Build traversal path once to follow physical LED order
  static bool pathInit = false;
  static uint16_t path[NUM_LEDS];
  static uint16_t pathLen = 0;
  if (!pathInit) {
    build_default_traversal(path, pathLen);
    pathInit = true;
  }

  // Wave phase 0..255 advances by beat8
  uint8_t phase = beat8(speed);
  const uint8_t wavelength = 20; // LEDs per cycle

  // Colors
  const CRGB startColor = CRGB(48, 0, 96);    // dark purple
  const CRGB endColor   = CRGB(0, 255, 255);  // neon blue

  // Optional: true 2D spatial wave using normalized coordinates
  static bool spatialInit = false;
  static float ledX[NUM_LEDS];
  static float ledY[NUM_LEDS];
  if (!spatialInit) {
    build_spatial_map(ledX, ledY);
    spatialInit = true;
  }

  // Render: wave moves left->right using x, modulated by y for slight vertical phase
  for (uint16_t idx = 0; idx < NUM_LEDS; ++idx) {
    float xNorm = ledX[idx];           // 0..1 left->right
    float yNorm = ledY[idx];           // 0..1 base->tip
    uint8_t x = (uint8_t)(xNorm * 255.0f);
    uint8_t y = (uint8_t)(yNorm * 64.0f); // small vertical phase
    uint8_t s = sin8(x - phase + y);
    uint8_t b = scale8(s, 220);
    CRGB col = blend(startColor, endColor, s);
    uint8_t nb = (b < 16) ? 16 : b;
    col.nscale8_video(nb);
    leds[idx] = col;
  }

  FastLED.show();
}

// =============================================
// ALIEN – white orb in the center of each hand that grows with proximity
// =============================================
static void updateAlienLedVisual(float dLeft, float dRight) {
  // Build spatial map once
  static bool spatialInit = false;
  static float ledX[NUM_LEDS];
  static float ledY[NUM_LEDS];
  if (!spatialInit) {
    build_spatial_map(ledX, ledY);
    spatialInit = true;
  }

  // Discrete grid sizes
  const uint8_t cols = hand_num_columns();
  const uint8_t rowsL = hand_num_rows(LEFT_HAND);
  const uint8_t rowsR = hand_num_rows(RIGHT_HAND);

  // LEFT HAND: anchor center to sequential LED 31 exactly
  auto leftSeqToColRow = [&](uint8_t seq, uint8_t &col, uint8_t &row){
    // Left order and cumulative run lengths
    Segment order[SEGMENT_COUNT] = { PINKY, RING, MIDDLE, INDEX, PALM, THUMB };
    uint8_t cum[SEGMENT_COUNT]   = { 10, 22, 35, 48, 53, 60 };
    uint8_t segIdx = 0; while (seq > cum[segIdx]) segIdx++;
    col = segIdx; // column index in left order
    uint8_t prevCum = (segIdx == 0) ? 0 : cum[segIdx - 1];
    uint8_t offset = (uint8_t)(seq - prevCum - 1); // 0-based into segment by sequence progression
    Segment seg = order[segIdx];
    uint8_t len = segmentLength(LEFT_HAND, seg);
    FingerDirection dir = segment_direction(LEFT_HAND, seg);
    // Normalize row from tip (0 at tip increasing toward base) so circle math aligns with your index labels
    if (dir == TIP_TO_BASE) row = offset; else row = (uint8_t)(len - 1 - offset);
  };

  uint8_t col0L, row0L; leftSeqToColRow(31, col0L, row0L);

  // RIGHT HAND: keep previous center near INDEX mid
  auto rightSeqToColRow = [&](uint8_t seq, uint8_t &col, uint8_t &row){
    Segment order[SEGMENT_COUNT] = { THUMB, PALM, INDEX, MIDDLE, RING, PINKY };
    uint8_t cum[SEGMENT_COUNT]   = { 7, 12, 25, 38, 50, 60 };
    uint8_t segIdx = 0; while (seq > cum[segIdx]) segIdx++;
    col = segIdx; // column index in right order
    uint8_t prevCum = (segIdx == 0) ? 0 : cum[segIdx - 1];
    uint8_t offset = (uint8_t)(seq - prevCum - 1);
    Segment seg = order[segIdx];
    uint8_t len = segmentLength(RIGHT_HAND, seg);
    FingerDirection dir = segment_direction(RIGHT_HAND, seg);
    // Normalize row from tip (0 at tip increasing toward base)
    if (dir == TIP_TO_BASE) row = offset; else row = (uint8_t)(len - 1 - offset);
  };

  uint8_t col0R, row0R; rightSeqToColRow(32, col0R, row0R); // anchor at seq 32 (middle segment center)

  // Convert selected grid cells to normalized x,y by averaging a small neighborhood
  auto gridCellToXY = [&](HandSide side, uint8_t col, uint8_t row, float& outX, float& outY) {
    uint16_t idx = hand_xy_to_index(side, col, row);
    if (idx == 0xFFFF) { outX = 0.5f; outY = 0.5f; return; }
    outX = ledX[idx]; outY = ledY[idx];
  };

  float cxL, cyL, cxR, cyR;
  gridCellToXY(LEFT_HAND, col0L, row0L, cxL, cyL);
  gridCellToXY(RIGHT_HAND, col0R, row0R, cxR, cyR);

  // Map distance 5..50 cm → radius 0.05..0.24 (closer = larger)
  auto mapRadius = [](float dCm) -> float {
    if (dCm <= 0) return 0.0f;
    float t = (dCm - 5.0f) / 45.0f; // 0..1
    if (t < 0) t = 0; if (t > 1) t = 1;
    float rMin = 0.05f, rMax = 0.24f;
    return rMin + (1.0f - t) * (rMax - rMin);
  };

  // Grid radius for left based on distance (integer, rows)
  auto mapRadiusGrid = [&](float dCm, uint8_t maxRows) -> uint8_t {
    if (dCm <= 0) return 0;
    float t = (dCm - 5.0f) / 45.0f; if (t < 0) t = 0; if (t > 1) t = 1;
    float r = (1.0f - t) * (float)maxRows * 0.9f; // up to ~90% of rows
    if (r < 1.0f) r = 1.0f;
    if (r > (float)maxRows) r = (float)maxRows;
    return (uint8_t)(r + 0.5f);
  };

  const uint8_t rL_grid = mapRadiusGrid(dLeft, rowsL);
  const uint8_t rR_grid = mapRadiusGrid(dRight, rowsR);

  // Draw black base
  fill_solid(leds, NUM_LEDS, CRGB::Black);

  // Accumulate intensities from both hands, clamp to 255
  for (uint16_t i = 0; i < NUM_LEDS; ++i) {
    float x = ledX[i];
    float y = ledY[i];
    // Left hand discrete circle using sequential indexing
    float iL = 0.0f;
    {
      // Determine if this physical index belongs to left hand
      if (i < NUM_LEDS/2) {
        // Find its sequential label approximately by column/row proximity
        // Iterate s and test circle in grid space (cheap due to small 60)
        for (uint8_t s = 1; s <= 60; ++s) {
          uint8_t c, r; leftSeqToColRow(s, c, r);
          int dc = (int)c - (int)col0L;
          int dr = (int)r - (int)row0L;
          if ((dc*dc + dr*dr) <= (int)rL_grid*(int)rL_grid) {
            uint16_t idx = left_seq_to_index(s);
            if (idx == i) { iL = 1.0f; break; }
          }
        }
      }
    }

    float iR = 0.0f;
    {
      if (i >= NUM_LEDS/2) {
        for (uint8_t s = 1; s <= 60; ++s) {
          uint8_t c, r; rightSeqToColRow(s, c, r);
          int dc = (int)c - (int)col0R;
          int dr = (int)r - (int)row0R;
          if ((dc*dc + dr*dr) <= (int)rR_grid*(int)rR_grid) {
            uint16_t idx = right_seq_to_index(s);
            if (idx == i) { iR = 1.0f; break; }
          }
        }
      }
    }

    uint16_t val = (uint16_t)((iL + iR) * 255.0f);
    if (val > 255) val = 255;
    // Small threshold to avoid lighting thumb/pinky when radius is tiny
    leds[i] = (val > 6) ? CRGB(val, val, val) : CRGB::Black;
  }

  FastLED.show();
}

// =============================================
// MINITHEREMIN – fireball cloud sweeping left pinky -> right pinky
// Controls:
//  - Right hand distance: speed (closer = faster)
//  - Left hand distance: size and heat/brightness (closer = larger, hotter)
// =============================================
static void updateThereminLedVisual(float dLeft, float dRight) {
  // Build spatial map once
  static bool spatialInit = false;
  static float ledX[NUM_LEDS];
  static float ledY[NUM_LEDS];
  static float minX = 0.0f, maxX = 1.0f;
  if (!spatialInit) {
    build_spatial_map(ledX, ledY);
    minX = 1.0f; maxX = 0.0f;
    for (uint16_t i = 0; i < NUM_LEDS; ++i) {
      if (ledX[i] < minX) minX = ledX[i];
      if (ledX[i] > maxX) maxX = ledX[i];
    }
    // Guard against degenerate range
    if (maxX - minX < 0.01f) { minX = 0.0f; maxX = 1.0f; }
    spatialInit = true;
  }

  // Presence flags
  auto inRange = [](float cm) -> bool { return cm >= 5.0f && cm <= 50.0f; };
  bool leftPresent = inRange(dLeft);
  bool rightPresent = inRange(dRight);

  // Map right-hand distance to speed (cycles per second)
  // Closer -> faster woosh
  float speedHz = 0.6f; // default slow drift
  if (rightPresent) {
    float t = (dRight - 5.0f) / 45.0f; if (t < 0) t = 0; if (t > 1) t = 1;
    // 5cm => 1.8 Hz, 50cm => 0.3 Hz
    speedHz = 0.3f + (1.0f - t) * 1.5f;
  }

  // Map left-hand distance to size and heat
  float sigma = 0.10f; // gaussian-ish half-width in x-space
  uint8_t heatBoost = 180; // max extra whiteness
  uint8_t brightnessBase = 140; // base brightness
  if (leftPresent) {
    float t = (dLeft - 5.0f) / 45.0f; if (t < 0) t = 0; if (t > 1) t = 1;
    // 5cm => large radius, hot/bright; 50cm => tight, dimmer
    sigma = 0.05f + (1.0f - t) * 0.13f; // 0.05 .. 0.18
    heatBoost = (uint8_t)(120 + (1.0f - t) * 135.0f); // 120 .. 255
    brightnessBase = (uint8_t)(110 + (1.0f - t) * 145.0f); // 110 .. 255
  } else if (rightPresent) {
    // Solo right-hand control: modest size/heat
    float t = (dRight - 5.0f) / 45.0f; if (t < 0) t = 0; if (t > 1) t = 1;
    sigma = 0.06f + (1.0f - t) * 0.10f;
    heatBoost = (uint8_t)(90 + (1.0f - t) * 110.0f);
    brightnessBase = (uint8_t)(100 + (1.0f - t) * 110.0f);
  }

  // Time-based center position sweeping strictly left->right and wrapping
  static unsigned long startMillis = millis();
  float seconds = (millis() - startMillis) / 1000.0f;
  float range = (maxX - minX);
  float pos01 = seconds * speedHz;
  // Cheap wrap without fmodf to avoid libm heavy calls on AVR
  pos01 = pos01 - (unsigned long)pos01; // keep fractional part only (0..1)
  float centerX = minX + pos01 * range;

  // Fixed RGB endpoints (orange -> yellow) to avoid any HSV wrap toward green
  const CRGB rgbOrange = CRGB(255, 96, 0);
  const CRGB rgbYellow = CRGB(255, 220, 0);

  // Clear frame to avoid leftover colors (keep spectrum strictly in our palette)
  fill_solid(leds, NUM_LEDS, CRGB::Black);

  // Render additive fireball
  for (uint16_t i = 0; i < NUM_LEDS; ++i) {
    float x = ledX[i];
    float y = ledY[i];
    float dx = x - centerX;
    // Normalize distance by sigma
    float nd = fabsf(dx) / sigma;
    // Soft bell curve approximation (no exp): 1 / (1 + a*d^2)
    float atten = 1.0f / (1.0f + 1.8f * nd * nd);
    if (atten < 0.02f) continue; // cheap cutoff

    // No vertical shimmer to ensure perceived motion is horizontal only
    float a = atten;
    if (a > 1.0f) a = 1.0f;

    // Brightness scaling from left-hand control
    uint8_t b = (uint8_t)constrain((int)(brightnessBase * a), 0, 255);

    // Base fire color between orange and yellow based on attenuation (hotter near center)
    uint8_t mix = (uint8_t)constrain((int)(a * 255.0f), 0, 255);
    CRGB col = blend(rgbOrange, rgbYellow, mix);
    // Apply brightness on blended RGB to keep palette bounds
    col.nscale8_video(b);
    // Direct assignment to keep color bounds; no additive blending
    leds[i] = col;
  }

  FastLED.show();
}
// Direct sensor reading with HC-SR04 reset fix (replaces NewPing)
float readDistanceWithReset(int trigPin, int echoPin) {
  // Check if echo pin is stuck HIGH from previous failed reading
  if (digitalRead(echoPin) == HIGH) {
    // Apply the fix: briefly drive echo pin LOW to reset sensor
    pinMode(echoPin, OUTPUT);
    digitalWrite(echoPin, LOW);
    delayMicroseconds(10);
    pinMode(echoPin, INPUT);
    delayMicroseconds(10);
  }
  
  // Send trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Wait for echo
  // Shorter timeout to reduce audio underruns while Mozzi runs
  // Balanced timeout: short enough to protect audio, long enough to detect hands reliably
  unsigned long duration = pulseIn(echoPin, HIGH, 16000UL);
  
  if (duration == 0) {
    return 0; // No echo detected (same as troubleshoot script)
  }
  
  float distance = (duration * 0.0343) / 2.0;
  
  if (distance > MAX_RANGE) {
    return MAX_RANGE + 10; // Return invalid value for out of range
  }
  
  return distance;
}

void updateDistanceSamples(float distance1, float distance2) {
  distance1Samples[sampleIndex] = distance1;
  distance2Samples[sampleIndex] = distance2;
  
  sampleIndex = (sampleIndex + 1) % SAMPLES;
  if (sampleIndex == 0) {
    samplesInitialized = true;
  }
}

float getAveragedDistance(float samples[]) {
  float sum = 0;
  int validSamples = 0;
  
  for (int i = 0; i < SAMPLES; i++) {
    // Include "very close" readings (hands covering sensors) in the valid range
    if (samples[i] >= (MIN_RANGE - 0.5) && samples[i] <= MAX_RANGE) {
      sum += samples[i];
      validSamples++;
    }
  }
  
  // Require at least 3 out of 5 valid samples to prevent false readings
  if (validSamples < 3) {
    return MAX_RANGE + 10; // Return invalid value
  }
  
  return sum / validSamples;
}




void loop() {
  unsigned long currentTime = millis();
  unsigned long sinceLastHand = currentTime - lastHandDetectedTime;
  // Give Mozzi priority to keep the audio buffer filled during ALIEN
  if (currentEffect == ALIEN) {
    // Follow Mozzi pattern: call hook once per loop
    alien_audio_hook();
  }
  
  // Read sensors (alternate per loop when ALIEN is active to reduce blocking)
  static float lastDistance1 = MAX_RANGE + 10;
  static float lastDistance2 = MAX_RANGE + 10;
  static bool readLeftThisLoop = true;
  if (currentEffect == ALIEN) {
    if (readLeftThisLoop) {
      lastDistance1 = readDistanceWithReset(trigPin1, echoPin1);
    } else {
      lastDistance2 = readDistanceWithReset(trigPin2, echoPin2);
    }
    readLeftThisLoop = !readLeftThisLoop;
  } else {
    lastDistance1 = readDistanceWithReset(trigPin1, echoPin1);
    lastDistance2 = readDistanceWithReset(trigPin2, echoPin2);
  }
  float distance1 = lastDistance1;
  float distance2 = lastDistance2;
  // Sanity: if left is stuck exactly at 60 repeatedly, try a quick echo line reset (same fix as readDistanceWithReset)
  static float prevDebugLeft = -999;
  static uint8_t stuckCount = 0;
  if ((int)distance1 == 60 && (int)prevDebugLeft == 60) {
    stuckCount++;
    if (stuckCount >= 2) {
      // Reset echo line pulse if sensor is latched
      pinMode(echoPin1, OUTPUT); digitalWrite(echoPin1, LOW); delayMicroseconds(10); pinMode(echoPin1, INPUT);
      stuckCount = 0;
    }
  } else {
    stuckCount = 0;
  }
  prevDebugLeft = distance1;
  
  updateDistanceSamples(distance1, distance2);

  float avgDistance1, avgDistance2;
  if (samplesInitialized) {
    avgDistance1 = getAveragedDistance(distance1Samples);
    avgDistance2 = getAveragedDistance(distance2Samples);
  } else {
    avgDistance1 = distance1;
    avgDistance2 = distance2;
  }

  bool inRange1, inRange2, handsDetected;
  updateHandsDetectionAndMode(avgDistance1, avgDistance2, inRange1, inRange2, handsDetected, currentTime);

  // Detect rising edge of hand presence to re-initialize the active effect
  static bool prevHandsDetected = false;

  // State machine for mode and effect switching
  if (handsDetected) {
    // Rising edge: set up whichever effect is currently selected by rotation
    if (!prevHandsDetected) {
      // Avoid re-initializing ALIEN (Mozzi) while already running
      if (currentEffect != ALIEN) {
        effect_setup(currentEffect);
        if (currentEffect == DJ_SCRATCH) {
          dj_scratch_start();
        }
      }
    }
    lastHandDetectedTime = currentTime;
    if (currentMode == ATTRACT_MODE) {
      currentMode = INTERACTIVE_MODE;
      // On entering interactive, leave current effect as-is
    }
  } else { // No hands detected
    if (currentMode == INTERACTIVE_MODE && sinceLastHand > INTERACTIVE_TIMEOUT) {
      currentMode = ATTRACT_MODE; // Back to attract after 10s
      lastEffectRotationTime = currentTime; // reset rotation cadence
    } else if (!DISABLE_EFFECT_ROTATION && sinceLastHand > EFFECT_SWITCH_TIMEOUT) {
      // Rotate effects every 5s of no hands
      if (currentTime - lastEffectRotationTime >= EFFECT_SWITCH_TIMEOUT) {
        // Keep hardware silent while rotating through effects with no hands
        effect_disable(currentEffect);
        currentEffect = nextEffect(currentEffect);
        // Defer setup until hands are detected to avoid any audio during idle
        lastEffectRotationTime = currentTime;
        Serial.print(F("Rotated effect to: "));
        Serial.println(effect_name(currentEffect));
      }
    }
  }

  // Update the current effect only if hands detected; otherwise ensure silence
  if (handsDetected) {
    // For ALIEN, updateControl() handles sensing; avoid double work
    if (currentEffect != ALIEN) {
      effect_update(currentEffect, distance1, distance2);
    }
    // DJ LED visual
    if (currentEffect == DJ_SCRATCH && currentMode == INTERACTIVE_MODE) {
      updateDJLedVisual(distance1, distance2);
    } else if (currentEffect == ALIEN && currentMode == INTERACTIVE_MODE) {
      // Re-enable ALIEN LED visual now that timing is stable
      updateAlienLedVisual(distance1, distance2);
      // Mozzi stepped at end of loop() - don't double-call here
    } else if (currentEffect == MINITHEREMIN && currentMode == INTERACTIVE_MODE) {
      updateThereminLedVisual(distance1, distance2);
    } else if (currentEffect == ROBOTS && currentMode == INTERACTIVE_MODE) {
      // Simple per-column rain head tied to envelope (first attempt, smoother look)
      static bool spatialInit = false;
      static float ledX[NUM_LEDS];
      static float ledY[NUM_LEDS];
      if (!spatialInit) { build_spatial_map(ledX, ledY); spatialInit = true; }

      // Decay frame
      for (int i = 0; i < NUM_LEDS; ++i) leds[i].fadeToBlackBy(64);

      uint8_t lvl = robots_get_level(); // 0..255
      CRGB gold = CRGB(255, 215, 0);
      // Map to rows
      uint8_t rowsL = hand_num_rows(LEFT_HAND);
      uint8_t rowsR = hand_num_rows(RIGHT_HAND);
      uint8_t hL = map(lvl, 0, 255, 0, rowsL - 1);
      uint8_t hR = map(lvl, 0, 255, 0, rowsR - 1);

      // Left hand outer→inner
      for (uint8_t col = 0; col < hand_num_columns(); ++col) {
        uint16_t idx = hand_xy_to_index(LEFT_HAND, col, hL);
        if (idx != 0xFFFF) { uint8_t b = (lvl < 32) ? 32 : lvl; CRGB c = gold; c.nscale8_video(b); leds[idx] = c; }
      }
      // Right hand inner→outer
      for (uint8_t col = 0; col < hand_num_columns(); ++col) {
        uint16_t idx = hand_xy_to_index(RIGHT_HAND, col, hR);
        if (idx != 0xFFFF) { uint8_t b = (lvl < 32) ? 32 : lvl; CRGB c = gold; c.nscale8_video(b); leds[idx] = c; }
      }

      FastLED.show();
    }
  } else {
    audio_all_off();
    // On falling edge, ensure current effect is fully disabled as well
    if (prevHandsDetected && !handsDetected) {
      effect_disable(currentEffect);
    }
    // Explicitly hard-mute Mozzi-based ALIEN on hands removed
    if (currentEffect == ALIEN) {
      // Signal ALIEN to mute immediately
      extern void alien_update(float, float);
      alien_update(999.0f, 999.0f); // will cause smoothVol=0 via presence logic
    }
  }

  // Update hand edge tracker
  prevHandsDetected = handsDetected;

  // Update LEDs: if an effect draws its own interactive frame, skip the background pass
  if (!((currentEffect == DJ_SCRATCH || currentEffect == ALIEN || currentEffect == MINITHEREMIN) && currentMode == INTERACTIVE_MODE)) {
    updateLEDs(avgDistance1, avgDistance2, inRange1, inRange2, currentTime);
  }

  // Send JSON data
  static LightMode lastReportedMode = ATTRACT_MODE;
  static bool lastHandsDetected = false;
  
  // Throttle JSON when ALIEN is active to avoid starving audio
  static unsigned long lastJsonMs = 0;
  unsigned long nowMs = millis();
  bool allowJson = true;
  if (currentEffect == ALIEN && currentMode == INTERACTIVE_MODE) {
    if (nowMs - lastJsonMs < 250) allowJson = false; // ~4 Hz max during ALIEN
  }
  if (allowJson && (handsDetected || lastHandsDetected || (currentMode != lastReportedMode))) {
    StaticJsonDocument<400> doc;
    
    doc["left"] = int(avgDistance1);
    doc["right"] = int(avgDistance2);
    doc["hands_detected"] = handsDetected;
    doc["mode"] = (currentMode == ATTRACT_MODE) ? "attract" : "interactive";
    doc["left_in_range"] = inRange1;
    doc["right_in_range"] = inRange2;
    const char* effectName = "dj_scratch";
    if (currentEffect == ALIEN) effectName = "alien";
    else if (currentEffect == ROBOTS) effectName = "robots";
    else if (currentEffect == MINITHEREMIN) effectName = "mini_theremin";
    doc["current_effect"] = effectName;
    doc["raw_distance1"] = distance1;
    doc["raw_distance2"] = distance2;
    
    serializeJson(doc, Serial);
    Serial.println();
    
    lastReportedMode = currentMode;
    lastHandsDetected = handsDetected;
    lastJsonMs = nowMs;
  }

  // Keep loop cooperative when not ALIEN
  if (currentEffect != ALIEN) {
    delay(20);
  }
}