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

#include <NewTone.h>
#define FASTLED_FORCE_BITBANG
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
const int echoPin1 = 11;
const int trigPin2 = 5;  // Right sensor
const int echoPin2 = 6; // Corrected from 12 to match hardware wiring and avoid audio conflict
// Distance-based detection constants (in cm)
const float MIN_RANGE = 5.0;                  // MinimTheremin effect initializedum detection range in cm
const float MAX_RANGE = 50.0;                 // Maximum detection range in cm

const unsigned long SENSOR_TIMEOUT = 30000;   // Sensor timeout in microseconds

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

EffectType currentEffect = DJ_SCRATCH; // Start with DJ Scratch

const unsigned long INTERACTIVE_TIMEOUT = 10000; // 10s no hands -> back to attract
const unsigned long EFFECT_SWITCH_TIMEOUT = 5000; // 5s no hands -> rotate effect
// Temporary: disable effect rotation to focus on DJ visuals
const bool DISABLE_EFFECT_ROTATION = true;
// Temporary: force ALIEN-only testing (no effect rotation, start in ALIEN)
const bool TEST_ALIEN_ONLY = true;


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
  // Stop NewTone
  noNewTone(12);
  // Disable DJ scratch Timer1 if active
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 &= ~_BV(OCIE1B);
  // Tri-state the pin
  pinMode(12, INPUT);
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
  unsigned long duration = pulseIn(echoPin, HIGH, 50000UL); // 50ms timeout
  
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
  
  // Read sensors
  float distance1 = readDistanceWithReset(trigPin1, echoPin1);
  float distance2 = readDistanceWithReset(trigPin2, echoPin2);
  
  updateDistanceSamples(distance1, distance2);

  float avgDistance1, avgDistance2;
  if (samplesInitialized) {
    avgDistance1 = getAveragedDistance(distance1Samples);
    avgDistance2 = getAveragedDistance(distance2Samples);
  } else {
    avgDistance1 = distance1;
    avgDistance2 = distance2;
  }

  bool inRange1 = (avgDistance1 >= MIN_RANGE && avgDistance1 <= MAX_RANGE);
  bool inRange2 = (avgDistance2 >= MIN_RANGE && avgDistance2 <= MAX_RANGE);
  bool handsDetected = inRange1 || inRange2;

  // Detect rising edge of hand presence to re-initialize the active effect
  static bool prevHandsDetected = false;

  // State machine for mode and effect switching
  if (handsDetected) {
    // Rising edge: set up whichever effect is currently selected by rotation
    if (!prevHandsDetected) {
      effect_setup(currentEffect);
      if (currentEffect == DJ_SCRATCH) {
        dj_scratch_start();
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
    effect_update(currentEffect, distance1, distance2);
    // DJ LED visual
    if (currentEffect == DJ_SCRATCH && currentMode == INTERACTIVE_MODE) {
      updateDJLedVisual(distance1, distance2);
    } else if (currentEffect == ALIEN && currentMode == INTERACTIVE_MODE) {
      updateAlienLedVisual(distance1, distance2);
    }
  } else {
    audio_all_off();
    // On falling edge, ensure current effect is fully disabled as well
    if (prevHandsDetected && !handsDetected) {
      effect_disable(currentEffect);
    }
  }

  // Update hand edge tracker
  prevHandsDetected = handsDetected;

  // Update LEDs: if DJ interactive, the DJ visual already drew the frame
  if (!((currentEffect == DJ_SCRATCH || currentEffect == ALIEN) && currentMode == INTERACTIVE_MODE)) {
    updateLEDs(avgDistance1, avgDistance2, inRange1, inRange2, currentTime);
  }

  // Send JSON data
  static LightMode lastReportedMode = ATTRACT_MODE;
  static bool lastHandsDetected = false;
  
  if (handsDetected || lastHandsDetected || (currentMode != lastReportedMode)) {
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
  }

  delay(20);
}