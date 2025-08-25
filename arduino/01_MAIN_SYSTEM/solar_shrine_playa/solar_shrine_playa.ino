/****************************************************************************
 Solar Shrine Playa - Modular Audio Effect System
 Cycles between DJ Scratch, Vocoder Robot, Robots, and Theremin effects every 5 seconds
 
 Effects included:
 - DJ Scratch: PROGMEM audio playback with scratching effects
 - Vocoder Robot: Classic robot voice effect using DJ audio as input
 - Robots: Musical theremin with pentatonic scale
 - Theremin: Automatic sawtooth wave pattern
*********************************************************************************/

#include <avr/pgmspace.h>
#include "audio_data.h"

#include "DjScratch.h"
#include "ScaleEffect.h"
#include "RobotsEffect.h"
#include "ThereminEffect.h"

// =============================================================================
// MOZZI CONFIGURATION
// =============================================================================
#include <MozziConfigValues.h>
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_2PIN_PWM

// Custom pin configuration to free up pin 11 for sensor
// Default: pin 11 (high) + pin 12 (low)
// Custom:  pin 12 (high) + pin 13 (low)
#define MOZZI_AUDIO_PIN_1 12
#define MOZZI_AUDIO_PIN_1_REGISTER OCR1B
#define MOZZI_AUDIO_PIN_1_LOW 13
#define MOZZI_AUDIO_PIN_1_LOW_REGISTER OCR1C

#include <MozziGuts.h>
#include <Oscil.h>
#include <LowPassFilter.h>
#include <RollingAverage.h>
#include <tables/sin2048_int8.h>
#include <tables/cos2048_int8.h>
#include <tables/triangle2048_int8.h>
#include <tables/triangle_valve_2048_int8.h>
#include <tables/saw2048_int8.h>
#include <math.h>
#include <ArduinoJson.h>

// =============================================================================
// LED SYSTEM (FastLED + LED mapping from modular sketch)
// =============================================================================
#define FASTLED_ALLOW_INTERRUPTS 1
#include <FastLED.h>
#define LED_PIN 3
#define NUM_LEDS 120
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
#include "led_mapping.h"

// LED visual prototypes
static void updateDJLedVisual(float dLeft, float dRight);
static void updateAlienLedVisual(float dLeft, float dRight);
static void updateThereminLedVisual(float dLeft, float dRight);
static void updateRobotsLedVisual(uint8_t level);

#define CONTROL_RATE 128

// =============================================================================
// DETECTION / AVERAGING (match modular sketch semantics)
// =============================================================================
const float MIN_RANGE = 5.0;   // in cm
const float MAX_RANGE = 50.0;  // in cm
const int SAMPLES = 5;
static float distance1Samples[SAMPLES];
static float distance2Samples[SAMPLES];
static int sampleIndex = 0;
static bool samplesInitialized = false;
static float lastDistance1 = MAX_RANGE + 10; // raw
static float lastDistance2 = MAX_RANGE + 10; // raw

static void updateDistanceSamples(float d1, float d2) {
  distance1Samples[sampleIndex] = d1;
  distance2Samples[sampleIndex] = d2;
  sampleIndex = (sampleIndex + 1) % SAMPLES;
  if (sampleIndex == 0) samplesInitialized = true;
}

static float getAveragedDistance(float samples[]) {
  float sum = 0; int valid = 0;
  for (int i = 0; i < SAMPLES; i++) {
    if (samples[i] >= (MIN_RANGE - 0.5f) && samples[i] <= MAX_RANGE) { sum += samples[i]; valid++; }
  }
  if (valid < 3) return MAX_RANGE + 10; // invalid
  return sum / valid;
}

// =============================================================================
// AUDIO MODE SYSTEM
// =============================================================================
enum AudioMode {
  MODE_DJ_SCRATCH = 0,
  MODE_VOCODER_ROBOT = 1,
  MODE_MOZZI_ROBOTS = 2,
  MODE_MOZZI_THEREMIN = 3
};

AudioMode currentMode = MODE_DJ_SCRATCH;
const unsigned long MODE_DURATION = 5000; // 5 seconds per mode
unsigned long lastModeChange = 0;

// =============================================================================
// SENSOR SYSTEM
// =============================================================================
#define TRIG1 10  // Left sensor trigger
#define ECHO1 11  // Left sensor echo
#define TRIG2 5   // Right sensor trigger
#define ECHO2 6   // Right sensor echo

// Note: DJ Scratch effect variables are now in the modular audio system section above

// =============================================================================
// SENSOR READING FUNCTIONS
// =============================================================================
float readSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Shorter timeout to avoid starving audio like modular sketch
  long duration = pulseIn(echoPin, HIGH, 16000);
  if (duration == 0) return 999; // No echo received
  
  float distance = duration * 0.034 / 2;
  return distance;
}

bool isHandPresent(float distance) {
  return (distance > 1 && distance < 20);
}

// =============================================================================
// SETUP FUNCTION
// =============================================================================
void setup() {
  Serial.begin(9600);
  // JSON-only output expected by TouchDesigner; avoid non-JSON chatter
  
  // Initialize sensor pins
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  
  // Initialize LEDs (match modular sketch)
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(150);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  // Initialize all effect modules
  DjScratch::setup();
  ScaleEffect::setup();
  RobotsEffect::setup();
  ThereminEffect::setup();
  
  // Start in DJ Scratch mode
  DjScratch::enter();
  lastModeChange = millis();

  // Initialize sample buffers
  for (int i = 0; i < SAMPLES; i++) { distance1Samples[i] = MAX_RANGE + 10; distance2Samples[i] = MAX_RANGE + 10; }
}

// =============================================================================
// DJ SCRATCH TIMER1 ISR
// =============================================================================
ISR(TIMER1_COMPB_vect) {
  if (currentMode == MODE_DJ_SCRATCH) {
    DjScratch::handleISR();
  } else if (currentMode == MODE_VOCODER_ROBOT) {
    // In vocoder mode, we still need to feed DJ audio to the vocoder
    // but we don't output it directly - the vocoder processes it
    DjScratch::handleISR();
    OCR1B = 200; // Silence DJ output, vocoder handles audio output
  } else {
    OCR1B = 200; // Silence when not in DJ mode
  }
}

// =============================================================================
// LED VISUALS (ported from modular sketch)
// =============================================================================

static void updateDJLedVisual(float dLeft, float dRight) {
  // Map right-hand distance to wave speed (closer = faster), 5..50 cm
  uint8_t speed = 60; // default
  if (dRight >= 5 && dRight <= 50) {
    speed = (uint8_t)map((int)dRight, 5, 50, 160, 40);
  }

  static bool spatialInit = false;
  static float ledX[NUM_LEDS];
  static float ledY[NUM_LEDS];
  if (!spatialInit) { build_spatial_map(ledX, ledY); spatialInit = true; }

  uint8_t phase = beat8(speed);
  const CRGB startColor = CRGB(48, 0, 96);    // dark purple
  const CRGB endColor   = CRGB(0, 255, 255);  // neon blue

  for (uint16_t idx = 0; idx < NUM_LEDS; ++idx) {
    float xNorm = ledX[idx];
    float yNorm = ledY[idx];
    uint8_t x = (uint8_t)(xNorm * 255.0f);
    uint8_t y = (uint8_t)(yNorm * 64.0f);
    uint8_t s = sin8(x - phase + y);
    uint8_t b = scale8(s, 220);
    CRGB col = blend(startColor, endColor, s);
    uint8_t nb = (b < 16) ? 16 : b;
    col.nscale8_video(nb);
    leds[idx] = col;
  }

  FastLED.show();
}

static void updateAlienLedVisual(float dLeft, float dRight) {
  static bool spatialInit = false;
  static float ledX[NUM_LEDS];
  static float ledY[NUM_LEDS];
  if (!spatialInit) { build_spatial_map(ledX, ledY); spatialInit = true; }

  const uint8_t rowsL = hand_num_rows(LEFT_HAND);
  const uint8_t rowsR = hand_num_rows(RIGHT_HAND);

  auto leftSeqToColRow = [&](uint8_t seq, uint8_t &col, uint8_t &row){
    Segment order[SEGMENT_COUNT] = { PINKY, RING, MIDDLE, INDEX, PALM, THUMB };
    uint8_t cum[SEGMENT_COUNT]   = { 10, 22, 35, 48, 53, 60 };
    uint8_t segIdx = 0; while (seq > cum[segIdx]) segIdx++;
    col = segIdx;
    uint8_t prevCum = (segIdx == 0) ? 0 : cum[segIdx - 1];
    uint8_t offset = (uint8_t)(seq - prevCum - 1);
    Segment seg = order[segIdx];
    uint8_t len = segmentLength(LEFT_HAND, seg);
    FingerDirection dir = segment_direction(LEFT_HAND, seg);
    if (dir == TIP_TO_BASE) row = offset; else row = (uint8_t)(len - 1 - offset);
  };

  uint8_t col0L, row0L; leftSeqToColRow(31, col0L, row0L);

  auto rightSeqToColRow = [&](uint8_t seq, uint8_t &col, uint8_t &row){
    Segment order[SEGMENT_COUNT] = { THUMB, PALM, INDEX, MIDDLE, RING, PINKY };
    uint8_t cum[SEGMENT_COUNT]   = { 7, 12, 25, 38, 50, 60 };
    uint8_t segIdx = 0; while (seq > cum[segIdx]) segIdx++;
    col = segIdx;
    uint8_t prevCum = (segIdx == 0) ? 0 : cum[segIdx - 1];
    uint8_t offset = (uint8_t)(seq - prevCum - 1);
    Segment seg = order[segIdx];
    uint8_t len = segmentLength(RIGHT_HAND, seg);
    FingerDirection dir = segment_direction(RIGHT_HAND, seg);
    if (dir == TIP_TO_BASE) row = offset; else row = (uint8_t)(len - 1 - offset);
  };

  uint8_t col0R, row0R; rightSeqToColRow(32, col0R, row0R);

  auto mapRadiusGrid = [&](float dCm, uint8_t maxRows) -> uint8_t {
    if (dCm <= 0) return 0;
    float t = (dCm - 5.0f) / 45.0f; if (t < 0) t = 0; if (t > 1) t = 1;
    float r = (1.0f - t) * (float)maxRows * 0.9f;
    if (r < 1.0f) r = 1.0f; if (r > (float)maxRows) r = (float)maxRows;
    return (uint8_t)(r + 0.5f);
  };

  const uint8_t rL_grid = mapRadiusGrid(dLeft, rowsL);
  const uint8_t rR_grid = mapRadiusGrid(dRight, rowsR);

  fill_solid(leds, NUM_LEDS, CRGB::Black);

  for (uint16_t i = 0; i < NUM_LEDS; ++i) {
    float iL = 0.0f, iR = 0.0f;
    if (i < NUM_LEDS/2) {
      for (uint8_t s = 1; s <= 60; ++s) {
        uint8_t c, r; leftSeqToColRow(s, c, r);
        int dc = (int)c - (int)col0L; int dr = (int)r - (int)row0L;
        if ((dc*dc + dr*dr) <= (int)rL_grid*(int)rL_grid) {
          uint16_t idx = left_seq_to_index(s); if (idx == i) { iL = 1.0f; break; }
        }
      }
    } else {
      for (uint8_t s = 1; s <= 60; ++s) {
        uint8_t c, r; rightSeqToColRow(s, c, r);
        int dc = (int)c - (int)col0R; int dr = (int)r - (int)row0R;
        if ((dc*dc + dr*dr) <= (int)rR_grid*(int)rR_grid) {
          uint16_t idx = right_seq_to_index(s); if (idx == i) { iR = 1.0f; break; }
        }
      }
    }
    uint16_t val = (uint16_t)((iL + iR) * 255.0f);
    if (val > 255) val = 255;
    leds[i] = (val > 6) ? CRGB(val, val, val) : CRGB::Black;
  }

  FastLED.show();
}

static void updateThereminLedVisual(float dLeft, float dRight) {
  static bool spatialInit = false;
  static float ledX[NUM_LEDS];
  static float ledY[NUM_LEDS];
  static float minX = 0.0f, maxX = 1.0f;
  if (!spatialInit) {
    build_spatial_map(ledX, ledY);
    minX = 1.0f; maxX = 0.0f;
    for (uint16_t i = 0; i < NUM_LEDS; ++i) { if (ledX[i] < minX) minX = ledX[i]; if (ledX[i] > maxX) maxX = ledX[i]; }
    if (maxX - minX < 0.01f) { minX = 0.0f; maxX = 1.0f; }
    spatialInit = true;
  }

  auto inRange = [](float cm) -> bool { return cm >= 5.0f && cm <= 50.0f; };
  bool leftPresent = inRange(dLeft);
  bool rightPresent = inRange(dRight);

  float speedHz = 0.6f;
  if (rightPresent) { float t = (dRight - 5.0f) / 45.0f; if (t < 0) t = 0; if (t > 1) t = 1; speedHz = 0.3f + (1.0f - t) * 1.5f; }

  float sigma = 0.10f; uint8_t brightnessBase = 140;
  if (leftPresent) {
    float t = (dLeft - 5.0f) / 45.0f; if (t < 0) t = 0; if (t > 1) t = 1;
    sigma = 0.05f + (1.0f - t) * 0.13f;
    brightnessBase = (uint8_t)(110 + (1.0f - t) * 145.0f);
  } else if (rightPresent) {
    float t = (dRight - 5.0f) / 45.0f; if (t < 0) t = 0; if (t > 1) t = 1;
    sigma = 0.06f + (1.0f - t) * 0.10f;
    brightnessBase = (uint8_t)(100 + (1.0f - t) * 110.0f);
  }

  static unsigned long startMillis = millis();
  float seconds = (millis() - startMillis) / 1000.0f;
  float range = (maxX - minX);
  float pos01 = seconds * speedHz; pos01 = pos01 - (unsigned long)pos01;
  float centerX = minX + pos01 * range;

  const CRGB rgbOrange = CRGB(255, 96, 0);
  const CRGB rgbYellow = CRGB(255, 220, 0);

  fill_solid(leds, NUM_LEDS, CRGB::Black);
  for (uint16_t i = 0; i < NUM_LEDS; ++i) {
    float dx = ledX[i] - centerX;
    float nd = fabsf(dx) / sigma;
    float atten = 1.0f / (1.0f + 1.8f * nd * nd);
    if (atten < 0.02f) continue;
    float a = atten; if (a > 1.0f) a = 1.0f;
    uint8_t b = (uint8_t)constrain((int)(brightnessBase * a), 0, 255);
    uint8_t mix = (uint8_t)constrain((int)(a * 255.0f), 0, 255);
    CRGB col = blend(rgbOrange, rgbYellow, mix);
    col.nscale8_video(b);
    leds[i] = col;
  }

  FastLED.show();
}

static void updateRobotsLedVisual(uint8_t lvl) {
  static bool spatialInit = false;
  static float ledX[NUM_LEDS];
  static float ledY[NUM_LEDS];
  if (!spatialInit) { build_spatial_map(ledX, ledY); spatialInit = true; }

  for (int i = 0; i < NUM_LEDS; ++i) leds[i].fadeToBlackBy(64);

  CRGB gold = CRGB(255, 215, 0);
  uint8_t rowsL = hand_num_rows(LEFT_HAND);
  uint8_t rowsR = hand_num_rows(RIGHT_HAND);
  uint8_t hL = map(lvl, 0, 255, 0, rowsL - 1);
  uint8_t hR = map(lvl, 0, 255, 0, rowsR - 1);

  for (uint8_t col = 0; col < hand_num_columns(); ++col) {
    uint16_t idx = hand_xy_to_index(LEFT_HAND, col, hL);
    if (idx != 0xFFFF) { uint8_t b = (lvl < 32) ? 32 : lvl; CRGB c = gold; c.nscale8_video(b); leds[idx] = c; }
  }
  for (uint8_t col = 0; col < hand_num_columns(); ++col) {
    uint16_t idx = hand_xy_to_index(RIGHT_HAND, col, hR);
    if (idx != 0xFFFF) { uint8_t b = (lvl < 32) ? 32 : lvl; CRGB c = gold; c.nscale8_video(b); leds[idx] = c; }
  }

  FastLED.show();
}

// =============================================================================
// MODE SWITCHING FUNCTIONS
// =============================================================================

// =============================================================================
// MODE SWITCHING FUNCTIONS
// =============================================================================



// =============================================================================
// MAIN LOOP FUNCTION
// =============================================================================
void loop() {
  // Check if it's time to switch modes
  if (millis() - lastModeChange >= MODE_DURATION) {
    switchToNextMode();
  }
  
  // CRITICAL: audioHook() must be called EVERY loop iteration for Mozzi
  // This is the key difference from our previous implementation
  if (currentMode == MODE_VOCODER_ROBOT || currentMode == MODE_MOZZI_ROBOTS || currentMode == MODE_MOZZI_THEREMIN) {
    audioHook();
  } else if (currentMode == MODE_DJ_SCRATCH) {
    // Handle DJ scratch controls (Mozzi handles its own controls in updateControl)
    static unsigned long lastSensorRead = 0;
    if (millis() - lastSensorRead >= 30) {  // Read sensors every 30ms for DJ scratch
      float d1 = readSensor(TRIG1, ECHO1);
      float d2 = readSensor(TRIG2, ECHO2);
      
      bool leftHand = isHandPresent(d1);
      bool rightHand = isHandPresent(d2);
      
      DjScratch::update(leftHand, rightHand, d1, d2);
      // LED visual for DJ mode
      updateDJLedVisual(d1, d2);
      // store raw for JSON
      lastDistance1 = d1; lastDistance2 = d2;
      updateDistanceSamples(d1, d2);
      lastSensorRead = millis();
    }
    
    delay(5);  // Small delay for DJ scratch mode
  }

  // Regular sensor sampling for JSON in Mozzi modes as well (lightweight cadence)
  static unsigned long lastJsonSample = 0;
  if (millis() - lastJsonSample >= 60) {
    float d1 = readSensor(TRIG1, ECHO1);
    float d2 = readSensor(TRIG2, ECHO2);
    lastDistance1 = d1; lastDistance2 = d2;
    updateDistanceSamples(d1, d2);
    lastJsonSample = millis();
  }

  // Build JSON identical to modular sketch
  float avgDistance1 = samplesInitialized ? getAveragedDistance(distance1Samples) : lastDistance1;
  float avgDistance2 = samplesInitialized ? getAveragedDistance(distance2Samples) : lastDistance2;
  bool inRange1 = (avgDistance1 >= MIN_RANGE && avgDistance1 <= MAX_RANGE);
  bool inRange2 = (avgDistance2 >= MIN_RANGE && avgDistance2 <= MAX_RANGE);
  bool handsDetected = inRange1 || inRange2;

  // Throttle during heavy Mozzi modes
  static unsigned long lastJsonMs = 0; unsigned long nowMs = millis(); bool allowJson = true;
  if ((currentMode == MODE_VOCODER_ROBOT || currentMode == MODE_MOZZI_ROBOTS || currentMode == MODE_MOZZI_THEREMIN)) {
    if (nowMs - lastJsonMs < 250) allowJson = false; // ~4 Hz
  }
  static bool lastHandsDetected = false; static int lastReportedModeInt = -1;
  auto modeString = handsDetected ? "interactive" : "attract";
  int modeInt = handsDetected ? 1 : 0;
  if (allowJson && (handsDetected || lastHandsDetected || (modeInt != lastReportedModeInt))) {
    StaticJsonDocument<400> doc;
    doc["left"] = int(avgDistance1);
    doc["right"] = int(avgDistance2);
    doc["hands_detected"] = handsDetected;
    doc["mode"] = modeString;
    doc["left_in_range"] = inRange1;
    doc["right_in_range"] = inRange2;
    const char* effectName = "dj_scratch";
    if (currentMode == MODE_VOCODER_ROBOT) effectName = "alien";
    else if (currentMode == MODE_MOZZI_ROBOTS) effectName = "robots";
    else if (currentMode == MODE_MOZZI_THEREMIN) effectName = "mini_theremin";
    doc["current_effect"] = effectName;
    doc["raw_distance1"] = lastDistance1;
    doc["raw_distance2"] = lastDistance2;
    serializeJson(doc, Serial);
    Serial.println();
    lastHandsDetected = handsDetected; lastReportedModeInt = modeInt; lastJsonMs = nowMs;
  }
}

void switchToNextMode() {
  AudioMode previousMode = currentMode;

  // Disable current mode
  switch (previousMode) {
    case MODE_DJ_SCRATCH:
      DjScratch::exit();
      break;
    case MODE_VOCODER_ROBOT:
      ScaleEffect::exit();
      break;
    case MODE_MOZZI_ROBOTS:
      RobotsEffect::exit();
      break;
    case MODE_MOZZI_THEREMIN:
      ThereminEffect::exit();
      break;
  }

  // Stop Mozzi if we are leaving a Mozzi mode and entering a non-Mozzi mode
  bool wasMozzi = (previousMode == MODE_VOCODER_ROBOT || previousMode == MODE_MOZZI_ROBOTS || previousMode == MODE_MOZZI_THEREMIN);
  currentMode = (AudioMode)((currentMode + 1) % 4);
  bool isMozzi = (currentMode == MODE_VOCODER_ROBOT || currentMode == MODE_MOZZI_ROBOTS || currentMode == MODE_MOZZI_THEREMIN);

  if (wasMozzi && !isMozzi) {
    stopMozzi();
  }

  lastModeChange = millis();

  // Enable new mode
  switch (currentMode) {
    case MODE_DJ_SCRATCH:
      DjScratch::enter();
      break;
    case MODE_VOCODER_ROBOT:
      ScaleEffect::enter();
      break;
    case MODE_MOZZI_ROBOTS:
      RobotsEffect::enter();
      break;
    case MODE_MOZZI_THEREMIN:
      ThereminEffect::enter();
      break;
  }

  // Start Mozzi if we are entering a Mozzi mode from a non-Mozzi mode
  if (!wasMozzi && isMozzi) {
    startMozzi();
  }
}

// =============================================================================
// MOZZI UPDATE CONTROL FUNCTION (Only called when in Mozzi modes)
// =============================================================================
void updateControl() {
  // Read sensors
  float d1 = readSensor(TRIG1, ECHO1); // Left hand
  float d2 = readSensor(TRIG2, ECHO2); // Right hand
  bool leftHand = isHandPresent(d1);
  bool rightHand = isHandPresent(d2);

  // Add this for debugging
  //Serial.print("Mode: ");
  //Serial.print(currentMode);
  //Serial.print(" | Left Sensor: ");
  //Serial.print(d1);
  //Serial.print(" cm, Hand: ");
  //Serial.print(leftHand);
  //Serial.print(" | Right Sensor: ");
  //Serial.print(d2);
  //Serial.print(" cm, Hand: ");
  //Serial.println(rightHand);
  
  // Update current Mozzi effect
  switch (currentMode) {
    case MODE_VOCODER_ROBOT:
      ScaleEffect::update(leftHand, rightHand, d1, d2);
      // LED: Alien orb visual
      updateAlienLedVisual(d1, d2);
      break;
    case MODE_MOZZI_ROBOTS:
      RobotsEffect::update(leftHand, rightHand, d1, d2);
      // LED: robots rain visual driven by envelope level
      updateRobotsLedVisual((uint8_t)constrain(RobotsEffect::level(), 0, 255));
      break;
    case MODE_MOZZI_THEREMIN:
      ThereminEffect::update(leftHand, rightHand, d1, d2);
      // LED: theremin fireball sweep
      updateThereminLedVisual(d1, d2);
      break;
    default:
      // Should not reach here in Mozzi modes
      break;
  }
}

// =============================================================================
// INDIVIDUAL EFFECT UPDATE FUNCTIONS
// =============================================================================


// =============================================================================
// MOZZI AUDIO OUTPUT FUNCTION (Only for Mozzi modes)
// =============================================================================
int updateAudio() {
  return audioOutput();
}

int audioOutput() {
  switch (currentMode) {
    case MODE_VOCODER_ROBOT:
      return ScaleEffect::audio();
    case MODE_MOZZI_ROBOTS:
      return RobotsEffect::audio();
    case MODE_MOZZI_THEREMIN:
      return ThereminEffect::audio();
    default:
      return 0; // DJ Scratch uses Timer1 ISR, not Mozzi
  }
}
