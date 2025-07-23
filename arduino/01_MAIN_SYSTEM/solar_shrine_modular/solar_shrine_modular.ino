/*
 * Solar Shrine - Modular Integration System
 * Combines dual-mode lighting, sensor detection, and DJ scratch audio
 * Designed as the main hub for modular functionality expansion
 * 
 * Hardware (Arduino Mega 2560):
 * - 2x HC-SR04 ultrasonic sensors (pins 5,6,10,11)
 * - WS2812B/WS2815 LED strip (pin 3)
 * - Audio: Pin 12 → 1K resistor → Amplifier right channel (left channel + ground → ground)
 * - WWZMDiB XH-M543 amplifier + Dayton Audio DAEX32QMB-4 exciter
 * 
 * Features:
 * - Attract/Interactive LED modes
 * - Hand detection with averaging
 * - DJ Scratch audio control (Left hand = Play/Pause, Right hand = Scratch)
 * - TouchDesigner JSON integration
 * - Modular architecture for future expansion
 * 
 * Libraries Required:
 * - FastLED
 * - ArduinoJson
 * - NewPing
 */

#include <ArduinoJson.h>
#include <FastLED.h>
#include <NewPing.h>
#include <avr/pgmspace.h>

// Audio data - include the DJ scratch audio
#include "audio_data.h"

// =============================================================================
// HARDWARE CONFIGURATION
// =============================================================================

// Sensor pins - Arduino Mega 2560
const int trigPin1 = 10;  // Left sensor trigger
const int echoPin1 = 11;  // Left sensor echo
const int trigPin2 = 5;   // Right sensor trigger
const int echoPin2 = 6;   // Right sensor echo

// NewPing sensor objects
NewPing sonar1(trigPin1, echoPin1, 200); // Left sensor, max 200cm
NewPing sonar2(trigPin2, echoPin2, 200); // Right sensor, max 200cm

// LED strip configuration
#define LED_PIN 3
#define NUM_LEDS 120
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

// Audio pin - Arduino Mega 2560 Timer1 OC1B
const int AUDIO_PIN = 12;

// =============================================================================
// SYSTEM CONSTANTS
// =============================================================================

// Range constants (in cm)
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

// Mode constants
enum LightMode {
  ATTRACT_MODE,
  INTERACTIVE_MODE
};

// Attract mode timing - optimized for smooth transitions
const float ATTRACT_PERIOD = 4000.0;  // 4 seconds for slightly faster, smoother cycle
const unsigned long INTERACTIVE_TIMEOUT = 10000;  // 10 seconds

// Hand detection averaging
const int SAMPLES = 5;

// =============================================================================
// SYSTEM STATE VARIABLES
// =============================================================================

// LED system state
LightMode currentMode = ATTRACT_MODE;
unsigned long lastHandDetectedTime = 0;
float attractPhaseOffset = 0.0;
bool usePhaseOffset = false;
CRGB lastLeftColor = CRGB::Yellow;
CRGB lastRightColor = CRGB::Yellow;

// Sensor averaging
float distance1Samples[SAMPLES];
float distance2Samples[SAMPLES];
int sampleIndex = 0;
bool samplesInitialized = false;

// Audio system state (DJ Scratch)
volatile int32_t audioSampleIndex = 0;
volatile uint8_t audioSampleCounter = 0;
volatile uint8_t playState = 0;
volatile uint8_t playbackSpeed = 5;
volatile int8_t scratchSpeed = 1;
volatile bool isScratchMode = false;

// Audio control state
bool audioActive = false;
unsigned long lastAudioStateChange = 0;

// Performance optimization variables
unsigned long lastLEDUpdate = 0;
const unsigned long LED_UPDATE_INTERVAL = 33;  // ~30Hz for smooth visuals
volatile bool audioInterruptBusy = false;

// =============================================================================
// AUDIO SYSTEM (DJ SCRATCH)
// =============================================================================

void setupAudioSystem() {
  pinMode(AUDIO_PIN, OUTPUT);  // Pin 12 (Timer1 OC1B)
  
  // Timer1 PWM - configured for OC1B (pin 12)
  TCCR1A = _BV(COM1B1) | _BV(WGM11);  // Clear OC1B on compare match, Fast PWM
  TCCR1B = _BV(WGM13) | _BV(CS10);    // Fast PWM, no prescaler
  ICR1 = 399;                         // 20kHz frequency
  OCR1B = ICR1 / 2;                   // 50% duty cycle (silence)
  TIMSK1 = _BV(OCIE1B);               // Enable Timer1 Compare B interrupt
  
  Serial.println(F("Audio System Ready - DJ Scratch Mode"));
}

// Timer1 Compare B interrupt - Audio generation (Optimized)
ISR(TIMER1_COMPB_vect) {
  // Quick exit if audio is off to minimize interrupt time
  if (playState == 0) {
    OCR1B = ICR1 / 2;  // Silence
    return;
  }
  
  audioInterruptBusy = true;  // Signal that interrupt is active
  
  audioSampleCounter++;
  uint8_t currentSpeed = isScratchMode ? 2 : playbackSpeed;
  
  if (audioSampleCounter >= currentSpeed) {
    audioSampleCounter = 0;
    
    // Bounds checking with faster logic
    if (audioSampleIndex < 0) audioSampleIndex = 0;
    if (audioSampleIndex >= AUDIO_SAMPLE_COUNT) audioSampleIndex = 0;
    
    // Optimized sample reading and processing
    uint8_t sample = pgm_read_byte(&audioData[audioSampleIndex]);
    int16_t amp = ((int16_t)sample - 128) << 2;  // Faster bit shift instead of multiply
    amp = constrain(amp, -128, 127);
    
    OCR1B = ((uint32_t)(amp + 128) * ICR1) >> 8;  // Faster bit shift division
    
    // Sample index advancement
    if (isScratchMode) {
      audioSampleIndex += scratchSpeed;
      if (audioSampleIndex < 0) audioSampleIndex = AUDIO_SAMPLE_COUNT - 1;
      if (audioSampleIndex >= AUDIO_SAMPLE_COUNT) audioSampleIndex = 0;
    } else {
      audioSampleIndex++;
      if (audioSampleIndex >= AUDIO_SAMPLE_COUNT) audioSampleIndex = 0;
    }
  }
  
  audioInterruptBusy = false;  // Signal that interrupt is complete
}

// =============================================================================
// SENSOR SYSTEM
// =============================================================================

float readDistanceWithReset(int trigPin, int echoPin) {
  // Optimized sensor reading with reduced blocking time
  
  // Quick echo pin reset if needed
  if (digitalRead(echoPin) == HIGH) {
    pinMode(echoPin, OUTPUT);
    digitalWrite(echoPin, LOW);
    delayMicroseconds(5);  // Reduced from 10
    pinMode(echoPin, INPUT);
    delayMicroseconds(5);  // Reduced from 10
  }
  
  // Fast trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reduced timeout to prevent long blocking - critical for smooth LEDs
  unsigned long duration = pulseIn(echoPin, HIGH, 20000UL); // 20ms timeout (was 50ms)
  
  if (duration == 0) {
    return MAX_RANGE + 10; // Return out-of-range instead of 0
  }
  
  // Faster distance calculation using bit shift
  float distance = (duration * 343) >> 11;  // Equivalent to * 0.0343 / 2, but faster
  
  if (distance > MAX_RANGE) {
    return MAX_RANGE + 10;
  }
  
  return distance;
}

float readDistanceNewPing(NewPing &sensor) {
  // Use ping_cm with reduced max distance for faster response
  unsigned int distance = sensor.ping_cm(MAX_RANGE + 5);  // Limit max distance to reduce blocking
  if (distance == 0) {
    return MAX_RANGE + 10;  // Return out-of-range for consistency
  }
  if (distance > MAX_RANGE) {
    return MAX_RANGE + 10;
  }
  return (float)distance;
}

void updateDistanceSamples(float distance1, float distance2) {
  distance1Samples[sampleIndex] = distance1;
  distance2Samples[sampleIndex] = distance2;
  
  sampleIndex = (sampleIndex + 1) % SAMPLES;
  
  if (!samplesInitialized && sampleIndex == 0) {
    samplesInitialized = true;
  }
}

float getAveragedDistance(float samples[]) {
  float sum = 0;
  int validSamples = 0;
  
  for (int i = 0; i < SAMPLES; i++) {
    if (samples[i] >= (MIN_RANGE - 0.5) && samples[i] <= MAX_RANGE) {
      sum += samples[i];
      validSamples++;
    }
  }
  
  if (validSamples < 3) {
    return MAX_RANGE + 10; // Invalid
  }
  
  return sum / validSamples;
}

// =============================================================================
// LED SYSTEM
// =============================================================================

CRGB getInteractiveColor(float distance) {
  if (distance < MIN_RANGE || distance > MAX_RANGE) {
    return CRGB::Black;
  }
  
  // Map distance to color: far = red, close = yellow
  float ratio = (distance - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
  ratio = constrain(ratio, 0.0, 1.0);
  
  int red = 255;
  int green = (int)(255 * (1.0 - ratio));
  int blue = 0;
  
  return CRGB(red, green, blue);
}

CRGB getAttractColor(unsigned long currentTime) {
  // Smooth attract color calculation - recalculate every time for fluid motion
  float phase;
  if (usePhaseOffset) {
    phase = attractPhaseOffset + (2.0 * PI * currentTime / ATTRACT_PERIOD);
    usePhaseOffset = false;
  } else {
    phase = 2.0 * PI * currentTime / ATTRACT_PERIOD;
  }
  
  // Smooth sinusoidal fade from yellow to red
  float sineValue = (sin(phase) + 1.0) * 0.5;
  
  // Use floating point for smoother transitions, then round
  int red = 255;
  int green = (int)(255.0 * sineValue + 0.5);  // Add 0.5 for proper rounding
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

void updateLEDs(float avgDistance1, float avgDistance2, bool handsDetected, unsigned long currentTime) {
  if (currentMode == ATTRACT_MODE) {
    // ATTRACT MODE: Maximum smoothness - update every single cycle
    CRGB attractColor = getAttractColor(currentTime);
    
    // Always update for maximum smoothness - no change detection
    fill_solid(leds, NUM_LEDS, attractColor);
    
    // Fastest possible LED update with minimal interrupt blocking
    noInterrupts();
    FastLED.show();
    interrupts();
    
  } else {  // INTERACTIVE_MODE
    // Rate limit interactive mode to preserve performance
    if (currentTime - lastLEDUpdate < LED_UPDATE_INTERVAL) {
      return;
    }
    
    // Skip if audio interrupt is busy
    if (audioInterruptBusy) {
      return;
    }
    
    // Left sensor (first half of strip)
    CRGB leftColor = getInteractiveColor(avgDistance1);
    if (leftColor == CRGB::Black) leftColor = lastLeftColor;
    else lastLeftColor = leftColor;
    
    // Right sensor (second half of strip)
    CRGB rightColor = getInteractiveColor(avgDistance2);
    if (rightColor == CRGB::Black) rightColor = lastRightColor;
    else lastRightColor = rightColor;
    
    // Fast LED array updates
    int halfPoint = NUM_LEDS / 2;
    fill_solid(&leds[0], halfPoint, leftColor);
    fill_solid(&leds[halfPoint], NUM_LEDS - halfPoint, rightColor);
    
    noInterrupts();
    FastLED.show();
    interrupts();
    
    lastLEDUpdate = currentTime;
  }
}

// =============================================================================
// AUDIO CONTROL SYSTEM
// =============================================================================

void updateAudioControl(float avgDistance1, float avgDistance2, bool leftHand, bool rightHand) {
  // LEFT HAND - Play/Stop Control
  static bool prevLeftHand = false;
  
  if (leftHand && !prevLeftHand) {
    if (playState == 0) {
      playState = 1;
      audioSampleIndex = 0;
      audioActive = true;
      lastAudioStateChange = millis();
      Serial.println(F("AUDIO: PLAY"));
    } else {
      playState = 0;
      audioActive = false;
      lastAudioStateChange = millis();
      Serial.println(F("AUDIO: STOP"));
    }
  }
  
  // RIGHT HAND - Scratch Control
  static bool prevRightHand = false;
  static unsigned long lastTransition = 0;
  static uint8_t transitionCount = 0;
  static int8_t scratchDirection = 1;
  static unsigned long lastRightHandDetected = 0;
  
  if (rightHand) {
    lastRightHandDetected = millis();
  }
  
  if (rightHand != prevRightHand) {
    lastTransition = millis();
    transitionCount++;
    scratchDirection = -scratchDirection;
  }
  
  if (millis() - lastTransition > 500) {
    transitionCount = 0;
  }
  
  // Stop scratching if no right hand for 200ms
  if (!rightHand && (millis() - lastRightHandDetected) > 200) {
    if (isScratchMode) {
      isScratchMode = false;
      scratchSpeed = 1;
      playbackSpeed = 5;
      Serial.println(F("AUDIO: NORMAL"));
    }
  }
  // Scratch mode - rapid transitions
  else if (rightHand && transitionCount >= 2 && (millis() - lastTransition) < 300) {
    isScratchMode = true;
    
    if (transitionCount >= 4) {
      scratchSpeed = scratchDirection * 6;
      Serial.println(F("AUDIO: SCRATCH++"));
    } else {
      scratchSpeed = scratchDirection * 3;
      Serial.println(F("AUDIO: SCRATCH"));
    }
  }
  // Distance speed control
  else if (rightHand && playState == 1) {
    isScratchMode = false;
    scratchSpeed = 1;
    
    float ratio = (avgDistance2 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    ratio = constrain(ratio, 0.0, 1.0);
    playbackSpeed = (uint8_t)(3 + (12 * ratio));  // 3 (fast) to 15 (slow)
  }
  
  prevLeftHand = leftHand;
  prevRightHand = rightHand;
}

// =============================================================================
// JSON OUTPUT SYSTEM
// =============================================================================

void sendJSONUpdate(float avgDistance1, float avgDistance2, bool handsDetected, bool inRange1, bool inRange2) {
  StaticJsonDocument<500> doc;
  
  // Sensor data
  doc["left"] = int(avgDistance1);
  doc["right"] = int(avgDistance2);
  doc["hands_detected"] = handsDetected;
  doc["left_in_range"] = inRange1;
  doc["right_in_range"] = inRange2;
  
  // LED system data
  doc["mode"] = (currentMode == ATTRACT_MODE) ? "attract" : "interactive";
  
  // Audio system data
  doc["audio_active"] = audioActive;
  doc["audio_playing"] = (playState == 1);
  doc["scratch_mode"] = isScratchMode;
  doc["playback_speed"] = playbackSpeed;
  
  // Color correlation values for TouchDesigner
  if (currentMode == INTERACTIVE_MODE) {
    float leftOrange = inRange1 ? (1.0 - (avgDistance1 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE)) : 
                                 (lastLeftColor.green / 255.0);
    float rightOrange = inRange2 ? (1.0 - (avgDistance2 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE)) : 
                                  (lastRightColor.green / 255.0);
    
    doc["left_orange_value"] = leftOrange;
    doc["right_orange_value"] = rightOrange;
  }
  
  // System info
  doc["system"] = "solar_shrine_modular";
  doc["timestamp"] = millis();
  
  serializeJson(doc, Serial);
  Serial.println();
}



// =============================================================================
// MAIN SYSTEM SETUP AND LOOP
// =============================================================================

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  // Initialize sensor pins
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  
  // Initialize LED system with optimized settings
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(150);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 2000);  // Power management
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  // Initialize sample arrays
  for (int i = 0; i < SAMPLES; i++) {
    distance1Samples[i] = MAX_RANGE + 10;
    distance2Samples[i] = MAX_RANGE + 10;
  }
  
  // Initialize audio system
  setupAudioSystem();
  
  // Startup sequence - LED flash
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();
  delay(500);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  Serial.println(F("Solar Shrine Modular System Ready"));
  Serial.println(F("Features: Dual-mode LEDs + DJ Scratch Audio"));
  Serial.println(F("Left Hand: Play/Pause | Right Hand: Scratch Control"));
  Serial.println(F("Performance: Optimized LED/Audio timing"));
}

void loop() {
  unsigned long currentTime = millis();
  
  // ATTRACT MODE: Minimal processing - prioritize LED smoothness
  if (currentMode == ATTRACT_MODE) {
    // Only do essential mode checking in attract mode
    static unsigned long lastModeCheck = 0;
    static float lastDistance1 = MAX_RANGE + 10;
    static float lastDistance2 = MAX_RANGE + 10;
    
    // Check for mode transition only every 50ms to minimize processing
    if (currentTime - lastModeCheck >= 50) {
      // Quick sensor check without heavy processing
      lastDistance1 = readDistanceWithReset(trigPin1, echoPin1);
      lastDistance2 = readDistanceNewPing(sonar2);
      
      bool handsNear = (lastDistance1 >= MIN_RANGE && lastDistance1 <= MAX_RANGE) ||
                       (lastDistance2 >= MIN_RANGE && lastDistance2 <= MAX_RANGE);
      
      if (handsNear) {
        currentMode = INTERACTIVE_MODE;
        lastHandDetectedTime = currentTime;
      }
      lastModeCheck = currentTime;
    }
    
    // LED updates every cycle for maximum smoothness
    updateLEDs(0, 0, false, currentTime);
    
    // Minimal delay for attract mode
    delay(2);  // ~500Hz for ultra-smooth attract mode
    return;
  }
  
  // INTERACTIVE MODE: Full processing
  static uint8_t taskCycle = 0;
  static float cachedDistance1 = MAX_RANGE + 10;
  static float cachedDistance2 = MAX_RANGE + 10;
  static unsigned long lastSensorRead = 0;
  
  // Read sensors every 40ms in interactive mode
  if (currentTime - lastSensorRead >= 40) {
    if (taskCycle == 0) {
      cachedDistance1 = readDistanceWithReset(trigPin1, echoPin1);
    } else {
      cachedDistance2 = readDistanceNewPing(sonar2);
    }
    taskCycle = (taskCycle + 1) % 2;
    lastSensorRead = currentTime;
  }
  
  // Update sample arrays with cached values
  updateDistanceSamples(cachedDistance1, cachedDistance2);
  
  // Get averaged distances
  float avgDistance1, avgDistance2;
  if (samplesInitialized) {
    avgDistance1 = getAveragedDistance(distance1Samples);
    avgDistance2 = getAveragedDistance(distance2Samples);
  } else {
    avgDistance1 = cachedDistance1;
    avgDistance2 = cachedDistance2;
  }
  
  // Check if hands are detected
  bool inRange1 = (avgDistance1 >= MIN_RANGE && avgDistance1 <= MAX_RANGE);
  bool inRange2 = (avgDistance2 >= MIN_RANGE && avgDistance2 <= MAX_RANGE);
  bool handsDetected = inRange1 || inRange2;
  
  if (handsDetected) {
    lastHandDetectedTime = currentTime;
  } else {
    if ((currentTime - lastHandDetectedTime) >= INTERACTIVE_TIMEOUT) {
      // Simplified transition back to attract mode
      calculatePhaseOffset(lastLeftColor.r > lastRightColor.r ? lastLeftColor : lastRightColor);
      currentMode = ATTRACT_MODE;
    }
  }
  
  // Update systems
  updateLEDs(avgDistance1, avgDistance2, handsDetected, currentTime);
  
  // Stagger heavy operations in interactive mode
  static uint8_t heavyTaskCycle = 0;
  switch (heavyTaskCycle) {
    case 0:
      updateAudioControl(avgDistance1, avgDistance2, inRange1, inRange2);
      break;
    case 2:
      sendJSONUpdate(avgDistance1, avgDistance2, handsDetected, inRange1, inRange2);
      break;
  }
  heavyTaskCycle = (heavyTaskCycle + 1) % 3;
  
  delay(8);  // ~125Hz for interactive mode
} 