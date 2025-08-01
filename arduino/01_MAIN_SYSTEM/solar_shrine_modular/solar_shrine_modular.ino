/*
 * Solar Shrine - Modular Integration System
 * Combines dual-mode lighting, sensor detection, and rotating audio effects
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
 * - Rotating Audio Effects: Theremin → DJ Scratch → Alien Sound → Robots Talking
 * - TouchDesigner JSON integration
 * - Modular architecture for future expansion
 * 
 * Audio Effect Rotation:
 * - Cover both sensors for 2+ seconds to cycle to next effect
 * - Order: Theremin → DJ Scratch → Alien Sound → Robots Talking → (repeat)
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

// NEW: Include modular audio effects system
#include "AudioEffects.h"

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

// Audio effect rotation constants
const unsigned long EFFECT_SWITCH_HOLD_TIME = 2000;  // 2 seconds to switch effects

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

// Audio effect rotation state
unsigned long bothHandsStartTime = 0;
bool bothHandsDetected = false;
bool effectSwitchInProgress = false;

// Performance optimization variables
unsigned long lastLEDUpdate = 0;
const unsigned long LED_UPDATE_INTERVAL = 33;  // ~30Hz for smooth visuals
volatile bool audioInterruptBusy = false;

// =============================================================================
// MASTER AUDIO INTERRUPT (Timer1 Compare B)
// =============================================================================

// Timer1 Compare B interrupt - Routes to active audio effect
ISR(TIMER1_COMPB_vect) {
  audioInterruptBusy = true;
  
  switch (currentAudioEffect) {
    case EFFECT_THEREMIN:
      // Simple sine wave synthesis for theremin
      if (theremin.active && theremin.volume > 0) {
        static uint32_t thereminPhase = 0;
        thereminPhase += (uint32_t)(theremin.frequency * 65536UL / 20000UL);
        
        // Simple sine approximation using triangle wave
        uint16_t phase16 = thereminPhase >> 16;
        int16_t triangle = (phase16 < 32768) ? (phase16 - 16384) : (49152 - phase16);
        triangle = triangle >> 6;  // Scale down
        
        int16_t sample = (triangle * theremin.volume) >> 8;
        sample = constrain(sample, -128, 127);
        OCR1B = ((uint32_t)(sample + 128) * ICR1) >> 8;
      } else {
        OCR1B = ICR1 / 2;  // Silence
      }
      break;
      
    case EFFECT_DJ_SCRATCH:
      // DJ scratch audio (existing logic)
      if (djScratch.playState == 0) {
        OCR1B = ICR1 / 2;  // Silence
        break;
      }
      
      djScratch.sampleCounter++;
      uint8_t currentSpeed = djScratch.isScratchMode ? 2 : djScratch.playbackSpeed;
      
      if (djScratch.sampleCounter >= currentSpeed) {
        djScratch.sampleCounter = 0;
        
        if (djScratch.sampleIndex < 0) djScratch.sampleIndex = 0;
        if (djScratch.sampleIndex >= AUDIO_SAMPLE_COUNT) djScratch.sampleIndex = 0;
        
        uint8_t sample = pgm_read_byte(&audioData[djScratch.sampleIndex]);
        int16_t amp = ((int16_t)sample - 128) << 2;
        amp = constrain(amp, -128, 127);
        
        OCR1B = ((uint32_t)(amp + 128) * ICR1) >> 8;
        
        if (djScratch.isScratchMode) {
          djScratch.sampleIndex += djScratch.scratchSpeed;
          if (djScratch.sampleIndex < 0) djScratch.sampleIndex = AUDIO_SAMPLE_COUNT - 1;
          if (djScratch.sampleIndex >= AUDIO_SAMPLE_COUNT) djScratch.sampleIndex = 0;
        } else {
          djScratch.sampleIndex++;
          if (djScratch.sampleIndex >= AUDIO_SAMPLE_COUNT) djScratch.sampleIndex = 0;
        }
      }
      break;
      
    case EFFECT_ALIEN_SOUND:
      // Alien sound FM synthesis
      if (alienSound.is_active) {
        alienSound.phase_accumulator += alienSound.phase_increment;
        alienSound.mod_phase += 50;
        
        uint8_t modulator = (alienSound.mod_phase >> 8) & 0xFF;
        uint8_t carrier = ((alienSound.phase_accumulator + (modulator * 4)) >> 8) & 0xFF;
        
        int16_t amp = (carrier > 128) ? 100 : -100;
        OCR1B = ((uint32_t)(amp + 128) * ICR1) / 255;
      } else {
        OCR1B = ICR1 / 2;  // Silence
      }
      break;
      
    case EFFECT_ROBOTS_TALKING:
      // Robot talking square wave synthesis
      if (robotTalk.is_active) {
        robotTalk.phase += 150;
        if ((robotTalk.phase & 0x1000) == 0) robotTalk.current_pattern++;
        
        uint8_t wave = robotTalk.phase >> 8;
        int16_t amp = 0;
        
        if (robotTalk.current_pattern & 1) {
          amp = (wave & 0x80) ? 120 : -120;  // Square wave
        } else {
          amp = (wave & 0x40) ? 80 : -80;    // Different pattern
        }
        
        OCR1B = ((uint32_t)(amp + 128) * ICR1) / 255;
      } else {
        OCR1B = ICR1 / 2;  // Silence
      }
      break;
      
    default:
      OCR1B = ICR1 / 2;  // Silence
      break;
  }
  
  audioInterruptBusy = false;
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
// AUDIO EFFECT ROTATION SYSTEM
// =============================================================================

void checkForEffectRotation(bool leftHand, bool rightHand, unsigned long currentTime) {
  bool bothHands = leftHand && rightHand;
  
  if (bothHands && !bothHandsDetected) {
    // Both hands just detected - start timer
    bothHandsDetected = true;
    bothHandsStartTime = currentTime;
    Serial.println(F("Both hands detected - hold for 2s to switch effect"));
  } else if (!bothHands && bothHandsDetected) {
    // Hands removed - cancel switch
    bothHandsDetected = false;
    effectSwitchInProgress = false;
    Serial.println(F("Effect switch cancelled"));
  } else if (bothHands && bothHandsDetected && !effectSwitchInProgress) {
    // Check if held long enough
    if (currentTime - bothHandsStartTime >= EFFECT_SWITCH_HOLD_TIME) {
      effectSwitchInProgress = true;
      
      // Cycle to next effect
      AudioEffect nextEffect;
      switch (currentAudioEffect) {
        case EFFECT_THEREMIN: nextEffect = EFFECT_DJ_SCRATCH; break;
        case EFFECT_DJ_SCRATCH: nextEffect = EFFECT_ALIEN_SOUND; break;
        case EFFECT_ALIEN_SOUND: nextEffect = EFFECT_ROBOTS_TALKING; break;
        case EFFECT_ROBOTS_TALKING: nextEffect = EFFECT_THEREMIN; break;
        default: nextEffect = EFFECT_THEREMIN; break;
      }
      
      switchAudioEffect(nextEffect);
      
      // LED flash to indicate effect change
      fill_solid(leds, NUM_LEDS, CRGB::Blue);
      FastLED.show();
      delay(200);
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      FastLED.show();
      delay(100);
    }
  }
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
  
  // Audio system data - NEW: Include current effect
  const char* effectName = "none";
  switch (currentAudioEffect) {
    case EFFECT_THEREMIN: effectName = "theremin"; break;
    case EFFECT_DJ_SCRATCH: effectName = "dj_scratch"; break;
    case EFFECT_ALIEN_SOUND: effectName = "alien_sound"; break;
    case EFFECT_ROBOTS_TALKING: effectName = "robots_talking"; break;
  }
  doc["current_audio_effect"] = effectName;
  
  // Effect-specific data
  switch (currentAudioEffect) {
    case EFFECT_THEREMIN:
      doc["audio_active"] = theremin.active;
      doc["frequency"] = theremin.frequency;
      doc["volume"] = theremin.volume;
      break;
    case EFFECT_DJ_SCRATCH:
      doc["audio_active"] = (djScratch.playState == 1);
      doc["audio_playing"] = (djScratch.playState == 1);
      doc["scratch_mode"] = djScratch.isScratchMode;
      doc["playback_speed"] = djScratch.playbackSpeed;
      break;
    case EFFECT_ALIEN_SOUND:
      doc["audio_active"] = alienSound.is_active;
      break;
    case EFFECT_ROBOTS_TALKING:
      doc["audio_active"] = robotTalk.is_active;
      break;
  }
  
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
  doc["system"] = "solar_shrine_modular_rotating";
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
  
  // NEW: Initialize modular audio system
  setupAudio();
  
  // Startup sequence - LED flash
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();
  delay(500);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  Serial.println(F("Solar Shrine Modular System Ready - Rotating Audio Effects"));
  Serial.println(F("Features: Dual-mode LEDs + Rotating Audio Effects"));
  Serial.println(F("Audio Effects: Theremin → DJ Scratch → Alien Sound → Robots Talking"));
  Serial.println(F("Hold both hands over sensors for 2s to cycle effects"));
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
  
  // NEW: Update current audio effect
  updateAudioEffects(avgDistance1, avgDistance2);
  
  // NEW: Check for effect rotation
  checkForEffectRotation(inRange1, inRange2, currentTime);
  
  // Stagger heavy operations in interactive mode
  static uint8_t heavyTaskCycle = 0;
  switch (heavyTaskCycle) {
    case 2:
      sendJSONUpdate(avgDistance1, avgDistance2, handsDetected, inRange1, inRange2);
      break;
  }
  heavyTaskCycle = (heavyTaskCycle + 1) % 3;
  
  delay(8);  // ~125Hz for interactive mode
} 