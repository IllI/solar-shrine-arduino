/*
 * Solar Shrine with Theremin Integration
 * Dual-mode lighting (attract/interactive) + PWM-based theremin audio generation
 * Uses granular synthesis approach inspired by Peter Knight's Auduino project
 * 
 * Hardware:
 * - 2x HC-SR04 ultrasonic sensors (pins 5,6,9,10)
 * - WS2812B/WS2815 LED strip (pin 3)
 * - WWZMDiB XH-M543 amplifier + Dayton Audio DAEX32QMB-4 exciter (pin 11)
 * 
 * Libraries Required:
 * - FastLED
 * - ArduinoJson  
 * - NewPing
 * 
 * IMPORTANT: NO NewTone library - using PWM timer interrupt approach instead
 * 
 * AUDIO DISABLE: Set ENABLE_AUDIO to false to disable audio completely
 */

#define ENABLE_AUDIO true  // Set to false to disable audio

#include <ArduinoJson.h>
#include <FastLED.h>
#include <NewPing.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Sensor pins
const int trigPin1 = 9;
const int echoPin1 = 10;
const int trigPin2 = 5;
const int echoPin2 = 6;

// NewPing sensor objects
NewPing sonar1(trigPin1, echoPin1, 200); // Left sensor, max 200cm
NewPing sonar2(trigPin2, echoPin2, 200); // Right sensor, max 200cm

// LED strip configuration
#define LED_PIN 3
#define NUM_LEDS 60        // Adjust to your strip length
#define LED_TYPE WS2812B   
#define COLOR_ORDER GRB    

CRGB leds[NUM_LEDS];

// Audio pin and PWM configuration
const int AUDIO_PIN = 11;

#if ENABLE_AUDIO
// PWM Audio generation variables (based on Auduino approach)
#define PWM_PIN 11
#define PWM_VALUE OCR2A
#define PWM_INTERRUPT TIMER2_OVF_vect

// Granular synthesis variables
uint16_t syncPhaseAcc = 0;
uint16_t syncPhaseInc = 0;
uint16_t grainPhaseAcc = 0;  
uint16_t grainPhaseInc = 0;
uint16_t grainAmp = 0;
uint8_t grainDecay = 0;
uint16_t grain2PhaseAcc = 0;
uint16_t grain2PhaseInc = 0;
uint16_t grain2Amp = 0;
uint8_t grain2Decay = 0;

// Phase increment mapping table (simplified from Auduino)
uint16_t mapPhaseInc(uint16_t input) {
  // Simple exponential mapping for frequency control
  // Input range 0-1023 maps to phase increments
  return (input * input) >> 6; // Exponential curve, scaled down
}
#endif

// Range constants (in cm)
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

// Theremin frequency ranges
const float MIN_FREQ = 80.0;    // Low bass tones
const float MAX_FREQ = 1200.0;  // High frequencies
const float BASE_FREQ = 220.0;  // A3 note as base

// Mode constants
enum LightMode {
  ATTRACT_MODE,
  INTERACTIVE_MODE
};

// State variables
LightMode currentMode = ATTRACT_MODE;
unsigned long lastHandDetectedTime = 0;
const unsigned long INTERACTIVE_TIMEOUT = 10000;  // 10 seconds
unsigned long lastToneTime = 0;
const unsigned long TONE_UPDATE_INTERVAL = 50;    // Update audio parameters every 50ms

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

// Theremin state
bool thereminActive = false;

#if ENABLE_AUDIO
// PWM Audio setup function
void audioOn() {
  // Set up PWM to 31.25kHz, phase accurate (Timer2)
  // Based on Auduino configuration for ATmega328
  TCCR2A = _BV(COM2A1) | _BV(WGM20);  // Phase correct PWM, output on OC2A (pin 11)
  TCCR2B = _BV(CS20);                 // No prescaling
  TIMSK2 = _BV(TOIE2);                // Enable overflow interrupt
}

// Timer2 overflow interrupt - this generates the audio samples
ISR(TIMER2_OVF_vect) {
  uint8_t value;
  uint16_t output;

  // Update sync phase accumulator
  syncPhaseAcc += syncPhaseInc;
  if (syncPhaseAcc < syncPhaseInc) {
    // Time to start the next grain
    grainPhaseAcc = 0;
    grainAmp = 0x7fff;
    grain2PhaseAcc = 0;
    grain2Amp = 0x7fff;
  }

  // Increment the phase of the grain oscillators
  grainPhaseAcc += grainPhaseInc;
  grain2PhaseAcc += grain2PhaseInc;

  // Convert phase into a triangle wave for grain 1
  value = (grainPhaseAcc >> 7) & 0xff;
  if (grainPhaseAcc & 0x8000) value = ~value;
  // Multiply by current grain amplitude to get sample
  output = value * (grainAmp >> 8);

  // Repeat for second grain
  value = (grain2PhaseAcc >> 7) & 0xff;
  if (grain2PhaseAcc & 0x8000) value = ~value;
  output += value * (grain2Amp >> 8);

  // Make the grain amplitudes decay by a factor every sample (exponential decay)
  grainAmp -= (grainAmp >> 8) * grainDecay;
  grain2Amp -= (grain2Amp >> 8) * grain2Decay;

  // Scale output to the available range, clipping if necessary
  output >>= 9;
  if (output > 255) output = 255;

  // Output to PWM
  PWM_VALUE = output;
}
#endif

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  // Audio pin setup
  pinMode(AUDIO_PIN, OUTPUT);
  digitalWrite(AUDIO_PIN, LOW);
  
#if ENABLE_AUDIO
  // Initialize PWM audio system
  audioOn();
#endif
  
  // FastLED setup
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(150);
  
  // Initialize all LEDs to off
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  // Initialize sample arrays
  for (int i = 0; i < SAMPLES; i++) {
    distance1Samples[i] = MAX_RANGE + 10;
    distance2Samples[i] = MAX_RANGE + 10;
  }
  
  // Audio test - play startup sequence
  playStartupSequence();
}

void playStartupSequence() {
#if ENABLE_AUDIO
  // Simple startup indication - brief audio test
  for (int i = 0; i < 3; i++) {
    // Set basic parameters for a brief tone
    syncPhaseInc = 1000 + (i * 500);  // Increasing frequency
    grainPhaseInc = 2000;
    grainDecay = 20;
    grain2PhaseInc = 2500;
    grain2Decay = 15;
    
    delay(150);
    
    // Stop audio
    syncPhaseInc = 0;
    grainPhaseInc = 0;
    grain2PhaseInc = 0;
    delay(50);
  }
  
  // Reset to silent state
  syncPhaseAcc = 0;
  grainPhaseAcc = 0;
  grain2PhaseAcc = 0;
  grainAmp = 0;
  grain2Amp = 0;
#else
  // Audio disabled - just a brief LED flash as startup indicator
  fill_solid(leds, NUM_LEDS, CRGB::White);
  FastLED.show();
  delay(200);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
#endif
}

float readDistanceNewPing(NewPing &sensor) {
  unsigned int distance = sensor.ping_cm();
  if (distance == 0 || distance > MAX_RANGE) {
    return MAX_RANGE + 10; // Return a clearly invalid value for no echo or out of range
  }
  return (float)distance;
}

float getAveragedDistance(float samples[]) {
  float sum = 0;
  int validSamples = 0;
  
  for (int i = 0; i < SAMPLES; i++) {
    if (samples[i] >= MIN_RANGE && samples[i] <= MAX_RANGE) {
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

void updateDistanceSamples(float distance1, float distance2) {
  distance1Samples[sampleIndex] = distance1;
  distance2Samples[sampleIndex] = distance2;
  
  sampleIndex = (sampleIndex + 1) % SAMPLES;
  
  if (!samplesInitialized && sampleIndex == 0) {
    samplesInitialized = true;
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

void updateThereminAudio(float distance1, float distance2, bool inRange1, bool inRange2) {
#if ENABLE_AUDIO
  if (!inRange1 && !inRange2) {
    // No hands detected - silence
    thereminActive = false;
    syncPhaseInc = 0;
    grainPhaseInc = 0;
    grain2PhaseInc = 0;
    grainDecay = 0;
    grain2Decay = 0;
    return;
  }
  
  thereminActive = true;
  
  if (inRange1 && inRange2) {
    // Both hands detected - use startup sequence parameter style
    float avgDistance = (distance1 + distance2) / 2.0;
    float difference = abs(distance1 - distance2);
    
    // Map average distance to sync frequency - same range as startup (1000-2000)
    float ratio = (avgDistance - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    ratio = constrain(ratio, 0.0, 1.0);
    syncPhaseInc = 1000 + (int)((1.0 - ratio) * 1000);  // 1000-2000 like startup
    
    // Map individual distances to grain frequencies - same as startup base values
    float ratio1 = (distance1 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    float ratio2 = (distance2 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    ratio1 = constrain(ratio1, 0.0, 1.0);
    ratio2 = constrain(ratio2, 0.0, 1.0);
    
    grainPhaseInc = 2000 + (int)((1.0 - ratio1) * 1000);   // 2000-3000 range
    grain2PhaseInc = 2500 + (int)((1.0 - ratio2) * 1000);  // 2500-3500 range
    
    // Use startup decay values with slight variation for difference
    grainDecay = 20 + (int)(difference);     // Base 20 like startup
    grain2Decay = 15 + (int)(difference);    // Base 15 like startup
    
  } else if (inRange1) {
    // Left hand only - lower frequency range
    float ratio = (distance1 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    ratio = constrain(ratio, 0.0, 1.0);
    
    syncPhaseInc = 800 + (int)((1.0 - ratio) * 700);   // 800-1500 range (lower)
    grainPhaseInc = 1800 + (int)((1.0 - ratio) * 800); // 1800-2600 range
    grain2PhaseInc = grainPhaseInc / 2;                 // Harmonic
    grainDecay = 20;    // Same as startup
    grain2Decay = 15;   // Same as startup
    
  } else if (inRange2) {
    // Right hand only - higher frequency range
    float ratio = (distance2 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    ratio = constrain(ratio, 0.0, 1.0);
    
    syncPhaseInc = 1200 + (int)((1.0 - ratio) * 1000);  // 1200-2200 range (higher)
    grainPhaseInc = 2200 + (int)((1.0 - ratio) * 1200); // 2200-3400 range
    grain2PhaseInc = grainPhaseInc * 1.2;                // Higher harmonic
    grainDecay = 20;    // Same as startup
    grain2Decay = 15;   // Same as startup
  }
#else
  // Audio disabled - just update the theremin state for JSON output
  thereminActive = (inRange1 || inRange2);
#endif
}

void updateLEDs(float avgDistance1, float avgDistance2, bool handsDetected, unsigned long currentTime) {
  if (currentMode == ATTRACT_MODE) {
    // Both sides show the same attract color
    CRGB attractColor = getAttractColor(currentTime);
    fill_solid(leds, NUM_LEDS, attractColor);
    
  } else {  // INTERACTIVE_MODE
    // Left sensor (first half of strip)
    CRGB leftColor = getInteractiveColor(avgDistance1);
    if (leftColor == CRGB::Black) leftColor = lastLeftColor;
    else lastLeftColor = leftColor;
    
    // Right sensor (second half of strip)
    CRGB rightColor = getInteractiveColor(avgDistance2);
    if (rightColor == CRGB::Black) rightColor = lastRightColor;
    else lastRightColor = rightColor;
    
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

void loop() {
  unsigned long currentTime = millis();
  
  // Read sensors using NewPing
  float distance1 = readDistanceNewPing(sonar1);
  float distance2 = readDistanceNewPing(sonar2);
  
  // Update sample arrays
  updateDistanceSamples(distance1, distance2);
  
  // Get averaged distances
  float avgDistance1, avgDistance2;
  if (samplesInitialized) {
    avgDistance1 = getAveragedDistance(distance1Samples);
    avgDistance2 = getAveragedDistance(distance2Samples);
  } else {
    avgDistance1 = distance1;
    avgDistance2 = distance2;
  }
  
  // Check if hands are detected with strict validation
  bool inRange1 = (avgDistance1 >= MIN_RANGE && avgDistance1 <= MAX_RANGE);
  bool inRange2 = (avgDistance2 >= MIN_RANGE && avgDistance2 <= MAX_RANGE);
  bool handsDetected = inRange1 || inRange2;
  
  // Additional validation - if we're in attract mode and getting brief detections, ignore them
  static int falseDetectionCount = 0;
  if (currentMode == ATTRACT_MODE && handsDetected) {
    falseDetectionCount++;
    if (falseDetectionCount < 3) {  // Require 3 consecutive detections to switch modes
      handsDetected = false;
      inRange1 = false;
      inRange2 = false;
    }
  } else {
    falseDetectionCount = 0;  // Reset counter when no hands or in interactive mode
  }
  
  // Mode state machine
  if (handsDetected) {
    if (currentMode == ATTRACT_MODE) {
      currentMode = INTERACTIVE_MODE;
    }
    lastHandDetectedTime = currentTime;
  } else {
    if (currentMode == INTERACTIVE_MODE && 
        (currentTime - lastHandDetectedTime) >= INTERACTIVE_TIMEOUT) {
      
      // Calculate smooth transition phase
      CRGB transitionColor = (lastLeftColor.r + lastLeftColor.g > lastRightColor.r + lastRightColor.g) ? 
                            lastLeftColor : lastRightColor;
      calculatePhaseOffset(transitionColor);
      
      currentMode = ATTRACT_MODE;
    }
  }
  
  // Update LED effects
  updateLEDs(avgDistance1, avgDistance2, handsDetected, currentTime);
  
  // Update theremin audio (less frequently for smoother sound)
  if (currentTime - lastToneTime >= TONE_UPDATE_INTERVAL) {
    updateThereminAudio(avgDistance1, avgDistance2, inRange1, inRange2);
    lastToneTime = currentTime;
  }
  
  // Create JSON document for TouchDesigner - only send when hands detected or mode changes
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
    doc["theremin_active"] = thereminActive;
    
    // Add granular synthesis parameters for TouchDesigner correlation
    if (thereminActive) {
#if ENABLE_AUDIO
      doc["sync_phase"] = syncPhaseInc;
      doc["grain1_phase"] = grainPhaseInc;
      doc["grain2_phase"] = grain2PhaseInc;
      doc["grain1_decay"] = grainDecay;
      doc["grain2_decay"] = grain2Decay;
#else
      doc["audio_disabled"] = true;
#endif
    }
    
    // Add color values for TouchDesigner correlation
    if (currentMode == INTERACTIVE_MODE) {
      float leftOrange = inRange1 ? (1.0 - (avgDistance1 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE)) : 
                                   (lastLeftColor.green / 255.0);
      float rightOrange = inRange2 ? (1.0 - (avgDistance2 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE)) : 
                                    (lastRightColor.green / 255.0);
      
      doc["left_orange_value"] = leftOrange;
      doc["right_orange_value"] = rightOrange;
    }
    
    // Serialize JSON to string and send
    serializeJson(doc, Serial);
    Serial.println();
    
    lastReportedMode = currentMode;
    lastHandsDetected = handsDetected;
  }
  
  delay(20);  // Faster loop for smoother theremin response
} 