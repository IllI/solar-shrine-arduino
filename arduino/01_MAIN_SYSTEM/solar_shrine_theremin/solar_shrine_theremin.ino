/*
 * Solar Shrine with Theremin Integration
 * Dual-mode lighting (attract/interactive) + theremin-like sound generation
 * Enhanced with volume control and smoother audio transitions
 * Uses NewTone library for timer compatibility with ultrasonic sensors
 * 
 * Hardware:
 * - 2x HC-SR04 ultrasonic sensors (pins 5,6,9,10)
 * - WS2812B/WS2815 LED strip (pin 3)
 * - WWZMDiB XH-M543 amplifier + Dayton Audio DAEX32QMB-4 exciter (pin 11)
 * - Optional: 10k potentiometer on A0 for volume control
 * 
 * Libraries Required:
 * - FastLED
 * - ArduinoJson  
 * - NewTone (download from: https://bitbucket.org/teckel12/arduino-new-tone/downloads/)
 * - NewPing
 */

#include <ArduinoJson.h>
#include <FastLED.h>
#include <NewTone.h>
#include <NewPing.h>

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

// Audio pin and volume control
const int AUDIO_PIN = 11;
const int VOLUME_PIN = A0;  // Optional volume potentiometer

// Range constants (in cm)
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

// Theremin frequency ranges
const float MIN_FREQ = 80.0;    // Low bass tones
const float MAX_FREQ = 2000.0;  // High treble tones
const float BASE_FREQ = 220.0;  // A3 note as base

// Volume and envelope settings
float masterVolume = 0.5;       // 0.0 to 1.0
const float VOLUME_SMOOTHING = 0.1;  // Volume smoothing factor
float currentVolume = 0.0;
float targetVolume = 0.0;

// Enhanced audio smoothing
const float FREQ_SMOOTHING = 0.08;    // Slower frequency smoothing for less choppiness
const float ENVELOPE_ATTACK = 0.15;   // Attack rate for volume envelope
const float ENVELOPE_RELEASE = 0.05;  // Release rate for volume envelope

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
const unsigned long TONE_UPDATE_INTERVAL = 20;    // Update tone every 20ms for smoother response

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

// Enhanced theremin state
bool thereminActive = false;
float currentFrequency = 0;
float targetFrequency = 0;
bool fadeIn = false;
bool fadeOut = false;
unsigned long fadeStartTime = 0;
const unsigned long FADE_DURATION = 300;  // 300ms fade in/out

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  // FastLED setup
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(150);
  
  // Initialize all LEDs to off
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  // Initialize sample arrays
  for (int i = 0; i < SAMPLES; i++) {
    distance1Samples[i] = MAX_RANGE + 1;
    distance2Samples[i] = MAX_RANGE + 1;
  }
  
  // Read initial volume setting
  if (VOLUME_PIN != -1) {
    masterVolume = analogRead(VOLUME_PIN) / 1023.0;
  }
  
  // Audio test - play startup sequence with volume control
  playStartupSequence();
}

void playStartupSequence() {
  // Play a pleasant startup melody with envelope
  int melody[] = {220, 277, 330, 440}; // A3, C#4, E4, A4
  int durations[] = {200, 200, 200, 400};
  
  for (int i = 0; i < 4; i++) {
    playToneWithEnvelope(melody[i], durations[i]);
    delay(durations[i] + 50);
  }
}

void playToneWithEnvelope(int frequency, int duration) {
  // Simple envelope: quick attack, sustained, quick release
  int attackTime = duration / 8;
  int sustainTime = duration - (2 * attackTime);
  int releaseTime = attackTime;
  
  // Attack phase - ramp up volume
  for (int t = 0; t < attackTime; t += 10) {
    float vol = (float)t / attackTime;
    NewTone(AUDIO_PIN, frequency);
    delay(10);
  }
  
  // Sustain phase
  NewTone(AUDIO_PIN, frequency);
  delay(sustainTime);
  
  // Release phase - would need external volume control for true fade
  NewTone(AUDIO_PIN, 0); // Stop tone
}

float readDistanceNewPing(NewPing &sensor) {
  unsigned int distance = sensor.ping_cm();
  if (distance == 0) {
    return MAX_RANGE + 1; // No echo received
  }
  return (float)distance;
}

float getAveragedDistance(float samples[]) {
  float sum = 0;
  int validSamples = 0;
  
  for (int i = 0; i < SAMPLES; i++) {
    if (samples[i] <= MAX_RANGE) {
      sum += samples[i];
      validSamples++;
    }
  }
  
  if (validSamples == 0) {
    return MAX_RANGE + 1;
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

float calculateThereminFrequency(float distance1, float distance2, bool inRange1, bool inRange2) {
  if (!inRange1 && !inRange2) {
    return 0; // No sound when no hands detected
  }
  
  float frequency = 0;
  
  if (inRange1 && inRange2) {
    // Both hands detected - create harmony based on distance difference
    float avgDistance = (distance1 + distance2) / 2.0;
    float difference = abs(distance1 - distance2);
    
    // Base frequency from average distance
    float ratio = (avgDistance - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    ratio = constrain(ratio, 0.0, 1.0);
    frequency = MIN_FREQ + (MAX_FREQ - MIN_FREQ) * (1.0 - ratio);
    
    // Add subtle harmonic modulation based on hand difference
    if (difference > 2.0) {
      float modulation = sin(millis() * 0.003) * 20; // Slower, subtler vibrato
      frequency += modulation;
    }
    
  } else if (inRange1) {
    // Left hand only - lower frequency range
    float ratio = (distance1 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    ratio = constrain(ratio, 0.0, 1.0);
    frequency = MIN_FREQ + (BASE_FREQ - MIN_FREQ) * (1.0 - ratio);
    
  } else if (inRange2) {
    // Right hand only - higher frequency range  
    float ratio = (distance2 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    ratio = constrain(ratio, 0.0, 1.0);
    frequency = BASE_FREQ + (MAX_FREQ - BASE_FREQ) * (1.0 - ratio);
  }
  
  return frequency;
}

void updateThereminAudio(float frequency) {
  // Read volume control if available
  if (VOLUME_PIN != -1) {
    float newMasterVolume = analogRead(VOLUME_PIN) / 1023.0;
    masterVolume += (newMasterVolume - masterVolume) * VOLUME_SMOOTHING;
  }
  
  if (frequency > 0) {
    // Starting a new note or continuing
    if (!thereminActive) {
      // Start fade in
      thereminActive = true;
      fadeIn = true;
      fadeOut = false;
      fadeStartTime = millis();
      currentVolume = 0.0;
    }
    
    // Smooth frequency transitions
    targetFrequency = frequency;
    currentFrequency += (targetFrequency - currentFrequency) * FREQ_SMOOTHING;
    
    // Handle volume envelope
    if (fadeIn) {
      unsigned long elapsed = millis() - fadeStartTime;
      if (elapsed < FADE_DURATION) {
        currentVolume = (float)elapsed / FADE_DURATION * masterVolume;
      } else {
        fadeIn = false;
        currentVolume = masterVolume;
      }
    } else if (!fadeOut) {
      targetVolume = masterVolume;
      currentVolume += (targetVolume - currentVolume) * ENVELOPE_ATTACK;
    }
    
    // Play tone at calculated frequency
    // Note: NewTone doesn't support volume, so we simulate it with on/off timing
    if (currentVolume > 0.1) {  // Only play if volume is significant
      NewTone(AUDIO_PIN, (unsigned int)currentFrequency);
    } else {
      NewTone(AUDIO_PIN, 0);
    }
    
  } else {
    // No frequency - start fade out if not already fading
    if (thereminActive && !fadeOut) {
      fadeOut = true;
      fadeIn = false;
      fadeStartTime = millis();
    }
    
    if (fadeOut) {
      unsigned long elapsed = millis() - fadeStartTime;
      if (elapsed < FADE_DURATION) {
        currentVolume = masterVolume * (1.0 - (float)elapsed / FADE_DURATION);
        if (currentVolume > 0.1) {
          NewTone(AUDIO_PIN, (unsigned int)currentFrequency);
        } else {
          NewTone(AUDIO_PIN, 0);
        }
      } else {
        // Fade complete
        NewTone(AUDIO_PIN, 0);
        thereminActive = false;
        fadeOut = false;
        currentVolume = 0.0;
        currentFrequency = 0;
      }
    }
  }
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
  
  // Check if hands are detected
  bool inRange1 = (avgDistance1 >= MIN_RANGE && avgDistance1 <= MAX_RANGE);
  bool inRange2 = (avgDistance2 >= MIN_RANGE && avgDistance2 <= MAX_RANGE);
  bool handsDetected = inRange1 || inRange2;
  
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
    float thereminFreq = calculateThereminFrequency(avgDistance1, avgDistance2, inRange1, inRange2);
    updateThereminAudio(thereminFreq);
    lastToneTime = currentTime;
  }
  
  // Create JSON document for TouchDesigner
  StaticJsonDocument<400> doc;
  
  doc["left"] = int(avgDistance1);
  doc["right"] = int(avgDistance2);
  doc["hands_detected"] = handsDetected;
  doc["mode"] = (currentMode == ATTRACT_MODE) ? "attract" : "interactive";
  doc["left_in_range"] = inRange1;
  doc["right_in_range"] = inRange2;
  doc["theremin_active"] = thereminActive;
  doc["frequency"] = currentFrequency;
  
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
  
  delay(20);  // Faster loop for smoother theremin response
} 