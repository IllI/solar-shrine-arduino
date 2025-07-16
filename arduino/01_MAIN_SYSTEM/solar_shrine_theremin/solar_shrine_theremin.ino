/*
 * Solar Shrine with Theremin Integration
 * Dual-mode lighting (attract/interactive) + Mozzi-based theremin audio generation
 * Optimized for Arduino Mega 2560 with Timer1 OC1B PWM audio output
 * 
 * Hardware (Arduino Mega 2560):
 * - 2x HC-SR04 ultrasonic sensors (pins 5,6,10,11)
 * - WS2812B/WS2815 LED strip (pin 3)
 * - Audio: Pin 12 → 1K resistor → Amplifier right channel (left channel + ground → ground)
 * - WWZMDiB XH-M543 amplifier + Dayton Audio DAEX32QMB-4 exciter
 * 
 * Libraries Required:
 * - FastLED
 * - ArduinoJson  
 * - NewPing
 * - Mozzi (download from https://sensorium.github.io/Mozzi/)
 * 
 * AUDIO DISABLE: Set ENABLE_AUDIO to false to disable audio completely
 */

#define ENABLE_AUDIO true  // Set to false to disable audio

#include <ArduinoJson.h>
#include <FastLED.h>
#include <NewPing.h>

#if ENABLE_AUDIO
#include <MozziGuts.h>
#include <Oscil.h> 
#include <RollingAverage.h>
#include <tables/triangle_valve_2048_int8.h>

Oscil <TRIANGLE_VALVE_2048_NUM_CELLS, AUDIO_RATE> osc(TRIANGLE_VALVE_2048_DATA);
RollingAverage <int, 4> freqAverage;
RollingAverage <int, 8> volAverage;

#define CONTROL_RATE 128
#endif

// Sensor pins (updated for your hardware changes)
const int trigPin1 = 10;  // Moved from pin 9
const int echoPin1 = 11;  // Moved from pin 10
const int trigPin2 = 5;
const int echoPin2 = 6;

// NewPing sensor objects
NewPing sonar1(trigPin1, echoPin1, 200); // Left sensor, max 200cm
NewPing sonar2(trigPin2, echoPin2, 200); // Right sensor, max 200cm

// LED strip configuration
#define LED_PIN 3
#define NUM_LEDS 120        // Adjust to your strip length
#define LED_TYPE WS2812B   
#define COLOR_ORDER GRB    

CRGB leds[NUM_LEDS];

// Audio pin - Arduino Mega 2560 Timer1 OC1B PWM output
const int AUDIO_PIN = 12;

#if ENABLE_AUDIO
// Mozzi audio variables (now pin 9 compatible!)
int audioVolume = 0;
int targetVolume = 0;
const int MIN_FREQ = 131;    // C3 in Hz
const int MAX_FREQ = 1046;   // C6 in Hz
const int BASE_FREQ = 220;   // A3 note as base frequency
const int FADE_SPEED = 4;    // Volume fade speed
#endif

// Range constants (in cm)
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

// Mode constants
enum LightMode {
  ATTRACT_MODE,
  INTERACTIVE_MODE
};

// State variables
LightMode currentMode = ATTRACT_MODE;
unsigned long lastHandDetectedTime = 0;
const unsigned long INTERACTIVE_TIMEOUT = 10000;  // 10 seconds

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
// Mozzi audio functions (now pin 9 compatible!)

// Required Mozzi function - called by audioHook()
void updateControl() {
  // This will be called from the main loop when sensors are read
}

// Required Mozzi function - generates audio samples
int updateAudio() {
  return (osc.next() * audioVolume) >> 8;  // Return audio sample with volume control
}
#endif

void setup() {
  Serial.begin(9600);
  delay(2000);
  
#if ENABLE_AUDIO
  // Initialize Mozzi audio system
  startMozzi(CONTROL_RATE);
  osc.setFreq(MIN_FREQ);  // Set initial frequency
#endif
  
  // Sensor pin setup (CRITICAL - was missing!)
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
  // Simple startup indication using Mozzi (adapted from MiniMin)
  int melody[] = {220, 277, 330, 440}; // A3, C#4, E4, A4
  for (int i = 0; i < 4; i++) {
    osc.setFreq(melody[i]);
    audioVolume = 128;  // Medium volume
    
    // Let Mozzi generate audio for 150ms
    unsigned long startTime = millis();
    while (millis() - startTime < 150) {
      audioHook();  // Required for Mozzi audio generation
    }
    
    audioVolume = 0;  // Silence
    
    // Silence for 50ms
    startTime = millis();
    while (millis() - startTime < 50) {
      audioHook();
    }
  }
  
  // Ensure silence after startup
  audioVolume = 0;
#else
  // Audio disabled - just a brief LED flash as startup indicator
  fill_solid(leds, NUM_LEDS, CRGB::White);
  FastLED.show();
  delay(200);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
#endif
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

float readDistanceNewPing(NewPing &sensor) {
  unsigned int distance = sensor.ping_cm();
  if (distance == 0) {
    // No echo detected - hand is covering sensor (very close)
    // Return minimum distance instead of invalid value (MiniMin approach)
    return MIN_RANGE - 0.5;  // Closer than minimum range
  }
  if (distance > MAX_RANGE) {
    return MAX_RANGE + 10; // Return invalid value for out of range
  }
  return (float)distance;
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
  int freq;
  
  // Check for any hand presence (including hands covering sensors)
  bool hand1Present = (distance1 <= MAX_RANGE);  // Any valid reading or very close
  bool hand2Present = (distance2 <= MAX_RANGE);  // Any valid reading or very close
  
  // Debug output - uncomment to see sensor readings and audio state
  // Serial.print("D1: "); Serial.print(distance1); 
  // Serial.print(" D2: "); Serial.print(distance2);
  // Serial.print(" H1: "); Serial.print(hand1Present);
  // Serial.print(" H2: "); Serial.print(hand2Present);
  // Serial.print(" Vol: "); Serial.println(audioVolume);
  
  if (!hand1Present && !hand2Present) {
    // Truly no hands detected - fade out volume (MiniMin approach)
    thereminActive = false;
    audioVolume = audioVolume - FADE_SPEED;
    if (audioVolume < 0) audioVolume = 0;
    return;
  }
  
  thereminActive = true;
  
  // Calculate frequency based on hand positions (MiniMin approach)
  if (hand1Present && hand2Present) {
    // Both hands detected - use average distance for frequency
    float workingDistance1 = constrain(distance1, MIN_RANGE - 0.5, MAX_RANGE);
    float workingDistance2 = constrain(distance2, MIN_RANGE - 0.5, MAX_RANGE);
    float avgDistance = (workingDistance1 + workingDistance2) / 2.0;
    
    // Map to frequency range - closer = higher frequency (like MiniMin)
    float ratio = (avgDistance - (MIN_RANGE - 0.5)) / (MAX_RANGE - (MIN_RANGE - 0.5));
    ratio = constrain(ratio, 0.0, 1.0);
    freq = MIN_FREQ + (MAX_FREQ - MIN_FREQ) * (1.0 - ratio);
    
  } else if (hand1Present) {
    // Left hand only - lower frequency range
    float workingDistance = constrain(distance1, MIN_RANGE - 0.5, MAX_RANGE);
    float ratio = (workingDistance - (MIN_RANGE - 0.5)) / (MAX_RANGE - (MIN_RANGE - 0.5));
    ratio = constrain(ratio, 0.0, 1.0);
    freq = MIN_FREQ + (BASE_FREQ - MIN_FREQ) * (1.0 - ratio);
    
  } else if (hand2Present) {
    // Right hand only - higher frequency range  
    float workingDistance = constrain(distance2, MIN_RANGE - 0.5, MAX_RANGE);
    float ratio = (workingDistance - (MIN_RANGE - 0.5)) / (MAX_RANGE - (MIN_RANGE - 0.5));
    ratio = constrain(ratio, 0.0, 1.0);
    freq = BASE_FREQ + (MAX_FREQ - BASE_FREQ) * (1.0 - ratio);
  }
  
  // Apply frequency smoothing and set oscillator frequency
  int smoothedFreq = freqAverage.next(freq);
  osc.setFreq(smoothedFreq);
  
  // Fade in volume when hands detected (MiniMin style)
  audioVolume = audioVolume + FADE_SPEED;
  if (audioVolume > 255) audioVolume = 255;
  
#else
  // Audio disabled - just update the theremin state for JSON output
  thereminActive = (hand1Present || hand2Present);
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
  
  // Read sensors - left sensor uses reset fix, right sensor uses working NewPing
  float distance1 = readDistanceWithReset(trigPin1, echoPin1);  // Left sensor with reset fix
  float distance2 = readDistanceNewPing(sonar2);                // Right sensor (working fine)
  
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
  
  // Update theremin audio (Mozzi approach)
  updateThereminAudio(avgDistance1, avgDistance2, inRange1, inRange2);
  
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
    
    // Add Mozzi audio parameters for TouchDesigner correlation
    if (thereminActive) {
#if ENABLE_AUDIO
      doc["audio_volume"] = audioVolume;
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

#if ENABLE_AUDIO
  // Required Mozzi audio hook - must be called frequently for audio generation
  audioHook();
#endif
  
  delay(10);  // Reduced delay for smoother audio with Mozzi
} 