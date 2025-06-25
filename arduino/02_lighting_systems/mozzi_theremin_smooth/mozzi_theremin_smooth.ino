/*
 * Solar Shrine Smooth Mozzi Theremin
 * Professional-quality theremin with volume control and smooth audio transitions
 * Uses Mozzi library for high-quality synthesis
 * 
 * Hardware:
 * - 2x HC-SR04 ultrasonic sensors (pins 5,6,9,10)
 * - WS2812B/WS2815 LED strip (pin 3)
 * - Audio output on pin 9 (Mozzi default)
 * - Optional: 10k potentiometer on A0 for master volume
 * 
 * Libraries Required:
 * - Mozzi (install via Library Manager)
 * - FastLED
 * - NewPing
 * - ArduinoJson
 */

#include <MozziGuts.h>
#include <Oscil.h>
#include <tables/sin2048_int8.h>  // Sine wave table
#include <tables/saw2048_int8.h>  // Saw wave table
#include <ADSR.h>                 // Envelope generator
#include <LowPassFilter.h>        // Audio filter
#include <Smooth.h>               // Control smoothing
#include <mozzi_midi.h>

#include <FastLED.h>
#include <NewPing.h>
#include <ArduinoJson.h>

#define CONTROL_RATE 256  // Higher control rate for smoother response

// Sensor pins
const int trigPin1 = 9;
const int echoPin1 = 10;
const int trigPin2 = 5;
const int echoPin2 = 6;

// NewPing sensor objects
NewPing sonar1(trigPin1, echoPin1, 200);
NewPing sonar2(trigPin2, echoPin2, 200);

// LED strip configuration
#define LED_PIN 3
#define NUM_LEDS 60
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

// Volume control
const int VOLUME_PIN = A0;
float masterVolume = 0.7;  // Start at 70% volume

// Range constants (in cm)
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

// Theremin frequency ranges
const float MIN_FREQ = 80.0;
const float MAX_FREQ = 2000.0;
const float BASE_FREQ = 220.0;

// Mozzi audio objects
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aSin(SIN2048_DATA);   // Main oscillator
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aVibrato(SIN2048_DATA); // Vibrato LFO
Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> aSaw(SAW2048_DATA);   // Alternative waveform

// Envelope for smooth attack/release
ADSR<CONTROL_RATE, AUDIO_RATE> envelope;

// Audio filter for warmth
LowPassFilter lpf;

// Control smoothing
Smooth<float> smoothFreq(0.85f);    // Smooth frequency changes
Smooth<float> smoothVolume(0.9f);   // Smooth volume changes
Smooth<int> smoothDistance1(0.8f);  // Smooth sensor 1
Smooth<int> smoothDistance2(0.8f);  // Smooth sensor 2

// Mode constants
enum LightMode {
  ATTRACT_MODE,
  INTERACTIVE_MODE
};

// State variables
LightMode currentMode = ATTRACT_MODE;
unsigned long lastHandDetectedTime = 0;
const unsigned long INTERACTIVE_TIMEOUT = 10000;

// Attract mode variables
const float ATTRACT_PERIOD = 5000.0;
float attractPhaseOffset = 0.0;
bool usePhaseOffset = false;

// Audio state
bool thereminActive = false;
float currentFrequency = 0;
float targetFrequency = 0;
float currentAmplitude = 0;
float targetAmplitude = 0;

// Vibrato settings
const float VIBRATO_DEPTH = 0.02;   // 2% frequency modulation
const float VIBRATO_RATE = 4.5;     // Hz

// Last known colors for transitions
CRGB lastLeftColor = CRGB::Yellow;
CRGB lastRightColor = CRGB::Yellow;

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  // FastLED setup
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(150);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  // Mozzi setup
  startMozzi(CONTROL_RATE);
  
  // Configure audio objects
  aSin.setFreq(440);
  aVibrato.setFreq(VIBRATO_RATE);
  
  // ADSR envelope settings (attack, decay, sustain, release in ms)
  envelope.setADLevels(255, 180);           // Attack and decay levels
  envelope.setTimes(100, 200, 50000, 400); // Smooth attack, quick release
  
  // Low-pass filter for warmth
  lpf.setResonance(120);
  lpf.setCutoffFreq(200);
  
  // Play startup chord
  playStartupChord();
}

void playStartupChord() {
  // Play a pleasant ascending chord
  int notes[] = {220, 277, 330, 440}; // A3, C#4, E4, A4
  
  for (int i = 0; i < 4; i++) {
    aSin.setFreq(notes[i]);
    envelope.noteOn();
    
    // Let note play for a bit
    for (int j = 0; j < 50; j++) {
      audioHook();
      delay(10);
    }
    
    envelope.noteOff();
    
    // Brief pause between notes
    for (int j = 0; j < 20; j++) {
      audioHook();
      delay(5);
    }
  }
}

float readDistanceNewPing(NewPing &sensor) {
  unsigned int distance = sensor.ping_cm();
  if (distance == 0) {
    return MAX_RANGE + 1;
  }
  return (float)distance;
}

CRGB getInteractiveColor(float distance) {
  if (distance < MIN_RANGE || distance > MAX_RANGE) {
    return CRGB::Black;
  }
  
  float ratio = (distance - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
  ratio = constrain(ratio, 0.0, 1.0);
  
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
  
  float sineValue = (sin(phase) + 1.0) / 2.0;
  
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
    return 0;
  }
  
  float frequency = 0;
  
  if (inRange1 && inRange2) {
    // Both hands - create harmony
    float avgDistance = (distance1 + distance2) / 2.0;
    float ratio = (avgDistance - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    ratio = constrain(ratio, 0.0, 1.0);
    frequency = MIN_FREQ + (MAX_FREQ - MIN_FREQ) * (1.0 - ratio);
    
  } else if (inRange1) {
    // Left hand - lower frequencies
    float ratio = (distance1 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    ratio = constrain(ratio, 0.0, 1.0);
    frequency = MIN_FREQ + (BASE_FREQ - MIN_FREQ) * (1.0 - ratio);
    
  } else if (inRange2) {
    // Right hand - higher frequencies
    float ratio = (distance2 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    ratio = constrain(ratio, 0.0, 1.0);
    frequency = BASE_FREQ + (MAX_FREQ - BASE_FREQ) * (1.0 - ratio);
  }
  
  return frequency;
}

void updateLEDs(float distance1, float distance2, bool handsDetected, unsigned long currentTime) {
  if (currentMode == ATTRACT_MODE) {
    CRGB attractColor = getAttractColor(currentTime);
    fill_solid(leds, NUM_LEDS, attractColor);
    
  } else {
    // Interactive mode
    CRGB leftColor = getInteractiveColor(distance1);
    if (leftColor == CRGB::Black) leftColor = lastLeftColor;
    else lastLeftColor = leftColor;
    
    CRGB rightColor = getInteractiveColor(distance2);
    if (rightColor == CRGB::Black) rightColor = lastRightColor;
    else lastRightColor = rightColor;
    
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

void updateControl() {
  unsigned long currentTime = millis();
  
  // Read and smooth sensor distances
  float distance1 = readDistanceNewPing(sonar1);
  float distance2 = readDistanceNewPing(sonar2);
  
  float smoothedDistance1 = smoothDistance1.next(distance1);
  float smoothedDistance2 = smoothDistance2.next(distance2);
  
  // Check if hands are detected
  bool inRange1 = (smoothedDistance1 >= MIN_RANGE && smoothedDistance1 <= MAX_RANGE);
  bool inRange2 = (smoothedDistance2 >= MIN_RANGE && smoothedDistance2 <= MAX_RANGE);
  bool handsDetected = inRange1 || inRange2;
  
  // Update volume from potentiometer if available
  if (VOLUME_PIN != -1) {
    int volumeReading = mozziAnalogRead(VOLUME_PIN);
    masterVolume = smoothVolume.next(volumeReading / 1023.0);
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
      
      CRGB transitionColor = (lastLeftColor.r + lastLeftColor.g > lastRightColor.r + lastRightColor.g) ? 
                            lastLeftColor : lastRightColor;
      calculatePhaseOffset(transitionColor);
      
      currentMode = ATTRACT_MODE;
    }
  }
  
  // Calculate theremin frequency
  float thereminFreq = calculateThereminFrequency(smoothedDistance1, smoothedDistance2, inRange1, inRange2);
  
  // Smooth frequency transitions
  if (thereminFreq > 0) {
    if (!thereminActive) {
      envelope.noteOn();
      thereminActive = true;
    }
    targetFrequency = thereminFreq;
    targetAmplitude = masterVolume;
  } else {
    if (thereminActive) {
      envelope.noteOff();
      thereminActive = false;
    }
    targetAmplitude = 0;
  }
  
  // Apply smoothing
  currentFrequency = smoothFreq.next(targetFrequency);
  currentAmplitude = smoothVolume.next(targetAmplitude);
  
  // Update oscillator frequency with vibrato
  float vibrato = aVibrato.next() * VIBRATO_DEPTH * currentFrequency;
  aSin.setFreq(currentFrequency + vibrato);
  
  // Update filter cutoff based on frequency for warmth
  lpf.setCutoffFreq(currentFrequency * 2);
  
  // Update LEDs
  updateLEDs(smoothedDistance1, smoothedDistance2, handsDetected, currentTime);
}

int updateAudio() {
  // Generate audio sample
  int sample = aSin.next();
  
  // Apply envelope
  sample = (int)((long)sample * envelope.next()) >> 8;
  
  // Apply low-pass filter for warmth
  sample = lpf.next(sample);
  
  // Apply volume control
  sample = (int)((long)sample * currentAmplitude * 255) >> 8;
  
  return sample;
}

void loop() {
  audioHook(); // Required for Mozzi
} 