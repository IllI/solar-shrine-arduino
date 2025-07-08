/*
 * Solar Shrine - Effect Rotator System
 * MEMORY-OPTIMIZED AUTO-ROTATION for Arduino Uno
 * Shared audio objects, efficient memory usage
 */

// ===========================================
// EFFECT CONFIGURATION
// ===========================================
#define ENABLE_DJ_SCRATCH false  // Set to true to enable DJ Scratch (Timer1 ISR)
                                // Note: DJ Scratch conflicts with Mozzi, so choose one or the other

// ===========================================
// LIBRARIES & INCLUDES
// ===========================================
#include <Arduino.h>

// Mozzi effects (1-3) - always included
#include <MozziGuts.h>
#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <RollingAverage.h>
#include <mozzi_midi.h>
#define CONTROL_RATE 128

// DJ Scratch needs audio data and avr/pgmspace
#if ENABLE_DJ_SCRATCH
  #include <avr/pgmspace.h>
  #include "audio_data.h"  // PROGMEM audio data
#endif

// ===========================================
// HARDWARE CONFIGURATION
// ===========================================
const int TRIG_PIN1 = 10;
const int ECHO_PIN1 = 11;
const int TRIG_PIN2 = 5;
const int ECHO_PIN2 = 6;
const int AUDIO_PIN = 9;
const int LED_PIN = 13;

const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

// ===========================================
// AUTO-ROTATION SYSTEM
// ===========================================
enum EffectType {
  EFFECT_MUSICAL_THEREMIN = 1,
  EFFECT_ALIEN_SOUND = 2,
  EFFECT_ROBOTS_TALKING = 3,
  EFFECT_DJ_SCRATCH = 4
};

int currentEffect = 1;  // Start with Musical Theremin
bool effectActive = false;
bool handsWereDetected = false;
unsigned long lastHandsDetectedTime = 0;
const unsigned long NEXT_EFFECT_DELAY = 5000;  // 5 seconds

// ===========================================
// SHARED AUDIO OBJECTS (Memory Optimized)
// ===========================================
// ONE oscillator shared by all effects
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> mainOsc(SIN2048_DATA);
// ONE vibrato oscillator shared by effects that need it
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> vibratoOsc(SIN2048_DATA);
// ONE rolling average shared by all effects
RollingAverage<int, 4> sharedFreqAverage;

// Small echo buffer for Alien Sound effect (reduced from 512 to 128)
#define ECHO_BUFFER_SIZE 128  // 128 ints = 256 bytes (was 1024 bytes)
int echoBuffer[ECHO_BUFFER_SIZE];
int echoIndex = 0;

// Robot talking frequencies (small array, keep in RAM for simplicity)
const float robotFreqs[] = {146.83, 164.81, 174.61, 196.00, 220.00, 246.94};
const int numFreqs = sizeof(robotFreqs) / sizeof(robotFreqs[0]);

// Effect-specific variables (minimal storage)
float currentNote = 146.83f;
unsigned long lastNoteTime = 0;
const unsigned long NOTE_HOLD_TIME = 500;

// DJ Scratch variables (only if enabled)
#if ENABLE_DJ_SCRATCH
volatile int32_t sampleIndex = 0;
volatile uint8_t sampleCounter = 0;
volatile uint8_t playState = 0;
volatile uint8_t playbackSpeed = 5;
volatile int8_t scratchSpeed = 1;
volatile bool isScratchMode = false;
#endif

// ===========================================
// EFFECT FUNCTIONS
// ===========================================

const char* getEffectName(int effect) {
  switch(effect) {
    case 1: return "Musical Theremin";
    case 2: return "Alien Sound";
    case 3: return "Robots Talking";
    case 4: return "DJ Scratch";
    default: return "Unknown";
  }
}

void switchToNextEffect() {
  // Increment effect number
  currentEffect++;
  
  // Skip DJ Scratch if not enabled
  if (currentEffect == 4 && !ENABLE_DJ_SCRATCH) {
    currentEffect = 1;  // Wrap to first effect
  }
  
  // Wrap around after effect 4
  if (currentEffect > 4) {
    currentEffect = 1;
  }
  
  Serial.println();
  Serial.println("=== AUTO-ROTATION ===");
  Serial.print("Effect ");
  Serial.print(currentEffect);
  Serial.print(": ");
  Serial.println(getEffectName(currentEffect));
  Serial.println("======================");
  
  // Initialize new effect
  initializeCurrentEffect();
}

void initializeCurrentEffect() {
  switch(currentEffect) {
    case 1: // Musical Theremin
      mainOsc.setFreq(440.0f);
      break;
      
    case 2: // Alien Sound
      mainOsc.setFreq(440.0f);
      vibratoOsc.setFreq(8.0f);
      // Clear echo buffer
      for (int i = 0; i < ECHO_BUFFER_SIZE; i++) {
        echoBuffer[i] = 0;
      }
      echoIndex = 0;
      break;
      
    case 3: // Robots Talking
      mainOsc.setFreq(146.83f);
      vibratoOsc.setFreq(6.0f);
      lastNoteTime = 0;
      currentNote = 146.83f;
      break;
      
    case 4: // DJ Scratch
      #if ENABLE_DJ_SCRATCH
        setupDJScratch();
      #endif
      break;
  }
}

// ===========================================
// SENSOR READING
// ===========================================
float readSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  unsigned long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 0.5;  // Very close reading
  
  float distance = (duration * 0.034) / 2.0;
  return distance;
}

// ===========================================
// MOZZI AUDIO FUNCTIONS
// ===========================================
void updateControl() {
  float leftDistance = readSensor(TRIG_PIN1, ECHO_PIN1);
  float rightDistance = readSensor(TRIG_PIN2, ECHO_PIN2);
  
  bool leftHand = (leftDistance >= MIN_RANGE && leftDistance <= MAX_RANGE);
  bool rightHand = (rightDistance >= MIN_RANGE && rightDistance <= MAX_RANGE);
  
  effectActive = (leftHand || rightHand);
  
  // Update current effect using shared objects
  switch(currentEffect) {
    case 1: updateMusicalTheremin(leftDistance, rightDistance, leftHand, rightHand); break;
    case 2: updateAlienSound(leftDistance, rightDistance, leftHand, rightHand); break;
    case 3: updateRobotsTalking(leftDistance, rightDistance, leftHand, rightHand); break;
  }
  
  // Handle auto-rotation
  handleAutoRotation(effectActive);
}

int updateAudio() {
  switch(currentEffect) {
    case 1: return updateMusicalThereminAudio();
    case 2: return updateAlienSoundAudio();
    case 3: return updateRobotsTalkingAudio();
    default: return 0;
  }
}

// ===========================================
// EFFECT 1: MUSICAL THEREMIN
// ===========================================
// Musical scale - C Major Pentatonic (always sounds good, no wrong notes)
int musicalScale[] = {
  131, 147, 165, 196, 220,           // C3 pentatonic
  262, 294, 330, 392, 440,           // C4 pentatonic (middle range)
  523, 587, 659, 784, 880,           // C5 pentatonic
  1047, 1175, 1319, 1568             // C6 pentatonic (high end)
};
const int SCALE_LENGTH = 19;

// Musical theremin state
int musicalThereminVolume = 0;
int baseFrequency = 440;  // A4 reference note
int musicalCurrentNote = 0;
int musicalTargetNote = 0;
unsigned long musicalLastNoteChange = 0;
const int NOTE_PERSISTENCE_MS = 75;  // Hold note for 75ms before changing

// Find closest note in musical scale
int findClosestNote(int frequency) {
  int bestNote = 0;
  int smallestDiff = abs(musicalScale[0] - frequency);
  
  for (int i = 1; i < SCALE_LENGTH; i++) {
    int diff = abs(musicalScale[i] - frequency);
    if (diff < smallestDiff) {
      smallestDiff = diff;
      bestNote = i;
    }
  }
  return bestNote;
}

// Apply musical note persistence (prevents accidental notes)
int applyNotePersistence(int newNote) {
  if (newNote != musicalTargetNote) {
    musicalTargetNote = newNote;
    musicalLastNoteChange = millis();
  }
  
  if (millis() - musicalLastNoteChange > NOTE_PERSISTENCE_MS) {
    musicalCurrentNote = musicalTargetNote;
  }
  
  return musicalCurrentNote;
}

void updateMusicalTheremin(float leftDistance, float rightDistance, bool leftHand, bool rightHand) {
  static const int MUSICAL_MIN_FREQ = 120;    // Research-proven musical low end
  static const int MUSICAL_MAX_FREQ = 1500;   // Research-proven musical high end
  static const int PITCH_NEAR = 2;            // Very close pitch detection (cm)
  static const int PITCH_FAR = 20;            // Far pitch detection (cm)
  static const int VOL_NEAR = 2;              // Very close volume detection (cm)
  static const int VOL_FAR = 18;              // Far volume detection (cm)
  
  // Process pitch (both hands or right hand for pitch)
  int rawFreq = 440;  // Default A4
  bool pitchActive = false;
  
  if (leftHand || rightHand) {
    float avgDistance = leftHand && rightHand ? (leftDistance + rightDistance) / 2.0 : 
                        rightHand ? rightDistance : leftDistance;
    
    if (avgDistance >= PITCH_NEAR && avgDistance <= PITCH_FAR) {
      // Map to musical frequency range
      rawFreq = map(avgDistance, PITCH_NEAR, PITCH_FAR, MUSICAL_MAX_FREQ, MUSICAL_MIN_FREQ);
      pitchActive = true;
    }
  }
  
  // Apply musical scale quantization with note persistence
  int noteIndex = findClosestNote(rawFreq);
  int persistentNote = applyNotePersistence(noteIndex);
  baseFrequency = musicalScale[persistentNote];
  
  // Smooth the frequency changes
  baseFrequency = sharedFreqAverage.next(baseFrequency);
  
  // Process volume (left hand primarily)
  int rawVolume = 0;
  bool volumeActive = false;
  
  if (leftHand || rightHand) {
    float volDistance = leftHand ? leftDistance : rightDistance;
    
    if (volDistance >= VOL_NEAR && volDistance <= VOL_FAR) {
      // Map to volume with exponential curve for natural response
      float linearVol = map(volDistance, VOL_NEAR, VOL_FAR, 255, 0) / 255.0;
      float expVol = linearVol * linearVol * linearVol;  // Cubic for gentle response
      rawVolume = (int)(expVol * 255);
      volumeActive = true;
    }
  }
  
  // Apply volume smoothing and fade
  if (volumeActive || pitchActive) {
    musicalThereminVolume = rawVolume;
  } else {
    musicalThereminVolume = 0;  // Fade out when no hands
  }
  
  // Apply gentle musical vibrato
  float vibratoMod = vibratoOsc.next() * 0.015;  // 1.5% frequency modulation
  int modulatedFreq = baseFrequency + (int)(baseFrequency * vibratoMod);
  
  // Set the main oscillator frequency
  mainOsc.setFreq(modulatedFreq);
  
  // Debug output (reduced frequency)
  static int debugCount = 0;
  if (++debugCount > 100) {
    Serial.print("Note: ");
    Serial.print(musicalScale[persistentNote]);
    Serial.print("Hz | Vol: ");
    Serial.print(musicalThereminVolume);
    Serial.print(" | ");
    if (pitchActive || volumeActive) Serial.print("ACTIVE");
    else Serial.print("SILENT");
    Serial.println();
    debugCount = 0;
  }
}

int updateMusicalThereminAudio() {
  int sample = mainOsc.next();
  return (sample * musicalThereminVolume) >> 8;
}

// ===========================================
// EFFECT 2: ALIEN SOUND
// ===========================================
void updateAlienSound(float leftDistance, float rightDistance, bool leftHand, bool rightHand) {
  if (effectActive) {
    float avgDistance = leftHand && rightHand ? (leftDistance + rightDistance) / 2.0 : 
                        leftHand ? leftDistance : rightDistance;
    
    // Alien frequency mapping with vibrato
    float baseFreq = map(avgDistance, MIN_RANGE, MAX_RANGE, 100.0, 2000.0);
    float vibratoAmount = (float)vibratoOsc.next() * 0.05;
    float frequency = baseFreq + (baseFreq * vibratoAmount);
    
    // Smooth using shared rolling average
    int smoothedFreq = sharedFreqAverage.next(frequency);
    mainOsc.setFreq(smoothedFreq);
  }
}

int updateAlienSoundAudio() {
  static int audioVolume = 0;
  if (effectActive) audioVolume = 180;
  else audioVolume = 0;
  
  int sample = mainOsc.next();
  
  // Add echo effect using small buffer
  int echoSample = echoBuffer[echoIndex];
  echoBuffer[echoIndex] = (sample + echoSample) >> 1;
  echoIndex = (echoIndex + 1) % ECHO_BUFFER_SIZE;
  
  int finalSample = (sample + echoSample) >> 1;
  return (finalSample * audioVolume) >> 8;
}

// ===========================================
// EFFECT 3: ROBOTS TALKING
// ===========================================
void updateRobotsTalking(float leftDistance, float rightDistance, bool leftHand, bool rightHand) {
  if (effectActive) {
    float avgDistance = leftHand && rightHand ? (leftDistance + rightDistance) / 2.0 : 
                        leftHand ? leftDistance : rightDistance;
    
    // Map to robot frequency index
    int freqIndex = map(avgDistance, MIN_RANGE, MAX_RANGE, 0, numFreqs - 1);
    freqIndex = constrain(freqIndex, 0, numFreqs - 1);
    
    // Note persistence logic
    if (millis() - lastNoteTime > NOTE_HOLD_TIME) {
      currentNote = robotFreqs[freqIndex];
      lastNoteTime = millis();
    }
    
    // Add vibrato
    float vibratoAmount = (float)vibratoOsc.next() * 0.03;
    float frequency = currentNote + (currentNote * vibratoAmount);
    
    // Smooth using shared rolling average
    int smoothedFreq = sharedFreqAverage.next(frequency);
    mainOsc.setFreq(smoothedFreq);
  }
}

int updateRobotsTalkingAudio() {
  static int audioVolume = 0;
  if (effectActive) audioVolume = 180;
  else audioVolume = 0;
  
  int sample = mainOsc.next();
  return (sample * audioVolume) >> 8;
}

// ===========================================
// EFFECT 4: DJ SCRATCH (Optional)
// ===========================================
#if ENABLE_DJ_SCRATCH
void setupDJScratch() {
  // Timer1 PWM setup
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = 399;
  OCR1A = ICR1 / 2;
  TIMSK1 = _BV(OCIE1A);
  
  Serial.println("DJ Scratch Ready");
}

// Timer1 ISR
ISR(TIMER1_COMPA_vect) {
  sampleCounter++;
  
  uint8_t currentSpeed = isScratchMode ? 2 : playbackSpeed;
  
  if (sampleCounter >= currentSpeed) {
    sampleCounter = 0;
    
    if (playState == 1) {
      if (sampleIndex < 0) sampleIndex = 0;
      if (sampleIndex >= AUDIO_SAMPLE_COUNT) sampleIndex = 0;
      
      uint8_t sample = pgm_read_byte(&audioData[sampleIndex]);
      int16_t amp = ((int16_t)sample - 128) * 4;
      amp = constrain(amp, -128, 127);
      
      OCR1A = ((uint32_t)(amp + 128) * ICR1) / 255;
      
      if (isScratchMode) {
        sampleIndex += scratchSpeed;
        if (sampleIndex < 0) sampleIndex = AUDIO_SAMPLE_COUNT - 1;
        if (sampleIndex >= AUDIO_SAMPLE_COUNT) sampleIndex = 0;
      } else {
        sampleIndex++;
        if (sampleIndex >= AUDIO_SAMPLE_COUNT) sampleIndex = 0;
      }
      
    } else {
      OCR1A = ICR1 / 2;
    }
  }
}

void updateDJScratch() {
  float leftDistance = readSensor(TRIG_PIN1, ECHO_PIN1);
  float rightDistance = readSensor(TRIG_PIN2, ECHO_PIN2);
  
  bool leftHand = (leftDistance >= MIN_RANGE && leftDistance <= MAX_RANGE);
  bool rightHand = (rightDistance >= MIN_RANGE && rightDistance <= MAX_RANGE);
  
  effectActive = (leftHand || rightHand);
  
  // LEFT HAND - Play/Stop
  if (leftHand) {
    if (playState == 0) {
      playState = 1;
      sampleIndex = 0;
      Serial.println("PLAY");
    }
  } else {
    if (playState == 1) {
      playState = 0;
      Serial.println("STOP");
    }
  }
  
  // RIGHT HAND - Scratch + Speed Control
  static bool prevRightHand = false;
  static unsigned long lastTransition = 0;
  static uint8_t transitionCount = 0;
  static int8_t scratchDirection = 1;
  
  if (rightHand != prevRightHand) {
    lastTransition = millis();
    transitionCount++;
    scratchDirection = -scratchDirection;
  }
  
  if (millis() - lastTransition > 500) {
    transitionCount = 0;
  }
  
  if (rightHand && transitionCount >= 2) {
    isScratchMode = true;
    scratchSpeed = scratchDirection * 3;
    Serial.println("SCRATCH");
  } else {
    isScratchMode = false;
    scratchSpeed = 1;
    
    if (rightHand) {
      float ratio = (rightDistance - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
      ratio = constrain(ratio, 0.0, 1.0);
      playbackSpeed = (uint8_t)(5 + (10 * ratio));
    }
  }
  
  prevRightHand = rightHand;
  
  // Handle auto-rotation
  handleAutoRotation(effectActive);
}
#endif

// ===========================================
// AUTO-ROTATION LOGIC
// ===========================================
void handleAutoRotation(bool currentHandsDetected) {
  if (currentHandsDetected) {
    lastHandsDetectedTime = millis();
    handsWereDetected = true;
  } else {
    if (handsWereDetected) {
      unsigned long elapsed = millis() - lastHandsDetectedTime;
      
      // Show countdown every second
      static unsigned long lastCountdownTime = 0;
      if (elapsed >= 1000 && elapsed < NEXT_EFFECT_DELAY) {
        if (millis() - lastCountdownTime >= 1000) {
          int secondsRemaining = (NEXT_EFFECT_DELAY - elapsed) / 1000;
          Serial.print("Auto-rotation in ");
          Serial.print(secondsRemaining);
          Serial.println("s...");
          lastCountdownTime = millis();
        }
      }
      
      if (elapsed >= NEXT_EFFECT_DELAY) {
        switchToNextEffect();
        handsWereDetected = false;
      }
    }
  }
}

// ===========================================
// SETUP & LOOP
// ===========================================
void setup() {
  Serial.begin(9600);
  delay(1000);
  
  // Pin setup
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);
  pinMode(AUDIO_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Show startup message
  Serial.println("Solar Shrine - AUTO-ROTATION Effect Rotator");
  Serial.println("Memory-optimized for Arduino Uno");
  Serial.println("Effects: 1.Musical 2.Alien 3.Robots");
  if (ENABLE_DJ_SCRATCH) {
    Serial.println("4.DJ Scratch ENABLED");
  } else {
    Serial.println("4.DJ Scratch DISABLED");
  }
  Serial.println();
  
  // Initialize Mozzi (for effects 1-3)
  if (!ENABLE_DJ_SCRATCH) {
    startMozzi(CONTROL_RATE);
  }
  
  // Initialize first effect
  initializeCurrentEffect();
  
  Serial.print("Starting: ");
  Serial.println(getEffectName(currentEffect));
  Serial.println();
}

void loop() {
  // Handle current effect
  if (currentEffect == 4 && ENABLE_DJ_SCRATCH) {
    #if ENABLE_DJ_SCRATCH
      updateDJScratch();
      delay(30);
    #endif
  } else {
    // Mozzi effects (1-3)
    audioHook();  // Required for Mozzi
  }
  
  // LED feedback
  digitalWrite(LED_PIN, effectActive ? HIGH : LOW);
} 