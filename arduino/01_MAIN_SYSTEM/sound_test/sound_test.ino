/*
 * Modular Audio Effect System - DJ Scratch + Mozzi Alien Effect
 * Completely separate audio methodologies:
 * - DJ Scratch: Timer1 PWM + ISR for PROGMEM audio playback
 * - Alien Effect: Mozzi library with proper audioHook() integration
 * 
 * Toggles between effects every 5 seconds with complete audio system isolation
 */

// =============================================================================
// AUDIO MODE SELECTION - Only one can be active at a time
// =============================================================================
enum AudioMode {
  MODE_DJ_SCRATCH = 0,
  MODE_MOZZI_ALIEN = 1
};

volatile AudioMode currentMode = MODE_DJ_SCRATCH;
unsigned long lastModeChange = 0;
const unsigned long MODE_DURATION = 5000; // 5 seconds per mode

// =============================================================================
// SENSOR SYSTEM (Shared between both modes)
// =============================================================================
#define TRIG1 10  // Left sensor trigger
#define ECHO1 8   // Left sensor echo  
#define TRIG2 5   // Right sensor trigger
#define ECHO2 6   // Right sensor echo

// =============================================================================
// DJ SCRATCH SYSTEM (Timer1 + ISR + PROGMEM)
// =============================================================================
#include <avr/pgmspace.h>
#include "whileDancing_medium_data.h"

// DJ Scratch pins and state
#define AUDIO_PIN_DJ 12  // Timer1 OC1B output
volatile int32_t sampleIndex = 0;
volatile uint8_t sampleCounter = 0;
volatile uint8_t playState = 1;  // Start playing immediately
volatile uint8_t playbackSpeed = 5;
volatile int8_t scratchSpeed = 1;
volatile bool isScratchMode = false;

// DJ Scratch Timer1 ISR
ISR(TIMER1_COMPB_vect) {
  if (currentMode != MODE_DJ_SCRATCH) {
    OCR1B = 200; // Silence when not in DJ mode
    return;
  }
  
  if (playState == 1 && sampleIndex < WHILEDANCING_MEDIUM_DATA_NUM_CELLS) {
    sampleCounter++;
    if (sampleCounter >= playbackSpeed) {
      sampleCounter = 0;
      
      // Get sample and convert int8_t to uint8_t (add 128)
      int8_t rawSample = pgm_read_byte(&WHILEDANCING_MEDIUM_DATA_DATA[sampleIndex]);
      uint8_t sample = (uint8_t)(rawSample + 128);
      
      // Apply amplification (same as working DJ scratch)
      uint16_t amplified = ((uint16_t)sample * 3) / 2;
      if (amplified > 255) amplified = 255;
      
      OCR1B = amplified;
      
      if (isScratchMode) {
        sampleIndex += scratchSpeed;
        if (sampleIndex < 0) sampleIndex = 0;
        if (sampleIndex >= WHILEDANCING_MEDIUM_DATA_NUM_CELLS) sampleIndex = WHILEDANCING_MEDIUM_DATA_NUM_CELLS - 1;
      } else {
        sampleIndex++;
      }
    }
  } else {
    OCR1B = 128; // Silence
    if (sampleIndex >= WHILEDANCING_MEDIUM_DATA_NUM_CELLS) {
      sampleIndex = 0; // Loop
    }
  }
}

void setupDJScratch() {
  Serial.println(F("Setting up DJ Scratch mode..."));
  
  // Configure Timer1 PWM for DJ scratch (EXACT same as working code)
  pinMode(AUDIO_PIN_DJ, OUTPUT);
  TCCR1A = _BV(COM1B1) | _BV(WGM11);  // Clear OC1B on compare match, Fast PWM
  TCCR1B = _BV(WGM13) | _BV(CS10);    // Fast PWM, no prescaler
  ICR1 = 399;                         // 20kHz frequency
  OCR1B = 200;                        // 50% duty cycle (silence)
  TIMSK1 = _BV(OCIE1B);               // Enable Timer1 Compare B interrupt
  
  // Reset DJ scratch state
  sampleIndex = 0;
  sampleCounter = 0;
  playState = 1;
  playbackSpeed = 5;
  scratchSpeed = 1;
  isScratchMode = false;
}

void disableDJScratch() {
  Serial.println(F("Disabling DJ Scratch mode..."));
  
  // Disable Timer1 interrupt
  TIMSK1 &= ~_BV(OCIE1B);
  
  // Reset Timer1 to default state
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1B = 0;
  
  // Set audio pin to low
  digitalWrite(AUDIO_PIN_DJ, LOW);
}

// =============================================================================
// MOZZI ALIEN SYSTEM (Mozzi Library + audioHook)
// =============================================================================

// Mozzi configuration - MUST be before MozziGuts.h
#include <MozziConfigValues.h>
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_2PIN_PWM

#include <MozziGuts.h>
#include <Oscil.h>
#include <tables/sin2048_int8.h>

#define CONTROL_RATE 128

// Mozzi oscillators
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> alienOsc(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, CONTROL_RATE> vibrato(SIN2048_DATA);

// Alien effect state
float alienVolume = 0.5;
float alienPitch = 440.0;
float vibratoRate = 6.5;

void setupMozziAlien() {
  Serial.println(F("Setting up Mozzi Alien mode..."));
  
  // Start Mozzi
  startMozzi(CONTROL_RATE);
  
  // Initialize oscillators
  alienOsc.setFreq(alienPitch);
  vibrato.setFreq(vibratoRate);
  
  // Reset alien state
  alienVolume = 0.5;
  alienPitch = 440.0;
}

void disableMozziAlien() {
  Serial.println(F("Disabling Mozzi Alien mode..."));
  
  // Stop Mozzi (this disables its timers)
  stopMozzi();
}

// Mozzi required functions - these MUST always be available when Mozzi is running
void updateControl() {
  // Only process if we're in Mozzi mode, otherwise do minimal processing
  if (currentMode == MODE_MOZZI_ALIEN) {
    // Read sensors directly (like the working alien effect)
    float d1 = readSensor(TRIG1, ECHO1);
    float d2 = readSensor(TRIG2, ECHO2);
    
    bool leftHand = (d1 >= 1.0 && d1 <= 20.0);
    bool rightHand = (d2 >= 1.0 && d2 <= 20.0);
    
    // Update alien controls
    if (leftHand) {
      alienVolume = map(d1 * 10, 10, 200, 255, 50);
    } else {
      alienVolume = 0;
    }
    
    if (rightHand) {
      alienPitch = map(d2 * 10, 10, 200, 800, 200);
    } else {
      alienPitch = 400;
    }
    
    // Update oscillators
    vibrato.setFreq(vibratoRate);
    alienOsc.setFreq(alienPitch);
  }
}

int updateAudio() {
  // Only generate audio if we're in Mozzi mode
  if (currentMode == MODE_MOZZI_ALIEN) {
    // Generate alien sound with vibrato (simplified version)
    int vibratoValue = vibrato.next();
    float modulation = 1.0 + (vibratoValue / 512.0) * 0.1; // 10% vibrato depth
    
    int sample = alienOsc.next();
    sample = (int)(sample * alienVolume * modulation / 255);
    
    return sample;
  }
  
  return 0; // Silence when not in Mozzi mode
}

// =============================================================================
// SENSOR SYSTEM (Shared between both modes)
// =============================================================================

float readSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  unsigned long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999.0;
  
  float distance = (duration * 0.034) / 2;
  return (distance > 60.0) ? 999.0 : distance;
}

// =============================================================================
// CONTROL HANDLERS (Mode-specific)
// =============================================================================

void handleDJScratchControls(bool leftHand, bool rightHand, float d2) {
  // LEFT HAND - Play/Stop
  if (leftHand) {
    if (playState == 0) {
      playState = 1;
      sampleIndex = 0;
      Serial.println(F("DJ: PLAY"));
    }
  } else {
    if (playState == 1) {
      playState = 0;
      Serial.println(F("DJ: STOP"));
    }
  }
  
  // RIGHT HAND - Scratch + Speed Control (same logic as working DJ scratch)
  static bool prevRightHand = false;
  static unsigned long lastTransition = 0;
  static uint8_t transitionCount = 0;
  static int8_t scratchDirection = 1;
  static unsigned long rightHandStartTime = 0;
  static unsigned long lastRightHandDetected = 0;
  
  if (rightHand) {
    lastRightHandDetected = millis();
  }
  
  if (rightHand != prevRightHand) {
    lastTransition = millis();
    transitionCount++;
    scratchDirection = -scratchDirection;
  }
  
  if (rightHand && !prevRightHand) {
    rightHandStartTime = millis();
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
      transitionCount = 0;
      Serial.println(F("DJ: NORMAL"));
    }
  }
  // Scratch mode - rapid transitions
  else if (rightHand && transitionCount >= 2 && (millis() - lastTransition) < 300) {
    isScratchMode = true;
    
    if (transitionCount >= 4) {
      scratchSpeed = scratchDirection * 6;
      Serial.println(F("DJ: SCRATCH+"));
    } else {
      scratchSpeed = scratchDirection * 3;
      Serial.println(F("DJ: SCRATCH"));
    }
  }
  // Distance speed control
  else if (rightHand && (millis() - rightHandStartTime) > 300) {
    isScratchMode = false;
    scratchSpeed = 1;
    
    float ratio = (d2 - 1.0) / (20.0 - 1.0);
    ratio = constrain(ratio, 0.0, 1.0);
    playbackSpeed = (uint8_t)(5 + (10 * ratio));  // 5 (normal) to 15 (slower)
    
    static unsigned long lastSpeedReport = 0;
    if (millis() - lastSpeedReport > 2000) {
      Serial.print(F("DJ: SPEED:"));
      Serial.println(playbackSpeed);
      lastSpeedReport = millis();
    }
  }
  
  prevRightHand = rightHand;
}

void handleAlienControls(bool leftHand, bool rightHand, float d1, float d2) {
  // Left hand controls volume (closer = louder)
  if (leftHand) {
    float volumeRatio = (20.0 - d1) / 19.0;  // Closer = higher ratio
    volumeRatio = constrain(volumeRatio, 0.0, 1.0);
    alienVolume = volumeRatio;
  } else {
    alienVolume = 0.3;  // Default volume when no left hand
  }
  
  // Right hand controls pitch (closer = higher pitch)
  if (rightHand) {
    float pitchRatio = (20.0 - d2) / 19.0;  // Closer = higher ratio
    pitchRatio = constrain(pitchRatio, 0.0, 1.0);
    alienPitch = 200.0 + (pitchRatio * 600.0);  // 200Hz to 800Hz
  } else {
    alienPitch = 440.0;  // Default pitch
  }
  
  // Status reporting
  static unsigned long lastAlienReport = 0;
  if (millis() - lastAlienReport > 2000) {
    Serial.print(F("Alien - Volume: "));
    Serial.print(alienVolume);
    Serial.print(F(", Pitch: "));
    Serial.print(alienPitch);
    Serial.println(F("Hz"));
    lastAlienReport = millis();
  }
}

// =============================================================================
// MAIN SETUP AND LOOP
// =============================================================================

void setup() {
  Serial.begin(9600);
  Serial.println(F("Modular Audio System Starting..."));
  
  // Setup sensor pins
  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  
  // Start with DJ Scratch mode
  setupDJScratch();
  lastModeChange = millis();
  
  Serial.println(F("Started in DJ SCRATCH mode"));
}

void loop() {
  // Check if it's time to toggle audio modes
  if (millis() - lastModeChange >= MODE_DURATION) {
    if (currentMode == MODE_DJ_SCRATCH) {
      // Switch to Mozzi Alien
      disableDJScratch();
      currentMode = MODE_MOZZI_ALIEN;
      setupMozziAlien();
      Serial.println(F("=== SWITCHED TO MOZZI ALIEN MODE ==="));
    } else {
      // Switch to DJ Scratch
      disableMozziAlien();
      currentMode = MODE_DJ_SCRATCH;
      setupDJScratch();
      Serial.println(F("=== SWITCHED TO DJ SCRATCH MODE ==="));
    }
    lastModeChange = millis();
  }
  
  // CRITICAL: audioHook() must be called EVERY loop iteration for Mozzi
  // This is the key difference from our previous implementation
  if (currentMode == MODE_MOZZI_ALIEN) {
    audioHook();
  } else if (currentMode == MODE_DJ_SCRATCH) {
    // Handle DJ scratch controls (Mozzi handles its own controls in updateControl)
    static unsigned long lastSensorRead = 0;
    if (millis() - lastSensorRead >= 30) {  // Read sensors every 30ms for DJ scratch
      float d1 = readSensor(TRIG1, ECHO1);
      float d2 = readSensor(TRIG2, ECHO2);
      
      bool leftHand = (d1 >= 1.0 && d1 <= 20.0);
      bool rightHand = (d2 >= 1.0 && d2 <= 20.0);
      
      handleDJScratchControls(leftHand, rightHand, d2);
      lastSensorRead = millis();
    }
    
    delay(5);  // Small delay for DJ scratch mode
  }
}
