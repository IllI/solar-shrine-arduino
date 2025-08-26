/*  Solar Shrine Rotation Test - Mozzi Theremin Echo + DJ Scratch
     Cycles between Mozzi theremin echo effect and DJ scratch effect every 10 seconds
     
     This demonstrates hardware-isolated rotation between two incompatible audio systems:
     - Mozzi theremin echo: 2-pin PWM synthesis with echo effects 
     - DJ scratch: Timer1 PWM with PROGMEM audio playback
     
     Based on successful rotation system from solar_shrine_playa.ino
     Using complete hardware teardown/reconstruction between modes
  
     The circuit: 
  
     Audio output on digital pin 12 (configured for Solar Shrine)
     Following MODULAR_AUDIO_SYSTEM_GUIDE.md and CUSTOM_SHIELD_WIRING.md
  
     Ultrasonic sensors (HC-SR04) as specified in CUSTOM_SHIELD_WIRING.md:
     Left sensor: Trig=Pin 10, Echo=Pin 11
     Right sensor: Trig=Pin 5, Echo=Pin 6 
  
    Mozzi documentation/API 
    https://sensorium.github.io/Mozzi/doc/html/index.html 
  
    Mozzi help/discussion/announcements: 
    https://groups.google.com/forum/#!forum/mozzi-users 
  
    Copyright 2012-2024 Tim Barrass and the Mozzi Team 
  
    Mozzi is licensed under the GNU Lesser General Public Licence (LGPL) Version 2.1 or later. 
 */ 
  
// =============================================================================
// AUDIO MODE SYSTEM - Hardware-isolated rotation between incompatible audio systems
// =============================================================================
enum AudioMode {
  MODE_MOZZI_THEREMIN_ECHO = 0,
  MODE_DJ_SCRATCH = 1
};

AudioMode currentMode = MODE_MOZZI_THEREMIN_ECHO;
static unsigned long lastModeChange = 0;
const unsigned long MODE_DURATION = 10000; // 10 seconds per mode

// =============================================================================
// DJ SCRATCH SYSTEM (PROGMEM audio playback)
// =============================================================================
#include <avr/pgmspace.h>
#include "audio_data.h"

// DJ Scratch audio state
volatile int32_t sampleIndex = 0;
volatile uint8_t sampleCounter = 0;
volatile uint8_t playState = 0;
volatile uint8_t playbackSpeed = 5;
volatile int8_t scratchSpeed = 1;
volatile bool isScratchMode = false;

// =============================================================================
// MOZZI THEREMIN ECHO SYSTEM
// =============================================================================
#define MOZZI_CONTROL_RATE 64 

// Configure Mozzi for pin 12 audio output (Solar Shrine configuration)
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_2PIN_PWM

// Custom pin configuration for Arduino Mega (matches solar_shrine_playa.ino)
#define MOZZI_AUDIO_PIN_1 12
#define MOZZI_AUDIO_PIN_1_REGISTER OCR1B
#define MOZZI_AUDIO_PIN_1_LOW 13
#define MOZZI_AUDIO_PIN_1_LOW_REGISTER OCR1C

#include <MozziGuts.h> 
#include <Oscil.h> // oscillator template 
#include <tables/sin2048_int8.h> // sine table for oscillator 
#include <RollingAverage.h> 
#include <ControlDelay.h> 
  
// Ultrasonic sensor pins (from CUSTOM_SHIELD_WIRING.md)
#define TRIG1 10  // Left sensor trigger
#define ECHO1 11  // Left sensor echo  
#define TRIG2 5   // Right sensor trigger
#define ECHO2 6   // Right sensor echo 
  
unsigned int echo_cells_1 = 32; 
unsigned int echo_cells_2 = 60; 
unsigned int echo_cells_3 = 127; 
  
ControlDelay <128, int> kDelay; // 2seconds 
  
// oscils to compare bumpy to averaged control input 
Oscil <SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> aSin0(SIN2048_DATA); 
Oscil <SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> aSin1(SIN2048_DATA); 
Oscil <SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> aSin2(SIN2048_DATA); 
Oscil <SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> aSin3(SIN2048_DATA); 
  
// use: RollingAverage <number_type, how_many_to_average> myThing 
RollingAverage <int, 32> kAverage; // how_many_to_average has to be power of 2 
int averaged;

// =============================================================================
// HARDWARE-ISOLATED MODE SWITCHING FUNCTIONS
// Critical: Complete hardware teardown prevents audio quality degradation
// =============================================================================
void disableMozziThereminEcho() {
  // CRITICAL: Complete Timer1 register clearing (prevents static noise)
  stopMozzi();
  
  // Clear ALL Timer1 registers to prevent conflicts
  TCCR1A = 0;  // Timer1 Control Register A
  TCCR1B = 0;  // Timer1 Control Register B  
  TIMSK1 = 0;  // Disable all Timer1 interrupts
  
  // Reset compare registers
  OCR1A = 0;
  OCR1B = 0;
  OCR1C = 0;
  ICR1 = 0;
  
  // Reset pins to INPUT to clear PWM (critical for clean handoff)
  pinMode(12, INPUT);
  pinMode(13, INPUT);
  
  delay(10);  // Hardware settling time
}

void setupMozziThereminEcho() {
  // CRITICAL: Clean Mozzi initialization after complete teardown
  // Reset pins to INPUT first to clear any previous PWM configuration
  pinMode(12, INPUT);
  pinMode(13, INPUT);
  
  // Small delay to ensure pins settle
  delay(10);
  
  // Start Mozzi with clean hardware state
  startMozzi();
}

void disableDJScratch() {
  // CRITICAL: Complete DJ Scratch teardown
  // Disable all Timer1 interrupts
  TIMSK1 = 0;
  
  // Clear Timer1 registers completely
  TCCR1A = 0;
  TCCR1B = 0;
  
  // Reset Timer1 compare registers
  OCR1A = 0;
  OCR1B = 0;
  OCR1C = 0;
  ICR1 = 0;
  
  // Reset pin 12 to INPUT to clear PWM
  pinMode(12, INPUT);
  
  delay(10);  // Hardware settling time
}

void setupDJScratch() {
  // CRITICAL: Clean DJ Scratch initialization
  // Ensure pin 12 is properly configured for OUTPUT
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);  // Start with silence
  
  // Clear any previous Timer1 configuration first
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;
  OCR1A = 0;
  OCR1B = 0;
  OCR1C = 0;
  ICR1 = 0;
  
  // Small delay to ensure registers are cleared
  delayMicroseconds(100);
  
  // Now configure Timer1 PWM for DJ scratch - OC1B (pin 12)
  TCCR1A = _BV(COM1B1) | _BV(WGM11);  // Clear OC1B on compare match, Fast PWM
  TCCR1B = _BV(WGM13) | _BV(CS10);    // Fast PWM, no prescaler
  ICR1 = 399;                         // 20kHz frequency
  OCR1B = ICR1 / 2;                   // 50% duty cycle (silence)
  TIMSK1 = _BV(OCIE1B);               // Enable Timer1 Compare B interrupt
}

// =============================================================================
// DJ SCRATCH TIMER1 ISR (only active in DJ scratch mode)
// =============================================================================
ISR(TIMER1_COMPB_vect) {
  if (currentMode != MODE_DJ_SCRATCH) {
    OCR1B = 200; // Silence when not in DJ mode
    return;
  }
  
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
      
      OCR1B = ((uint32_t)(amp + 128) * ICR1) / 255;
      
      if (isScratchMode) {
        sampleIndex += scratchSpeed;
        if (sampleIndex < 0) sampleIndex = AUDIO_SAMPLE_COUNT - 1;
        if (sampleIndex >= AUDIO_SAMPLE_COUNT) sampleIndex = 0;
      } else {
        sampleIndex++;
        if (sampleIndex >= AUDIO_SAMPLE_COUNT) sampleIndex = 0;
      }
      
    } else {
      OCR1B = ICR1 / 2; // Silence
    }
  }
}

// =============================================================================
// MODE SWITCHING LOGIC
// =============================================================================
void switchToNextMode() {
  AudioMode previousMode = currentMode;

  // Hardware-Isolated Mode Teardown (complete hardware isolation)
  switch (previousMode) {
    case MODE_MOZZI_THEREMIN_ECHO:
      disableMozziThereminEcho();
      break;
    case MODE_DJ_SCRATCH:
      disableDJScratch();
      break;
  }

  // Switch to next mode
  currentMode = (AudioMode)((currentMode + 1) % 2);
  
  // Hardware-Isolated Mode Setup (immediately initialize new mode)
  switch (currentMode) {
    case MODE_MOZZI_THEREMIN_ECHO:
      setupMozziThereminEcho();
      Serial.println(F("MODE: Mozzi Theremin Echo"));
      break;
    case MODE_DJ_SCRATCH:
      setupDJScratch();
      // Reset DJ scratch state
      playState = 0;
      sampleIndex = 0;
      isScratchMode = false;
      scratchSpeed = 1;
      playbackSpeed = 5;
      Serial.println(F("MODE: DJ Scratch"));
      break;
  }
  
  lastModeChange = millis();
}

// =============================================================================
// SENSOR READING FUNCTIONS (used by both modes)
// ============================================================================= 
  
// Sensor reading function (from solar_shrine_playa)
float readSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999; // No echo received
  
  float distance = duration * 0.034 / 2;
  return distance;
}

bool isHandPresent(float distance) {
  return (distance > 1 && distance < 20);
}

// Gate audio when no hands detected (like solar_shrine_playa)
static volatile bool g_handsActive = false;

void setup(){ 
  Serial.begin(9600);
  Serial.println(F("Starting Solar Shrine Rotation Test"));
  Serial.println(F("Mozzi Theremin Echo + DJ Scratch"));
  Serial.println(F("Audio output on pin 12, rotating every 10 seconds"));
  
  // Initialize sensor pins
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  
  // Initialize Mozzi effect parameters
  kDelay.set(echo_cells_1); 
  
  // Start in Mozzi theremin echo mode (like the working prototype)
  setupMozziThereminEcho();
  lastModeChange = millis();
  Serial.println(F("MODE: Mozzi Theremin Echo (starting mode)"));
} 
  
  
void updateControl(){ 
  // Only process when in Mozzi mode
  if (currentMode != MODE_MOZZI_THEREMIN_ECHO) {
    g_handsActive = false;
    return;
  }
  
  // Read both ultrasonic sensors
  float leftDistance = readSensor(TRIG1, ECHO1);
  float rightDistance = readSensor(TRIG2, ECHO2);
  
  // Check if any hands are present
  bool leftHand = isHandPresent(leftDistance);
  bool rightHand = isHandPresent(rightDistance);
  g_handsActive = (leftHand || rightHand);
  
  // Use right hand for primary frequency control (like ThereminEffect)
  int sensor_input = 0;
  if (rightHand) {
    // Map distance to frequency range (closer = higher frequency)
    float normalizedDistance = (rightDistance - 1.0) / 19.0; // 1-20cm range
    normalizedDistance = 1.0 - normalizedDistance; // Invert so closer = higher
    sensor_input = (int)(normalizedDistance * 1023); // Convert to 0-1023 range
  } else {
    sensor_input = 200; // Default frequency when no hand
  }
  
  // Apply smoothing and echo effects
  averaged = kAverage.next(sensor_input); 
  aSin0.setFreq(averaged); 
  aSin1.setFreq(kDelay.next(averaged)); 
  aSin2.setFreq(kDelay.read(echo_cells_2)); 
  aSin3.setFreq(kDelay.read(echo_cells_3)); 
} 
  
  
AudioOutput_t updateAudio(){ 
  // Only output audio when in Mozzi mode
  if (currentMode != MODE_MOZZI_THEREMIN_ECHO) {
    return MonoOutput::from8Bit(0);
  }
  
  // Hard gate: never output audio without hands detected (like solar_shrine_playa)
  if (!g_handsActive) {
    return MonoOutput::from8Bit(0);
  }
  
  // Restore the better-sounding fromAlmostNBit implementation with much higher amplification
  return MonoOutput::fromAlmostNBit(14, 
    24*((int)aSin0.next()+aSin1.next()+(aSin2.next()>>1) 
    +(aSin3.next()>>2)) 
  ); 
} 
  
  
void loop(){ 
  // EXACT same pattern as working mozzi_theremin_echo prototype
  // Check for mode switching every 10 seconds
  if (millis() - lastModeChange >= MODE_DURATION) {
    switchToNextMode();
  }
  
  // CRITICAL: audioHook() must be called EVERY loop iteration for Mozzi
  if (currentMode == MODE_MOZZI_THEREMIN_ECHO) {
    audioHook();
  }
  
  // Handle DJ scratch controls when in DJ mode (like prototype)
  if (currentMode == MODE_DJ_SCRATCH) {
    float d1 = readSensor(TRIG1, ECHO1);
    float d2 = readSensor(TRIG2, ECHO2);
    
    bool leftHand = isHandPresent(d1);
    bool rightHand = isHandPresent(d2);
    
    // DJ SCRATCH CONTROLS (from dj_scratch_progmem_mega.ino)
    // LEFT HAND - Play/Stop
    static bool prevLeftHand = false;
    if (leftHand && !prevLeftHand) {
      if (playState == 0) {
        playState = 1;
        sampleIndex = 0;
        Serial.println(F("DJ: PLAY"));
      } else {
        playState = 0;
        Serial.println(F("DJ: STOP"));
      }
    }
    prevLeftHand = leftHand;
    
    // RIGHT HAND - Scratch + Speed Control
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
        Serial.print(F("DJ: SPEED="));
        Serial.println(playbackSpeed);
        lastSpeedReport = millis();
      }
    }
    
    prevRightHand = rightHand;
    
    // Simple delay like prototype
    delay(30);
  }
}