/*
 * Effect Rotator - Ultra Minimal Conditional Compilation
 * ONLY ONE EFFECT COMPILES AT A TIME - MINIMUM MEMORY USAGE
 * 
 * To switch effects, uncomment the desired effect and recompile/upload:
 */

// UNCOMMENT EXACTLY ONE OF THESE:
#define EFFECT_DJ_SCRATCH           // 159KB PROGMEM audio
// #define EFFECT_MUSICAL_THEREMIN  // Mozzi synthesis
// #define EFFECT_ALIEN_SOUND       // Spacey FM synthesis  
// #define EFFECT_ROBOTS_TALKING    // Robotic square waves

// Minimal sensor reading function (no NewPing library)
float readSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) return 0.5;
  return duration * 0.034 / 2.0;
}

// Sensor pins
const int trigPin1 = 10, echoPin1 = 11;
const int trigPin2 = 5, echoPin2 = 6;
const float MIN_RANGE = 1.0, MAX_RANGE = 20.0;

// ==================== EFFECT 1: DJ SCRATCH ====================
#ifdef EFFECT_DJ_SCRATCH
#include <avr/pgmspace.h>
#include "audio_data.h"

volatile int32_t sampleIndex = 0;
volatile uint8_t sampleCounter = 0;
volatile uint8_t playState = 0;
volatile uint8_t playbackSpeed = 5;
volatile int8_t scratchSpeed = 1;
volatile bool isScratchMode = false;

void setupEffect() {
  pinMode(9, OUTPUT);
  pinMode(trigPin1, OUTPUT); pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT); pinMode(echoPin2, INPUT);
  
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = 399;
  OCR1A = ICR1 / 2;
  TIMSK1 = _BV(OCIE1A);
  Serial.println(F("DJ"));
}

ISR(TIMER1_COMPA_vect) {
  sampleCounter++;
  if (sampleCounter >= (isScratchMode ? 2 : playbackSpeed)) {
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

void updateEffect(float d1, float d2, bool leftHand, bool rightHand) {
  if (leftHand && playState == 0) {
    playState = 1;
    sampleIndex = 0;
    Serial.println(F("PLAY"));
  } else if (!leftHand && playState == 1) {
    playState = 0;
    Serial.println(F("STOP"));
  }
  
  static bool prevRightHand = false;
  static unsigned long lastTransition = 0;
  static uint8_t transitionCount = 0;
  static int8_t scratchDirection = 1;
  
  if (rightHand != prevRightHand) {
    lastTransition = millis();
    transitionCount++;
    scratchDirection = -scratchDirection;
  }
  
  if (millis() - lastTransition > 500) transitionCount = 0;
  
  if (rightHand && transitionCount >= 2 && (millis() - lastTransition) < 300) {
    isScratchMode = true;
    scratchSpeed = scratchDirection * 3;
    Serial.println(F("SCRATCH"));
  } else if (!rightHand) {
    isScratchMode = false;
    scratchSpeed = 1;
    playbackSpeed = 5;
  }
  
  prevRightHand = rightHand;
}

// ==================== EFFECT 2: MUSICAL THEREMIN ====================
#elif defined(EFFECT_MUSICAL_THEREMIN)
#include <MozziGuts.h>
#include <Oscil.h>
#include <tables/triangle_valve_2048_int8.h>

#define CONTROL_RATE 128
Oscil <TRIANGLE_VALVE_2048_NUM_CELLS, AUDIO_RATE> osc(TRIANGLE_VALVE_2048_DATA);
int audioVolume = 0;

void setupEffect() {
  pinMode(trigPin1, OUTPUT); pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT); pinMode(echoPin2, INPUT);
  
  startMozzi(CONTROL_RATE);
  osc.setFreq(220);
  Serial.println(F("Theremin"));
}

void updateControl() { }

int updateAudio() {
  return (osc.next() * audioVolume) >> 8;
}

void updateEffect(float d1, float d2, bool leftHand, bool rightHand) {
  if (leftHand || rightHand) {
    float avgDistance = (d1 + d2) / 2.0;
    if (avgDistance >= MIN_RANGE && avgDistance <= MAX_RANGE) {
      float ratio = (avgDistance - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
      int freq = 220 + (880 - 220) * (1.0 - ratio);
      osc.setFreq(freq);
      audioVolume = 200;
      Serial.println(freq);
    }
  } else {
    audioVolume = 0;
  }
}

// ==================== EFFECT 3: ALIEN SOUND ====================
#elif defined(EFFECT_ALIEN_SOUND)
uint16_t alienPhase = 0;
uint16_t alienMod = 0;
bool alienActive = false;

void setupEffect() {
  pinMode(9, OUTPUT);
  pinMode(trigPin1, OUTPUT); pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT); pinMode(echoPin2, INPUT);
  
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = 399;
  OCR1A = ICR1 / 2;
  TIMSK1 = _BV(OCIE1A);
  Serial.println(F("Alien"));
}

ISR(TIMER1_COMPA_vect) {
  if (alienActive) {
    alienPhase += 200;
    alienMod += 50;
    
    uint8_t modulator = (alienMod >> 8) & 0xFF;
    uint8_t carrier = ((alienPhase + (modulator * 4)) >> 8) & 0xFF;
    
    int16_t amp = (carrier > 128) ? 100 : -100;
    OCR1A = ((uint32_t)(amp + 128) * ICR1) / 255;
  } else {
    OCR1A = ICR1 / 2;
  }
}

void updateEffect(float d1, float d2, bool leftHand, bool rightHand) {
  alienActive = (leftHand || rightHand);
  if (alienActive) Serial.println(F("Alien"));
}

// ==================== EFFECT 4: ROBOTS TALKING ====================
#elif defined(EFFECT_ROBOTS_TALKING)
uint16_t robotPhase = 0;
uint8_t robotPattern = 0;
bool robotActive = false;

void setupEffect() {
  pinMode(9, OUTPUT);
  pinMode(trigPin1, OUTPUT); pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT); pinMode(echoPin2, INPUT);
  
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = 399;
  OCR1A = ICR1 / 2;
  TIMSK1 = _BV(OCIE1A);
  Serial.println(F("Robot"));
}

ISR(TIMER1_COMPA_vect) {
  if (robotActive) {
    robotPhase += 150;
    if ((robotPhase & 0x1000) == 0) robotPattern++;
    
    uint8_t wave = robotPhase >> 8;
    int16_t amp = 0;
    
    if (robotPattern & 1) {
      amp = (wave & 0x80) ? 120 : -120;
    } else {
      amp = (wave & 0x40) ? 80 : -80;
    }
    
    OCR1A = ((uint32_t)(amp + 128) * ICR1) / 255;
  } else {
    OCR1A = ICR1 / 2;
  }
}

void updateEffect(float d1, float d2, bool leftHand, bool rightHand) {
  robotActive = (leftHand || rightHand);
  if (robotActive) Serial.println(F("Robot"));
}

#endif

// ==================== MINIMAL SETUP AND LOOP ====================
void setup() {
  Serial.begin(9600);
  setupEffect();
  delay(500);
}

void loop() {
  float d1 = readSensor(trigPin1, echoPin1);
  float d2 = readSensor(trigPin2, echoPin2);
  
  bool leftHand = (d1 >= MIN_RANGE && d1 <= MAX_RANGE);
  bool rightHand = (d2 >= MIN_RANGE && d2 <= MAX_RANGE);
  
  updateEffect(d1, d2, leftHand, rightHand);
  
#ifdef EFFECT_MUSICAL_THEREMIN
  audioHook();
#endif
  
  delay(30);
} 