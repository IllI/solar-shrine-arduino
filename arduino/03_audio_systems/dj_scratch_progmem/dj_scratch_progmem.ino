/*
 * DJ Scratch PROGMEM - Optimized Version
 * Left hand = play/stop, Right hand = scratch/speed control
 */

#include <avr/pgmspace.h>
#include "audio_data.h"

// Pins
#define TRIG1 10
#define ECHO1 11
#define TRIG2 5  
#define ECHO2 6

// Audio state
volatile int32_t sampleIndex = 0;
volatile uint8_t sampleCounter = 0;
volatile uint8_t playState = 0;
volatile uint8_t playbackSpeed = 5;
volatile int8_t scratchSpeed = 1;
volatile bool isScratchMode = false;

void setup() {
  Serial.begin(9600);
  pinMode(9, OUTPUT);
  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  
  // Timer1 PWM
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = 399;
  OCR1A = ICR1 / 2;
  TIMSK1 = _BV(OCIE1A);
  
  Serial.println(F("DJ Ready"));
}

float readSensor(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  unsigned long duration = pulseIn(echo, HIGH, 30000UL);
  if (duration == 0) return 0.5;
  return duration * 0.034 / 2.0;
}

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

void loop() {
  float d1 = readSensor(TRIG1, ECHO1);
  float d2 = readSensor(TRIG2, ECHO2);
  
  bool leftHand = (d1 >= 1.0 && d1 <= 20.0);
  bool rightHand = (d2 >= 1.0 && d2 <= 20.0);
  
  // LEFT HAND - Play/Stop
  if (leftHand) {
    if (playState == 0) {
      playState = 1;
      sampleIndex = 0;
      Serial.println(F("PLAY"));
    }
  } else {
    if (playState == 1) {
      playState = 0;
      Serial.println(F("STOP"));
    }
  }
  
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
      Serial.println(F("NORMAL"));
    }
  }
  // Scratch mode - rapid transitions
  else if (rightHand && transitionCount >= 2 && (millis() - lastTransition) < 300) {
    isScratchMode = true;
    
    if (transitionCount >= 4) {
      scratchSpeed = scratchDirection * 6;
      Serial.println(F("SCRATCH+"));
    } else {
      scratchSpeed = scratchDirection * 3;
      Serial.println(F("SCRATCH"));
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
      Serial.print(F("SPEED:"));
      Serial.println(playbackSpeed);
      lastSpeedReport = millis();
    }
  }
  
  prevRightHand = rightHand;
  
  delay(30);
} 