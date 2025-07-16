/*
 * DJ Scratch PROGMEM - Arduino Mega 2560 Version
 * Left hand = play/stop, Right hand = scratch/speed control
 * 
 * MEGA CHANGES:
 * - Audio output moved to pin 12 (Timer1 OC1B on Mega)
 * - Pin 9 already used for audio output (user's hardware)
 * - Pin 11 used for sensor echo (user's hardware)
 * - Left sensor kept on pins 10/11 (echo pin matches hardware)
 */

#include <avr/pgmspace.h>
#include "audio_data.h"

// Pins - UPDATED FOR MEGA WITH HARDWARE CONSTRAINTS
#define TRIG1 10  // Left sensor trigger
#define ECHO1 11  // Left sensor echo (user's hardware)
#define TRIG2 5   // Right sensor trigger
#define ECHO2 6   // Right sensor echo
#define AUDIO_PIN 12  // Timer1 OC1B output on Mega (pin 12)

// Audio state
volatile int32_t sampleIndex = 0;
volatile uint8_t sampleCounter = 0;
volatile uint8_t playState = 0;
volatile uint8_t playbackSpeed = 5;
volatile int8_t scratchSpeed = 1;
volatile bool isScratchMode = false;

void setup() {
  Serial.begin(9600);
  pinMode(AUDIO_PIN, OUTPUT);  // Pin 12 for Mega
  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  
  // Timer1 PWM - configured for OC1B (pin 12)
  TCCR1A = _BV(COM1B1) | _BV(WGM11);  // Clear OC1B on compare match, Fast PWM
  TCCR1B = _BV(WGM13) | _BV(CS10);    // Fast PWM, no prescaler
  ICR1 = 399;                         // 20kHz frequency
  OCR1B = ICR1 / 2;                   // 50% duty cycle (silence)
  TIMSK1 = _BV(OCIE1B);               // Enable Timer1 Compare B interrupt
  
  Serial.println(F("DJ Ready - Mega Version"));
  Serial.println(F("Audio on pin 12 (Timer1 OC1B)"));
  Serial.println(F("Left sensor: pins 10/11, Right sensor: pins 5/6"));
}

float readSensor(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  unsigned long duration = pulseIn(echo, HIGH, 30000UL);
  if (duration == 0) return 0.5;
  return duration * 0.034 / 2.0;
}

ISR(TIMER1_COMPB_vect) {
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
      OCR1B = ICR1 / 2;
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