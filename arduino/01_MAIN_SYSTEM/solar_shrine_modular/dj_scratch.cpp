#include "dj_scratch.h"
#include <avr/pgmspace.h>
#include "audio_data.h" // This file will be created next

// Audio state
volatile int32_t djSampleIndex = 0;
volatile uint8_t sampleCounter = 0;
volatile uint8_t playState = 0;
volatile uint8_t playbackSpeed = 5;
volatile int8_t scratchSpeed = 1;
volatile bool isScratchMode = false;

void dj_scratch_setup() {
  pinMode(12, OUTPUT);  // Use pin 12 for Timer1 PWM on Mega
  
  // Timer1 PWM - configured for OC1B (pin 12)
  TCCR1A = _BV(COM1B1) | _BV(WGM11); // Non-inverting mode on OC1B
  TCCR1B = _BV(WGM13) | _BV(CS10);    // Fast PWM, no prescaler
  ICR1 = 399;                         // 16MHz / (399 + 1) = 40kHz
  OCR1B = ICR1 / 2;                   // 50% duty cycle (silence)
  TIMSK1 = _BV(OCIE1B);               // Enable Timer1 Compare B interrupt
}

void dj_scratch_disable() {
    TIMSK1 &= ~_BV(OCIE1B); // Disable Timer1 interrupt
    playState = 0;
    OCR1B = ICR1 / 2; // Set to silence
}

ISR(TIMER1_COMPB_vect) {
  sampleCounter++;
  
  uint8_t currentSpeed = isScratchMode ? 2 : playbackSpeed;
  
  if (sampleCounter >= currentSpeed) {
    sampleCounter = 0;
    
    if (playState == 1) {
      if (djSampleIndex < 0) djSampleIndex = 0;
      if (djSampleIndex >= AUDIO_SAMPLE_COUNT) djSampleIndex = 0;
      
      uint8_t sample = pgm_read_byte(&audioData[djSampleIndex]);
      int16_t amp = ((int16_t)sample - 128) * 4;
      amp = constrain(amp, -128, 127);
      
      OCR1B = ((uint32_t)(amp + 128) * ICR1) / 255;
      
      if (isScratchMode) {
        djSampleIndex += scratchSpeed;
        if (djSampleIndex < 0) djSampleIndex = AUDIO_SAMPLE_COUNT - 1;
        if (djSampleIndex >= AUDIO_SAMPLE_COUNT) djSampleIndex = 0;
      } else {
        djSampleIndex++;
        if (djSampleIndex >= AUDIO_SAMPLE_COUNT) djSampleIndex = 0;
      }
      
    } else {
      OCR1B = ICR1 / 2;
    }
  }
}

void dj_scratch_start() {
    playState = 1;
    djSampleIndex = 0;
}

void dj_scratch_update(float d1, float d2) {
    bool rightHand = (d2 > 0.5 && d2 < 20.0);

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

    if (!rightHand && (millis() - lastRightHandDetected) > 200) {
        if (isScratchMode) {
            isScratchMode = false;
            scratchSpeed = 1;
            playbackSpeed = 5;
            transitionCount = 0;
        }
    } else if (rightHand && transitionCount >= 2 && (millis() - lastTransition) < 300) {
        isScratchMode = true;
        if (transitionCount >= 4) {
            scratchSpeed = scratchDirection * 6;
        } else {
            scratchSpeed = scratchDirection * 3;
        }
    } else if (rightHand && (millis() - rightHandStartTime) > 300) {
        isScratchMode = false;
        scratchSpeed = 1;
        float ratio = (d2 - 1.0) / (20.0 - 1.0);
        ratio = constrain(ratio, 0.0, 1.0);
        playbackSpeed = (uint8_t)(5 + (10 * ratio));
    }

    prevRightHand = rightHand;
}