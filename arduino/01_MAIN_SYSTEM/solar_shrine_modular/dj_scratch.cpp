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

// Distance thresholds (in centimeters)
static const int MIN_CM = 5;
static const int MAX_CM = 50;

void dj_scratch_setup() {
  // Pin 9 and Timer2 control for DJ scratch audio
  pinMode(9, OUTPUT);  // Use pin 9 for Timer2 PWM on Mega
  
  // Timer2 PWM setup for 20kHz Fast PWM
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // Non-inverting mode on OC2B, Fast PWM
  TCCR2B = _BV(CS20);                              // No prescaler
  OCR2A = 199;                                     // 16MHz / (199 + 1) = 80kHz / 4 = 20kHz
  OCR2B = OCR2A / 2;                               // 50% duty cycle (silence)
  TIMSK2 = _BV(OCIE2B);                            // Enable Timer2 Compare B interrupt
}

void dj_scratch_disable() {
    TIMSK2 &= ~_BV(OCIE2B); // Disable Timer2 interrupt
    playState = 0;
    OCR2B = OCR2A / 2; // Set to silence
    // Reset Timer2 to default state
    TCCR2A = 0;
    TCCR2B = 0;
    pinMode(9, INPUT); // tri-state pin 9
}

ISR(TIMER2_COMPB_vect) {
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
      
      OCR2B = ((uint32_t)(amp + 128) * OCR2A) / 255;
      
      if (isScratchMode) {
        djSampleIndex += scratchSpeed;
        if (djSampleIndex < 0) djSampleIndex = AUDIO_SAMPLE_COUNT - 1;
        if (djSampleIndex >= AUDIO_SAMPLE_COUNT) djSampleIndex = 0;
      } else {
        djSampleIndex++;
        if (djSampleIndex >= AUDIO_SAMPLE_COUNT) djSampleIndex = 0;
      }
      
    } else {
      OCR2B = OCR2A / 2;
    }
  }
}

void dj_scratch_start() {
  playState = 1;
  djSampleIndex = 0;
}

void dj_scratch_update(float d1, float d2) {
  // Use raw distances from sensors
  bool rightInRange = (d2 >= MIN_CM && d2 <= MAX_CM);
  bool leftInRange = (d1 >= MIN_CM && d1 <= MAX_CM);

  // If the right hand is not in range, stop playback completely
  if (!rightInRange) {
    playState = 0;           // Stop audio in ISR
    isScratchMode = false;
    OCR1B = ICR1 / 2;        // Ensure silence duty
    return;
  }

  static bool prevRightHand = false;
  static unsigned long lastTransition = 0;
  static uint8_t transitionCount = 0;
  static int8_t scratchDirection = 1;
  static unsigned long rightHandStartTime = 0;
    static unsigned long lastRightHandDetected = 0;

    if (rightInRange) {
        lastRightHandDetected = millis();
    }

    if (rightInRange != prevRightHand) {
        lastTransition = millis();
        transitionCount++;
        scratchDirection = -scratchDirection;
    }

    if (rightInRange && !prevRightHand) {
        rightHandStartTime = millis();
    }

    if (millis() - lastTransition > 500) {
        transitionCount = 0;
    }

    if (!rightInRange && (millis() - lastRightHandDetected) > 200) {
        if (isScratchMode) {
            isScratchMode = false;
            scratchSpeed = 1;
            playbackSpeed = 5;
            transitionCount = 0;
        }
    } else if (rightInRange && transitionCount >= 2 && (millis() - lastTransition) < 300) {
        isScratchMode = true;
        if (transitionCount >= 4) {
            scratchSpeed = scratchDirection * 6;
        } else {
            scratchSpeed = scratchDirection * 3;
        }
    } else if (rightInRange && (millis() - rightHandStartTime) > 300) {
        isScratchMode = false;
        scratchSpeed = 1;
        float ratio = (d2 - (float)MIN_CM) / (float)(MAX_CM - MIN_CM);
        ratio = constrain(ratio, 0.0, 1.0);
        playbackSpeed = (uint8_t)(5 + (10 * ratio));
    }

    // Ensure playback is running only when the right hand is present
    playState = 1; // enable audio ISR output
    prevRightHand = rightInRange;
}