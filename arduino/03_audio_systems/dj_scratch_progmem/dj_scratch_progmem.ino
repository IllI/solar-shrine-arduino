/*
 * DJ Scratch PROGMEM - ENHANCED SCRATCHING
 * Left hand = immediate play/stop, Right hand = rapid in/out scratching
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
volatile uint8_t playState = 0;  // 0=stopped, 1=playing
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
  OCR1A = ICR1 / 2;  // Start silent
  TIMSK1 = _BV(OCIE1A);
  
  Serial.println("DJ Ready - Enhanced Scratching");
}

float readSensor(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  unsigned long duration = pulseIn(echo, HIGH, 30000UL);
  if (duration == 0) return 0.5;  // Very close
  return duration * 0.034 / 2.0;  // Convert to cm
}

ISR(TIMER1_COMPA_vect) {
  sampleCounter++;
  
  // Variable speed based on scratch mode
  uint8_t targetSpeed = isScratchMode ? 2 : 5;  // Faster when scratching
  
  if (sampleCounter >= targetSpeed) {
    sampleCounter = 0;
    
    if (playState == 1) {
      // Ensure bounds are correct
      if (sampleIndex < 0) sampleIndex = 0;
      if (sampleIndex >= AUDIO_SAMPLE_COUNT) sampleIndex = 0;  // Loop
      
      uint8_t sample = pgm_read_byte(&audioData[sampleIndex]);
      int16_t amp = ((int16_t)sample - 128) * 4;
      amp = constrain(amp, -128, 127);
      
      OCR1A = ((uint32_t)(amp + 128) * ICR1) / 255;
      
      // Advance by scratch speed (more dramatic range)
      sampleIndex += scratchSpeed;
      
      // Handle negative wraparound
      if (sampleIndex < 0) sampleIndex = AUDIO_SAMPLE_COUNT - 1;
      
    } else {
      // Not playing - silence
      OCR1A = ICR1 / 2;
    }
  }
}

void loop() {
  float d1 = readSensor(TRIG1, ECHO1);
  float d2 = readSensor(TRIG2, ECHO2);
  
  bool leftHand = (d1 >= 1.0 && d1 <= 20.0);
  bool rightHand = (d2 >= 1.0 && d2 <= 20.0);
  
  // LEFT HAND LOGIC - Immediate play/stop
  if (leftHand) {
    if (playState == 0) {
      // Start playing from beginning
      playState = 1;
      sampleIndex = 0;
      Serial.println("PLAY");
    }
  } else {
    if (playState == 1) {
      // Stop immediately
      playState = 0;
      Serial.println("STOP");
    }
  }
  
  // RIGHT HAND LOGIC - Rapid in/out detection for scratching
  static bool prevRightHand = false;
  static uint8_t transitionCount = 0;
  static unsigned long lastTransition = 0;
  static unsigned long scratchTimer = 0;
  static int8_t scratchDirection = 1;
  
  // Detect state changes (in/out transitions)
  if (rightHand != prevRightHand) {
    transitionCount++;
    lastTransition = millis();
    
    // Alternate scratch direction on each transition
    scratchDirection = -scratchDirection;
    
    Serial.print("TRANSITION ");
    Serial.println(transitionCount);
  }
  
  // Check for rapid transitions (scratching detected)
  if (millis() - lastTransition < 200) {  // Within 200ms of last transition
    if (transitionCount >= 2) {  // At least 2 transitions (in->out->in or out->in->out)
      if (leftHand) {  // Only scratch when left hand is playing
        isScratchMode = true;
        scratchTimer = millis();
        
        // More dramatic scratch speeds based on transition count
        if (transitionCount >= 6) {
          scratchSpeed = scratchDirection * 8;  // Very fast scratch
        } else if (transitionCount >= 4) {
          scratchSpeed = scratchDirection * 5;  // Fast scratch
        } else {
          scratchSpeed = scratchDirection * 3;  // Medium scratch
        }
        
        Serial.print("SCRATCH ACTIVE - Speed: ");
        Serial.println(scratchSpeed);
      }
    }
  } else {
    // No recent transitions - reset
    if (transitionCount > 0) {
      transitionCount = 0;
      Serial.println("SCRATCH RESET");
    }
  }
  
  // Scratch mode timeout
  if (isScratchMode && (millis() - scratchTimer > 300)) {
    isScratchMode = false;
    scratchSpeed = 1;  // Back to normal speed
    Serial.println("SCRATCH TIMEOUT");
  }
  
  // Normal playback when not scratching
  if (!isScratchMode) {
    scratchSpeed = 1;
  }
  
  prevRightHand = rightHand;
  
  // Debug output (minimal)
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 500) {
    Serial.print("L:");
    Serial.print(leftHand ? "1" : "0");
    Serial.print(" R:");
    Serial.print(rightHand ? "1" : "0");
    Serial.print(" Trans:");
    Serial.print(transitionCount);
    Serial.print(" Scratch:");
    Serial.print(isScratchMode ? "ON" : "OFF");
    Serial.print(" Speed:");
    Serial.println(scratchSpeed);
    lastDebug = millis();
  }
  
  delay(30);  // Faster loop for better transition detection
} 