/*
 * DJ Scratch PROGMEM Audio Player - MEMORY OPTIMIZED
 * Combines high-quality PROGMEM audio with DJ scratching effects
 * Optimized for Arduino UNO memory constraints
 * 
 * CONTROL:
 * - Left Hand: Play/Pause/Loop control  
 * - Right Hand: Scratch effects
 */

#include <avr/pgmspace.h>
#include "audio_data.h"

// Sensor pins (no NewPing library to save memory)
const int trigPin1 = 10, echoPin1 = 11;  // Left sensor
const int trigPin2 = 5, echoPin2 = 6;    // Right sensor

// Audio state
volatile bool isPlaying = false;
volatile int32_t sampleIndex = 0;
volatile uint8_t sampleCounter = 0;

// Scratch variables
int prevDistance2 = 0;
float playbackSpeed = 1.0;
const int SCRATCH_THRESHOLD = 5;
const float MAX_SPEED = 4.0;

// Control state
bool leftHand = false, rightHand = false;
unsigned long lastLeft = 0, lastRight = 0;
bool loopEnabled = true;
uint8_t playbackState = 0; // 0=stopped, 1=playing, 2=paused

void setup() {
  Serial.begin(9600);
  
  // Setup pins
  pinMode(9, OUTPUT);
  pinMode(trigPin1, OUTPUT); pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT); pinMode(echoPin2, INPUT);
  
  // Setup Timer1 Phase Correct PWM
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = 399;  // 20kHz
  OCR1A = ICR1 / 2;
  TIMSK1 = _BV(OCIE1A);
  
  Serial.println(F("DJ Ready"));
}

// Optimized distance reading (no NewPing library)
int readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  unsigned long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 0;
  
  int distance = duration / 58;  // Convert to cm
  return (distance > 2 && distance < 20) ? distance : 0;
}

// Timer interrupt - optimized for memory
ISR(TIMER1_COMPA_vect) {
  sampleCounter++;
  
  uint8_t target = (uint8_t)(5.0 / abs(playbackSpeed));
  if (target < 1) target = 1;
  if (target > 20) target = 20;
  
  if (sampleCounter >= target) {
    sampleCounter = 0;
    
    if (playbackState == 1) {  // Playing
      if (sampleIndex < 0) sampleIndex = 0;
      if (sampleIndex >= AUDIO_SAMPLE_COUNT) {
        if (loopEnabled) {
          sampleIndex = 0;
        } else {
          playbackState = 0;
          OCR1A = ICR1 / 2;
          return;
        }
      }
      
      uint8_t sample = pgm_read_byte(&audioData[sampleIndex]);
      
      // 4x amplification
      int16_t amp = ((int16_t)sample - 128) * 4;
      if (amp > 127) amp = 127;
      if (amp < -128) amp = -128;
      
      uint16_t pwm = ((uint32_t)(amp + 128) * ICR1) / 255;
      OCR1A = pwm;
      
      sampleIndex += (playbackSpeed >= 0) ? 1 : -1;
    } else {
      OCR1A = ICR1 / 2;  // Silence
    }
  }
}

void updateControls() {
  // Left hand control
  int dist1 = readDistance(trigPin1, echoPin1);
  bool currentLeft = (dist1 > 0);
  
  if (currentLeft) {
    lastLeft = millis();
    if (!leftHand) {
      leftHand = true;
      
      if (playbackState == 0 || playbackState == 2) {
        playbackState = 1;  // Play
        Serial.println(F("Play"));
      } else {
        playbackState = 2;  // Pause
        Serial.println(F("Pause"));
      }
    } else if (dist1 < 4) {  // Very close = loop toggle
      static unsigned long lastToggle = 0;
      if (millis() - lastToggle > 1000) {
        loopEnabled = !loopEnabled;
        Serial.print(F("Loop: "));
        Serial.println(loopEnabled ? F("ON") : F("OFF"));
        lastToggle = millis();
      }
    }
  } else if (millis() - lastLeft > 200) {
    leftHand = false;
  }
  
  // Right hand scratch control
  int dist2 = readDistance(trigPin2, echoPin2);
  bool currentRight = (dist2 > 0);
  
  if (currentRight) {
    lastRight = millis();
    
    if (!rightHand) {
      rightHand = true;
      prevDistance2 = dist2;
    } else {
      int velocity = dist2 - prevDistance2;
      
      if (abs(velocity) > SCRATCH_THRESHOLD) {
        playbackSpeed = constrain(velocity / 5.0, -MAX_SPEED, MAX_SPEED);
        if (abs(playbackSpeed) < 0.1) {
          playbackSpeed = (playbackSpeed >= 0) ? 0.1 : -0.1;
        }
      } else {
        playbackSpeed = 1.0;
      }
      
      prevDistance2 = dist2;
    }
  } else if (millis() - lastRight > 200) {
    if (rightHand) {
      rightHand = false;
      playbackSpeed = 1.0;
    }
  }
}

void loop() {
  updateControls();
  
  // Minimal status output
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 3000) {
    Serial.print(F("Pos: "));
    Serial.print((float)sampleIndex / AUDIO_SAMPLE_COUNT * 100, 0);
    Serial.print(F("% Speed: "));
    Serial.println(playbackSpeed, 1);
    lastStatus = millis();
  }
  
  delay(50);
} 