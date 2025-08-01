/*
 * Dual Track Audio Mixer
 * Based on working methodology from dj_scratch_progmem_mega
 * 
 * Functionality:
 * - Left hand detected: Play beat on loop
 * - Right hand detected: Play singing overlay on loop
 * - Both hands: Mix both audio tracks together
 */

#include "beat1_data_chunked.h"
#include "overlay_data_chunked.h"

// Pin definitions
const int LEFT_HAND_PIN = A0;   // Left hand sensor
const int RIGHT_HAND_PIN = A1;  // Right hand sensor (not used yet)
const int AUDIO_OUTPUT_PIN = 12; // PWM audio output (Timer1 OC1B)

// Audio playback variables
volatile uint16_t beatSampleIndex = 0;
volatile uint16_t singingSampleIndex = 0;
volatile bool isBeatPlaying = false;
volatile bool isSingingPlaying = false;

// Sensor variables
int leftHandValue = 0;
int rightHandValue = 0;
bool leftHandDetected = false;
bool rightHandDetected = false;
const int HAND_THRESHOLD = 100; // Adjust based on your sensor

void setup() {
  Serial.begin(9600);
  
  // Initialize sensor pins
  pinMode(LEFT_HAND_PIN, INPUT);
  pinMode(RIGHT_HAND_PIN, INPUT);
  
  // Initialize audio output pin
  pinMode(AUDIO_OUTPUT_PIN, OUTPUT);
  
  // Setup Timer1 for PWM audio output
  setupTimer1();
  
  Serial.println("Dual Track Audio Mixer Ready");
  Serial.println("Left hand: beat track, Right hand: singing overlay");
}

void setupTimer1() {
  // Timer1 PWM - configured for OC1B (pin 12) - matches working dj_scratch_progmem_mega
  TCCR1A = _BV(COM1B1) | _BV(WGM11);  // Clear OC1B on compare match, Fast PWM
  TCCR1B = _BV(WGM13) | _BV(CS10);    // Fast PWM, no prescaler
  ICR1 = 399;                         // 20kHz frequency
  OCR1B = ICR1 / 2;                   // 50% duty cycle (silence)
  TIMSK1 = _BV(OCIE1B);               // Enable Timer1 Compare B interrupt
}

// Timer1 Compare Match B interrupt - Audio sample output with mixing
ISR(TIMER1_COMPB_vect) {
  static uint8_t sampleCounter = 0;
  sampleCounter++;
  
  // Sample rate control - play every 2.5 cycles for 8kHz (20kHz / 8kHz = 2.5)
  if (sampleCounter >= 3) {
    sampleCounter = 0;
    
    int16_t mixedSample = 0; // Start with silence (0 in signed)
    uint8_t activeChannels = 0;
    
    // Mix beat track if playing
    if (isBeatPlaying) {
      uint8_t beatSample = read_beat1Data(beatSampleIndex);
      mixedSample += (int16_t)beatSample - 128; // Convert to signed (-128 to +127)
      activeChannels++;
      
      // Advance beat sample index
      beatSampleIndex++;
      if (beatSampleIndex >= BEAT1_SAMPLE_COUNT) {
        beatSampleIndex = 0; // Loop beat
      }
    }
    
    // Mix singing track if playing
    if (isSingingPlaying) {
      uint8_t singingSample = read_singingData(singingSampleIndex);
      mixedSample += (int16_t)singingSample - 128; // Convert to signed (-128 to +127)
      activeChannels++;
      
      // Advance singing sample index
      singingSampleIndex++;
      if (singingSampleIndex >= SINGING_SAMPLE_COUNT) {
        singingSampleIndex = 0; // Loop singing
      }
    }
    
    // Apply mixing and prevent clipping
    if (activeChannels > 0) {
      // Scale down if multiple channels to prevent clipping
      if (activeChannels > 1) {
        mixedSample = mixedSample / 2; // Simple mixing: average the channels
      }
      
      // Apply amplitude and constrain to prevent clipping
      mixedSample = mixedSample * 4; // Amplify
      mixedSample = constrain(mixedSample, -128, 127);
      
      // Convert back to unsigned (0-255) and output to PWM
      uint8_t finalSample = (uint8_t)(mixedSample + 128);
      OCR1B = ((uint32_t)finalSample * ICR1) / 255;
    } else {
      // Output silence (50% duty cycle)
      OCR1B = ICR1 / 2;
    }
  }
}

void loop() {
  // Read both hand sensors
  leftHandValue = analogRead(LEFT_HAND_PIN);
  rightHandValue = analogRead(RIGHT_HAND_PIN);
  leftHandDetected = (leftHandValue > HAND_THRESHOLD);
  rightHandDetected = (rightHandValue > HAND_THRESHOLD);
  
  // Control beat track based on left hand
  if (leftHandDetected && !isBeatPlaying) {
    // Start beat
    isBeatPlaying = true;
    beatSampleIndex = 0; // Start from beginning
    Serial.println("Beat started");
  } else if (!leftHandDetected && isBeatPlaying) {
    // Stop beat
    isBeatPlaying = false;
    Serial.println("Beat stopped");
  }
  
  // Control singing track based on right hand
  if (rightHandDetected && !isSingingPlaying) {
    // Start singing
    isSingingPlaying = true;
    singingSampleIndex = 0; // Start from beginning
    Serial.println("Singing overlay started");
  } else if (!rightHandDetected && isSingingPlaying) {
    // Stop singing
    isSingingPlaying = false;
    Serial.println("Singing overlay stopped");
  }
  
  // Debug output every 500ms
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 500) {
    Serial.print("Left: ");
    Serial.print(leftHandValue);
    Serial.print(" (Beat: ");
    Serial.print(isBeatPlaying ? "ON" : "OFF");
    Serial.print("), Right: ");
    Serial.print(rightHandValue);
    Serial.print(" (Singing: ");
    Serial.print(isSingingPlaying ? "ON" : "OFF");
    Serial.println(")");
    lastDebug = millis();
  }
  
  delay(10); // Small delay for stability
}