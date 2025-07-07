/*
 * Solar Shrine Timer-Based Audio Player
 * Based on Robin2's working method: https://forum.arduino.cc/t/very-simple-wav-file-player/350303
 * 
 * This method uses Timer2 in fast PWM mode with interrupts for precise timing
 * 
 * Hardware Setup:
 * - Audio output: Pin 3 (OC2B) 
 * - RC Low-pass filter: 1kΩ resistor + 10µF capacitor to ground
 * - Connect filtered output to amplifier input
 * 
 * Audio Requirements:
 * - 8-bit, mono, 11,025Hz sample rate
 * - Raw PCM data (no WAV headers)
 * - Stored in PROGMEM to save RAM
 */

#include <FastLED.h>
#include <NewPing.h>

// Audio sample data - PROCESSED VERSION (remove WAV headers, convert to raw PCM)
// This needs to be your audio converted to raw 8-bit PCM samples at 11,025Hz
const unsigned int SAMPLE_LENGTH = 1000;
const unsigned char PROGMEM audioSample[SAMPLE_LENGTH] = {
  // Sample values should be 0-255 representing audio amplitude
  // These values need to come from properly processed audio file
  128, 130, 135, 140, 145, 150, 155, 160, 165, 170, 175, 180, 185, 190, 195, 200,
  205, 210, 215, 220, 225, 230, 235, 240, 245, 250, 255, 250, 245, 240, 235, 230,
  225, 220, 215, 210, 205, 200, 195, 190, 185, 180, 175, 170, 165, 160, 155, 150,
  145, 140, 135, 130, 128, 125, 120, 115, 110, 105, 100, 95, 90, 85, 80, 75,
  70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 15, 10, 5, 0, 5,
  // ... continuing pattern for demo - replace with your actual audio data
  10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85,
  90, 95, 100, 105, 110, 115, 120, 125, 128, 130, 135, 140, 145, 150, 155, 160,
  165, 170, 175, 180, 185, 190, 195, 200, 205, 210, 215, 220, 225, 230, 235, 240,
  245, 250, 255, 250, 245, 240, 235, 230, 225, 220, 215, 210, 205, 200, 195, 190,
  185, 180, 175, 170, 165, 160, 155, 150, 145, 140, 135, 130, 128, 125, 120, 115,
  // Fill remaining with test pattern...
  128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
  // ... continue this pattern or replace with actual converted audio data
};

// Pin definitions
const int trigPin1 = 10;
const int echoPin1 = 11;
const int trigPin2 = 5;
const int echoPin2 = 6;
const int LED_PIN = 2;      // Changed from 3 since Pin 3 is used for audio
const int AUDIO_PIN = 3;    // OC2B - Timer2 PWM output

// LED configuration
#define NUM_LEDS 60
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

// Sensor objects
NewPing sonar1(trigPin1, echoPin1, 200);
NewPing sonar2(trigPin2, echoPin2, 200);

// Audio playback variables
volatile unsigned int sampleIndex = 0;
volatile bool isPlaying = false;
volatile bool audioEnabled = false;

// Range constants
const float MIN_RANGE = 2.0;
const float MAX_RANGE = 30.0;

// State variables
bool handsDetected = false;
unsigned long lastHandTime = 0;
const unsigned long HAND_TIMEOUT = 5000;

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  Serial.println("=== Timer-Based Audio Player ===");
  Serial.println("Based on: https://forum.arduino.cc/t/very-simple-wav-file-player/350303");
  Serial.println();
  
  // Initialize FastLED (Pin 2 instead of 3)
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(150);
  
  // Initialize audio pin
  pinMode(AUDIO_PIN, OUTPUT);
  
  // Setup Timer2 for 11,025Hz audio playback
  setupTimer2Audio();
  
  Serial.println("Hardware Requirements:");
  Serial.println("- Pin 3 → 1kΩ resistor → Amplifier input");
  Serial.println("- Pin 3 → 1kΩ resistor → 10µF cap → Ground");
  Serial.println("- This creates RC low-pass filter for audio");
  Serial.println();
  
  // Test startup pattern
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();
  delay(1000);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  Serial.println("Ready - wave hands to play audio sample");
  Serial.println("Commands: 't' = test audio, 's' = stop");
}

void setupTimer2Audio() {
  // Configure Timer2 for fast PWM mode at 11,025Hz
  // Based on Robin2's working configuration
  
  Serial.println("Configuring Timer2 for audio output...");
  
  // Stop timer during configuration
  TCCR2B = 0;
  
  // Fast PWM mode with OC2B (Pin 3) output
  TCCR2A = 0b00100011;  // Fast PWM mode, OC2B connected
  
  // Set prescaler for 2MHz clock (16MHz / 8)
  TCCR2B = 0b00000010;  // Clock/8 prescaler
  
  // Set TOP value for ~11,025Hz frequency
  // 2MHz / 181 ≈ 11,049Hz (close to 11,025Hz)
  OCR2A = 180;  // TOP value (frequency control)
  OCR2B = 90;   // Initial duty cycle (50%)
  
  // Clear interrupt flags
  TIFR2 = 0xFF;
  
  Serial.println("Timer2 configured:");
  Serial.print("- Target frequency: 11,025Hz");
  Serial.print("- Actual frequency: ~");
  Serial.print(2000000L / 181);
  Serial.println("Hz");
  Serial.println("- Output pin: 3 (OC2B)");
  
  // Timer interrupt will be enabled when audio starts
}

void startAudioPlayback() {
  if (!isPlaying) {
    Serial.println("Starting audio playback...");
    sampleIndex = 0;
    isPlaying = true;
    audioEnabled = true;
    
    // Enable timer overflow interrupt
    TIMSK2 |= (1 << TOIE2);
    
    Serial.println("Timer2 overflow interrupt enabled");
  }
}

void stopAudioPlayback() {
  if (isPlaying) {
    Serial.println("Stopping audio playback...");
    isPlaying = false;
    audioEnabled = false;
    
    // Disable timer interrupt
    TIMSK2 &= ~(1 << TOIE2);
    
    // Set output to mid-level (silence)
    OCR2B = 90;  // 50% duty cycle = silence for AC-coupled audio
    
    Serial.println("Timer2 interrupt disabled");
  }
}

// Timer2 overflow interrupt - called at ~11,025Hz when enabled
ISR(TIMER2_OVF_vect) {
  if (audioEnabled && isPlaying) {
    // Read next sample from PROGMEM
    unsigned char sample = pgm_read_byte(&audioSample[sampleIndex]);
    
    // Scale sample to Timer2 range (0-180)
    // Convert 8-bit sample (0-255) to PWM range (0-OCR2A)
    OCR2B = (sample * (long)OCR2A) >> 8;
    
    // Advance to next sample
    sampleIndex++;
    if (sampleIndex >= SAMPLE_LENGTH) {
      sampleIndex = 0;  // Loop the sample
    }
  }
}

void handleSerialCommands() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 't':
        Serial.println("Manual audio test...");
        startAudioPlayback();
        break;
        
      case 's':
        Serial.println("Manual stop...");
        stopAudioPlayback();
        break;
        
      case 'i':
        Serial.print("Playing: "); Serial.print(isPlaying);
        Serial.print(" | Sample: "); Serial.print(sampleIndex);
        Serial.print(" | OCR2B: "); Serial.println(OCR2B);
        break;
        
      default:
        Serial.println("Commands: t=test, s=stop, i=info");
        break;
    }
  }
}

void loop() {
  // Handle serial commands
  handleSerialCommands();
  
  // Read sensors
  float distance1 = sonar1.ping_cm();
  float distance2 = sonar2.ping_cm();
  
  // Convert 0 readings
  if (distance1 == 0) distance1 = MAX_RANGE + 1;
  if (distance2 == 0) distance2 = MAX_RANGE + 1;
  
  // Hand detection
  bool currentHands = false;
  if (distance1 >= MIN_RANGE && distance1 <= MAX_RANGE) currentHands = true;
  if (distance2 >= MIN_RANGE && distance2 <= MAX_RANGE) currentHands = true;
  
  // Update hand state
  if (currentHands) {
    if (!handsDetected) {
      Serial.println("Hands detected!");
      handsDetected = true;
      startAudioPlayback();
    }
    lastHandTime = millis();
  } else {
    if (handsDetected && (millis() - lastHandTime > HAND_TIMEOUT)) {
      Serial.println("Hand timeout");
      handsDetected = false;
      stopAudioPlayback();
    }
  }
  
  // Update LEDs
  if (handsDetected) {
    if (isPlaying) {
      fill_solid(leds, NUM_LEDS, CRGB::Green);  // Playing
    } else {
      fill_solid(leds, NUM_LEDS, CRGB::Yellow); // Hands detected
    }
  } else {
    fill_solid(leds, NUM_LEDS, CRGB::Red);      // Idle
  }
  
  FastLED.show();
  
  // Debug output
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 3000) {
    Serial.print("D1: "); Serial.print(distance1);
    Serial.print(" D2: "); Serial.print(distance2);
    Serial.print(" | Hands: "); Serial.print(handsDetected);
    Serial.print(" | Playing: "); Serial.print(isPlaying);
    Serial.print(" | Sample: "); Serial.println(sampleIndex);
    lastDebug = millis();
  }
  
  delay(50);
} 