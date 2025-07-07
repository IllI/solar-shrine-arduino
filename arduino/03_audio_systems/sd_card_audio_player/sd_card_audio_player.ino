/*
 * Solar Shrine SD Card Audio Player
 * Using TMRpcm library for proper Arduino audio playback
 * 
 * Based on research from: https://simple-circuit.com/arduino-wave-audio-player-sd-card/
 * This method uses higher PWM frequencies and proper audio processing
 * 
 * Hardware Required:
 * - Arduino UNO
 * - MicroSD card module (SPI pins: 10,11,12,13)
 * - SD card with WAV files (16kHz, 8-bit, mono, PCM format)
 * - Audio amplifier + speaker
 * - 2x HC-SR04 ultrasonic sensors
 * - WS2812B LED strip
 * 
 * Libraries Required:
 * - TMRpcm (install via Library Manager)
 * - SD (built-in)
 * - SPI (built-in)
 * - FastLED
 * - NewPing
 */

#include <SPI.h>
#include <SD.h>
#include <TMRpcm.h>
#include <FastLED.h>
#include <NewPing.h>

// Audio setup
TMRpcm audio;
const int SD_CS_PIN = 10;  // SD card chip select pin
const int SPEAKER_PIN = 9; // Audio output pin

// Sensor pins
const int trigPin1 = 5;
const int echoPin1 = 6;
const int trigPin2 = 7;
const int echoPin2 = 8;

// LED setup
const int LED_PIN = 3;
const int NUM_LEDS = 60;
CRGB leds[NUM_LEDS];

// Sensor objects
NewPing sonar1(trigPin1, echoPin1, 200);
NewPing sonar2(trigPin2, echoPin2, 200);

// Audio files
String audioFiles[] = {"sample1.wav", "sample2.wav", "sample3.wav"};
int currentFile = 0;
int totalFiles = 3;

// Range constants
const float MIN_RANGE = 2.0;
const float MAX_RANGE = 30.0;

// State variables
bool handsDetected = false;
bool isPlaying = false;
unsigned long lastHandTime = 0;
const unsigned long HAND_TIMEOUT = 5000;

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  Serial.println("=== Solar Shrine SD Card Audio Player ===");
  
  // Initialize SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Failed!");
    Serial.println("Check SD card connection and formatting (FAT32)");
    while(true) {
      // Flash red LED to indicate SD card error
      fill_solid(leds, NUM_LEDS, CRGB::Red);
      FastLED.show();
      delay(500);
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      FastLED.show();
      delay(500);
    }
  }
  Serial.println("OK!");
  
  // Initialize FastLED
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(150);
  
  // Initialize TMRpcm
  audio.speakerPin = SPEAKER_PIN;
  audio.setVolume(6);    // 0-7 volume range
  audio.quality(1);      // 1 = high quality (2x oversampling)
  
  // List files on SD card
  Serial.println("Files on SD card:");
  listFiles();
  
  // Test audio
  Serial.println("Playing test audio...");
  if (SD.exists("test.wav")) {
    audio.play("test.wav");
    delay(2000);
    audio.stopPlayback();
  }
  
  // Startup LED sequence
  fill_solid(leds, NUM_LEDS, CRGB::Green);
  FastLED.show();
  delay(1000);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  Serial.println("Ready - wave hands to play audio");
}

void listFiles() {
  File root = SD.open("/");
  
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    
    if (!entry.isDirectory()) {
      String fileName = entry.name();
      if (fileName.endsWith(".WAV") || fileName.endsWith(".wav")) {
        Serial.print("  ");
        Serial.print(fileName);
        Serial.print(" (");
        Serial.print(entry.size());
        Serial.println(" bytes)");
      }
    }
    entry.close();
  }
  root.close();
}

void updateHandDetection() {
  // Read sensors
  float distance1 = sonar1.ping_cm();
  float distance2 = sonar2.ping_cm();
  
  // Convert 0 to max range
  if (distance1 == 0) distance1 = MAX_RANGE + 1;
  if (distance2 == 0) distance2 = MAX_RANGE + 1;
  
  // Check for hands
  bool currentHands = false;
  if (distance1 >= MIN_RANGE && distance1 <= MAX_RANGE) currentHands = true;
  if (distance2 >= MIN_RANGE && distance2 <= MAX_RANGE) currentHands = true;
  
  // Update state
  if (currentHands) {
    if (!handsDetected) {
      Serial.println("Hands detected!");
      handsDetected = true;
    }
    lastHandTime = millis();
  } else {
    if (handsDetected && (millis() - lastHandTime > HAND_TIMEOUT)) {
      Serial.println("Hand timeout - stopping audio");
      handsDetected = false;
    }
  }
}

void updateAudioPlayback() {
  if (handsDetected) {
    if (!isPlaying && !audio.isPlaying()) {
      // Start playing audio
      String fileName = audioFiles[currentFile];
      if (SD.exists(fileName)) {
        Serial.print("Playing: ");
        Serial.println(fileName);
        audio.play(fileName);
        isPlaying = true;
      } else {
        Serial.print("File not found: ");
        Serial.println(fileName);
      }
    }
  } else {
    if (isPlaying) {
      Serial.println("Stopping audio playback");
      audio.stopPlayback();
      isPlaying = false;
    }
  }
}

void updateLEDs() {
  if (handsDetected) {
    if (isPlaying) {
      // Green when playing
      fill_solid(leds, NUM_LEDS, CRGB::Green);
    } else {
      // Yellow when hands detected but not playing
      fill_solid(leds, NUM_LEDS, CRGB::Yellow);
    }
  } else {
    // Red when idle
    fill_solid(leds, NUM_LEDS, CRGB::Red);
  }
  
  FastLED.show();
}

void loop() {
  // Update hand detection
  updateHandDetection();
  
  // Update audio playback
  updateAudioPlayback();
  
  // Update LEDs
  updateLEDs();
  
  // Check if playback finished
  if (isPlaying && !audio.isPlaying()) {
    isPlaying = false;
    Serial.println("Playback finished");
  }
  
  // Debug output
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 3000) {
    Serial.print("Hands: ");
    Serial.print(handsDetected);
    Serial.print(" | Playing: ");
    Serial.print(isPlaying);
    Serial.print(" | TMRpcm Playing: ");
    Serial.println(audio.isPlaying());
    lastDebug = millis();
  }
  
  delay(100);
} 