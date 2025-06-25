/*
 * Speaker Test with Hand Sensor Integration
 * WWZMDiB XH-M543 amplifier + Dayton Audio DAEX32QMB-4 Exciter
 * Beeps when hands are detected by ultrasonic sensors
 */

#include <ArduinoJson.h>
#include <FastLED.h>

// Sensor pins
const int trigPin1 = 9;
const int echoPin1 = 10;
const int trigPin2 = 5;
const int echoPin2 = 6;

// LED strip configuration
#define LED_PIN 3
#define NUM_LEDS 20
#define LED_TYPE WS2815
#define COLOR_ORDER RGB

CRGB leds[NUM_LEDS];

// Speaker/Amplifier pin
const int speakerPin = 11;

// Range constants (in cm)
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

// Beep settings
const int beepFrequency = 1000;  // 1kHz beep
const int beepDuration = 200;    // 200ms beep
unsigned long lastBeepTime = 0;
const unsigned long beepInterval = 500;  // Minimum 500ms between beeps

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  // Sensor setup
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  
  // Speaker setup
  pinMode(speakerPin, OUTPUT);
  digitalWrite(speakerPin, LOW);
  
  // FastLED setup
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(100);
  
  // Initialize all LEDs to off
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  // Test beep on startup
  tone(11, 1000); // 1kHz tone on pin 11
  delay(500);
  noTone(11);
}

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  float duration = pulseIn(echoPin, HIGH);
  return (duration * 0.0343) / 2;
}

void playBeep(int frequency, int duration) {
  // Generate square wave for the specified duration
  unsigned long startTime = millis();
  float period = 1000000.0 / frequency;  // Period in microseconds
  float halfPeriod = period / 2;
  
  while (millis() - startTime < duration) {
    digitalWrite(speakerPin, HIGH);
    delayMicroseconds(halfPeriod);
    digitalWrite(speakerPin, LOW);
    delayMicroseconds(halfPeriod);
  }
}

void playHandsDetectedBeep() {
  unsigned long currentTime = millis();
  
  // Only beep if enough time has passed since last beep
  if (currentTime - lastBeepTime >= beepInterval) {
    playBeep(beepFrequency, beepDuration);
    lastBeepTime = currentTime;
  }
}

void loop() {
  // Read sensors
  float distance1 = readDistance(trigPin1, echoPin1);
  float distance2 = readDistance(trigPin2, echoPin2);
  
  // Check if hands are detected (within range)
  bool inRange1 = (distance1 >= MIN_RANGE && distance1 <= MAX_RANGE);
  bool inRange2 = (distance2 >= MIN_RANGE && distance2 <= MAX_RANGE);
  bool hands_detected = inRange1 && inRange2;

  if (hands_detected) {
    // Lower pitch and reduce loudness
    float avgDistance = (distance1 + distance2) / 2.0;
    int minFreq = 100;
    int maxFreq = 600;
    int freq = map((int)avgDistance, (int)MIN_RANGE, (int)MAX_RANGE, maxFreq, minFreq); // closer = higher pitch
    int period = 1000000 / freq;
    int onTime = period * 0.2; // 20% duty cycle for lower volume
    int offTime = period - onTime;
    unsigned long startTime = micros();
    while (micros() - startTime < 10000) { // play for 10ms per loop
      digitalWrite(speakerPin, HIGH);
      delayMicroseconds(onTime);
      digitalWrite(speakerPin, LOW);
      delayMicroseconds(offTime);
    }
  } else {
    digitalWrite(speakerPin, LOW);
  }

  // Create JSON document for TouchDesigner
  StaticJsonDocument<200> doc;
  doc["left"] = int(distance1);
  doc["right"] = int(distance2);
  doc["hands_detected"] = hands_detected;
  
  // Serialize JSON to string and send
  serializeJson(doc, Serial);
  Serial.println();
  
  delay(10);
}
