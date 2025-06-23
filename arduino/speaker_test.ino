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
  playBeep(500, 100);  // Short startup beep
  delay(200);
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
  // ENHANCED SPEAKER TEST - Multiple frequencies and patterns
  Serial.println("Testing speaker - you should hear/feel vibration...");
  
  // Test 1: Low frequency (100Hz) - should feel vibration
  Serial.println("Test 1: 100Hz");
  for (int i = 0; i < 1000; i++) {
    digitalWrite(speakerPin, HIGH);
    delayMicroseconds(5000);  // 100Hz = 10000us period / 2
    digitalWrite(speakerPin, LOW);
    delayMicroseconds(5000);
  }
  delay(1000);
  
  // Test 2: Mid frequency (1kHz) - should hear tone
  Serial.println("Test 2: 1kHz");
  for (int i = 0; i < 1000; i++) {
    digitalWrite(speakerPin, HIGH);
    delayMicroseconds(500);   // 1kHz = 1000us period / 2
    digitalWrite(speakerPin, LOW);
    delayMicroseconds(500);
  }
  delay(1000);
  
  // Test 3: High frequency (2kHz) - should hear higher pitch
  Serial.println("Test 3: 2kHz");
  for (int i = 0; i < 1000; i++) {
    digitalWrite(speakerPin, HIGH);
    delayMicroseconds(250);   // 2kHz = 500us period / 2
    digitalWrite(speakerPin, LOW);
    delayMicroseconds(250);
  }
  delay(1000);
  
  // Test 4: Check if pin is actually changing
  Serial.println("Test 4: Pin state check");
  digitalWrite(speakerPin, HIGH);
  Serial.print("Pin 11 HIGH, voltage should be ~5V. Reading: ");
  Serial.println(digitalRead(speakerPin));
  delay(2000);
  
  digitalWrite(speakerPin, LOW);
  Serial.print("Pin 11 LOW, voltage should be ~0V. Reading: ");
  Serial.println(digitalRead(speakerPin));
  delay(2000);
  
  Serial.println("=== Test cycle complete ===");
  delay(3000);
  
  // Uncomment the sensor code below once speaker test is complete
  /*
  // Read sensors
  float distance1 = readDistance(trigPin1, echoPin1);
  float distance2 = readDistance(trigPin2, echoPin2);
  
  // Check if hands are detected (within range)
  bool inRange1 = (distance1 >= MIN_RANGE && distance1 <= MAX_RANGE);
  bool inRange2 = (distance2 >= MIN_RANGE && distance2 <= MAX_RANGE);
  bool hands_detected = inRange1 && inRange2;
  
  // Control LED strip based on which hand is closer
  if (hands_detected) {
    // Play beep when hands are detected
    playHandsDetectedBeep();
    
    // Calculate the difference between left and right sensor distances
    float difference = distance1 - distance2;
    float maxDifference = MAX_RANGE - MIN_RANGE;
    
    // Map difference to 0-255 for color transition
    int colorValue = map(constrain((difference + maxDifference) * 100, 0, 2 * maxDifference * 100), 
                        0, 2 * maxDifference * 100, 0, 255);
    
    // Create color: Red = Left closer, Blue = Right closer
    CRGB stripColor = CRGB(255 - colorValue, 0, colorValue);
    fill_solid(leds, NUM_LEDS, stripColor);
  } else {
    // Turn all LEDs off when no hands detected
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  }
  FastLED.show();
  
  // Create JSON document for TouchDesigner
  StaticJsonDocument<200> doc;
  doc["left"] = int(distance1);
  doc["right"] = int(distance2);
  doc["hands_detected"] = hands_detected;
  
  // Serialize JSON to string and send
  serializeJson(doc, Serial);
  Serial.println();
  
  delay(10);
  */
}
