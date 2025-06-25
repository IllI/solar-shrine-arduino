/*
 * Solar Shrine - JSON Integration for TouchDesigner
 * Outputs sensor data in JSON format for TouchDesigner integration
 * 
 * Pin Configuration:
 * - Pins 9,10: Ultrasonic sensor 1 (Trig, Echo)
 * - Pins 5,6: Ultrasonic sensor 2 (Trig, Echo)  
 * - Pin 3: LED strip (optional)
 * 
 * Features:
 * - JSON formatted output
 * - TouchDesigner compatibility
 * - Basic LED control
 * - Hand detection logic
 */

#include <ArduinoJson.h>
#include <FastLED.h>

// Sensor pins
const int trigPin1 = 9;
const int echoPin1 = 10;
const int trigPin2 = 5;
const int echoPin2 = 6;

// LED strip configuration (optional)
#define LED_PIN 3
#define NUM_LEDS 60
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

// Range constants
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  // Sensor setup
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  
  // LED setup (optional)
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(100);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  Serial.println("Solar Shrine JSON Integration Started");
}

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  float duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.0343) / 2;
  
  if (distance <= 0 || duration == 0) {
    return MAX_RANGE + 1;
  }
  
  return distance;
}

void updateBasicLEDs(float distance1, float distance2) {
  // Simple LED visualization
  bool inRange1 = (distance1 >= MIN_RANGE && distance1 <= MAX_RANGE);
  bool inRange2 = (distance2 >= MIN_RANGE && distance2 <= MAX_RANGE);
  
  CRGB color1 = inRange1 ? CRGB::Green : CRGB::Red;
  CRGB color2 = inRange2 ? CRGB::Green : CRGB::Red;
  
  // Left half
  for (int i = 0; i < NUM_LEDS/2; i++) {
    leds[i] = color1;
  }
  
  // Right half
  for (int i = NUM_LEDS/2; i < NUM_LEDS; i++) {
    leds[i] = color2;
  }
  
  FastLED.show();
}

void loop() {
  // Read sensors
  float distance1 = readDistance(trigPin1, echoPin1);
  float distance2 = readDistance(trigPin2, echoPin2);
  
  // Check if hands detected
  bool inRange1 = (distance1 >= MIN_RANGE && distance1 <= MAX_RANGE);
  bool inRange2 = (distance2 >= MIN_RANGE && distance2 <= MAX_RANGE);
  bool handsDetected = inRange1 || inRange2;
  
  // Update LEDs
  updateBasicLEDs(distance1, distance2);
  
  // Create JSON document
  StaticJsonDocument<200> doc;
  
  doc["timestamp"] = millis();
  doc["left"] = int(distance1);
  doc["right"] = int(distance2);
  doc["left_in_range"] = inRange1;
  doc["right_in_range"] = inRange2;
  doc["hands_detected"] = handsDetected;
  doc["system"] = "solar_shrine";
  
  // Add normalized values for TouchDesigner
  if (inRange1) {
    float normalized1 = (distance1 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    doc["left_normalized"] = normalized1;
  } else {
    doc["left_normalized"] = -1; // Out of range indicator
  }
  
  if (inRange2) {
    float normalized2 = (distance2 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    doc["right_normalized"] = normalized2;
  } else {
    doc["right_normalized"] = -1; // Out of range indicator
  }
  
  // Serialize and send JSON
  serializeJson(doc, Serial);
  Serial.println();
  
  delay(50); // 20Hz update rate
} 