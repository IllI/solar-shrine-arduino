/*
 * Dual Ultrasonic Sensor Reader with JSON Output
 */

#include <ArduinoJson.h>

// Sensor pins
const int trigPin1 = 9;
const int echoPin1 = 10;
const int trigPin2 = 5;
const int echoPin2 = 6;

// Range constants (in cm)
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

void loop() {
  // Read sensors
  float distance1 = readDistance(trigPin1, echoPin1);
  float distance2 = readDistance(trigPin2, echoPin2);
  
  // Check if hands are detected (within range)
  bool inRange1 = (distance1 >= MIN_RANGE && distance1 <= MAX_RANGE);
  bool inRange2 = (distance2 >= MIN_RANGE && distance2 <= MAX_RANGE);
  bool hands_detected = inRange1 && inRange2;
  
  // Create JSON document
  StaticJsonDocument<200> doc;
  
  // Add values to JSON
  doc["left"] = int(distance1);
  doc["right"] = int(distance2);
  doc["hands_detected"] = hands_detected;
  
  // Serialize JSON to string and send
  serializeJson(doc, Serial);
  Serial.println();  // Add newline for readability
  
  delay(10);
} 
