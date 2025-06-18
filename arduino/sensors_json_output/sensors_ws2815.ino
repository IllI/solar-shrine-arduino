/*
 * Dual Ultrasonic Sensor Reader with JSON Output and FastLED Control
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
#define NUM_LEDS 20        // Adjust this to match your LED strip length
#define LED_TYPE WS2815    // WS2815 LED type
#define COLOR_ORDER RGB    // WS2815 strips typically use RGB order

CRGB leds[NUM_LEDS];

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
  
  // FastLED setup
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(100);  // Set brightness (0-255)
  
  // Initialize all LEDs to off
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
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
  
  // Control LED strip based on which hand is closer
  if (hands_detected) {
    // Calculate the difference between left and right sensor distances
    float difference = distance1 - distance2;  // Positive = left closer, Negative = right closer
    
    // Normalize the difference to 0-255 range
    // Maximum possible difference is around 19cm (MAX_RANGE - MIN_RANGE)
    float maxDifference = MAX_RANGE - MIN_RANGE;
    
    // Map difference to 0-255: 
    // Left much closer = 0 (red), Right much closer = 255 (blue), Equal distances = 127 (purple)
    int colorValue = map(constrain((difference + maxDifference) * 100, 0, 2 * maxDifference * 100), 
                        0, 2 * maxDifference * 100, 0, 255);
    
    // Create color based on which hand is closer
    // Red = Left hand closer (distance1 < distance2)
    // Blue = Right hand closer (distance2 < distance1)
    // Purple = Equal distances
    CRGB stripColor = CRGB(255 - colorValue, 0, colorValue);
    
    // Fill entire strip with the calculated color
    fill_solid(leds, NUM_LEDS, stripColor);
  } else {
    // Turn all LEDs off when no hands detected
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  }
  FastLED.show();  // Update the LED strip
  
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
