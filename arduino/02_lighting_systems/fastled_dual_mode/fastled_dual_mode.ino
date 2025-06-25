/*
 * Dual Ultrasonic Sensor Reader with Advanced FastLED Effects
 * Two modes: Attract mode (sinusoidal fade) and Interactive mode (distance-based colors)
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
#define NUM_LEDS 60        // Adjust this to match your LED strip length
#define LED_TYPE WS2812B   // Change if using different LED type
#define COLOR_ORDER GRB    // Most WS2812B strips use GRB order

CRGB leds[NUM_LEDS];

// Range constants (in cm)
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

// Mode constants
enum LightMode {
  ATTRACT_MODE,
  INTERACTIVE_MODE
};

// State variables
LightMode currentMode = ATTRACT_MODE;
unsigned long lastHandDetectedTime = 0;
const unsigned long INTERACTIVE_TIMEOUT = 10000;  // 10 seconds

// Attract mode variables
const float ATTRACT_PERIOD = 5000.0;  // 5 seconds in milliseconds
float attractPhaseOffset = 0.0;  // For smooth transition back to attract mode
bool usePhaseOffset = false;

// Hand detection averaging
const int SAMPLES = 5;
float distance1Samples[SAMPLES];
float distance2Samples[SAMPLES];
int sampleIndex = 0;
bool samplesInitialized = false;

// Last known interactive colors for smooth transition
CRGB lastLeftColor = CRGB::Yellow;
CRGB lastRightColor = CRGB::Yellow;

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
  FastLED.setBrightness(150);  // Set brightness (0-255)
  
  // Initialize all LEDs to off
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  // Initialize sample arrays
  for (int i = 0; i < SAMPLES; i++) {
    distance1Samples[i] = MAX_RANGE + 1;  // Initialize to "no detection"
    distance2Samples[i] = MAX_RANGE + 1;
  }
}

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  float duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.0343) / 2;
  
  // Return a large value if reading seems invalid
  if (distance <= 0 || duration == 0) {
    return MAX_RANGE + 1;
  }
  
  return distance;
}

float getAveragedDistance(float samples[]) {
  float sum = 0;
  int validSamples = 0;
  
  for (int i = 0; i < SAMPLES; i++) {
    if (samples[i] <= MAX_RANGE) {  // Only count valid readings
      sum += samples[i];
      validSamples++;
    }
  }
  
  if (validSamples == 0) {
    return MAX_RANGE + 1;  // No valid readings
  }
  
  return sum / validSamples;
}

void updateDistanceSamples(float distance1, float distance2) {
  distance1Samples[sampleIndex] = distance1;
  distance2Samples[sampleIndex] = distance2;
  
  sampleIndex = (sampleIndex + 1) % SAMPLES;
  
  if (!samplesInitialized && sampleIndex == 0) {
    samplesInitialized = true;
  }
}

CRGB getInteractiveColor(float distance) {
  if (distance < MIN_RANGE || distance > MAX_RANGE) {
    return CRGB::Black;  // No hand detected
  }
  
  // Map distance to color: far = red, close = yellow
  float ratio = (distance - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
  ratio = constrain(ratio, 0.0, 1.0);
  
  // Red to Yellow interpolation through Orange
  int red = 255;
  int green = (int)(255 * (1.0 - ratio));  // More green when closer (yellow)
  int blue = 0;
  
  return CRGB(red, green, blue);
}

CRGB getAttractColor(unsigned long currentTime) {
  float phase;
  
  if (usePhaseOffset) {
    // Start from the calculated phase offset for smooth transition
    phase = attractPhaseOffset + (2.0 * PI * currentTime / ATTRACT_PERIOD);
    usePhaseOffset = false;  // Only use offset once
  } else {
    // Normal sinusoidal phase
    phase = 2.0 * PI * currentTime / ATTRACT_PERIOD;
  }
  
  // Sinusoidal fade from yellow to red
  float sineValue = (sin(phase) + 1.0) / 2.0;  // Normalize to 0-1
  
  int red = 255;
  int green = (int)(255 * sineValue);  // Yellow when sine = 1, Red when sine = 0
  int blue = 0;
  
  return CRGB(red, green, blue);
}

void calculatePhaseOffset(CRGB currentColor) {
  // Calculate what phase would produce the current color
  // currentColor should be some shade between red and yellow
  float greenRatio = currentColor.green / 255.0;
  
  // Reverse the sine calculation: greenRatio = (sin(phase) + 1) / 2
  // sin(phase) = 2 * greenRatio - 1
  float sineValue = 2.0 * greenRatio - 1.0;
  sineValue = constrain(sineValue, -1.0, 1.0);
  
  attractPhaseOffset = asin(sineValue);
  usePhaseOffset = true;
}

void updateLEDs(float avgDistance1, float avgDistance2, bool handsDetected, unsigned long currentTime) {
  if (currentMode == ATTRACT_MODE) {
    // Both sides show the same attract color
    CRGB attractColor = getAttractColor(currentTime);
    fill_solid(leds, NUM_LEDS, attractColor);
    
  } else {  // INTERACTIVE_MODE
    // Left sensor (first half of strip)
    CRGB leftColor = getInteractiveColor(avgDistance1);
    if (leftColor == CRGB::Black) leftColor = lastLeftColor;  // Maintain last color if no hand
    else lastLeftColor = leftColor;
    
    // Right sensor (second half of strip)
    CRGB rightColor = getInteractiveColor(avgDistance2);
    if (rightColor == CRGB::Black) rightColor = lastRightColor;  // Maintain last color if no hand
    else lastRightColor = rightColor;
    
    // Apply colors to respective halves
    int halfPoint = NUM_LEDS / 2;
    for (int i = 0; i < halfPoint; i++) {
      leds[i] = leftColor;
    }
    for (int i = halfPoint; i < NUM_LEDS; i++) {
      leds[i] = rightColor;
    }
  }
  
  FastLED.show();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read sensors
  float distance1 = readDistance(trigPin1, echoPin1);
  float distance2 = readDistance(trigPin2, echoPin2);
  
  // Update sample arrays
  updateDistanceSamples(distance1, distance2);
  
  // Get averaged distances (only if we have enough samples)
  float avgDistance1, avgDistance2;
  if (samplesInitialized) {
    avgDistance1 = getAveragedDistance(distance1Samples);
    avgDistance2 = getAveragedDistance(distance2Samples);
  } else {
    avgDistance1 = distance1;
    avgDistance2 = distance2;
  }
  
  // Check if hands are detected (within range)
  bool inRange1 = (avgDistance1 >= MIN_RANGE && avgDistance1 <= MAX_RANGE);
  bool inRange2 = (avgDistance2 >= MIN_RANGE && avgDistance2 <= MAX_RANGE);
  bool handsDetected = inRange1 || inRange2;  // Any hand triggers interactive mode
  
  // Mode state machine
  if (handsDetected) {
    if (currentMode == ATTRACT_MODE) {
      // Switch to interactive mode
      currentMode = INTERACTIVE_MODE;
    }
    lastHandDetectedTime = currentTime;
  } else {
    // Check timeout for returning to attract mode
    if (currentMode == INTERACTIVE_MODE && 
        (currentTime - lastHandDetectedTime) >= INTERACTIVE_TIMEOUT) {
      
      // Calculate smooth transition phase based on current colors
      CRGB transitionColor = (lastLeftColor.r + lastLeftColor.g > lastRightColor.r + lastRightColor.g) ? 
                            lastLeftColor : lastRightColor;  // Use brighter color
      calculatePhaseOffset(transitionColor);
      
      currentMode = ATTRACT_MODE;
    }
  }
  
  // Update LED effects
  updateLEDs(avgDistance1, avgDistance2, handsDetected, currentTime);
  
  // Create JSON document for TouchDesigner
  StaticJsonDocument<300> doc;
  
  // Add values to JSON
  doc["left"] = int(avgDistance1);
  doc["right"] = int(avgDistance2);
  doc["hands_detected"] = handsDetected;
  doc["mode"] = (currentMode == ATTRACT_MODE) ? "attract" : "interactive";
  doc["left_in_range"] = inRange1;
  doc["right_in_range"] = inRange2;
  
  // Add color values for TouchDesigner correlation
  if (currentMode == INTERACTIVE_MODE) {
    // Calculate orange values (0-1 scale where 0=red, 1=yellow)
    float leftOrange = inRange1 ? (1.0 - (avgDistance1 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE)) : 
                                 (lastLeftColor.green / 255.0);
    float rightOrange = inRange2 ? (1.0 - (avgDistance2 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE)) : 
                                  (lastRightColor.green / 255.0);
    
    doc["left_orange_value"] = leftOrange;
    doc["right_orange_value"] = rightOrange;
  }
  
  // Serialize JSON to string and send
  serializeJson(doc, Serial);
  Serial.println();  // Add newline for readability
  
  delay(50);  // Smooth animation delay
} 