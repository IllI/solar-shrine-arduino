/*
 * Solar Shrine with TaskScheduler - Multi-Effect Audio System
 * Handles JSON output, effect rotation, LEDs, and audio concurrently
 * Uses TaskScheduler for proper concurrent process management
 * 
 * Hardware:
 * - 2x HC-SR04 ultrasonic sensors (pins 5,6,10,11)
 * - WS2812B/WS2815 LED strip (pin 3)  
 * - Audio output on pin 9
 * 
 * Libraries Required:
 * - TaskScheduler (install via Library Manager)
 * - FastLED
 * - ArduinoJson
 * - NewPing
 * - MozziGuts (for theremin effect)
 */

#include <TaskScheduler.h>
#include <ArduinoJson.h>
#include <FastLED.h>
#include <NewPing.h>

// Mozzi for theremin effect
#include <MozziGuts.h>
#include <Oscil.h>
#include <tables/triangle_valve_2048_int8.h>
Oscil <TRIANGLE_VALVE_2048_NUM_CELLS, AUDIO_RATE> osc(TRIANGLE_VALVE_2048_DATA);
#define CONTROL_RATE 128

// Sensor pins
const int trigPin1 = 10;
const int echoPin1 = 11;
const int trigPin2 = 5;
const int echoPin2 = 6;

NewPing sonar1(trigPin1, echoPin1, 200);
NewPing sonar2(trigPin2, echoPin2, 200);

// LED strip configuration
#define LED_PIN 3
#define NUM_LEDS 60
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

// Range constants
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

// ===========================================
// SHARED STATE VARIABLES
// ===========================================
struct SharedState {
  float leftDistance = 0;
  float rightDistance = 0;
  bool leftHandDetected = false;
  bool rightHandDetected = false;
  bool handsDetected = false;
  unsigned long lastHandTime = 0;
  
  // Mode state
  enum LightMode { ATTRACT_MODE, INTERACTIVE_MODE } currentMode = ATTRACT_MODE;
  
  // Effect rotation
  enum EffectType { 
    EFFECT_THEREMIN = 0,
    EFFECT_ALIEN_SOUND = 1,
    EFFECT_ROBOTS_TALKING = 2,
    EFFECT_DJ_SCRATCH = 3
  } currentEffect = EFFECT_THEREMIN;
  
  // Audio state
  bool thereminActive = false;
  int audioVolume = 0;
  float currentFrequency = 220.0;
  
  // JSON output control
  bool jsonNeedsUpdate = false;
  
} sharedState;

// ===========================================
// TASK SCHEDULER SETUP
// ===========================================
Scheduler taskScheduler;

// Task definitions with different priorities and intervals
Task sensorTask(20, TASK_FOREVER, &readSensors);           // 20ms - High priority
Task audioTask(5, TASK_FOREVER, &updateAudio);             // 5ms - Highest priority
Task ledTask(33, TASK_FOREVER, &updateLEDs);               // 33ms - Medium priority
Task rotationTask(5000, TASK_FOREVER, &checkEffectRotation); // 5s - Low priority
Task jsonTask(100, TASK_FOREVER, &sendJSONUpdate);         // 100ms - Low priority

// ===========================================
// SENSOR READING TASK
// ===========================================
void readSensors() {
  sharedState.leftDistance = readDistanceNewPing(sonar1);
  sharedState.rightDistance = readDistanceNewPing(sonar2);
  
  sharedState.leftHandDetected = (sharedState.leftDistance >= MIN_RANGE && sharedState.leftDistance <= MAX_RANGE);
  sharedState.rightHandDetected = (sharedState.rightDistance >= MIN_RANGE && sharedState.rightDistance <= MAX_RANGE);
  sharedState.handsDetected = sharedState.leftHandDetected || sharedState.rightHandDetected;
  
  if (sharedState.handsDetected) {
    sharedState.lastHandTime = millis();
    if (sharedState.currentMode == SharedState::ATTRACT_MODE) {
      sharedState.currentMode = SharedState::INTERACTIVE_MODE;
      sharedState.jsonNeedsUpdate = true;
    }
  } else {
    if (sharedState.currentMode == SharedState::INTERACTIVE_MODE && 
        (millis() - sharedState.lastHandTime) >= 10000) {
      sharedState.currentMode = SharedState::ATTRACT_MODE;
      sharedState.jsonNeedsUpdate = true;
    }
  }
}

// ===========================================
// AUDIO UPDATE TASK
// ===========================================
void updateAudio() {
  switch (sharedState.currentEffect) {
    case SharedState::EFFECT_THEREMIN:
      updateThereminAudio();
      break;
    case SharedState::EFFECT_ALIEN_SOUND:
      updateAlienAudio();
      break;
    case SharedState::EFFECT_ROBOTS_TALKING:
      updateRobotAudio();
      break;
    case SharedState::EFFECT_DJ_SCRATCH:
      updateDJScratchAudio();
      break;
  }
  
  // Always call Mozzi audio hook for theremin effect
  if (sharedState.currentEffect == SharedState::EFFECT_THEREMIN) {
    audioHook();
  }
}

void updateThereminAudio() {
  if (sharedState.handsDetected) {
    float avgDistance = (sharedState.leftDistance + sharedState.rightDistance) / 2.0;
    float ratio = (avgDistance - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    ratio = constrain(ratio, 0.0, 1.0);
    
    sharedState.currentFrequency = 131 + (1046 - 131) * (1.0 - ratio);
    osc.setFreq(sharedState.currentFrequency);
    sharedState.audioVolume = 200;
    sharedState.thereminActive = true;
  } else {
    sharedState.audioVolume = 0;
    sharedState.thereminActive = false;
  }
}

void updateAlienAudio() {
  // Placeholder for alien sound synthesis
  sharedState.thereminActive = sharedState.handsDetected;
}

void updateRobotAudio() {
  // Placeholder for robot voice synthesis
  sharedState.thereminActive = sharedState.handsDetected;
}

void updateDJScratchAudio() {
  // Placeholder for DJ scratch audio
  sharedState.thereminActive = sharedState.handsDetected;
}

// Required Mozzi functions
void updateControl() {
  // Called by audioHook()
}

int updateAudio() {
  return (osc.next() * sharedState.audioVolume) >> 8;
}

// ===========================================
// LED UPDATE TASK
// ===========================================
void updateLEDs() {
  if (sharedState.currentMode == SharedState::ATTRACT_MODE) {
    // Attract mode - pulsing effect
    float phase = 2.0 * PI * millis() / 5000.0;
    float sineValue = (sin(phase) + 1.0) / 2.0;
    int green = (int)(255 * sineValue);
    fill_solid(leds, NUM_LEDS, CRGB(255, green, 0));
    
  } else {
    // Interactive mode - split by effect
    CRGB effectColor;
    switch (sharedState.currentEffect) {
      case SharedState::EFFECT_THEREMIN: effectColor = CRGB::Blue; break;
      case SharedState::EFFECT_ALIEN_SOUND: effectColor = CRGB::Green; break;
      case SharedState::EFFECT_ROBOTS_TALKING: effectColor = CRGB::Purple; break;
      case SharedState::EFFECT_DJ_SCRATCH: effectColor = CRGB::Red; break;
    }
    
    // Apply hand-based intensity
    if (sharedState.handsDetected) {
      int leftIntensity = map(sharedState.leftDistance, MIN_RANGE, MAX_RANGE, 255, 50);
      int rightIntensity = map(sharedState.rightDistance, MIN_RANGE, MAX_RANGE, 255, 50);
      
      // Left half
      CRGB leftColor = effectColor;
      if (sharedState.leftHandDetected) {
        leftColor.fadeToBlackBy(255 - leftIntensity);
      } else {
        leftColor.fadeToBlackBy(200);
      }
      
      // Right half
      CRGB rightColor = effectColor;
      if (sharedState.rightHandDetected) {
        rightColor.fadeToBlackBy(255 - rightIntensity);
      } else {
        rightColor.fadeToBlackBy(200);
      }
      
      int halfPoint = NUM_LEDS / 2;
      for (int i = 0; i < halfPoint; i++) {
        leds[i] = leftColor;
      }
      for (int i = halfPoint; i < NUM_LEDS; i++) {
        leds[i] = rightColor;
      }
    } else {
      fill_solid(leds, NUM_LEDS, CRGB::Black);
    }
  }
  
  FastLED.show();
}

// ===========================================
// EFFECT ROTATION TASK
// ===========================================
void checkEffectRotation() {
  // Only rotate if no hands detected for 5 seconds
  if (!sharedState.handsDetected && 
      (millis() - sharedState.lastHandTime) >= 5000) {
    
    // Rotate to next effect
    int nextEffect = (sharedState.currentEffect + 1) % 4;
    sharedState.currentEffect = static_cast<SharedState::EffectType>(nextEffect);
    
    // Reset audio state for new effect
    sharedState.audioVolume = 0;
    sharedState.thereminActive = false;
    
    // Trigger JSON update
    sharedState.jsonNeedsUpdate = true;
    
    // Debug output
    Serial.print("Rotated to effect: ");
    switch (sharedState.currentEffect) {
      case SharedState::EFFECT_THEREMIN: Serial.println("Theremin"); break;
      case SharedState::EFFECT_ALIEN_SOUND: Serial.println("Alien Sound"); break;
      case SharedState::EFFECT_ROBOTS_TALKING: Serial.println("Robots Talking"); break;
      case SharedState::EFFECT_DJ_SCRATCH: Serial.println("DJ Scratch"); break;
    }
    
    // Reset rotation timer
    sharedState.lastHandTime = millis();
  }
}

// ===========================================
// JSON OUTPUT TASK
// ===========================================
void sendJSONUpdate() {
  // Only send updates when something has changed OR hands are detected
  if (sharedState.jsonNeedsUpdate || sharedState.handsDetected) {
    StaticJsonDocument<500> doc;
    
    // Basic sensor data
    doc["left"] = int(sharedState.leftDistance);
    doc["right"] = int(sharedState.rightDistance);
    doc["hands_detected"] = sharedState.handsDetected;
    doc["left_in_range"] = sharedState.leftHandDetected;
    doc["right_in_range"] = sharedState.rightHandDetected;
    
    // Mode and effect information
    doc["mode"] = (sharedState.currentMode == SharedState::ATTRACT_MODE) ? "attract" : "interactive";
    doc["current_effect"] = sharedState.currentEffect;
    doc["effect_name"] = getEffectName(sharedState.currentEffect);
    
    // Audio state
    doc["theremin_active"] = sharedState.thereminActive;
    doc["audio_volume"] = sharedState.audioVolume;
    doc["current_frequency"] = sharedState.currentFrequency;
    
    // Color information for TouchDesigner
    if (sharedState.currentMode == SharedState::INTERACTIVE_MODE) {
      float leftIntensity = sharedState.leftHandDetected ? 
        (1.0 - (sharedState.leftDistance - MIN_RANGE) / (MAX_RANGE - MIN_RANGE)) : 0.0;
      float rightIntensity = sharedState.rightHandDetected ? 
        (1.0 - (sharedState.rightDistance - MIN_RANGE) / (MAX_RANGE - MIN_RANGE)) : 0.0;
      
      doc["left_intensity"] = leftIntensity;
      doc["right_intensity"] = rightIntensity;
    }
    
    // Send JSON
    serializeJson(doc, Serial);
    Serial.println();
    
    sharedState.jsonNeedsUpdate = false;
  }
}

String getEffectName(SharedState::EffectType effect) {
  switch (effect) {
    case SharedState::EFFECT_THEREMIN: return "theremin";
    case SharedState::EFFECT_ALIEN_SOUND: return "alien_sound";
    case SharedState::EFFECT_ROBOTS_TALKING: return "robots_talking";
    case SharedState::EFFECT_DJ_SCRATCH: return "dj_scratch";
    default: return "unknown";
  }
}

// ===========================================
// UTILITY FUNCTIONS
// ===========================================
float readDistanceNewPing(NewPing &sensor) {
  unsigned int distance = sensor.ping_cm();
  if (distance == 0) {
    return MIN_RANGE - 0.5;
  }
  if (distance > MAX_RANGE) {
    return MAX_RANGE + 10;
  }
  return (float)distance;
}

// ===========================================
// SETUP AND MAIN LOOP
// ===========================================
void setup() {
  Serial.begin(9600);
  delay(1000);
  
  // Initialize Mozzi
  startMozzi(CONTROL_RATE);
  osc.setFreq(220);
  
  // Initialize FastLED
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(150);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  // Initialize TaskScheduler
  taskScheduler.init();
  
  // Add and enable all tasks
  taskScheduler.addTask(sensorTask);
  taskScheduler.addTask(audioTask);
  taskScheduler.addTask(ledTask);
  taskScheduler.addTask(rotationTask);
  taskScheduler.addTask(jsonTask);
  
  sensorTask.enable();
  audioTask.enable();
  ledTask.enable();
  rotationTask.enable();
  jsonTask.enable();
  
  Serial.println("Solar Shrine TaskScheduler Ready");
  Serial.println("Effects: Theremin -> Alien Sound -> Robots Talking -> DJ Scratch");
}

void loop() {
  taskScheduler.execute();  // This single line handles ALL concurrent processes!
} 