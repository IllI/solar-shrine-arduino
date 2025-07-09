/*
 * Solar Shrine Effect Rotator - Optimized for Different Audio Timing Requirements
 * 
 * This system handles 4 different audio effects with their specific timing needs:
 * 1. Theremin (Mozzi, CONTROL_RATE 128)
 * 2. Alien Sound (Timer1 ISR FM synthesis)
 * 3. Robots Talking (Mozzi, CONTROL_RATE 256)
 * 4. DJ Scratch (Timer1 ISR with PROGMEM audio)
 * 
 * KEY INSIGHT: Each effect has different timing requirements that cannot be 
 * handled by TaskScheduler. We use effect-specific timing approaches.
 * 
 * Hardware:
 * - 2x HC-SR04 sensors (pins 5,6,10,11)
 * - WS2812B LED strip (pin 3)
 * - Audio on pin 9
 */

#include <ArduinoJson.h>
#include <FastLED.h>
#include <NewPing.h>
#include <math.h>

// Mozzi includes (for Theremin and Robots effects)
#include <MozziGuts.h>
#include <Oscil.h>
#include <RollingAverage.h>
#include <tables/triangle_valve_2048_int8.h>
#include <tables/sin2048_int8.h>
#include <tables/cos2048_int8.h>

// Hardware setup
const int trigPin1 = 10, echoPin1 = 11;
const int trigPin2 = 5, echoPin2 = 6;
NewPing sonar1(trigPin1, echoPin1, 200);
NewPing sonar2(trigPin2, echoPin2, 200);

// LED setup
#define LED_PIN 3
#define NUM_LEDS 60
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

// Range constants
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

// Effect definitions
enum EffectType {
  EFFECT_THEREMIN = 0,
  EFFECT_ALIEN = 1,
  EFFECT_ROBOTS = 2,
  EFFECT_DJ_SCRATCH = 3
};

// System state
struct SystemState {
  EffectType currentEffect;
  unsigned long lastHandDetected;
  unsigned long lastEffectChange;
  unsigned long lastJSONSend;
  bool handsDetected;
  float distance1, distance2;
  bool inRange1, inRange2;
  int audioVolume;
  bool effectActive;
  bool mozziActive;
  bool timer1Active;
  int currentControlRate;
};

SystemState state;

// Timing constants
const unsigned long EFFECT_ROTATION_INTERVAL = 5000;  // 5 seconds
const unsigned long JSON_SEND_INTERVAL = 100;        // 100ms
const unsigned long LED_UPDATE_INTERVAL = 33;        // ~30fps
const unsigned long SENSOR_READ_INTERVAL = 20;       // 50Hz sensor reading

// Last update times for non-blocking timing
unsigned long lastSensorRead = 0;
unsigned long lastLEDUpdate = 0;

// Effect names for JSON output
const char* effectNames[] = {"Theremin", "Alien", "Robots", "DJ Scratch"};

// LED colors for each effect
CRGB effectColors[] = {
  CRGB::Blue,     // Theremin
  CRGB::Green,    // Alien
  CRGB::Purple,   // Robots
  CRGB::Red       // DJ Scratch
};

// =============================================================================
// EFFECT 1: THEREMIN (Mozzi-based, 128 Hz control rate)
// =============================================================================
#define CONTROL_RATE_THEREMIN 128
Oscil <TRIANGLE_VALVE_2048_NUM_CELLS, AUDIO_RATE> thereminOsc(TRIANGLE_VALVE_2048_DATA);
RollingAverage <int, 4> thereminFreqAvg;
RollingAverage <int, 8> thereminVolAvg;

void setupTheremin() {
  if (state.timer1Active) {
    TIMSK1 = 0;  // Disable Timer1 interrupt
    state.timer1Active = false;
  }
  
  startMozzi(CONTROL_RATE_THEREMIN);
  thereminOsc.setFreq(220);
  state.mozziActive = true;
  state.currentControlRate = CONTROL_RATE_THEREMIN;
}

void updateThereminControl() {
  if (!state.handsDetected) {
    state.audioVolume = state.audioVolume - 4;
    if (state.audioVolume < 0) state.audioVolume = 0;
    return;
  }
  
  // Calculate frequency based on hand positions (from minimin_theremin_test)
  float avgDistance = (state.distance1 + state.distance2) / 2.0;
  if (avgDistance < MIN_RANGE) avgDistance = MIN_RANGE;
  if (avgDistance > MAX_RANGE) avgDistance = MAX_RANGE;
  
  int freq = 220 + (880 - 220) * (1.0 - (avgDistance - MIN_RANGE) / (MAX_RANGE - MIN_RANGE));
  freq = constrain(freq, 220, 880);
  
  int smoothedFreq = thereminFreqAvg.next(freq);
  thereminOsc.setFreq(smoothedFreq);
  
  state.audioVolume = state.audioVolume + 4;
  if (state.audioVolume > 255) state.audioVolume = 255;
}

int updateThereminAudio() {
  return (thereminOsc.next() * state.audioVolume) >> 8;
}

// =============================================================================
// EFFECT 2: ALIEN SOUND (Timer1 ISR-based FM synthesis)
// =============================================================================
volatile uint16_t alienPhase = 0;
volatile uint16_t alienMod = 0;
volatile bool alienActive = false;
volatile uint8_t alienVolume = 0;

void setupAlien() {
  if (state.mozziActive) {
    // Stop Mozzi
    state.mozziActive = false;
  }
  
  // Setup Timer1 for alien sound (from alien_sound_effect approach)
  pinMode(9, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = 399;  // ~40kHz sample rate
  OCR1A = ICR1 / 2;
  TIMSK1 = _BV(OCIE1A);
  
  state.timer1Active = true;
}

// Combined ISR for both Timer1 effects (Alien and DJ Scratch)
ISR(TIMER1_COMPA_vect) {
  if (state.currentEffect == EFFECT_ALIEN) {
    // Alien Sound ISR
    if (alienActive && alienVolume > 0) {
      alienPhase += 200;
      alienMod += 50;
      
      uint8_t modulator = (alienMod >> 8) & 0xFF;
      uint8_t carrier = ((alienPhase + (modulator * 4)) >> 8) & 0xFF;
      
      int16_t amp = (carrier > 128) ? (alienVolume / 4) : -(alienVolume / 4);
      OCR1A = ((uint32_t)(amp + 128) * ICR1) / 255;
    } else {
      OCR1A = ICR1 / 2;  // Silence
    }
  } else if (state.currentEffect == EFFECT_DJ_SCRATCH) {
    // DJ Scratch ISR
    djSampleCounter++;
    
    uint8_t currentSpeed = djScratchMode ? 2 : djPlaybackSpeed;
    
    if (djSampleCounter >= currentSpeed) {
      djSampleCounter = 0;
      
      if (djPlayState == 1) {
        if (djSampleIndex < 0) djSampleIndex = 0;
        if (djSampleIndex >= DJ_BUFFER_SIZE) djSampleIndex = 0;
        
        uint8_t sample = djAudioBuffer[djSampleIndex];
        int16_t amp = ((int16_t)sample - 128) * 4;
        amp = constrain(amp, -128, 127);
        
        OCR1A = ((uint32_t)(amp + 128) * ICR1) / 255;
        
        if (djScratchMode) {
          djSampleIndex += djScratchSpeed;
          if (djSampleIndex < 0) djSampleIndex = DJ_BUFFER_SIZE - 1;
          if (djSampleIndex >= DJ_BUFFER_SIZE) djSampleIndex = 0;
        } else {
          djSampleIndex++;
          if (djSampleIndex >= DJ_BUFFER_SIZE) djSampleIndex = 0;
        }
      } else {
        OCR1A = ICR1 / 2;
      }
    }
  } else {
    OCR1A = ICR1 / 2;  // Silence for other effects
  }
}

void updateAlienControl() {
  alienActive = state.handsDetected;
  if (state.handsDetected) {
    alienVolume = 200;
    state.audioVolume = 200;
  } else {
    alienVolume = 0;
    state.audioVolume = 0;
  }
}

// =============================================================================
// EFFECT 3: ROBOTS TALKING (Mozzi-based, 256 Hz control rate)
// =============================================================================
#define CONTROL_RATE_ROBOTS 256
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> robotsOsc(SIN2048_DATA);
Oscil <COS2048_NUM_CELLS, CONTROL_RATE_ROBOTS> robotsModulator(COS2048_DATA);

void setupRobots() {
  if (state.timer1Active) {
    TIMSK1 = 0;  // Disable Timer1 interrupt
    state.timer1Active = false;
  }
  
  startMozzi(CONTROL_RATE_ROBOTS);
  robotsOsc.setFreq(440);
  robotsModulator.setFreq(5);
  state.mozziActive = true;
  state.currentControlRate = CONTROL_RATE_ROBOTS;
}

void updateRobotsControl() {
  if (!state.handsDetected) {
    state.audioVolume = state.audioVolume - 4;
    if (state.audioVolume < 0) state.audioVolume = 0;
    return;
  }
  
  // Robot-like frequency modulation (from robots_talking approach)
  float modulation = robotsModulator.next() * 0.3;
  int baseFreq = 200 + (state.distance1 + state.distance2) * 10;
  robotsOsc.setFreq(baseFreq + (int)(baseFreq * modulation));
  
  state.audioVolume = state.audioVolume + 4;
  if (state.audioVolume > 255) state.audioVolume = 255;
}

int updateRobotsAudio() {
  int sample = robotsOsc.next();
  // Add square wave distortion for robot effect
  sample = (sample > 0) ? 127 : -127;
  return (sample * state.audioVolume) >> 8;
}

// =============================================================================
// EFFECT 4: DJ SCRATCH (Timer1 ISR with audio buffer simulation)
// =============================================================================
volatile int32_t djSampleIndex = 0;
volatile uint8_t djSampleCounter = 0;
volatile uint8_t djPlayState = 0;
volatile uint8_t djPlaybackSpeed = 5;
volatile bool djScratchMode = false;
volatile int8_t djScratchSpeed = 1;

// Simple audio buffer simulation (instead of PROGMEM)
const int DJ_BUFFER_SIZE = 512;
volatile uint8_t djAudioBuffer[DJ_BUFFER_SIZE];

void setupDJScratch() {
  if (state.mozziActive) {
    state.mozziActive = false;
  }
  
  // Initialize audio buffer with simple waveform
  for (int i = 0; i < DJ_BUFFER_SIZE; i++) {
    djAudioBuffer[i] = 128 + (sin(i * 0.1) * 100); // Simple sine wave
  }
  
  // Setup Timer1 for DJ scratch (from dj_scratch_progmem approach)
  pinMode(9, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = 399;
  OCR1A = ICR1 / 2;
  TIMSK1 = _BV(OCIE1A);
  
  state.timer1Active = true;
}



void updateDJScratchControl() {
  // Left hand = play/stop, Right hand = scratch
  djPlayState = state.handsDetected ? 1 : 0;
  djScratchMode = state.inRange1 && state.inRange2;  // Both hands = scratch mode
  
  if (djScratchMode) {
    djScratchSpeed = (state.distance1 < state.distance2) ? 2 : -2;
  } else {
    djScratchSpeed = 1;
  }
  
  state.audioVolume = djPlayState ? 200 : 0;
}

// =============================================================================
// CORE SYSTEM FUNCTIONS
// =============================================================================

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  // Initialize state
  state.currentEffect = EFFECT_THEREMIN;
  state.lastHandDetected = 0;
  state.lastEffectChange = 0;
  state.lastJSONSend = 0;
  state.handsDetected = false;
  state.effectActive = false;
  state.mozziActive = false;
  state.timer1Active = false;
  state.audioVolume = 0;
  
  // LED setup
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(150);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  // Start with first effect
  switchToEffect(EFFECT_THEREMIN);
  
  Serial.println("Solar Shrine Effect Rotator Ready");
  Serial.println("Effects: Theremin -> Alien -> Robots -> DJ Scratch");
}

void switchToEffect(EffectType newEffect) {
  if (newEffect == state.currentEffect) return;
  
  Serial.print("Switching to effect: ");
  Serial.println(effectNames[newEffect]);
  
  // Cleanup current effect
  if (state.timer1Active) {
    TIMSK1 = 0;  // Disable Timer1 interrupt
    state.timer1Active = false;
  }
  
  state.currentEffect = newEffect;
  state.lastEffectChange = millis();
  
  // Initialize new effect
  switch (newEffect) {
    case EFFECT_THEREMIN:
      setupTheremin();
      break;
    case EFFECT_ALIEN:
      setupAlien();
      break;
    case EFFECT_ROBOTS:
      setupRobots();
      break;
    case EFFECT_DJ_SCRATCH:
      setupDJScratch();
      break;
  }
}

void readSensors() {
  unsigned long currentTime = millis();
  if (currentTime - lastSensorRead < SENSOR_READ_INTERVAL) return;
  lastSensorRead = currentTime;
  
  // Read sensors
  unsigned int distance1 = sonar1.ping_cm();
  unsigned int distance2 = sonar2.ping_cm();
  
  state.distance1 = (distance1 == 0) ? MIN_RANGE - 0.5 : (float)distance1;
  state.distance2 = (distance2 == 0) ? MIN_RANGE - 0.5 : (float)distance2;
  
  // Determine if hands are in range
  state.inRange1 = (state.distance1 >= MIN_RANGE && state.distance1 <= MAX_RANGE);
  state.inRange2 = (state.distance2 >= MIN_RANGE && state.distance2 <= MAX_RANGE);
  state.handsDetected = state.inRange1 || state.inRange2;
  
  if (state.handsDetected) {
    state.lastHandDetected = currentTime;
  }
}

void updateAudioControl() {
  // Update audio based on current effect
  switch (state.currentEffect) {
    case EFFECT_THEREMIN:
      updateThereminControl();
      break;
    case EFFECT_ALIEN:
      updateAlienControl();
      break;
    case EFFECT_ROBOTS:
      updateRobotsControl();
      break;
    case EFFECT_DJ_SCRATCH:
      updateDJScratchControl();
      break;
  }
}

void updateLEDs() {
  unsigned long currentTime = millis();
  if (currentTime - lastLEDUpdate < LED_UPDATE_INTERVAL) return;
  lastLEDUpdate = currentTime;
  
  CRGB color = effectColors[state.currentEffect];
  
  if (state.handsDetected) {
    // Interactive mode - show hand positions
    int halfPoint = NUM_LEDS / 2;
    
    for (int i = 0; i < halfPoint; i++) {
      if (state.inRange1) {
        float intensity = 1.0 - (state.distance1 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
        leds[i] = color;
        leds[i].fadeToBlackBy(255 - (intensity * 255));
      } else {
        leds[i] = CRGB::Black;
      }
    }
    
    for (int i = halfPoint; i < NUM_LEDS; i++) {
      if (state.inRange2) {
        float intensity = 1.0 - (state.distance2 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
        leds[i] = color;
        leds[i].fadeToBlackBy(255 - (intensity * 255));
      } else {
        leds[i] = CRGB::Black;
      }
    }
  } else {
    // Attract mode - pulse effect color
    float pulse = (sin(millis() * 0.003) + 1.0) / 2.0;
    fill_solid(leds, NUM_LEDS, color);
    fadeToBlackBy(leds, NUM_LEDS, 255 - (pulse * 255));
  }
  
  FastLED.show();
}

void checkEffectRotation() {
  unsigned long currentTime = millis();
  
  // Check if we should rotate effects (5 seconds after hands removed)
  if (!state.handsDetected && 
      (currentTime - state.lastHandDetected) >= EFFECT_ROTATION_INTERVAL &&
      (currentTime - state.lastEffectChange) >= EFFECT_ROTATION_INTERVAL) {
    
    EffectType nextEffect = (EffectType)((state.currentEffect + 1) % 4);
    switchToEffect(nextEffect);
  }
}

void sendJSONUpdate() {
  unsigned long currentTime = millis();
  if (currentTime - state.lastJSONSend < JSON_SEND_INTERVAL) return;
  state.lastJSONSend = currentTime;
  
  StaticJsonDocument<400> doc;
  doc["left"] = int(state.distance1);
  doc["right"] = int(state.distance2);
  doc["hands_detected"] = state.handsDetected;
  doc["left_in_range"] = state.inRange1;
  doc["right_in_range"] = state.inRange2;
  doc["current_effect"] = state.currentEffect;
  doc["effect_name"] = effectNames[state.currentEffect];
  doc["audio_volume"] = state.audioVolume;
  doc["mozzi_active"] = state.mozziActive;
  doc["timer1_active"] = state.timer1Active;
  
  if (state.mozziActive) {
    doc["control_rate"] = state.currentControlRate;
  }
  
  serializeJson(doc, Serial);
  Serial.println();
}

// Required Mozzi functions (called when Mozzi is active)
void updateControl() {
  // This is called by Mozzi when active - we handle it in updateAudio()
}

AudioOutput_t updateAudio() {
  // Called by Mozzi for audio generation
  switch (state.currentEffect) {
    case EFFECT_THEREMIN:
      return updateThereminAudio();
    case EFFECT_ROBOTS:
      return updateRobotsAudio();
    default:
      return 0;
  }
}

void loop() {
  // Core sensor reading
  readSensors();
  
  // Update audio (effect-specific)
  updateAudioControl();
  
  // Update LEDs
  updateLEDs();
  
  // Check for effect rotation
  checkEffectRotation();
  
  // Send JSON updates
  sendJSONUpdate();
  
  // Handle Mozzi audio hook if active
  if (state.mozziActive) {
    audioHook();
  }
  
  // Adaptive timing based on active audio system
  if (state.mozziActive) {
    delay(8);  // ~125Hz for Mozzi compatibility
  } else {
    delay(10); // Standard 100Hz for Timer1 effects
  }
} 