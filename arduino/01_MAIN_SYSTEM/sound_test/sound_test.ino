/****************************************************************************
 * Sound Test: Alien, Robots, and Theremin - Sensor Reactive
 *
 * This sketch cycles between three sound effects every 5 seconds:
 * 1. ALIEN: Sensor-reactive vibrato with hand proximity control
 * 2. ROBOTS: Hand-controlled robotic speech patterns
 * 3. THEREMIN: Sensor-controlled theremin with C Minor Pentatonic scale
 *
 * Uses HC-SR04 sensors for hand detection and audio modulation.
 * Based on solar_shrine_modular.ino sensor logic.
****************************************************************************/

#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_2PIN_PWM
#include <MozziGuts.h>
#include <Oscil.h>
#include <RollingAverage.h>
#include <tables/sin2048_int8.h>
#include <tables/cos2048_int8.h>
#include <tables/triangle_valve_2048_int8.h>

// =============================================================================
// SENSOR CONFIGURATION (from solar_shrine_modular.ino)
// =============================================================================

// Pin assignments for HC-SR04 sensors
// Left sensor (Pitch)
const int trigPin1 = 10;
const int echoPin1 = 8;
// Right sensor (Volume)
const int trigPin2 = 5;
const int echoPin2 = 6;

// Distance-based detection constants (in cm)
const float MIN_RANGE = 5.0;
const float MAX_RANGE = 50.0;
const unsigned long SENSOR_TIMEOUT = 12000;

// Hand detection averaging
const int SAMPLES = 5;
float distance1Samples[SAMPLES];
float distance2Samples[SAMPLES];
int sampleIndex = 0;
bool samplesInitialized = false;
// Robot effect variables and functions moved inline to avoid header conflicts
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> robotOsc(SIN2048_DATA);
const int robotPitchPattern[] = {220, 330, 440, 550, 660, 770, 880, 990};
const int ROBOT_PITCH_PATTERN_LENGTH = sizeof(robotPitchPattern) / sizeof(int);
int currentRobotPitchIndex = 0;
unsigned long lastRobotPitchChangeTime = 0;
const unsigned long ROBOT_NOTE_DURATION = 100;
unsigned long lastRobotChange = 0;
int robotPatternIndex = 0;

// Sensor state variables
float avgDistance1 = MAX_RANGE + 10;
float avgDistance2 = MAX_RANGE + 10;
bool inRange1 = false;
bool inRange2 = false;
bool handsDetected = false;

void updateRobots() {
  // Cycle through the pitch pattern every 150ms to create the "talking" effect
  if (millis() - lastRobotPitchChangeTime > 150) {
    lastRobotPitchChangeTime = millis();
    currentRobotPitchIndex = (currentRobotPitchIndex + 1) % ROBOT_PITCH_PATTERN_LENGTH;
    robotOsc.setFreq(robotPitchPattern[currentRobotPitchIndex]);
  }
}

int audioOutputRobots() {
  return robotOsc.next();
}

// --- Variables for ALIEN effect ---
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> osc(SIN2048_DATA);
Oscil <COS2048_NUM_CELLS, CONTROL_RATE> vibrato(COS2048_DATA);
float vibratoDepth = 0.03;
float vibratoRate = 5.5;
int alien_vol = 0;

#define CONTROL_RATE 128

// --- Variables for THEREMIN effect ---
Oscil<TRIANGLE_VALVE_2048_NUM_CELLS, AUDIO_RATE> thereminOsc(TRIANGLE_VALVE_2048_DATA);
RollingAverage<int, 4> pAverage;
RollingAverage<int, 8> vAverage;
int averaged;

// Theremin sensor pins
const int volOut = 5;
const int volIn = 6;
const int pitchOut = 10;
const int pitchIn = 7;

// Theremin thresholds
const int pitchLowThreshold = 800;
const int pitchHighThreshold = 30;
const int volLowThreshold = 600;
const int volHighThreshold = 30;

// Theremin frequency range
const int lowestFreq = 131;
const int highestFreq = 1046;

// C Minor Pentatonic scale
int notes[] = {131,131,131,156,156,175,175,196,196,196,233,233,
               262,262,262,311,311,349,349,392,392,392,466,466,
               523,523,523,622,622,698,698,783,783,783,932,932,
               1046};

int pitchTimeOut = pitchLowThreshold * 8;
int volTimeOut = volLowThreshold * 8;
int smoothVol = 0;
int vol = 0;

// Find nearest note in scale for theremin
int nearest(int x) {
  int idx = 0;
  int distance = abs(notes[idx] - x);
  for (int i = 1; i < 37; i++) {
    int d = abs(notes[i] - x);
    if (d < distance) {
      idx = i;
      distance = d;
    }
  }
  return idx;
}

void updateTheremin() {
  int dur, vDur, freq;
  long distance;
  int jitter;
  
  // Pitch control (right hand)
  digitalWrite(pitchOut, HIGH);
  delayMicroseconds(10);
  digitalWrite(pitchOut, LOW);
  
  dur = pulseIn(pitchIn, HIGH, pitchTimeOut);
  
  if (dur < 5) {
    freq = highestFreq;
  } else {
    distance = dur / 6;
    if (distance >= pitchLowThreshold) distance = pitchLowThreshold;
    if (distance < pitchHighThreshold) distance = pitchHighThreshold;
    freq = map(distance, pitchHighThreshold, pitchLowThreshold, highestFreq, lowestFreq);
  }
  
  jitter = random(-5, 5);
  averaged = pAverage.next(freq);
  thereminOsc.setFreq(averaged + jitter);
  
  // Volume control (left hand)
  digitalWrite(volOut, HIGH);
  delayMicroseconds(10);
  digitalWrite(volOut, LOW);
  
  vDur = pulseIn(volIn, HIGH, volTimeOut);
  
  if (vDur < 5) {
    vol = vol - 4;
    if (vol < 0) vol = 0;
    distance = volLowThreshold;
  } else {
    vol = vol + 4;
    if (vol > 255) vol = 255;
    distance = vDur / 6;
    if (distance > volLowThreshold) distance = volLowThreshold;
    if (distance < volHighThreshold) distance = volHighThreshold;
  }
  
  int mappedVol = map(distance, volHighThreshold, volLowThreshold, 255, 0);
  smoothVol = vAverage.next(mappedVol);
}

int audioOutputTheremin() {
  int sample = (thereminOsc.next() * smoothVol) >> 8;
  return sample;
}

// =============================================================================
// SENSOR-REACTIVE EFFECT FUNCTIONS
// =============================================================================

void updateAlienSensorReactive() {
  // Base frequency controlled by left hand (pitch sensor)
  int baseFreq = 440; // Default A4
  if (inRange1) {
    // Map distance to frequency: closer = higher pitch
    baseFreq = map((int)avgDistance1, MIN_RANGE, MAX_RANGE, 880, 220);
  }
  
  // Vibrato rate controlled by right hand (volume sensor)
  float currentVibratoRate = vibratoRate;
  if (inRange2) {
    // Map distance to vibrato rate: closer = faster vibrato
    currentVibratoRate = map((int)avgDistance2, MIN_RANGE, MAX_RANGE, 15.0, 2.0);
    vibrato.setFreq(currentVibratoRate);
  }
  
  // Apply vibrato
  int jitter = random(-5, 5);
  int vibrato_val = vibrato.next();
  int vibrato_freq = (int)((float)baseFreq * vibratoDepth * vibrato_val / 256);
  osc.setFreq(baseFreq + vibrato_freq + jitter);
  
  // Volume based on hand presence
  if (handsDetected) {
    alien_vol = 200;
  } else {
    alien_vol = 50; // Quiet when no hands
  }
}

void updateRobotsSensorReactive() {
  // Base robot pattern timing
  static unsigned long baseRobotInterval = ROBOT_NOTE_DURATION;
  
  // Speed controlled by right hand
  unsigned long currentInterval = baseRobotInterval;
  if (inRange2) {
    // Map distance to speed: closer = faster talking
    currentInterval = map((int)avgDistance2, MIN_RANGE, MAX_RANGE, 50, 300);
  }
  
  // Pitch range controlled by left hand
  if (millis() - lastRobotChange >= currentInterval) {
    lastRobotChange = millis();
    robotPatternIndex = (robotPatternIndex + 1) % 8;
    
    int basePitch = robotPitchPattern[robotPatternIndex];
    
    // Modify pitch based on left hand position
    if (inRange1) {
      // Map distance to pitch multiplier: closer = higher pitch
      float pitchMultiplier = map((int)avgDistance1, MIN_RANGE, MAX_RANGE, 2.0, 0.5);
      basePitch = (int)(basePitch * pitchMultiplier);
    }
    
    robotOsc.setFreq(basePitch);
  }
}

// --- Effect Switching Logic ---
enum Effect {
  ALIEN,
  ROBOTS,
  THEREMIN
};
Effect currentEffect = ALIEN;
unsigned long lastEffectChangeTime = 0;
const unsigned long effectSwitchInterval = 5000; // 5 seconds

// Robot effect is now inline - no separate class needed

void setup(){
  Serial.begin(9600);
  Serial.println("Sound Test: Alien, Robots, and Theremin - Sensor Reactive");
  
  // Initialize HC-SR04 sensor pins
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  
  // Initialize theremin sensor pins
  pinMode(pitchOut, OUTPUT);
  pinMode(pitchIn, INPUT);
  pinMode(volOut, OUTPUT);
  pinMode(volIn, INPUT);
  digitalWrite(pitchOut, LOW);
  digitalWrite(volOut, LOW);
  
  // Initialize sample arrays
  for (int i = 0; i < SAMPLES; i++) {
    distance1Samples[i] = MAX_RANGE + 10;
    distance2Samples[i] = MAX_RANGE + 10;
  }
  
  // Initialize theremin oscillator
  thereminOsc.setFreq(220);
  
  startMozzi(CONTROL_RATE);
  vibrato.setFreq(vibratoRate); // Set vibrato LFO rate for alien effect
}

void loop(){
  audioHook();
}

// =============================================================================
// SENSOR READING FUNCTIONS (from solar_shrine_modular.ino)
// =============================================================================

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  unsigned long duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT);
  
  if (duration == 0) {
    return MAX_RANGE + 10; // No echo = out of range
  }
  
  float distance = (duration * 0.034) / 2; // Convert to cm
  return distance;
}

float getAverageDistance(float samples[], float newSample) {
  samples[sampleIndex] = newSample;
  sampleIndex = (sampleIndex + 1) % SAMPLES;
  
  float sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += samples[i];
  }
  return sum / SAMPLES;
}

void updateSensors() {
  // Read raw distances
  float dist1 = readDistance(trigPin1, echoPin1);
  float dist2 = readDistance(trigPin2, echoPin2);
  
  // Apply rolling average
  avgDistance1 = getAverageDistance(distance1Samples, dist1);
  avgDistance2 = getAverageDistance(distance2Samples, dist2);
  
  // Update hand detection flags
  inRange1 = (avgDistance1 >= MIN_RANGE && avgDistance1 <= MAX_RANGE);
  inRange2 = (avgDistance2 >= MIN_RANGE && avgDistance2 <= MAX_RANGE);
  handsDetected = inRange1 || inRange2;
}

void updateControl(){
  // Update sensor readings
  updateSensors();
  
  // Check if it's time to switch effects
  if (millis() - lastEffectChangeTime > effectSwitchInterval) {
    lastEffectChangeTime = millis();
    if (currentEffect == ALIEN) {
      currentEffect = ROBOTS;
      Serial.println("Switching to ROBOTS");
    } else if (currentEffect == ROBOTS) {
      currentEffect = THEREMIN;
      Serial.println("Switching to THEREMIN");
    } else {
      currentEffect = ALIEN;
      Serial.println("Switching to ALIEN");
      // Re-initialize alien vibrato rate when switching back
      vibrato.setFreq(vibratoRate);
    }
  }

  // Update the control logic for the currently active effect
  switch (currentEffect) {
    case ALIEN:
      updateAlienSensorReactive();
      break;
    case ROBOTS:
      updateRobotsSensorReactive();
      break;
    case THEREMIN:
      updateTheremin();
      break;
  }
}

int updateAudio(){
  switch (currentEffect) {
    case ALIEN:
      {
        int out = osc.next();
        return (out * alien_vol) >> 8;
      }
    case ROBOTS:
      return audioOutputRobots();
    case THEREMIN:
      return audioOutputTheremin();
    default:
      return 0;
  }
}