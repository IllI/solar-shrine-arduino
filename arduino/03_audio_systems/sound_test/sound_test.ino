/****************************************************************************
 Sound Test - Effect Rotation
 Cycles between Alien, Robots, and Theremin effects every 2 seconds
 
 Effects included:
 - Alien: Vibrato sine wave effect
 - Robots: Robotic talking pattern
 - Theremin: Sensor-based theremin with C Minor Pentatonic scale
*********************************************************************************/

// Configure Mozzi for 2-pin PWM mode
#include <MozziConfigValues.h>
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_2PIN_PWM

#include <MozziGuts.h>
#include <Oscil.h>
#include <LowPassFilter.h>
#include <RollingAverage.h>
#include <tables/sin2048_int8.h>
#include <tables/cos2048_int8.h>
#include <tables/triangle_valve_2048_int8.h>

#define CONTROL_RATE 128

// Effect timing
unsigned long lastEffectChange = 0;
const unsigned long EFFECT_DURATION = 2000; // 2 seconds per effect
int currentEffect = 0; // 0=Alien, 1=Robots, 2=Theremin

// === ALIEN EFFECT ===
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> alienOsc(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, CONTROL_RATE> vibrato(SIN2048_DATA);
float vibratoRate = 6.5f;

// === ROBOTS EFFECT ===
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> robotOsc(SIN2048_DATA);
int robotPitchPattern[] = {220, 330, 440, 330, 220, 165, 220, 330};
int robotPatternIndex = 0;
unsigned long lastRobotChange = 0;
const unsigned long ROBOT_NOTE_DURATION = 200;

// === THEREMIN EFFECT ===
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

void setup() {
  Serial.begin(9600);
  Serial.println("Sound Test - Effect Rotation");
  Serial.println("Cycling: Alien -> Robots -> Theremin");
  
  // Initialize alien effect
  alienOsc.setFreq(440);
  vibrato.setFreq(vibratoRate);
  
  // Initialize robot effect
  robotOsc.setFreq(robotPitchPattern[0]);
  
  // Initialize theremin sensor pins
  pinMode(pitchOut, OUTPUT);
  pinMode(pitchIn, INPUT);
  pinMode(volOut, OUTPUT);
  pinMode(volIn, INPUT);
  digitalWrite(pitchOut, LOW);
  digitalWrite(volOut, LOW);
  
  // Initialize theremin oscillator
  thereminOsc.setFreq(220);
  
  startMozzi(CONTROL_RATE);
}

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

void updateControl() {
  // Check if it's time to switch effects
  if (millis() - lastEffectChange >= EFFECT_DURATION) {
    currentEffect = (currentEffect + 1) % 3;
    lastEffectChange = millis();
    
    switch (currentEffect) {
      case 0:
        Serial.println("Effect: Alien");
        break;
      case 1:
        Serial.println("Effect: Robots Talking");
        break;
      case 2:
        Serial.println("Effect: Theremin");
        break;
    }
  }
  
  // Update current effect
  switch (currentEffect) {
    case 0: // Alien effect
      updateAlien();
      break;
    case 1: // Robots effect
      updateRobots();
      break;
    case 2: // Theremin effect
      updateTheremin();
      break;
  }
}

void updateAlien() {
  float vibrato_val = (float)vibrato.next() / 128.0f;
  float freq = 440.0f + (vibrato_val * 50.0f);
  alienOsc.setFreq(freq);
}

void updateRobots() {
  if (millis() - lastRobotChange >= ROBOT_NOTE_DURATION) {
    robotPatternIndex = (robotPatternIndex + 1) % 8;
    robotOsc.setFreq(robotPitchPattern[robotPatternIndex]);
    lastRobotChange = millis();
  }
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

int updateAudio() {
  switch (currentEffect) {
    case 0: // Alien effect
      return audioOutputAlien();
    case 1: // Robots effect
      return audioOutputRobots();
    case 2: // Theremin effect
      return audioOutputTheremin();
    default:
      return 0;
  }
}

int audioOutputAlien() {
  return alienOsc.next();
}

int audioOutputRobots() {
  return robotOsc.next();
}

int audioOutputTheremin() {
  int sample = (thereminOsc.next() * smoothVol) >> 8;
  return sample;
}

void loop() {
  audioHook();
}