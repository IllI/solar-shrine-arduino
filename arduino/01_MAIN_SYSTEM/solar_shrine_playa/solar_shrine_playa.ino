/****************************************************************************
 Solar Shrine Playa - Modular Audio Effect System
 Cycles between DJ Scratch, Alien, Robots, and Theremin effects every 5 seconds
 
 Effects included:
 - DJ Scratch: Vibrato sine wave effect (simulated)
 - Alien: Vibrato sine wave effect
 - Robots: Robotic talking pattern
 - Theremin: Sensor-based theremin with C Minor Pentatonic scale
*********************************************************************************/

#include <avr/pgmspace.h>
#include "audio_data.h"

// Mozzi configuration - MUST be before MozziGuts.h
#include <MozziConfigValues.h>
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_2PIN_PWM

// Custom pin configuration to free up pin 11 for sensor
// Default: pin 11 (high) + pin 12 (low)
// Custom:  pin 12 (high) + pin 13 (low)
#define MOZZI_AUDIO_PIN_1 12
#define MOZZI_AUDIO_PIN_1_REGISTER OCR1B
#define MOZZI_AUDIO_PIN_1_LOW 13
#define MOZZI_AUDIO_PIN_1_LOW_REGISTER OCR1C

#include <MozziGuts.h>
#include <Oscil.h>
#include <LowPassFilter.h>
#include <RollingAverage.h>
#include <tables/sin2048_int8.h>
#include <tables/cos2048_int8.h>
#include <tables/triangle2048_int8.h>
#include <tables/triangle_valve_2048_int8.h>
#include <tables/saw2048_int8.h>

#define CONTROL_RATE 128

// =============================================================================
// AUDIO MODE SYSTEM
// =============================================================================
enum AudioMode {
  MODE_DJ_SCRATCH = 0,
  MODE_MOZZI_ALIEN = 1,
  MODE_MOZZI_ROBOTS = 2,
  MODE_MOZZI_THEREMIN = 3
};

AudioMode currentMode = MODE_DJ_SCRATCH;
const unsigned long MODE_DURATION = 5000; // 5 seconds per mode
unsigned long lastModeChange = 0;

// =============================================================================
// DJ SCRATCH SYSTEM (PROGMEM + Timer1)
// =============================================================================
volatile int32_t sampleIndex = 0;
volatile uint8_t sampleCounter = 0;
volatile uint8_t playState = 0;
volatile uint8_t playbackSpeed = 5;
volatile int8_t scratchSpeed = 1;
volatile bool isScratchMode = false;

// =============================================================================
// SENSOR SYSTEM
// =============================================================================
#define TRIG1 10  // Left sensor trigger
#define ECHO1 11  // Left sensor echo
#define TRIG2 5   // Right sensor trigger
#define ECHO2 6   // Right sensor echo

// Note: DJ Scratch effect variables are now in the modular audio system section above

// =============================================================================
// ALIEN EFFECT
// =============================================================================
// Alien effect variables (sophisticated theremin-style)
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> alienOsc(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> alienOscHarm(SIN2048_DATA);
Oscil <COS2048_NUM_CELLS, CONTROL_RATE> alienVibrato(COS2048_DATA);
RollingAverage <int, 4> alienPitchAvg;
RollingAverage <int, 8> alienVolAvg;
RollingAverage <int, 4> alienPitchVelAvg;
RollingAverage <int, 4> alienVolVelAvg;
int alienBaseFreq = 440;
int alienSmoothVol = 0;
int alienVol = 0;
float alienVibratoDepth = 0.03;
float alienVibratoRate = 5.5;
int alienHarmMix = 0;
int alienPrevPitchDur = 0;
int alienPrevVolDur = 0;
bool alienMuteOutput = false;
// Echo buffer for alien effect
#define ALIEN_ECHO_BUFFER_SIZE 256
int alienEchoBuffer[ALIEN_ECHO_BUFFER_SIZE];
int alienEchoIndex = 0;
float alienEchoMix = 0.25;
int alienEchoDelay = 128;
int alienLpfState = 0;
int alienLpfAlpha = 180;

// =============================================================================
// ROBOTS EFFECT
// =============================================================================
Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> robotOsc(SAW2048_DATA);
int robotPitchPattern[] = {220, 330, 440, 330, 220, 165, 220, 330};
int robotPatternIndex = 0;
unsigned long lastRobotChange = 0;
const unsigned long ROBOT_NOTE_DURATION = 200;
float robotVolume = 0.7f;

// =============================================================================
// THEREMIN EFFECT
// =============================================================================
Oscil<TRIANGLE_VALVE_2048_NUM_CELLS, AUDIO_RATE> thereminOsc(TRIANGLE_VALVE_2048_DATA);
RollingAverage<int, 4> pAverage;
RollingAverage<int, 8> vAverage;
int averaged;

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

// =============================================================================
// SENSOR READING FUNCTIONS
// =============================================================================
float readSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999; // No echo received
  
  float distance = duration * 0.034 / 2;
  return distance;
}

bool isHandPresent(float distance) {
  return (distance > 1 && distance < 20);
}

// =============================================================================
// SETUP FUNCTION
// =============================================================================
void setup() {
  Serial.begin(9600);
  Serial.println("Solar Shrine Playa - Modular Audio Effect System");
  Serial.println("Cycling: DJ Scratch -> Alien -> Robots -> Theremin");
  
  // Initialize sensor pins
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  
  // Effects will be initialized by their respective setup functions
  
  // Start in DJ Scratch mode
  setupDJScratch();
  lastModeChange = millis();
}

// =============================================================================
// DJ SCRATCH TIMER1 ISR
// =============================================================================
ISR(TIMER1_COMPB_vect) {
  if (currentMode != MODE_DJ_SCRATCH) {
    OCR1B = 200; // Silence when not in DJ mode
    return;
  }
  
  sampleCounter++;
  
  uint8_t currentSpeed = isScratchMode ? 2 : playbackSpeed;
  
  if (sampleCounter >= currentSpeed) {
    sampleCounter = 0;
    
    if (playState == 1) {
      if (sampleIndex < 0) sampleIndex = 0;
      if (sampleIndex >= AUDIO_SAMPLE_COUNT) sampleIndex = 0;
      
      uint8_t sample = pgm_read_byte(&audioData[sampleIndex]);
      int16_t amp = ((int16_t)sample - 128) * 4;
      amp = constrain(amp, -128, 127);
      
      OCR1B = ((uint32_t)(amp + 128) * ICR1) / 255;
      
      if (isScratchMode) {
        sampleIndex += scratchSpeed;
        if (sampleIndex < 0) sampleIndex = AUDIO_SAMPLE_COUNT - 1;
        if (sampleIndex >= AUDIO_SAMPLE_COUNT) sampleIndex = 0;
      } else {
        sampleIndex++;
        if (sampleIndex >= AUDIO_SAMPLE_COUNT) sampleIndex = 0;
      }
      
    } else {
      OCR1B = ICR1 / 2; // Silence
    }
  }
}

// =============================================================================
// MODE SWITCHING FUNCTIONS
// =============================================================================
void setupDJScratch() {
  Serial.println(F("Setting up DJ Scratch mode..."));
  
  // Configure Timer1 for PWM audio on pin 12 (OC1B)
  pinMode(12, OUTPUT);
  TCCR1A = _BV(COM1B1) | _BV(WGM11);  // Clear OC1B on compare match, Fast PWM
  TCCR1B = _BV(WGM13) | _BV(CS10);    // Fast PWM, no prescaler
  ICR1 = 399;                         // 20kHz frequency
  OCR1B = 200;                        // 50% duty cycle (silence)
  TIMSK1 = _BV(OCIE1B);               // Enable Timer1 Compare B interrupt
  
  // Reset DJ scratch state
  sampleIndex = 0;
  sampleCounter = 0;
  playState = 1;  // Start playing immediately
  playbackSpeed = 5;
  scratchSpeed = 1;
  isScratchMode = false;
  
  Serial.println(F("Mode: DJ Scratch (PROGMEM + Timer1)"));
}

void disableDJScratch() {
  TIMSK1 &= ~_BV(OCIE1B); // Disable Timer1 interrupt
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1B = 0;
  digitalWrite(12, LOW);
}

void setupMozziAlien() {
  // Initialize alien effect
  alienOsc.setFreq(440);
  alienOscHarm.setFreq(880);
  alienVibrato.setFreq(alienVibratoRate);
  alienVol = 0;
  alienSmoothVol = 0;
  alienBaseFreq = 440;
  
  // Initialize alien echo buffer to silence
  for (int i = 0; i < ALIEN_ECHO_BUFFER_SIZE; i++) {
    alienEchoBuffer[i] = 0;
  }
  
  startMozzi(CONTROL_RATE);
  Serial.println("Mode: Mozzi Alien");
}

void disableMozziAlien() {
  stopMozzi();
}

// =============================================================================
// THEREMIN HELPER FUNCTION
// =============================================================================
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

// =============================================================================
// MAIN LOOP FUNCTION
// =============================================================================
void loop() {
  // Check if it's time to switch modes
  if (millis() - lastModeChange >= MODE_DURATION) {
    switchToNextMode();
  }
  
  // CRITICAL: audioHook() must be called EVERY loop iteration for Mozzi
  // This is the key difference from our previous implementation
  if (currentMode == MODE_MOZZI_ALIEN || currentMode == MODE_MOZZI_ROBOTS || currentMode == MODE_MOZZI_THEREMIN) {
    audioHook();
  } else if (currentMode == MODE_DJ_SCRATCH) {
    // Handle DJ scratch controls (Mozzi handles its own controls in updateControl)
    static unsigned long lastSensorRead = 0;
    if (millis() - lastSensorRead >= 30) {  // Read sensors every 30ms for DJ scratch
      float d1 = readSensor(TRIG1, ECHO1);
      float d2 = readSensor(TRIG2, ECHO2);
      
      bool leftHand = isHandPresent(d1);
      bool rightHand = isHandPresent(d2);
      
      updateDJScratch(leftHand, rightHand, d1, d2);
      lastSensorRead = millis();
    }
    
    delay(5);  // Small delay for DJ scratch mode
  }
}

void switchToNextMode() {
  // Disable current mode
  switch (currentMode) {
    case MODE_DJ_SCRATCH:
      disableDJScratch();
      break;
    case MODE_MOZZI_ALIEN:
    case MODE_MOZZI_ROBOTS:
    case MODE_MOZZI_THEREMIN:
      disableMozziAlien();
      break;
  }
  
  // Switch to next mode
  currentMode = (AudioMode)((currentMode + 1) % 4);
  lastModeChange = millis();
  
  // Enable new mode
  switch (currentMode) {
    case MODE_DJ_SCRATCH:
      setupDJScratch();
      break;
    case MODE_MOZZI_ALIEN:
      setupMozziAlien();
      break;
    case MODE_MOZZI_ROBOTS:
      setupMozziAlien(); // Same setup as alien
      Serial.println("Mode: Mozzi Robots");
      break;
    case MODE_MOZZI_THEREMIN:
      setupMozziAlien(); // Same setup as alien
      Serial.println("Mode: Mozzi Theremin");
      break;
  }
}

// =============================================================================
// MOZZI UPDATE CONTROL FUNCTION (Only called when in Mozzi modes)
// =============================================================================
void updateControl() {
  // Read sensors
  float d1 = readSensor(TRIG1, ECHO1); // Left hand
  float d2 = readSensor(TRIG2, ECHO2); // Right hand
  bool leftHand = isHandPresent(d1);
  bool rightHand = isHandPresent(d2);
  
  // Update current Mozzi effect
  switch (currentMode) {
    case MODE_MOZZI_ALIEN:
      updateAlien(leftHand, rightHand, d1, d2);
      break;
    case MODE_MOZZI_ROBOTS:
      updateRobots();
      break;
    case MODE_MOZZI_THEREMIN:
      updateTheremin();
      break;
    default:
      // Should not reach here in Mozzi modes
      break;
  }
}

// =============================================================================
// INDIVIDUAL EFFECT UPDATE FUNCTIONS
// =============================================================================
void updateDJScratch(bool leftHand, bool rightHand, float d1, float d2) {
  // Left hand controls play/stop
  if (leftHand) {
    if (playState == 0) {
      playState = 1; // Start playing
      Serial.println("DJ: Play");
    }
  } else {
    if (playState == 1) {
      playState = 0; // Stop playing
      Serial.println("DJ: Stop");
    }
  }
  
  // Right hand controls scratch/speed
  if (rightHand) {
    // Map distance to scratch parameters
    int mappedDistance = map(d2 * 10, 20, 200, 0, 100);
    mappedDistance = constrain(mappedDistance, 0, 100);
    
    // Detect rapid transitions for scratching
    static int lastMappedDistance = 50;
    int distanceChange = abs(mappedDistance - lastMappedDistance);
    
    if (distanceChange > 20) {
      // Rapid movement = scratch mode
      isScratchMode = true;
      scratchSpeed = map(mappedDistance, 0, 100, -3, 3);
      if (scratchSpeed == 0) scratchSpeed = 1;
      Serial.print("DJ: Scratch Speed: ");
      Serial.println(scratchSpeed);
    } else {
      // Slow movement = speed control
      isScratchMode = false;
      playbackSpeed = map(mappedDistance, 0, 100, 10, 1);
      playbackSpeed = constrain(playbackSpeed, 1, 10);
    }
    
    lastMappedDistance = mappedDistance;
  } else {
    // No right hand = normal playback
    isScratchMode = false;
    playbackSpeed = 5; // Normal speed
  }
}

void updateAlien(bool leftHand, bool rightHand, float d1, float d2) {
  // Sophisticated alien theremin effect based on alien_sound_effect.ino
  int dur = d2 * 58.8;  // Convert distance to microseconds (right sensor for pitch)
  int vDur = d1 * 58.8; // Convert distance to microseconds (left sensor for volume)
  int freq;
  int targetFreq;
  long pitchDistance;
  long volDistance;
  int jitter;
  
  // Sensor thresholds (in microseconds)
  const int pitchLowThreshold = 800;
  const int pitchHighThreshold = 30;
  const int volLowThreshold = 600;
  const int volHighThreshold = 30;
  const int lowestFreq = 131;
  const int highestFreq = 1046;
  
  // === PITCH CONTROL (Right hand) ===
  if (dur < 5) {
    freq = highestFreq;
    pitchDistance = pitchHighThreshold;
  } else {
    pitchDistance = dur / 6;
    if (pitchDistance >= pitchLowThreshold) pitchDistance = pitchLowThreshold;
    if (pitchDistance < pitchHighThreshold) pitchDistance = pitchHighThreshold;
    freq = map(pitchDistance, pitchHighThreshold, pitchLowThreshold, highestFreq, lowestFreq);
  }
  
  // Add slight random jitter for organic sound
  jitter = random(-5, 5);
  int averaged = alienPitchAvg.next(freq);
  alienBaseFreq = averaged + jitter;
  
  // === VOLUME CONTROL (Left hand) ===
  if (vDur < 5) {
    alienVol = alienVol - 4;
    if (alienVol < 0) alienVol = 0;
    volDistance = volLowThreshold;
  } else {
    alienVol = alienVol + 4;
    if (alienVol > 255) alienVol = 255;
    volDistance = vDur / 6;
    if (volDistance > volLowThreshold) volDistance = volLowThreshold;
    if (volDistance < volHighThreshold) volDistance = volHighThreshold;
  }
  
  // Check if hands are present
  bool rightPresent = (dur >= 5);
  bool leftPresent = (vDur >= 5);
  if (!rightPresent && !leftPresent) {
    alienSmoothVol = 0;
    alienMuteOutput = true;
    alienEchoMix = 0.0f;
    return;
  } else {
    alienMuteOutput = false;
  }
  
  // Map distance to volume with logarithmic curve
  int linearVol = map(volDistance, volHighThreshold, volLowThreshold, 255, 0);
  
  // Right-hand solo fallback
  if (!leftPresent && rightPresent) {
    int pdClamped = (int)constrain(pitchDistance, pitchHighThreshold, pitchLowThreshold);
    int soloLinearVol = map(pdClamped, pitchHighThreshold, pitchLowThreshold, 200, 60);
    if (soloLinearVol > linearVol) linearVol = soloLinearVol;
  }
  
  // Apply logarithmic volume curve
  float normalizedVol = linearVol / 255.0;
  float logVol = normalizedVol * normalizedVol;
  int mappedVol = (int)(logVol * 255);
  alienSmoothVol = alienVolAvg.next(mappedVol);
  
  // === Motion reactivity ===
  int pitchVel = abs(dur - alienPrevPitchDur);
  int volVel = abs(vDur - alienPrevVolDur);
  int pitchVelSm = alienPitchVelAvg.next(pitchVel);
  int volVelSm = alienVolVelAvg.next(volVel);
  alienPrevPitchDur = dur;
  alienPrevVolDur = vDur;
  
  float pitchVelNorm = min(1.0f, pitchVelSm / 400.0f);
  float volVelNorm = min(1.0f, volVelSm / 400.0f);
  
  // Dynamic vibrato
  float baselineDepth = 0.01f + (normalizedVol * 0.07f);
  float motionDepth = pitchVelNorm * 0.03f;
  alienVibratoDepth = baselineDepth + motionDepth;
  if (alienVibratoDepth > 0.15f) alienVibratoDepth = 0.15f;
  
  alienVibratoRate = 4.0f + (volVelNorm * 6.0f);
  alienVibrato.setFreq(alienVibratoRate);
  
  // Apply vibrato
  float vibratoAmount = alienVibrato.next() * alienVibratoDepth;
  int modulatedFreq = alienBaseFreq + (int)(alienBaseFreq * vibratoAmount);
  if (modulatedFreq < lowestFreq) modulatedFreq = lowestFreq;
  if (modulatedFreq > highestFreq) modulatedFreq = highestFreq;
  alienOsc.setFreq(modulatedFreq);
  
  // Reactive harmonic content
  alienHarmMix = (int)(pitchVelNorm * 160.0f);
  int harmFreq = modulatedFreq * 2;
  if (harmFreq > 3000) harmFreq = 3000;
  alienOscHarm.setFreq(harmFreq);
  
  // Gesture-controlled echo
  alienEchoMix = 0.10f + (normalizedVol * 0.35f);
  int minDelay = 80;
  int maxDelay = ALIEN_ECHO_BUFFER_SIZE - 1;
  int pd = (int)constrain(pitchDistance, pitchHighThreshold, pitchLowThreshold);
  alienEchoDelay = map(pd, pitchHighThreshold, pitchLowThreshold, minDelay, maxDelay);
  
  // Map right-hand distance to tone brightness
  int minAlpha = 40;
  int maxAlpha = 230;
  alienLpfAlpha = map(pd, pitchHighThreshold, pitchLowThreshold, maxAlpha, minAlpha);
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
  
  // Pitch control (right hand sensor)
  digitalWrite(TRIG2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG2, LOW);
  
  dur = pulseIn(ECHO2, HIGH, pitchTimeOut);
  
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
  
  // Volume control (left hand sensor)
  digitalWrite(TRIG1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1, LOW);
  
  vDur = pulseIn(ECHO1, HIGH, volTimeOut);
  
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

// =============================================================================
// MOZZI AUDIO OUTPUT FUNCTION (Only for Mozzi modes)
// =============================================================================
int updateAudio() {
  return audioOutput();
}

int audioOutput() {
  switch (currentMode) {
    case MODE_MOZZI_ALIEN:
      return audioOutputAlien();
    case MODE_MOZZI_ROBOTS:
      return audioOutputRobots();
    case MODE_MOZZI_THEREMIN:
      return audioOutputTheremin();
    default:
      return 0; // DJ Scratch uses Timer1 ISR, not Mozzi
  }
}

// =============================================================================
// INDIVIDUAL AUDIO OUTPUT FUNCTIONS
// =============================================================================

int audioOutputAlien() {
  // Hard mute when no hands present
  if (alienMuteOutput) {
    alienLpfState = 0; // reset filter memory
    return 0;
  }
  
  // Generate base and harmonic samples
  int baseSample = (alienOsc.next() * alienSmoothVol) >> 8;
  int harmSample = (alienOscHarm.next() * alienSmoothVol) >> 8;
  
  // Blend harmonic content based on motion
  int combined = baseSample + ((harmSample * alienHarmMix) >> 8);
  
  // Right-hand tone control: simple low-pass filter (closer = brighter)
  alienLpfState = alienLpfState + (((combined - alienLpfState) * alienLpfAlpha) >> 8);
  int filtered = alienLpfState;
  
  // Echo effect
  int echoPos = (alienEchoIndex - alienEchoDelay + ALIEN_ECHO_BUFFER_SIZE) % ALIEN_ECHO_BUFFER_SIZE;
  int echoSample = alienEchoBuffer[echoPos];
  int mixedSample = filtered + (int)(echoSample * alienEchoMix);
  
  // Store current filtered sample in echo buffer
  alienEchoBuffer[alienEchoIndex] = filtered;
  alienEchoIndex++;
  if (alienEchoIndex >= ALIEN_ECHO_BUFFER_SIZE) alienEchoIndex = 0;
  
  // Clip to prevent distortion
  if (mixedSample > 127) mixedSample = 127;
  if (mixedSample < -128) mixedSample = -128;
  
  return mixedSample;
}

int audioOutputRobots() {
  return (robotOsc.next() * robotVolume);
}

int audioOutputTheremin() {
  int sample = (thereminOsc.next() * smoothVol) >> 8;
  return sample;
}
