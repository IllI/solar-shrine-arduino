/****************************************************************************     
 MiniMin Theremin Test - Classic Theremin Sound with Vibrato
 Based on techno-womble's proven design, adapted for Solar Shrine hardware
 
 Original: Created 15 Dec 2019, Modified 23 Jan 2020, V1.2
 By John Potter (techno-womble)
 
 ENHANCEMENTS FOR CLASSIC THEREMIN SOUND:
 - Warm sine wave oscillator (instead of tinny triangle)
 - Classic theremin vibrato (5.5Hz wobble)
 - Logarithmic volume response (more natural)
 - Increased sensor sensitivity 
 - Debug output for tuning
 - C Minor Pentatonic scale
 
 SOUND CHARACTERISTICS:
 - Warm, organic theremin tone
 - Natural vibrato modulation  
 - Smooth volume transitions
 - Professional theremin feel
 
 TUNING TIPS:
 - Adjust vibratoRate (Hz) for faster/slower wobble
 - Adjust vibratoDepth (0.0-0.1) for more/less vibrato
 - Modify threshold values if sensitivity needs tweaking
 - Watch Serial Monitor for real-time sensor feedback
 
 Hardware:
 - 2x HC-SR04 sensors on pins 5,6,10,11 
 - Audio output on pin 9 (Mozzi default)
*********************************************************************************/

//   download Mozzi library from https://sensorium.github.io/Mozzi/ 

// Configure Mozzi for 2-pin PWM mode on Arduino Mega (CRITICAL FOR AUDIO)
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_2PIN_PWM

#include <MozziGuts.h>
#include <Oscil.h> 
#include <RollingAverage.h>
#include <tables/sin2048_int8.h>                          // warmer sine wave for theremin sound
#include <tables/cos2048_int8.h>                          // for vibrato LFO

// Main oscillator - sine wave for warm theremin sound
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> osc(SIN2048_DATA);

// Reactive harmonic oscillator (adds brightness on quick movements)
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> oscHarm(SIN2048_DATA);

// Vibrato LFO for classic theremin wobble
Oscil <COS2048_NUM_CELLS, CONTROL_RATE> vibrato(COS2048_DATA);

#define CONTROL_RATE 128         

const boolean stepMode = false;  // Set to true for note-snapping mode



// C Minor Pentatonic scale - sounds beautiful and forgiving
int notes[] = {131,131,131,156,156,175,175,196,196,196,233,233,       // C3 octave
               262,262,262,311,311,349,349,392,392,392,466,466,       // C4 octave  
               523,523,523,622,622,698,698,783,783,783,932,932,       // C5 octave
               1046};                                                  // C6

RollingAverage <int, 4> pAverage;             // Pitch averaging
RollingAverage <int, 8> vAverage;             // Volume averaging
int averaged;

// Motion reactivity
RollingAverage <int, 4> pitchVelAvg;          // Smoothed right-hand movement
RollingAverage <int, 4> volVelAvg;            // Smoothed left-hand movement
int prevPitchDur = 0;
int prevVolDur = 0;
int harmMix = 0;                               // 0-255, how much harmonic to blend

// Theremin-style vibrato settings
float vibratoDepth = 0.03;                   // 3% frequency modulation  
float vibratoRate = 5.5;                     // 5.5 Hz vibrato rate
int baseFreq = 0;                            // Base frequency before vibrato

// Pin assignments for Solar Shrine Custom Shield with Mozzi on Mega
// Pin assignments for Solar Shrine Custom Shield with Mozzi on Mega
const int pitchOut = 10;                      // Left Sensor Trig
const int pitchIn = 11;                       // Left Sensor Echo
const int volOut = 5;                         // Right Sensor Trig
const int volIn = 6;                          // Right Sensor Echo

// Sensor thresholds (in microseconds, not cm) - INCREASED for better sensitivity
const int pitchLowThreshold = 800;            // Farthest pitch distance (was 450)
const int pitchHighThreshold = 30;            // Closest pitch distance (was 50)
const int volLowThreshold = 600;              // Farthest volume distance (was 200)
const int volHighThreshold = 30;              // Closest volume distance (was 50)

// Frequency range
const int lowestFreq = 131;                   // C3 in Hz
const int highestFreq = 1046;                 // C6 in Hz

// Timeout calculations
int pitchTimeOut = pitchLowThreshold * 8;
int volTimeOut = volLowThreshold * 8;
int smoothVol;
int vol = 0;
const bool allowRightHandSolo = true;          // when true, right hand alone produces a low-level sound

// Hard mute when neither hand is present
volatile bool muteOutput = false;

// Lightweight echo/delay (gesture-controlled)
#define ECHO_BUFFER_SIZE 256
int echoBuffer[ECHO_BUFFER_SIZE];
int echoIndex = 0;
float echoMix = 0.25;                         // 0.0 - 1.0
int echoDelay = 128;                          // in samples (0..ECHO_BUFFER_SIZE-1)

// Right-hand-controlled tone (simple low-pass filter)
int lpfState = 0;                              // filter memory/sample history
int lpfAlpha = 180;                            // 0..255, higher = brighter

void setup(){
  // Initialize pitch sensor pins (right sensor)
  pinMode(pitchOut, OUTPUT);                   
  pinMode(pitchIn, INPUT);
  digitalWrite(pitchOut, LOW);

  // Initialize volume sensor pins (left sensor)  
  pinMode(volOut, OUTPUT);                   
  pinMode(volIn, INPUT);
  digitalWrite(volOut, LOW);
  
  // Initialize echo buffer to silence
  for (int i = 0; i < ECHO_BUFFER_SIZE; i++) {
    echoBuffer[i] = 0;
  }
  
  Serial.begin(9600);
  Serial.println("MiniMin Theremin Test - Enhanced with Echo!");
  Serial.println("Right hand = Pitch, Left hand = Volume");
  Serial.println("Cover sensors with hands to play");
  Serial.println("Sensor readings will be shown below:");
  Serial.println("PitchTime | VolTime | Frequency | Volume");
  
  startMozzi(CONTROL_RATE);                   // Start Mozzi audio
  osc.setFreq(220);                           // Set initial frequency (A3)
  vibrato.setFreq(vibratoRate);               // Set vibrato LFO rate
  oscHarm.setFreq(440);
}

// Find nearest note in scale (for stepMode)
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
  
void updateControl(){
  int dur;
  int vDur;
  int freq;
  int targetFreq;
  long pitchDistance;
  long volDistance;
  int jitter;

  // === PITCH CONTROL (Right hand) ===
  digitalWrite(pitchOut, HIGH);
  delayMicroseconds(10);
  digitalWrite(pitchOut, LOW);

  dur = pulseIn(pitchIn, HIGH, pitchTimeOut);
  
  if (dur < 5) {
    // No echo detected - hand very close or no hand
    freq = highestFreq;  // Very close = high frequency
    pitchDistance = pitchHighThreshold; // treat as closest distance
  } else {
    // Echo detected - map distance to frequency
    pitchDistance = dur / 6;  // Convert to approximate distance
    if (pitchDistance >= pitchLowThreshold) pitchDistance = pitchLowThreshold;
    if (pitchDistance < pitchHighThreshold) pitchDistance = pitchHighThreshold;
    freq = map(pitchDistance, pitchHighThreshold, pitchLowThreshold, highestFreq, lowestFreq);
  }   
  
  // Add slight random jitter for organic sound
  jitter = random(-5, 5); 
  
  if (stepMode == true) {
    // Snap to nearest note in scale
    targetFreq = pAverage.next(notes[nearest(freq)]);
    baseFreq = targetFreq + jitter;
  } else {
    // Smooth continuous frequency
    averaged = pAverage.next(freq); 
    baseFreq = averaged + jitter;
  }
  
  // Defer vibrato application until after volume sensor processing

  // === VOLUME CONTROL (Left hand) ===
  digitalWrite(volOut, HIGH);
  delayMicroseconds(10);
  digitalWrite(volOut, LOW);

  vDur = pulseIn(volIn, HIGH, volTimeOut);
  
  if (vDur < 5) {
    // No echo detected - fade out
    vol = vol - 4;                             
    if (vol < 0) vol = 0;
    volDistance = volLowThreshold;  // Treat as far distance
  } else {
    // Echo detected - fade in and map distance to volume
    vol = vol + 4;                            
    if (vol > 255) vol = 255;
    volDistance = vDur / 6;
    if (volDistance > volLowThreshold) volDistance = volLowThreshold;
    if (volDistance < volHighThreshold) volDistance = volHighThreshold;
  }
  
  // If neither hand is present, hard-mute output and skip processing
  bool rightPresent = (dur >= 5);
  bool leftPresent = (vDur >= 5);
  if (!rightPresent && !leftPresent) {
    smoothVol = 0;
    muteOutput = true;
    echoMix = 0.0f;              // ensure no residual echo
    return;
  } else {
    muteOutput = false;
  }

  // Map distance to volume with logarithmic curve for natural sound
  int linearVol = map(volDistance, volHighThreshold, volLowThreshold, 255, 0);
  
  // Right-hand solo fallback: if left hand not detected, let right hand set a quiet floor volume
  if (allowRightHandSolo && !leftPresent && rightPresent) {
    int pdClamped = (int)constrain(pitchDistance, pitchHighThreshold, pitchLowThreshold);
    // closer = louder floor, farther = quieter floor
    int soloLinearVol = map(pdClamped, pitchHighThreshold, pitchLowThreshold, 200, 60);
    if (soloLinearVol > linearVol) linearVol = soloLinearVol;
  }
  
  // Apply logarithmic volume curve (sounds more natural than linear)
  float normalizedVol = linearVol / 255.0;
  float logVol = normalizedVol * normalizedVol;  // Square for logarithmic feel
  int mappedVol = (int)(logVol * 255);
  
  smoothVol = vAverage.next(mappedVol);

  // === Interactivity: react to hand motion and position ===
  int pitchVel = abs(dur - prevPitchDur);
  int volVel = abs(vDur - prevVolDur);
  int pitchVelSm = pitchVelAvg.next(pitchVel);
  int volVelSm = volVelAvg.next(volVel);
  prevPitchDur = dur;
  prevVolDur = vDur;

  float pitchVelNorm = min(1.0f, pitchVelSm / 400.0f);  // tune divisor to taste
  float volVelNorm = min(1.0f, volVelSm / 400.0f);

  // Dynamic vibrato: closer left hand -> deeper vibrato; quick movements -> even deeper/faster
  float baselineDepth = 0.01f + (normalizedVol * 0.07f);
  float motionDepth = pitchVelNorm * 0.03f;
  vibratoDepth = baselineDepth + motionDepth;           // 0.01 .. ~0.11
  if (vibratoDepth > 0.15f) vibratoDepth = 0.15f;

  vibratoRate = 4.0f + (volVelNorm * 6.0f);             // 4 .. 10 Hz depending on movement
  vibrato.setFreq(vibratoRate);

  // Apply vibrato now that we have dynamic parameters
  float vibratoAmount = vibrato.next() * vibratoDepth;
  int modulatedFreq = baseFreq + (int)(baseFreq * vibratoAmount);
  if (modulatedFreq < lowestFreq) modulatedFreq = lowestFreq;
  if (modulatedFreq > highestFreq) modulatedFreq = highestFreq;
  osc.setFreq(modulatedFreq);

  // Reactive harmonic content: more on faster right-hand movement
  harmMix = (int)(pitchVelNorm * 160.0f);               // 0..160 (out of 255)
  int harmFreq = modulatedFreq * 2;
  if (harmFreq > 3000) harmFreq = 3000;                 // avoid aliasing
  oscHarm.setFreq(harmFreq);

  // Gesture-controlled echo: closer left hand -> more echo; farther right hand -> longer delay
  echoMix = 0.10f + (normalizedVol * 0.35f);            // 0.10 .. 0.45
  int minDelay = 80;
  int maxDelay = ECHO_BUFFER_SIZE - 1;
  int pd = (int)constrain(pitchDistance, pitchHighThreshold, pitchLowThreshold);
  echoDelay = map(pd, pitchHighThreshold, pitchLowThreshold, minDelay, maxDelay);
  // Map right-hand distance to tone brightness
  int minAlpha = 40;                           // darker
  int maxAlpha = 230;                          // brighter
  lpfAlpha = map(pd, pitchHighThreshold, pitchLowThreshold, maxAlpha, minAlpha);
  
  // Debug output - REDUCED frequency to avoid audio interference
  static int debugCounter = 0;
  debugCounter++;
  if (debugCounter > 50) {  // Print every 50 cycles (much less frequent)
    Serial.print(dur);
    Serial.print(" | ");
    Serial.print(vDur);
    Serial.print(" | ");
    Serial.print(averaged);
    Serial.print("Hz | Vol:");
    Serial.print(smoothVol);
    Serial.print(" | ");
    Serial.print("VibD:");
    Serial.print(vibratoDepth, 3);
    Serial.print(" VibR:");
    Serial.print(vibratoRate, 2);
    Serial.print(" Harm:");
    Serial.print(harmMix);
    Serial.print(" | ");
    if (dur < 5) Serial.print("P:CLOSE");
    else if (dur > pitchLowThreshold) Serial.print("P:FAR");
    else Serial.print("P:MID");
    Serial.print(" ");
    if (vDur < 5) Serial.print("V:CLOSE");
    else if (vDur > volLowThreshold) Serial.print("V:FAR");
    else Serial.print("V:MID");
    Serial.println();
    debugCounter = 0;
  }
}

 int updateAudio(){
  // Hard mute when no hands present
  if (muteOutput) {
    lpfState = 0;                // reset filter memory
    // don't advance echo buffer; also ensure no direct output
    return 0;
  }

  // Generate base and harmonic samples
  int baseSample = (osc.next() * smoothVol) >> 8;
  int harmSample = (oscHarm.next() * smoothVol) >> 8;

  // Blend harmonic content based on motion
  int combined = baseSample + ((harmSample * harmMix) >> 8);

  // Right-hand tone control: simple low-pass filter (closer = brighter)
  lpfState = lpfState + (((combined - lpfState) * lpfAlpha) >> 8);
  int filtered = lpfState;

  // Echo effect
  int echoPos = (echoIndex - echoDelay + ECHO_BUFFER_SIZE) % ECHO_BUFFER_SIZE;
  int echoSample = echoBuffer[echoPos];
  int mixedSample = filtered + (int)(echoSample * echoMix);

  // Store current filtered sample in echo buffer
  echoBuffer[echoIndex] = filtered;
  echoIndex++;
  if (echoIndex >= ECHO_BUFFER_SIZE) echoIndex = 0;

  // Clip to prevent distortion
  if (mixedSample > 127) mixedSample = 127;
  if (mixedSample < -128) mixedSample = -128;

  return mixedSample;
 }

void loop(){
  audioHook();                 // Required for Mozzi
}