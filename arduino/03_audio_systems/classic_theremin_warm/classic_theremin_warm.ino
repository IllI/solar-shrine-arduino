/****************************************************************************     
 Classic Theremin - Warm Sound Fix
 Based on working minimin_theremin_test.ino + sine wave for warmth
 
 FIXES APPLIED TO ELIMINATE "TINNY" SOUND:
 - Sine wave oscillator (research shows this eliminates harsh harmonics)
 - Enhanced sensor sensitivity (from working test version)
 - Logarithmic volume response (more natural)
 - Working vibrato settings (5.5Hz like test version) 
 - Musical frequency range with scale quantization
 
 Original MiniMin: Created 15 Dec 2019, Modified 23 Jan 2020, V1.2
 By John Potter (techno-womble)
 
 COMBINES:
 - Audio Physics: Sine waves = warm, Triangle waves = tinny  
 - Working sensor logic from minimin_theremin_test.ino
 - Classic theremin vibrato for musical expression
 
 Hardware:
 - 2x HC-SR04 sensors on pins 5,6,10,11 
 - Audio output on pin 9 (Mozzi default)
*********************************************************************************/

#include <MozziGuts.h>
#include <Oscil.h> 
#include <RollingAverage.h>
#include <tables/sin2048_int8.h>                          // Sine wave for warm sound
#include <tables/cos2048_int8.h>                          // For gentle vibrato

// Main oscillator - sine wave for warm theremin sound (research-based fix)
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> osc(SIN2048_DATA);

// Gentle vibrato for classic theremin wobble
Oscil <COS2048_NUM_CELLS, CONTROL_RATE> vibrato(COS2048_DATA);

#define CONTROL_RATE 128         

// Scale quantization mode (from original MiniMin design)
const boolean stepMode = false;  // Set to true for note-snapping mode

// Musical scales from original MiniMin project
// Uncomment the scale you want to use:

/*int notes[] = {131,139,147,156,165,175,185,196,208,220,233,246,       // chromatic scale
               262,277,294,311,330,349,370,392,415,440,466,494,
               523,554,587,622,659,698,739,783,830,880,932,988,
               1046}; */

/*int notes[] = {131,131,147,147,165,175,175,196,196,220,220,246,         // major scale
               262,262,294,294,330,349,349,392,392,440,440,494,
               523,523,587,587,659,698,698,783,783,880,880,988,
               1046}; */
               
/*int notes[] = {131,131,147,147,156,175,175,196,196,220,220,246,         // minor scale
               262,262,294,294,311,349,349,392,392,440,440,494,
               523,523,587,587,622,698,698,783,783,880,880,988,
               1046}; */

int notes[] = {131,131,131,156,156,175,175,196,196,196,233,233,       // C minor pentatonic (sounds great!)
               262,262,262,311,311,349,349,392,392,392,466,466,
               523,523,523,622,622,698,698,783,783,783,932,932,
               1046}; 

RollingAverage <int, 4> pAverage;             // Pitch averaging (from MiniMin)
RollingAverage <int, 8> vAverage;             // Volume averaging  
int averaged;

// Classic theremin vibrato settings (from working test version)
float vibratoDepth = 0.03;                   // 3% frequency modulation 
float vibratoRate = 5.5;                     // 5.5 Hz vibrato rate
int baseFreq = 0;                            // Base frequency before vibrato

// Pin assignments (your hardware)
const int volOut = 5;                         // Volume sensor trigger (left sensor)
const int volIn = 6;                          // Volume sensor echo (left sensor)
const int pitchOut = 10;                      // Pitch sensor trigger (right sensor) 
const int pitchIn = 11;                       // Pitch sensor echo (right sensor)

// Enhanced sensor thresholds (from working minimin_theremin_test)
const int pitchLowThreshold = 800;            // Farthest pitch distance (enhanced)
const int pitchHighThreshold = 30;            // Closest pitch distance (enhanced)
const int volLowThreshold = 600;              // Farthest volume distance (enhanced)
const int volHighThreshold = 30;              // Closest volume distance (enhanced)

// Frequency range (from MiniMin but optimized for warmth)
const int lowestFreq = 131;                   // C3 in Hz
const int highestFreq = 1046;                 // C6 in Hz

// Timeout calculations (from MiniMin)
int pitchTimeOut = pitchLowThreshold * 8;
int volTimeOut = volLowThreshold * 8;
int smoothVol;
int vol = 0;

void setup(){
  // Initialize sensor pins (MiniMin approach)
  pinMode(pitchOut, OUTPUT);                   
  pinMode(pitchIn, INPUT);
  digitalWrite(pitchOut, LOW);

  pinMode(volOut, OUTPUT);                   
  pinMode(volIn, INPUT);
  digitalWrite(volOut, LOW);
  
  Serial.begin(9600);
  Serial.println("Classic Theremin - Warm Sound (Sine Wave Fix)");
  Serial.println("Based on MiniMin proven design + audio research");
  Serial.println("Right hand = Pitch, Left hand = Volume");
  Serial.println("Scale: C Minor Pentatonic (change in code for other scales)");
  
  startMozzi(CONTROL_RATE);                   
  osc.setFreq(220);                           // Set initial frequency (A3)
  vibrato.setFreq(vibratoRate);               // Set classic vibrato rate
}

// Find nearest note in scale (from original MiniMin)
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
  long distance;
  int jitter;

  // === PITCH CONTROL (MiniMin proven approach) ===
  digitalWrite(pitchOut, HIGH);
  delayMicroseconds(10);
  digitalWrite(pitchOut, LOW);

  dur = pulseIn(pitchIn, HIGH, pitchTimeOut);
  
  if (dur < 5) {
    // No echo detected - hand very close or no hand
    freq = highestFreq;  // Very close = high frequency (like working version)
  } else {
    // Echo detected - map distance to frequency
    distance = dur / 6;  // Convert to approximate distance
    if (distance >= pitchLowThreshold) distance = pitchLowThreshold;
    if (distance < pitchHighThreshold) distance = pitchHighThreshold;
    freq = map(distance, pitchHighThreshold, pitchLowThreshold, highestFreq, lowestFreq);
  }   
  
  // Add slight random jitter for organic sound (from MiniMin)
  jitter = random(-5, 5); 
  
  if (stepMode == true) {
    // Snap to nearest note in scale (MiniMin feature)
    targetFreq = pAverage.next(notes[nearest(freq)]);
    baseFreq = targetFreq + jitter;
  } else {
    // Smooth continuous frequency
    averaged = pAverage.next(freq); 
    baseFreq = averaged + jitter;
  }
  
  // Apply gentle classic theremin vibrato (research-based warmth)
  float vibratoAmount = vibrato.next() * vibratoDepth;
  int modulatedFreq = baseFreq + (int)(baseFreq * vibratoAmount);
  osc.setFreq(modulatedFreq);

  // === VOLUME CONTROL (MiniMin proven approach) ===
  digitalWrite(volOut, HIGH);
  delayMicroseconds(10);
  digitalWrite(volOut, LOW);

  vDur = pulseIn(volIn, HIGH, volTimeOut);
  
  if (vDur < 5) {
    // No echo detected - fade out
    vol = vol - 4;                             
    if (vol < 0) vol = 0;
    distance = volLowThreshold;  // Treat as far distance
  } else {
    // Echo detected - fade in and map distance to volume
    vol = vol + 4;                            
    if (vol > 255) vol = 255;
    distance = vDur / 6;
    if (distance > volLowThreshold) distance = volLowThreshold;
    if (distance < volHighThreshold) distance = volHighThreshold;
  }
  
  // Map distance to volume with logarithmic curve for natural sound (from working version)
  int linearVol = map(distance, volHighThreshold, volLowThreshold, 255, 0);
  
  // Apply logarithmic volume curve (sounds more natural than linear)
  float normalizedVol = linearVol / 255.0;
  float logVol = normalizedVol * normalizedVol;  // Square for logarithmic feel
  int mappedVol = (int)(logVol * 255);
  
  smoothVol = vAverage.next(mappedVol);
  
  // Debug output - REDUCED frequency to avoid audio interference
  static int debugCounter = 0;
  debugCounter++;
  if (debugCounter > 50) {  // Print every 50 cycles (like working version)
    Serial.print(dur);
    Serial.print(" | ");
    Serial.print(vDur);
    Serial.print(" | ");
    Serial.print(averaged);
    Serial.print("Hz | Vol:");
    Serial.print(smoothVol);
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
  // Generate sine wave sample (research fix for "tinny" sound)
  return (osc.next() * smoothVol) >> 8;
}

void loop(){
  audioHook();                 // Required for Mozzi
} 