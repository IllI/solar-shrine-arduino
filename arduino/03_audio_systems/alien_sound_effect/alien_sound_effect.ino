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

void setup(){
  // Initialize pitch sensor pins (right sensor)
  pinMode(pitchOut, OUTPUT);                   
  pinMode(pitchIn, INPUT);
  digitalWrite(pitchOut, LOW);

  // Initialize volume sensor pins (left sensor)  
  pinMode(volOut, OUTPUT);                   
  pinMode(volIn, INPUT);
  digitalWrite(volOut, LOW);
  

  
  Serial.begin(9600);
  Serial.println("MiniMin Theremin Test - Enhanced with Echo!");
  Serial.println("Right hand = Pitch, Left hand = Volume");
  Serial.println("Cover sensors with hands to play");
  Serial.println("Sensor readings will be shown below:");
  Serial.println("PitchTime | VolTime | Frequency | Volume");
  
  startMozzi(CONTROL_RATE);                   // Start Mozzi audio
  osc.setFreq(220);                           // Set initial frequency (A3)
  vibrato.setFreq(vibratoRate);               // Set vibrato LFO rate
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
  long distance;
  int jitter;

  // === PITCH CONTROL (Right hand) ===
  digitalWrite(pitchOut, HIGH);
  delayMicroseconds(10);
  digitalWrite(pitchOut, LOW);

  dur = pulseIn(pitchIn, HIGH, pitchTimeOut);
  
  if (dur < 5) {
    // No echo detected - hand very close or no hand
    freq = highestFreq;  // Very close = high frequency
  } else {
    // Echo detected - map distance to frequency
    distance = dur / 6;  // Convert to approximate distance
    if (distance >= pitchLowThreshold) distance = pitchLowThreshold;
    if (distance < pitchHighThreshold) distance = pitchHighThreshold;
    freq = map(distance, pitchHighThreshold, pitchLowThreshold, highestFreq, lowestFreq);
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
  
  // Apply classic theremin vibrato
  float vibratoAmount = vibrato.next() * vibratoDepth;
  int modulatedFreq = baseFreq + (int)(baseFreq * vibratoAmount);
  osc.setFreq(modulatedFreq);

  // === VOLUME CONTROL (Left hand) ===
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
  
  // Map distance to volume with logarithmic curve for natural sound
  int linearVol = map(distance, volHighThreshold, volLowThreshold, 255, 0);
  
  // Apply logarithmic volume curve (sounds more natural than linear)
  float normalizedVol = linearVol / 255.0;
  float logVol = normalizedVol * normalizedVol;  // Square for logarithmic feel
  int mappedVol = (int)(logVol * 255);
  
  smoothVol = vAverage.next(mappedVol);
  
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
  // Generate the main oscillator sample
  int sample = (osc.next() * smoothVol) >> 8;
  
  // Return clean sample
  return sample;
}

void loop(){
  audioHook();                 // Required for Mozzi
}