/****************************************************************************     
 MiniMin Theremin Test - Enhanced with Echo and Improved Sensitivity
 Based on techno-womble's proven design, adapted for Solar Shrine hardware
 
 Original: Created 15 Dec 2019, Modified 23 Jan 2020, V1.2
 By John Potter (techno-womble)
 
 ENHANCEMENTS:
 - Echo/delay effect for richer sound
 - Increased sensor sensitivity (detects hands from farther away)  
 - Debug output to help tune hand movements
 - C Minor Pentatonic scale (sounds great!)
 
 TUNING TIPS:
 - Adjust echoMix (0.0-1.0) for more/less echo
 - Adjust echoDelay (samples) for echo timing
 - Modify threshold values if sensitivity needs tweaking
 - Watch Serial Monitor for real-time sensor feedback
 
 Hardware:
 - 2x HC-SR04 sensors on pins 5,6,10,11 
 - Audio output on pin 12 (Mozzi 2-pin PWM mode)
*********************************************************************************/

//   download Mozzi library from https://sensorium.github.io/Mozzi/ 

// Configure Mozzi for 2-pin PWM mode for output on pin 12
#include <MozziConfigValues.h>
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_2PIN_PWM

#include <MozziGuts.h>
#include <Oscil.h> 
#include <RollingAverage.h>
#include <tables/triangle_valve_2048_int8.h>              // oscillator wavetable data

Oscil <TRIANGLE_VALVE_2048_NUM_CELLS, AUDIO_RATE> osc(TRIANGLE_VALVE_2048_DATA);

#define CONTROL_RATE 128         

const boolean stepMode = false;  // Set to true for note-snapping mode

// Echo/Delay effect settings - REDUCED for Arduino Uno memory limits
#define ECHO_BUFFER_SIZE 256     // Smaller buffer (256 * 2 bytes = 512 bytes)
int echoBuffer[ECHO_BUFFER_SIZE];
int echoIndex = 0;
float echoMix = 0.3;            // Echo volume (0.0 to 1.0)
int echoDelay = 128;            // Echo delay in samples (shorter but still audible)

// C Minor Pentatonic scale - sounds beautiful and forgiving
int notes[] = {131,131,131,156,156,175,175,196,196,196,233,233,       // C3 octave
               262,262,262,311,311,349,349,392,392,392,466,466,       // C4 octave  
               523,523,523,622,622,698,698,783,783,783,932,932,       // C5 octave
               1046};                                                  // C6

RollingAverage <int, 4> pAverage;             // Pitch averaging
RollingAverage <int, 8> vAverage;             // Volume averaging
int averaged;

// Pin assignments adapted for your hardware
const int volOut = 5;                         // Volume sensor trigger (left sensor)
const int volIn = 6;                          // Volume sensor echo (left sensor)
const int pitchOut = 10;                      // Pitch sensor trigger (right sensor) 
const int pitchIn = 7;                        // Pitch sensor echo (right sensor) - MOVED FROM 11 to avoid Mozzi conflict

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
    osc.setFreq(targetFreq + jitter); 
  } else {
    // Smooth continuous frequency
    averaged = pAverage.next(freq); 
    osc.setFreq(averaged + jitter); 
  }

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
  
  // Map distance to volume (closer = louder)
  int mappedVol = map(distance, volHighThreshold, volLowThreshold, 255, 0);
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
  
  // ECHO TEMPORARILY DISABLED - uncomment below to re-enable
  /*
  // Add echo effect - get sample from echoDelay positions back
  int echoPos = (echoIndex - echoDelay + ECHO_BUFFER_SIZE) % ECHO_BUFFER_SIZE;
  int echoSample = echoBuffer[echoPos];
  int mixedSample = sample + (int)(echoSample * echoMix);
  
  // Store current sample in echo buffer for future echo
  echoBuffer[echoIndex] = sample;
  
  // Move to next position in circular buffer
  echoIndex++;
  if (echoIndex >= ECHO_BUFFER_SIZE) {
    echoIndex = 0;
  }
  
  // Clip to prevent distortion
  if (mixedSample > 127) mixedSample = 127;
  if (mixedSample < -128) mixedSample = -128;
  
  return mixedSample;
  */
  
  // Return clean sample without echo
  return sample;
}

void loop(){
  audioHook();                 // Required for Mozzi
}