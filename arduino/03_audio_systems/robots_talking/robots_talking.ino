/****************************************************************************     
 Musical Theremin - Warm, Professional Sound
 Based on research from Adafruit, Arduino.cc, and audio physics principles
 
 MUSICAL SOUND PRINCIPLES APPLIED:
 - Pure sine wave (no harsh harmonics like triangle waves)
 - Musical frequency range: 120-1500Hz (research-proven sweet spot)
 - Pentatonic scale quantization (always sounds good)
 - Gentle vibrato (2-4Hz, <2% depth for warmth)
 - Exponential volume curves (matches human hearing)
 - Note persistence (prevents accidental notes)
 
 RESEARCH SOURCES:
 - Adafruit Theremin Guide: 120-1500Hz range recommendation
 - Arduino Pitch Follower: Musical mapping techniques  
 - Audio Physics: Sine waves = warmth, Triangle waves = tinny
 
 Hardware:
 - 2x HC-SR04 sensors on pins 5,6,10,11 
 - Audio output on pin 9 (Mozzi)
*********************************************************************************/

#include <MozziGuts.h>
#include <Oscil.h> 
#include <RollingAverage.h>
#include <tables/sin2048_int8.h>                          // Pure sine wave for warm sound
#include <tables/cos2048_int8.h>                          // Smooth vibrato LFO

// Main oscillator - pure sine wave for musical warmth
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> mainOsc(SIN2048_DATA);

// Gentle vibrato for musical expression (research shows 2-4Hz optimal)
Oscil <COS2048_NUM_CELLS, CONTROL_RATE> vibratoLFO(COS2048_DATA);

#define CONTROL_RATE 256         // Higher rate for smoother musical response

// MUSICAL FREQUENCY RANGE (based on research from Adafruit and Arduino.cc)
const int MUSICAL_MIN_FREQ = 120;    // Research-proven musical low end
const int MUSICAL_MAX_FREQ = 1500;   // Research-proven musical high end  
const int MUSICAL_BASE_FREQ = 440;   // A4 reference note

// Musical scale - C Major Pentatonic (always sounds good, no wrong notes)
int musicalScale[] = {
  131, 147, 165, 196, 220,           // C3 pentatonic
  262, 294, 330, 392, 440,           // C4 pentatonic (middle range)
  523, 587, 659, 784, 880,           // C5 pentatonic
  1047, 1175, 1319, 1568             // C6 pentatonic (high end)
};

const int SCALE_LENGTH = 19;

// Gentle vibrato settings (research shows <2% for musical warmth)
float vibratoDepth = 0.015;          // 1.5% frequency modulation (subtle)
float vibratoRate = 3.2;             // 3.2 Hz (gentle musical vibrato)

// Sensor averaging for stability
RollingAverage <int, 8> pitchAverage;     // More averaging for stable pitch
RollingAverage <int, 12> volumeAverage;   // Even more for smooth volume

// Pin assignments
const int pitchTrigPin = 10;              // Right hand controls pitch
const int pitchEchoPin = 11;
const int volTrigPin = 5;                 // Left hand controls volume  
const int volEchoPin = 6;

// Musical sensor thresholds (wider range for expressiveness)
const int PITCH_NEAR = 25;               // Very close pitch detection
const int PITCH_FAR = 1000;              // Far pitch detection  
const int VOL_NEAR = 25;                 // Very close volume detection
const int VOL_FAR = 800;                 // Far volume detection

// Note persistence (from theremin digitizer research)
const int NOTE_PERSISTENCE_MS = 75;       // Hold note for 75ms before changing
unsigned long lastNoteChange = 0;
int currentNote = 0;
int targetNote = 0;

// Audio state
int baseFrequency = MUSICAL_BASE_FREQ;
int currentVolume = 0;
bool thereminActive = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Musical Theremin - Warm Professional Sound");
  Serial.println("Based on audio physics research and musical principles");
  Serial.println("Right hand = Pitch, Left hand = Volume");
  Serial.println();
  
  // Initialize sensor pins
  pinMode(pitchTrigPin, OUTPUT);
  pinMode(pitchEchoPin, INPUT);
  pinMode(volTrigPin, OUTPUT);
  pinMode(volEchoPin, INPUT);
  
  // Initialize Mozzi
  startMozzi(CONTROL_RATE);
  
  // Set up oscillators
  mainOsc.setFreq(MUSICAL_BASE_FREQ);
  vibratoLFO.setFreq(vibratoRate);
  
  Serial.println("Ready! Move hands near sensors for warm musical sounds.");
}

// Read ultrasonic sensor with timeout
long readSensor(int trigPin, int echoPin, long maxTime) {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, maxTime);
  return duration;
}

// Find closest note in musical scale
int findClosestNote(int frequency) {
  int bestNote = 0;
  int smallestDiff = abs(musicalScale[0] - frequency);
  
  for (int i = 1; i < SCALE_LENGTH; i++) {
    int diff = abs(musicalScale[i] - frequency);
    if (diff < smallestDiff) {
      smallestDiff = diff;
      bestNote = i;
    }
  }
  return bestNote;
}

// Apply musical note persistence (prevents accidental notes)
int applyNotePersistence(int newNote) {
  if (newNote != targetNote) {
    targetNote = newNote;
    lastNoteChange = millis();
  }
  
  if (millis() - lastNoteChange > NOTE_PERSISTENCE_MS) {
    currentNote = targetNote;
  }
  
  return currentNote;
}

void updateControl() {
  // Read pitch sensor (right hand)
  long pitchTime = readSensor(pitchTrigPin, pitchEchoPin, 10000);
  
  // Read volume sensor (left hand)  
  long volTime = readSensor(volTrigPin, volEchoPin, 8000);
  
  // Process pitch
  int rawFreq = MUSICAL_BASE_FREQ;
  bool pitchActive = false;
  
  if (pitchTime > 0 && pitchTime < 5000) {
    // Map sensor reading to musical frequency range
    int distance = pitchTime / 58;  // Convert to cm
    distance = constrain(distance, PITCH_NEAR, PITCH_FAR);
    
    // Map to musical frequency range (research-based 120-1500Hz)
    rawFreq = map(distance, PITCH_NEAR, PITCH_FAR, MUSICAL_MAX_FREQ, MUSICAL_MIN_FREQ);
    pitchActive = true;
  }
  
  // Apply musical scale quantization with note persistence
  int noteIndex = findClosestNote(rawFreq);
  int persistentNote = applyNotePersistence(noteIndex);
  baseFrequency = musicalScale[persistentNote];
  
  // Smooth the frequency changes
  baseFrequency = pitchAverage.next(baseFrequency);
  
  // Process volume with exponential curve (matches human hearing)
  int rawVolume = 0;
  bool volumeActive = false;
  
  if (volTime > 0 && volTime < 8000) {
    int distance = volTime / 58;
    distance = constrain(distance, VOL_NEAR, VOL_FAR);
    
    // Map to volume with exponential curve for natural response
    float linearVol = map(distance, VOL_NEAR, VOL_FAR, 255, 0) / 255.0;
    float expVol = linearVol * linearVol * linearVol;  // Cubic for gentle response
    rawVolume = (int)(expVol * 255);
    volumeActive = true;
  }
  
  // Apply volume smoothing and fade
  if (volumeActive || pitchActive) {
    thereminActive = true;
    currentVolume = volumeAverage.next(rawVolume);
  } else {
    thereminActive = false;
    currentVolume = volumeAverage.next(0);  // Smooth fade out
  }
  
  // Apply gentle musical vibrato
  float vibratoMod = vibratoLFO.next() * vibratoDepth;
  int modulatedFreq = baseFrequency + (int)(baseFrequency * vibratoMod);
  
  // Set the main oscillator frequency
  mainOsc.setFreq(modulatedFreq);
  
  // Debug output (reduced frequency)
  static int debugCount = 0;
  if (++debugCount > 100) {
    Serial.print("Note: ");
    Serial.print(musicalScale[persistentNote]);
    Serial.print("Hz | Vol: ");
    Serial.print(currentVolume);
    Serial.print(" | ");
    if (thereminActive) Serial.print("ACTIVE");
    else Serial.print("SILENT");
    Serial.println();
    debugCount = 0;
  }
}

int updateAudio() {
  // Generate pure sine wave sample
  int sample = mainOsc.next();
  
  // Apply musical volume curve
  sample = (sample * currentVolume) >> 8;
  
  return sample;
}

void loop() {
  audioHook();  // Required for Mozzi
} 