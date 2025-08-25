/*  Solar Shrine Theremin Echo Test
     Theremin-like instrument with long echoes using ultrasonic sensors
     Based on Mozzi sonification library example
  
     Demonstrates ControlDelay() for echoing control values, 
     and smoothing ultrasonic sensor input with RollingAverage(). 
  
     The circuit: 
  
     Audio output on digital pin 12 (configured for Solar Shrine)
     Following MODULAR_AUDIO_SYSTEM_GUIDE.md and CUSTOM_SHIELD_WIRING.md
  
     Ultrasonic sensors (HC-SR04) as specified in CUSTOM_SHIELD_WIRING.md:
     Left sensor: Trig=Pin 10, Echo=Pin 11
     Right sensor: Trig=Pin 5, Echo=Pin 6 
  
    Mozzi documentation/API 
    https://sensorium.github.io/Mozzi/doc/html/index.html 
  
    Mozzi help/discussion/announcements: 
    https://groups.google.com/forum/#!forum/mozzi-users 
  
    Copyright 2012-2024 Tim Barrass and the Mozzi Team 
  
    Mozzi is licensed under the GNU Lesser General Public Licence (LGPL) Version 2.1 or later. 
 */ 
  
#define MOZZI_CONTROL_RATE 64 

// Configure Mozzi for pin 12 audio output (Solar Shrine configuration)
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_2PIN_PWM

// Custom pin configuration for Arduino Mega (matches solar_shrine_playa.ino)
#define MOZZI_AUDIO_PIN_1 12
#define MOZZI_AUDIO_PIN_1_REGISTER OCR1B
#define MOZZI_AUDIO_PIN_1_LOW 13
#define MOZZI_AUDIO_PIN_1_LOW_REGISTER OCR1C

#include <MozziGuts.h> 
#include <Oscil.h> // oscillator template 
#include <tables/sin2048_int8.h> // sine table for oscillator 
#include <RollingAverage.h> 
#include <ControlDelay.h> 
  
// Ultrasonic sensor pins (from CUSTOM_SHIELD_WIRING.md)
#define TRIG1 10  // Left sensor trigger
#define ECHO1 11  // Left sensor echo  
#define TRIG2 5   // Right sensor trigger
#define ECHO2 6   // Right sensor echo 
  
unsigned int echo_cells_1 = 32; 
unsigned int echo_cells_2 = 60; 
unsigned int echo_cells_3 = 127; 
  
ControlDelay <128, int> kDelay; // 2seconds 
  
// oscils to compare bumpy to averaged control input 
Oscil <SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> aSin0(SIN2048_DATA); 
Oscil <SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> aSin1(SIN2048_DATA); 
Oscil <SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> aSin2(SIN2048_DATA); 
Oscil <SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> aSin3(SIN2048_DATA); 
  
// use: RollingAverage <number_type, how_many_to_average> myThing 
RollingAverage <int, 32> kAverage; // how_many_to_average has to be power of 2 
int averaged;

// Gate audio when no hands detected (like solar_shrine_playa)
static volatile bool g_handsActive = false; 
  
// Sensor reading function (from solar_shrine_playa)
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

void setup(){ 
  Serial.begin(9600);
  Serial.println("Starting Solar Shrine Theremin Echo Test");
  Serial.println("Audio output on pin 12");
  Serial.println("Using ultrasonic sensors for control");
  
  // Initialize sensor pins
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  
  kDelay.set(echo_cells_1); 
  startMozzi(); 
} 
  
  
void updateControl(){ 
  // Read both ultrasonic sensors
  float leftDistance = readSensor(TRIG1, ECHO1);
  float rightDistance = readSensor(TRIG2, ECHO2);
  
  // Check if any hands are present
  bool leftHand = isHandPresent(leftDistance);
  bool rightHand = isHandPresent(rightDistance);
  g_handsActive = (leftHand || rightHand);
  
  // Use right hand for primary frequency control (like ThereminEffect)
  int sensor_input = 0;
  if (rightHand) {
    // Map distance to frequency range (closer = higher frequency)
    float normalizedDistance = (rightDistance - 1.0) / 19.0; // 1-20cm range
    normalizedDistance = 1.0 - normalizedDistance; // Invert so closer = higher
    sensor_input = (int)(normalizedDistance * 1023); // Convert to 0-1023 range
  } else {
    sensor_input = 200; // Default frequency when no hand
  }
  
  // Apply smoothing and echo effects
  averaged = kAverage.next(sensor_input); 
  aSin0.setFreq(averaged); 
  aSin1.setFreq(kDelay.next(averaged)); 
  aSin2.setFreq(kDelay.read(echo_cells_2)); 
  aSin3.setFreq(kDelay.read(echo_cells_3)); 
} 
  
  
AudioOutput_t updateAudio(){ 
  // Hard gate: never output audio without hands detected (like solar_shrine_playa)
  if (!g_handsActive) {
    return MonoOutput::from8Bit(0);
  }
  
  // Restore the better-sounding fromAlmostNBit implementation with much higher amplification
  return MonoOutput::fromAlmostNBit(14, 
    24*((int)aSin0.next()+aSin1.next()+(aSin2.next()>>1) 
    +(aSin3.next()>>2)) 
  ); 
} 
  
  
void loop(){ 
  audioHook(); 
}