#ifndef AUDIO_EFFECTS_H
#define AUDIO_EFFECTS_H

#include <Arduino.h>

// Audio pin configuration
#define AUDIO_PIN 12  // Timer1 OC1B on Arduino Mega

// Function prototypes for the theremin effect
void theremin_setup();
void theremin_update(float distance1, float distance2);
void theremin_disable();

#endif // AUDIO_EFFECTS_H