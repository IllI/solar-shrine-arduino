#ifndef ALIEN_H
#define ALIEN_H

#include <Arduino.h>

void alien_setup();
void alien_disable();
void alien_update(float distanceLeft, float distanceRight);
void alien_audio_hook();
void alien_get_distances(float &outLeftCm, float &outRightCm);
void alien_sense(float &outLeftCm, float &outRightCm);
bool alien_is_test_tone_enabled();

#endif // ALIEN_H
