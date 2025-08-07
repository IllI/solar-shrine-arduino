#ifndef ALIEN_H
#define ALIEN_H

#include <Arduino.h>

void alien_setup();
void alien_disable();
void alien_update(float distanceLeft, float distanceRight);

#endif // ALIEN_H
