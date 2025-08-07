#ifndef ROBOTS_H
#define ROBOTS_H

#include <Arduino.h>

void robots_setup();
void robots_disable();
void robots_update(float distanceLeft, float distanceRight);

#endif // ROBOTS_H
