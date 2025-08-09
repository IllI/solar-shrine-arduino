#ifndef ROBOTS_H
#define ROBOTS_H

#include <Arduino.h>

void robots_setup();
void robots_disable();
void robots_update(float distanceLeft, float distanceRight);

// Telemetry for LED visuals
uint8_t robots_get_level(); // 0..255 envelope / volume proxy

#endif // ROBOTS_H
