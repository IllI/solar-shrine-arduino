#ifndef THEREMIN_H
#define THEREMIN_H

#include <Arduino.h>

void theremin_setup();
void theremin_disable();
void theremin_update(float distance1, float distance2);

#endif // THEREMIN_H