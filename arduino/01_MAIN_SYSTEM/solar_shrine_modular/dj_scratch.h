#ifndef DJ_SCRATCH_H
#define DJ_SCRATCH_H

#include <Arduino.h>

void dj_scratch_setup();
void dj_scratch_disable();
void dj_scratch_start();
void dj_scratch_update(float d1, float d2);

#endif // DJ_SCRATCH_H