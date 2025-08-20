#ifndef DJ_SCRATCH_H
#define DJ_SCRATCH_H

#include <Arduino.h>

// External variable declarations
extern volatile int32_t djSampleIndex;
extern volatile uint8_t sampleCounter;
extern volatile uint8_t playState;
extern volatile uint8_t playbackSpeed;
extern volatile int8_t scratchSpeed;
extern volatile bool isScratchMode;

void dj_scratch_setup();
void dj_scratch_disable();
void dj_scratch_start();
void dj_scratch_update(float d1, float d2);

#endif // DJ_SCRATCH_H