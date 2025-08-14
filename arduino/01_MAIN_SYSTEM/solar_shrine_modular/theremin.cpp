#include "theremin.h"
// NewTone library disabled to prevent timer conflicts with Mozzi
// #include "NewTone.h"

const int AUDIO_PIN = 12;

// Range constants (in cm)
const float MIN_RANGE_THEREMIN = 5.0;
const float MAX_RANGE_THEREMIN = 50.0;

void theremin_setup() {
  pinMode(AUDIO_PIN, OUTPUT);
#if TEST_VERBOSE
  Serial.println(F("Theremin effect initialized."));
#endif
}

void theremin_disable() {
  // NewTone disabled - no audio output needed for theremin when using Mozzi
  // noNewTone(AUDIO_PIN);
#if TEST_VERBOSE
  Serial.println(F("Theremin effect disabled."));
#endif
}

void theremin_update(float distance1, float distance2) {
    bool handInRange1 = (distance1 >= MIN_RANGE_THEREMIN && distance1 <= MAX_RANGE_THEREMIN);
    bool handInRange2 = (distance2 >= MIN_RANGE_THEREMIN && distance2 <= MAX_RANGE_THEREMIN);
    if (handInRange2) {
        // NewTone disabled - theremin effect not used when Mozzi alien effect is active
        // int frequency = map(distance2, MIN_RANGE_THEREMIN, MAX_RANGE_THEREMIN, 1000, 200);
        // NewTone(AUDIO_PIN, frequency);
    } else if (handInRange1) { // Fallback to left hand if right is out of range
        // NewTone disabled - theremin effect not used when Mozzi alien effect is active
        // int frequency = map(distance1, MIN_RANGE_THEREMIN, MAX_RANGE_THEREMIN, 1000, 200);
        // NewTone(AUDIO_PIN, frequency);
    } else {
        // NewTone disabled - no audio cleanup needed
        // noNewTone(AUDIO_PIN);
    }
}