#include "theremin.h"
#include "NewTone.h"

const int AUDIO_PIN = 12;

// Range constants (in cm)
const float MIN_RANGE_THEREMIN = 5.0;
const float MAX_RANGE_THEREMIN = 50.0;

void theremin_setup() {
  pinMode(AUDIO_PIN, OUTPUT);
  Serial.println(F("Theremin effect initialized."));
}

void theremin_disable() {
  noNewTone(AUDIO_PIN);
  Serial.println(F("Theremin effect disabled."));
}

void theremin_update(float distance1, float distance2) {
    bool handInRange1 = (distance1 >= MIN_RANGE_THEREMIN && distance1 <= MAX_RANGE_THEREMIN);
    bool handInRange2 = (distance2 >= MIN_RANGE_THEREMIN && distance2 <= MAX_RANGE_THEREMIN);
    if (handInRange2) {
        int frequency = map(distance2, MIN_RANGE_THEREMIN, MAX_RANGE_THEREMIN, 1000, 200);
        NewTone(AUDIO_PIN, frequency);
    } else if (handInRange1) { // Fallback to left hand if right is out of range
        int frequency = map(distance1, MIN_RANGE_THEREMIN, MAX_RANGE_THEREMIN, 1000, 200);
        NewTone(AUDIO_PIN, frequency);
    } else {
        noNewTone(AUDIO_PIN);
    }
}