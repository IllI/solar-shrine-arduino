# Mozzi Pin Configuration for Solar Shrine Theremin

## Audio Output Pins (Arduino Mega 2560)

**CRITICAL:** Mozzi uses pins 11 & 12 for 2-pin PWM audio output on Arduino Mega.
- **Pin 11:** Audio output (primary) - used internally by Mozzi
- **Pin 12:** Audio output (secondary) - Connect this to your amplifier/speaker
- **NEVER use single-pin PWM mode** - it produces no audio output

## Sensor Pin Assignments

### Volume Control (Left Hand Sensor)
- **Trigger Pin:** 10
- **Echo Pin:** 7 (CHANGED from 11 to avoid Mozzi conflict)

### Pitch Control (Right Hand Sensor)
- **Trigger Pin:** 5
- **Echo Pin:** 6

## Mozzi Configuration

```cpp
// Configure Mozzi for 2-pin PWM mode
#include <MozziConfigValues.h>
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_2PIN_PWM

#include <MozziGuts.h>
// ... other includes

void setup() {
  // ... sensor setup
  startMozzi(CONTROL_RATE);  // Initialize Mozzi
}

void loop() {
  audioHook();  // Required for Mozzi operation
}
```

## Wiring Notes

1. **Audio Output:** Connect pin 12 to your amplifier input
2. **Power:** Ensure adequate power supply for both Arduino and sensors
3. **Sensor Wiring:** Standard HC-SR04 connections (VCC, GND, Trig, Echo)

## Troubleshooting

- **No Audio:** Check that pin 12 is connected to amplifier
- **Distorted Audio:** Verify power supply stability
- **Sensor Issues:** Ensure no pin conflicts with Mozzi audio pins (11, 12)

## Version History

- **v1.0:** Initial configuration with pin conflict resolution
- **v1.1:** Corrected pin assignments to match working configuration
- Pin 11 moved from volume sensor echo to dedicated Mozzi audio output
- Volume sensor echo moved to pin 7
- Confirmed 2-pin PWM mode is required for audio output