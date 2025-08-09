# Alien Sound Effect

## Overview
This Arduino sketch creates an alien-like theremin sound effect using the Mozzi audio library. It was recovered from git history during the project cleanup.

## Features
- **Warm sine wave theremin** with classic vibrato
- **Gesture-reactive vibrato**: depth and rate adapt to hand distance and speed
- **Motion-driven harmonics**: quick pitch-hand moves add a bright harmonic layer
- **Gesture-controlled echo**: left-hand distance sets echo mix; right-hand distance sets delay
- **C Minor Pentatonic scale** quantization (optional)
- **Dual sensor control**: Right hand = pitch, Left hand = volume
- **Enhanced sensitivity** with debug output

## Hardware Requirements
- 2x HC-SR04 ultrasonic sensors (pins 5,6,10,11)
- Audio output on pin 9 (Mozzi default)
- Arduino Uno or compatible

## Sound Characteristics
- Warm, organic theremin tone
- Natural vibrato modulation (5.5Hz)
- Echo/delay effect for alien ambiance
- Smooth volume transitions
- Professional theremin feel

## Usage
1. Install Mozzi library from https://sensorium.github.io/Mozzi/
2. Upload sketch to Arduino
3. Move hands near sensors to create alien sounds
4. Right hand controls pitch, left hand controls volume

## Tuning
- **Vibrato**: now dynamic. Baseline scales with left-hand distance; movement increases depth/rate.
- **Harmonics**: `harmMix` is automatic; move the right hand quickly to hear added brightness.
- **Echo**: `echoMix` rises as the left hand gets closer; `echoDelay` grows as the right hand moves farther.
- Watch Serial Monitor for sensor and modulation readouts (VibD, VibR, Harm).

## Status
✅ Recovered from git history (commit b93c0d2)
🔧 Ready for use and further development 