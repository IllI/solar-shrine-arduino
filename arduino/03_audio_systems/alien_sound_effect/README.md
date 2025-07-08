# Alien Sound Effect

## Overview
This Arduino sketch creates an alien-like theremin sound effect using the Mozzi audio library. It was recovered from git history during the project cleanup.

## Features
- **Warm sine wave theremin** with classic vibrato
- **Echo/delay effect** for alien-like sound
- **C Minor Pentatonic scale** quantization
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
- Adjust `vibratoRate` for faster/slower wobble
- Modify `vibratoDepth` for more/less vibrato
- Tune `echoMix` for echo intensity
- Watch Serial Monitor for sensor feedback

## Status
✅ Recovered from git history (commit b93c0d2)
🔧 Ready for use and further development 