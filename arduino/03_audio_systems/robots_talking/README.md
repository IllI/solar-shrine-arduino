# Robots Talking Effect

## Overview
This Arduino sketch creates robot-like conversational sounds using a warm, professional theremin engine. It was recovered from git history during the project cleanup.

## Features
- **Pure sine wave** for warm, musical sound
- **Research-based frequency range** (120-1500Hz)
- **C Major Pentatonic scale** quantization (no wrong notes)
- **Gentle vibrato** (3.2Hz, 1.5% depth)
- **Note persistence** (prevents accidental notes)
- **Exponential volume curves** (matches human hearing)

## Hardware Requirements
- 2x HC-SR04 ultrasonic sensors (pins 5,6,10,11)
- Audio output on pin 9 (Mozzi default)
- Arduino Uno or compatible

## Sound Characteristics
- Warm, professional theremin tone
- Musical scale quantization
- Gentle vibrato for expression
- Smooth volume transitions
- Robot-like conversational quality

## Research Sources
- Adafruit Theremin Guide: 120-1500Hz range recommendation
- Arduino Pitch Follower: Musical mapping techniques
- Audio Physics: Sine waves = warmth, Triangle waves = tinny

## Usage
1. Install Mozzi library from https://sensorium.github.io/Mozzi/
2. Upload sketch to Arduino
3. Move hands near sensors to create robot talking sounds
4. Right hand controls pitch, left hand controls volume

## Musical Features
- **Pentatonic Scale**: Always sounds good, no dissonant notes
- **Note Persistence**: 75ms hold prevents accidental notes
- **Exponential Volume**: Matches human hearing perception
- **Gentle Vibrato**: 1.5% modulation for warmth

## Status
✅ Recovered from git history (commit 4e9a1c0)
🔧 Ready for use and further development 