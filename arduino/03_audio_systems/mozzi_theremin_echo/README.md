# Solar Shrine Theremin Echo Test

## Overview
This is a test implementation of a theremin-like audio effect with echo capabilities using ultrasonic sensors and the Mozzi audio synthesis library. It's designed to help troubleshoot and test audio functionality for the Solar Shrine project.

## Features
- **Theremin-like Control**: Hand-proximity frequency control using HC-SR04 ultrasonic sensors
- **Echo effects**: Multiple delayed echoes create atmospheric soundscapes
- **Rolling average smoothing**: Reduces noise from analog sensor input
- **Solar Shrine compatible**: Configured for pin 12 audio output and ultrasonic sensors as specified in project guidelines

## Hardware Configuration

### Audio Output
- **Pin 12 and 13**: PWM audio output (2-pin PWM mode for Arduino Mega)
- **Configuration**: 2-pin PWM mode for compatibility with Solar Shrine system

### Ultrasonic Sensors
- **Left sensor**: Trigger=Pin 10, Echo=Pin 11
- **Right sensor**: Trigger=Pin 5, Echo=Pin 6
- **Sensor Configuration**: As specified in CUSTOM_SHIELD_WIRING.md

## Technical Details

### Mozzi Configuration
```cpp
#define MOZZI_AUDIO_PIN_1 12
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_PWM
```

### Control Rate
- **64 Hz**: Optimized for responsive control while maintaining audio quality

### Echo System
- **3 echo delays**: 32, 60, and 127 cells
- **Rolling average**: 32-sample smoothing for stable control
- **Multiple oscillators**: Creates rich harmonic content

## Usage
1. Connect HC-SR04 ultrasonic sensors as specified in the hardware configuration
2. Connect audio output from pin 12 to amplifier/speaker
3. Upload sketch to Arduino Mega 2560
4. Move your right hand near the right sensor (5-20cm range) to control frequency
5. Listen for the echo effects that create ambient soundscapes
6. Closer hand proximity = higher frequency

## Integration Notes
This effect can be integrated into the main Solar Shrine modular system by:
1. Adapting the control input to use ultrasonic sensors instead of LDR
2. Following the effect class pattern used in other Mozzi effects
3. Ensuring proper mode switching and audio isolation

## Compatibility
- **Arduino Mega 2560**: Primary target platform
- **Mozzi Library**: Requires Mozzi v2.0 or later
- **Solar Shrine Shield**: Compatible with custom shield wiring

## Testing

✅ **Compilation Status**: Successfully compiles (5916 bytes program, 1198 bytes RAM)
- Upload to Arduino Mega 2560
- Connect LDR to pin A0 with appropriate pull-down resistor
- Connect audio output from pin 12 (and pin 13 for 2-pin PWM mode)
- Cover/uncover LDR to hear theremin-like effects with echo
- **Audio quality**: Clear, loud theremin with echo harmonics

## Compilation Results
- **Program Storage**: 8142 bytes (3% of Arduino Mega)
- **Dynamic Memory**: 1243 bytes (15% of available RAM)
- **Status**: ✅ Compilation successful with Arduino Mega 2-pin PWM configuration
- **Pin Configuration**: Matches `solar_shrine_playa.ino` system requirements
- **High volume**: 14-bit depth with 24x amplification (doubled from 12x)

This standalone sketch allows testing of:
- Pin 12 audio output functionality
- Mozzi library integration
- Echo and delay effects
- Analog input processing

Use this as a foundation for developing more complex Mozzi effects for the Solar Shrine project.