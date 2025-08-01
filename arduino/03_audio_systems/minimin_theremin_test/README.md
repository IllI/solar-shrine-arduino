# MiniMin Theremin Test - Classic Design

## Overview
This is the **classic MiniMin theremin** based on techno-womble's proven design, enhanced with echo effects and optimized for Solar Shrine hardware. This represents the gold standard of Arduino theremin implementations.

## Original Credit
- **Created**: 15 Dec 2019, Modified 23 Jan 2020, V1.2
- **By**: John Potter (techno-womble)
- **Adapted for**: Solar Shrine hardware with enhancements

## Features
- **Warm sine wave oscillator** (not tinny triangle waves)
- **Classic theremin vibrato** (5.5Hz wobble)
- **Logarithmic volume response** (natural sound curve)
- **Enhanced sensor sensitivity** with tuning options
- **C Minor Pentatonic scale**   (beautiful and forgiving)
- **Echo/delay effects** (currently disabled for stability)
- **Real-time debug output** for tuning

## Sound Characteristics
- **Warm, organic theremin tone** - professional quality
- **Natural vibrato modulation** - classic 5.5Hz wobble
- **Smooth volume transitions** - logarithmic curve
- **Professional theremin feel** - based on proven design
- **Optional note snapping** - stepMode for scale quantization

## Hardware Requirements
- 2x HC-SR04 ultrasonic sensors (pins 5,6,10,11)
- Audio output on pin 9 (Mozzi default)
- Arduino Uno or compatible

## Control Scheme
- **Right Hand (pins 10,11)**: Pitch control (C3 to C6 range)
- **Left Hand (pins 5,6)**: Volume control with logarithmic curve
- **Very Close Hand**: Maximum frequency/volume
- **No Hand**: Smooth fade out

## Tuning Parameters
```cpp
// Vibrato settings
float vibratoDepth = 0.03;     // 3% frequency modulation
float vibratoRate = 5.5;       // 5.5 Hz classic rate

// Sensor sensitivity (microseconds)
int pitchLowThreshold = 800;   // Farthest pitch detection
int pitchHighThreshold = 30;   // Closest pitch detection
int volLowThreshold = 600;     // Farthest volume detection  
int volHighThreshold = 30;     // Closest volume detection

// Echo effects (disabled by default)
float echoMix = 0.3;           // Echo volume (0.0 to 1.0)
int echoDelay = 128;           // Echo delay in samples
```

## Advanced Features
- **Step Mode**: Set `stepMode = true` for note quantization
- **Organic Jitter**: Random ±5Hz for natural sound
- **Echo Buffer**: 256 samples (512 bytes) for delay effects
- **Rolling Averages**: Pitch (4 samples) and Volume (8 samples)
- **Debug Output**: Real-time sensor feedback

## Usage Instructions
1. Install Mozzi library from https://sensorium.github.io/Mozzi/
2. Upload sketch to Arduino
3. Open Serial Monitor for real-time feedback
4. Move hands near sensors to play classic theremin sounds
5. Adjust tuning parameters as needed

## Debug Output Format
```
PitchTime | VolTime | Frequency | Volume | P:STATE V:STATE
800 | 400 | 523Hz | Vol:180 | P:MID V:MID
```

## Echo Effects
Echo effects are **temporarily disabled** for stability. To re-enable:
1. Uncomment the echo code in `updateAudio()`
2. Adjust `echoMix` and `echoDelay` as desired
3. Monitor memory usage (512 bytes for echo buffer)

## Why This Design?
- **Proven Track Record**: Based on techno-womble's successful MiniMin
- **Professional Sound**: Sine waves + vibrato = classic theremin
- **Stable Performance**: Well-tested sensor thresholds
- **Extensive Documentation**: Clear code comments and tuning guides
- **Research-Based**: Logarithmic volume, musical frequencies

## Status
✅ Recovered from git history (commit 5e52d69)  
✅ Classic proven design by techno-womble  
✅ Enhanced for Solar Shrine hardware  
🔧 Ready for immediate use or further development

## Comparison to Other Effects
- **vs Alien Effect**: More traditional, less experimental
- **vs Robot Effect**: Continuous vs quantized scales
- **vs DJ Scratch**: Musical instrument vs sound effect 