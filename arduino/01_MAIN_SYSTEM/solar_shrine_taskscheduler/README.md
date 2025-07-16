# Solar Shrine Effect Rotator - Optimized Audio Timing

## Overview
This system handles 4 different audio effects with their specific timing requirements, solving the fundamental problem that TaskScheduler interferes with audio generation timing.

## Key Insight: Why TaskScheduler Fails with Audio

**The Problem**: TaskScheduler uses variable timing intervals that disrupt the precise timing required for audio generation. Different audio libraries have different timing requirements:

1. **Mozzi Library**: Requires consistent `audioHook()` calls at ~7.8kHz for proper audio synthesis
2. **Timer1 ISR**: Requires uninterrupted hardware timer interrupts at ~40kHz for sample playback
3. **Different Control Rates**: Mozzi effects use different control rates (128 Hz vs 256 Hz)

**The Solution**: Effect-specific timing approaches that preserve the precise requirements of each audio system.

## Supported Effects

### 1. Theremin (Mozzi-based, 128 Hz control rate)
- **Source**: Based on `minimin_theremin_test/minimin_theremin_test.ino`
- **Audio**: Pure sine wave with vibrato
- **Timing**: Mozzi CONTROL_RATE 128, requires `audioHook()` every ~8ms
- **LED Color**: Blue
- **Controls**: Distance controls frequency, both hands create harmonic blend

### 2. Alien Sound (Timer1 ISR FM synthesis)
- **Source**: Based on `alien_sound_effect/alien_sound_effect.ino`
- **Audio**: FM synthesis with modulated carrier wave
- **Timing**: Timer1 ISR at ~40kHz, cannot be interrupted
- **LED Color**: Green
- **Controls**: Hand presence activates alien-like modulated tones

### 3. Robots Talking (Mozzi-based, 256 Hz control rate)
- **Source**: Based on `robots_talking/robots_talking.ino`
- **Audio**: Square wave distortion with frequency modulation
- **Timing**: Mozzi CONTROL_RATE 256, requires `audioHook()` every ~4ms
- **LED Color**: Purple
- **Controls**: Distance controls base frequency with robotic modulation

### 4. DJ Scratch (Timer1 ISR with audio buffer)
- **Source**: Based on `dj_scratch_progmem/dj_scratch_progmem.ino`
- **Audio**: Sample playback with scratch control
- **Timing**: Timer1 ISR at ~40kHz with variable playback speed
- **LED Color**: Red
- **Controls**: Left hand = play/stop, Right hand = scratch direction

## Technical Implementation

### Audio System Switching
The system cleanly switches between Mozzi and Timer1 ISR approaches:

```cpp
void switchToEffect(EffectType newEffect) {
  // Cleanup current effect
  if (state.timer1Active) {
    TIMSK1 = 0;  // Disable Timer1 interrupt
    state.timer1Active = false;
  }
  
  // Initialize new effect with appropriate timing
  switch (newEffect) {
    case EFFECT_THEREMIN:
      setupTheremin();  // Starts Mozzi with 128 Hz control rate
      break;
    case EFFECT_ALIEN:
      setupAlien();     // Starts Timer1 ISR at 40kHz
      break;
    // ... etc
  }
}
```

### Timing Optimization
Different timing approaches based on active audio system:

```cpp
void loop() {
  // Core functions with non-blocking timing
  readSensors();        // 50Hz (20ms intervals)
  updateAudio();        // Effect-specific timing
  updateLEDs();         // 30fps (33ms intervals)
  checkEffectRotation(); // 5 second intervals
  sendJSONUpdate();     // 100ms intervals
  
  // Critical: Mozzi audio hook when active
  if (state.mozziActive) {
    audioHook();
  }
  
  // Adaptive timing based on active audio system
  if (state.mozziActive) {
    delay(8);  // ~125Hz for Mozzi compatibility
  } else {
    delay(10); // Standard 100Hz for Timer1 effects
  }
}
```

### Effect Rotation Logic
Effects rotate automatically after 5 seconds of no hand detection:

```
Theremin -> Alien -> Robots -> DJ Scratch -> (repeat)
```

## Hardware Requirements

### Sensors
- 2x HC-SR04 ultrasonic sensors
- Left sensor (pins 5,6): Volume/left hand control
- Right sensor (pins 10,11): Pitch/right hand control

### LED Strip
- WS2812B/WS2815 LED strip on pin 3
- 60 LEDs (configurable via `NUM_LEDS`)

### Audio Output (Arduino Mega 2560)
- Pin 12: Audio output (Timer1 OC1B PWM - compatible with both Mozzi and Timer1)
- Wiring: Right channel → 1K resistor → Pin 12, Left channel + Ground → Ground
- Recommended: WWZMDiB XH-M543 amplifier + Dayton Audio DAEX32QMB-4 exciter

## JSON Output for TouchDesigner

Enhanced JSON output includes effect-specific information:

```json
{
  "left": 15,
  "right": 8,
  "hands_detected": true,
  "left_in_range": true,
  "right_in_range": true,
  "current_effect": 0,
  "effect_name": "Theremin",
  "audio_volume": 200,
  "mozzi_active": true,
  "timer1_active": false,
  "control_rate": 128
}
```

## Libraries Required

- **ArduinoJson**: For TouchDesigner communication
- **FastLED**: For LED strip control
- **NewPing**: For ultrasonic sensor readings
- **Mozzi**: For high-quality audio synthesis (effects 1 & 3)

## Installation

1. Install required libraries through Arduino IDE Library Manager
2. Download Mozzi library from: https://sensorium.github.io/Mozzi/
3. Upload to Arduino Uno/Nano with appropriate hardware connections
4. Open Serial Monitor at 9600 baud to see effect switching and JSON output

## Performance Characteristics

### Memory Usage
- **Theremin**: ~15KB program memory (Mozzi + basic oscillators)
- **Alien**: ~12KB program memory (Timer1 ISR + FM synthesis)
- **Robots**: ~16KB program memory (Mozzi + dual oscillators)
- **DJ Scratch**: ~13KB program memory (Timer1 ISR + audio buffer)

### Timing Precision
- **Sensor Reading**: 50Hz (20ms intervals)
- **LED Updates**: 30fps (33ms intervals)
- **JSON Updates**: 10Hz (100ms intervals)
- **Audio Generation**: Effect-specific (125Hz for Mozzi, 40kHz for Timer1)

## Troubleshooting

### Audio Issues
- **Buzzing/Distortion**: Check that only one audio system is active at a time
- **No Audio**: Verify pin 12 connections and audio system state
- **Wiring**: Ensure Right channel → 1K resistor → Pin 12, Left channel + Ground → Ground
- **Timing Problems**: Ensure `audioHook()` is called consistently for Mozzi effects

### Effect Switching
- **Stuck on One Effect**: Check sensor readings and hand detection logic
- **Rapid Switching**: Verify 5-second rotation interval timing
- **Memory Issues**: Monitor available RAM, especially with DJ Scratch buffer

### LED Issues
- **Wrong Colors**: Verify effect color mapping in `effectColors[]` array
- **Timing Lag**: Check LED update interval (33ms) and FastLED show() calls

## Future Enhancements

1. **PROGMEM Audio**: Replace DJ Scratch simulation with actual PROGMEM audio data
2. **Effect Customization**: Add parameters for vibrato depth, modulation rates, etc.
3. **Touch Integration**: Add capacitive touch sensors for direct effect selection
4. **MIDI Output**: Add MIDI note output for external synthesizer control
5. **WiFi Control**: Add ESP32 support for wireless effect control

## Research Sources

- **Mozzi Documentation**: https://sensorium.github.io/Mozzi/
- **Timer1 ISR Techniques**: Arduino Timer1 interrupt programming
- **Audio Physics**: Sine wave synthesis and FM modulation principles
- **Theremin Design**: Classic theremin frequency mapping and control techniques 