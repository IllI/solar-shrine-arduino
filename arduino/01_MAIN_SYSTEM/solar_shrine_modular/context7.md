# Arduino Mega 2560 Hardware Context for Solar Shrine Audio Refactoring

## Hardware Platform
- **Target Platform:** Arduino Mega 2560
- **Custom Shield:** Red prototyping shield with JST connectors
- **Audio Output:** Pin 12 with 1KΩ current limiting resistor

## Pin Assignments (From CUSTOM_SHIELD_WIRING.md)

### Audio System
- **Pin 12:** Audio output (with 1KΩ resistor to amplifier)
- **Mozzi Configuration:** 2-pin PWM mode (pins 11 & 12 on Mega)

### Sensor System
- **Left Ultrasonic Sensor (HC-SR04):**
  - Trigger Pin: 10
  - Echo Pin: 11
- **Right Ultrasonic Sensor (HC-SR04):**
  - Trigger Pin: 5
  - Echo Pin: 6

### LED System
- **Pin 3:** WS2812B LED strip data line

### Power Distribution
- **5V Rail:** Powers all sensors and LED strip
- **GND Rail:** Common ground for all components

## Audio Architecture Considerations

### DJ Scratch Mode (PROGMEM)
- Uses Timer1 PWM on Pin 12 (OC1B)
- Direct hardware control via ISR
- No Mozzi dependency

### Mozzi Modes (Alien, Robots, Theremin)
- Requires Mozzi library initialization
- Uses pins 11 & 12 for 2-pin PWM output
- Needs audioHook() calls in main loop

### Pin Conflict Resolution
- Pin 11 shared between Mozzi and left sensor echo
- Requires careful mode switching to avoid conflicts
- DJ Scratch mode must disable Mozzi completely
- Mozzi modes must not interfere with Timer1 setup

## Hardware Wiring Summary
```
Arduino Mega 2560 Pin Assignments:
┌─────────────────────────────────────────┐
│  Pin │ Function        │ Connection     │
├──────┼─────────────────┼────────────────┤
│  5   │ Right Sensor    │ HC-SR04 Trig   │
│  6   │ Right Sensor    │ HC-SR04 Echo   │
│  10  │ Left Sensor     │ HC-SR04 Trig   │
│  11  │ Left Sensor     │ HC-SR04 Echo   │
│  12  │ DJ/Mozzi Audio  │ PWM Output      │
│  13  │ Mozzi Audio     │ PWM Output      │
│  5V  │ Power Rail      │ All Components │
│  GND │ Ground Rail     │ All Components │
└─────────────────────────────────────────┘
```

# Working Audio System Reference (from sound_test)

## Successful Implementation from MODULAR_AUDIO_SYSTEM_GUIDE.md

### Key Success Factors:
1. **Complete Audio System Isolation** - No timer conflicts between Timer1 (DJ scratch) and Mozzi's internal timers
2. **Proper audioHook() Timing** - Called every loop iteration when in Mozzi mode
3. **Hardware Pin Isolation** - Pin 11 exclusively for sensor, Pin 12 shared with mode isolation
4. **Executive Intelligence System** - Proper mode switching with complete teardown/setup

### Critical Implementation Details:

#### audioHook() Timing (CRITICAL)
```cpp
void loop() {
  // Mode switching logic...
  
  // CRITICAL: audioHook() must be called EVERY loop iteration for Mozzi
  if (currentMode == MODE_MOZZI_ALIEN) {
    audioHook();  // Called unconditionally when in Mozzi mode
  }
}
```

#### Mozzi Function Structure
```cpp
void updateControl() {
  // Only process if we're in Mozzi mode
  if (currentMode == MODE_MOZZI_ALIEN) {
    // Direct sensor reading and control processing
  }
}

int updateAudio() {
  // Only generate audio if we're in Mozzi mode
  if (currentMode == MODE_MOZZI_ALIEN) {
    // Audio generation with vibrato
    return sample;
  }
  return 0; // Silence when not in Mozzi mode
}
```

#### Timer1 ISR Protection
```cpp
ISR(TIMER1_COMPB_vect) {
  if (currentMode != MODE_DJ_SCRATCH) {
    OCR1B = 200; // Silence when not in DJ mode
    return;
  }
  // DJ scratch audio processing...
}
```

### Pin Configuration Resolution:
- **DJ Scratch Mode**: Pin 12 only (Timer1 OC1B)
- **Mozzi Modes**: Pins 12+13 (2-pin PWM mode)
- **Left Sensor**: Pin 11 exclusively for echo input
- **Complete hardware isolation** eliminates software workarounds

### Mode Switching Architecture:
```cpp
enum AudioMode {
  MODE_DJ_SCRATCH = 0,
  MODE_MOZZI_ALIEN = 1
};

// Complete Mode Isolation Functions
void setupDJScratch()    // Configure Timer1, enable ISR
void disableDJScratch()  // Disable Timer1, reset pins
void setupMozziAlien()   // Start Mozzi, initialize oscillators
void disableMozziAlien() // Stop Mozzi, disable timers
```│  Pin │ Function        │ Connection     │
├──────┼─────────────────┼────────────────┤
│  3   │ LED Data        │ WS2812B Strip  │
│  5   │ Right Sensor    │ HC-SR04 Trig   │
│  6   │ Right Sensor    │ HC-SR04 Echo   │
│  10  │ Left Sensor     │ HC-SR04 Trig   │
│  11  │ Left Sensor     │ HC-SR04 Echo   │
│  12  │ Audio Output    │ 1KΩ → Amplifier│
│  5V  │ Power Rail      │ All Components │
│  GND │ Ground Rail     │ All Components │
└─────────────────────────────────────────┘
```

This context ensures the audio refactoring maintains compatibility with the existing Arduino Mega hardware setup and custom shield wiring.