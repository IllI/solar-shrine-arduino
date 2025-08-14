# Alternative: 2PIN_PWM Mode with Custom Pin Mapping

If single-pin PWM doesn't work well, we can try 2PIN_PWM mode but remap the pins.

According to the forum discussion and documentation:
- Default Arduino Mega 2PIN_PWM uses pins 11 (high bits) and 12 (low bits)  
- We need to avoid pin 11 (left sensor echo)
- We could potentially remap to use pins 12 (high bits) and 13 (low bits)

Configuration would be:
```cpp
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_2PIN_PWM
#define MOZZI_AUDIO_PIN_1 12          // High bits
#define MOZZI_AUDIO_PIN_1_LOW 13      // Low bits  
#define MOZZI_AUDIO_RATE 16384
#define MOZZI_PWM_RATE 125000         // Higher rate for 2PIN mode
#define CONTROL_RATE 128
```

This would give us the same high-quality 14-bit audio as the working sketch, but on pins 12+13 instead of 11+12.

Hardware setup would need two resistors (3.9k and 499k) as shown in Mozzi documentation.
