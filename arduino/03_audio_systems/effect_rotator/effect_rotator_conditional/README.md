# Effect Rotator - Ultra Minimal

## Arduino IDE Compatible Structure ✅
- Folder: `effect_rotator_conditional/`
- Sketch: `effect_rotator_conditional.ino`
- Audio Data: `audio_data.h` (for DJ Scratch effect)

## Memory Optimization
**ULTRA MINIMAL** - Removed all non-essential libraries:
- ❌ No FastLED (no LED support)
- ❌ No ArduinoJson (no JSON output)
- ❌ No NewPing (direct pulseIn() instead)
- ❌ No RollingAverage (direct frequency setting)

## How to Use
1. **Open in Arduino IDE**: File → Open → Select `effect_rotator_conditional.ino`
2. **Choose Effect**: Edit the `#define` lines at the top:
   ```cpp
   // UNCOMMENT EXACTLY ONE:
   #define EFFECT_DJ_SCRATCH           // Should fit now!
   // #define EFFECT_MUSICAL_THEREMIN  // Mozzi only
   // #define EFFECT_ALIEN_SOUND       // Timer ISR only
   // #define EFFECT_ROBOTS_TALKING    // Timer ISR only
   ```
3. **Compile and Upload**: Minimal memory usage!

## Effects (Audio Only)
- **DJ Scratch**: PROGMEM audio with scratch controls
- **Musical Theremin**: Basic Mozzi synthesis
- **Alien Sound**: FM synthesis 
- **Robots Talking**: Square wave patterns

## Hardware
- **Sensors**: HC-SR04 on pins 5,6,10,11
- **Audio**: Pin 9 output
- **Range**: 1-20cm hand detection
- **LEDs**: REMOVED to save memory 