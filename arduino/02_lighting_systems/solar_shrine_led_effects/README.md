# Solar Shrine LED Effects

## Overview
Standalone LED effects system extracted from `solar_shrine_theremin.ino` for testing and experimentation with lighting patterns. This system features dual-mode lighting: attract mode and interactive mode.

## Hardware Requirements
- **Arduino Uno/Nano** (or compatible)
- **2x HC-SR04 ultrasonic sensors**
  - Left sensor: Trig pin 10, Echo pin 11
  - Right sensor: Trig pin 5, Echo pin 6
- **WS2812B/WS2815 LED strip** (pin 3)
- **Power supply** appropriate for your LED strip length

## Libraries Required
```
FastLED
ArduinoJson
NewPing
```

## Features

### Three-Mode System
1. **Attract Mode** (Default)
   - Sinusoidal fade from yellow to red across all LEDs
   - 5-second cycle period
   - Smooth color transitions
   - Automatically activated when idle timeout expires

2. **Interactive Mode**
   - Activates when hands are detected within 1-20cm range
   - Left half responds to left sensor (pin 10/11)
   - Right half responds to right sensor (pin 5/6)
   - Color mapping: Far = Red, Close = Yellow (through orange)
   - Updates colors in real-time based on hand position

3. **Idle Mode**
   - Activated when hands are removed from Interactive Mode
   - Maintains the last interactive colors (frozen effect)
   - Lasts for 10 seconds before returning to Attract Mode
   - Preserves the lighting state for smooth transitions

### Smart Features
- **Hand Detection Averaging**: Uses 5-sample rolling average to prevent false readings
- **False Detection Prevention**: Requires 3 consecutive detections to switch from attract to interactive mode
- **Color Persistence**: Colors persist for 1 second after hand detection loss to handle involuntary movements
- **Smooth Transitions**: Calculates phase offset for seamless mode switching
- **Startup Sequence**: Rainbow sweep and fade on boot

## Configuration

### LED Strip Settings
```cpp
#define LED_PIN 3
#define NUM_LEDS 120        // Adjust to your strip length
#define LED_TYPE WS2812B   
#define COLOR_ORDER GRB    
```

### Detection Range
```cpp
const float MIN_RANGE = 1.0;   // Minimum detection distance (cm)
const float MAX_RANGE = 20.0;  // Maximum detection distance (cm)
```

### Timing
```cpp
const unsigned long INTERACTIVE_TIMEOUT = 10000;     // 10 seconds (idle timeout)
const float ATTRACT_PERIOD = 5000.0;                 // 5 seconds (attract cycle)
const unsigned long COLOR_PERSIST_TIMEOUT = 1000;    // 1 second (color persistence)
```

## Usage

1. **Upload Code**: Open `solar_shrine_led_effects.ino` in Arduino IDE and upload
2. **Monitor Serial**: Open Serial Monitor at 9600 baud to see JSON sensor data
3. **Test Modes**: 
   - Watch attract mode cycle through colors
   - Wave hands near sensors to trigger interactive mode
   - Remove hands to see idle mode (frozen interactive colors)
   - Wait 10 seconds in idle mode to return to attract mode

## JSON Output
The system outputs JSON data for monitoring and integration:
```json
{
  "left": 15,
  "right": 8,
  "hands_detected": false,
  "mode": "idle",
  "left_in_range": false,
  "right_in_range": false,
  "left_orange_value": 0.7,
  "right_orange_value": 0.4
}
```

**Mode Values**: `"attract"`, `"interactive"`, `"idle"`

## Customization Ideas

### Color Schemes
- Modify `getInteractiveColor()` for different color mappings
- Change `getAttractColor()` for different attract patterns
- Add HSV color space for smoother transitions

### Effects
- Add breathing effects in attract mode
- Implement sparkle or twinkle effects
- Create wave patterns across the strip
- Add multiple attract mode patterns

### Sensor Response
- Adjust averaging sample count
- Modify detection range
- Change false detection thresholds
- Add exponential smoothing
- Modify `COLOR_PERSIST_TIMEOUT` for different color persistence durations

## Troubleshooting

### No LED Response
- Check LED strip power supply
- Verify LED_PIN and NUM_LEDS settings
- Test with simple FastLED examples

### Sensor Issues
- Verify sensor wiring (VCC, GND, Trig, Echo)
- Check sensor range (1-20cm optimal)
- Monitor Serial output for sensor readings

### Mode Switching Problems
- Adjust false detection count threshold
- Modify timeout values
- Check sensor placement for interference

### Color Persistence Issues
- Colors may persist for 1 second after hand removal (by design)
- Adjust `COLOR_PERSIST_TIMEOUT` if colors stick too long or not long enough
- Monitor Serial output to debug color update timing

## Integration
This system can be easily integrated with:
- TouchDesigner (via JSON serial output)
- Audio systems (sensor data correlation)
- Other Arduino modules (shared sensor data)
- Web interfaces (via serial bridge)

---

*Extracted from solar_shrine_theremin.ino for standalone LED testing and experimentation.* 