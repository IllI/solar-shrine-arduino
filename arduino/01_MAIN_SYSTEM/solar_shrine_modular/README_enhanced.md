# Solar Shrine - Modular Theremin System

An interactive art installation combining dual-mode LED lighting, ultrasonic sensor detection, and a `tone()`-based theremin for Arduino.

## Features

### Audio System
- **`tone()`-based Theremin**: Simple and reliable audio output using the standard Arduino `tone()` function.
- **Pitch Control**: Left sensor controls the pitch of the theremin.

### Dual-Mode LED System
- **Attract Mode**: Smooth color transitions when no hands detected
- **Interactive Mode**: Real-time hand position visualization
- **Performance Optimized**: ~500Hz attract mode, ~125Hz interactive mode

### Advanced Sensor Processing
- **Dual HC-SR04 Sensors**: Left and right hand detection
- **Averaging Algorithm**: 5-sample moving average for stability
- **Range Mapping**: 1-20cm detection range with smooth transitions

## Hardware Requirements

### Arduino Mega 2560
- **Sensors**: 2x HC-SR04 ultrasonic sensors
  - Left sensor: Trigger pin 10, Echo pin 11
  - Right sensor: Trigger pin 5, Echo pin 6
- **LEDs**: WS2812B/WS2815 LED strip on pin 3 (120 LEDs)
- **Audio**: Pin 12 → Amplifier input
- **Amplifier**: WWZMDiB XH-M543 + Dayton Audio DAEX32QMB-4 exciter

### Required Libraries
```arduino
#include <ArduinoJson.h>  // JSON communication
#include <FastLED.h>      // LED control
#include <NewPing.h>      // Ultrasonic sensors

```

## Installation

1. **Install Required Libraries**:
   - ArduinoJson (latest version)
   - FastLED (latest version)
   - NewPing (latest version)

2. **Upload Code**:
   - Open `solar_shrine_modular.ino` in Arduino IDE
   - Select "Arduino Mega 2560" as board
   - Upload to your Arduino

3. **Hardware Connections**:
   ```
   HC-SR04 Left:  Trig=10, Echo=11, VCC=5V, GND=GND
   HC-SR04 Right: Trig=5,  Echo=6,  VCC=5V, GND=GND
   LED Strip:     Data=3,  VCC=5V,  GND=GND
   Audio Out:     Pin 12 → 1K resistor → Amplifier
   ```

## Audio Control

### Sensor Control Mapping
- **Left Sensor (1-20cm)**: Controls pitch/frequency.
- **Right Sensor (1-20cm)**: Currently unused for audio, but contributes to hand detection for LED mode.

## System Architecture

### Modular Design
- **AudioEffects.h**: Defines effect types and state structures
- **theremin.h/.cpp**: `tone()`-based theremin implementation.
- **solar_shrine_modular.ino**: Main system integration

### Performance Optimizations
- **`tone()` Function**: Uses built-in timer interrupts for audio generation, simplifying the code.
- **Staggered Processing**: Heavy operations distributed across cycles.
- **Sensor Caching**: Reduced blocking time for smooth LED updates.

## JSON Output

The system outputs real-time data via Serial (9600 baud) for TouchDesigner integration:

```json
{
  "left": 15,
  "right": 8,
  "hands_detected": true,
  "mode": "interactive",
  "left_in_range": true,
  "right_in_range": true
}
```

## Configuration

### Range Settings
```cpp
const float MIN_RANGE = 1.0;   // Minimum detection distance (cm)
const float MAX_RANGE = 20.0;  // Maximum detection distance (cm)
```

### LED Configuration
```cpp
#define NUM_LEDS 120           // Number of LEDs in strip
#define LED_TYPE WS2812B       // LED strip type
FastLED.setBrightness(150);    // LED brightness (0-255)
```



## Troubleshooting

### Common Issues
1. **No Audio Output**: Verify pin 12 connection and amplifier.
2. **Erratic Sensor Readings**: Ensure sensors have clear line of sight
3. **LED Flickering**: Check power supply capacity (5V, 2A minimum)


### Debug Output
Enable Serial Monitor (9600 baud) to view:

- Sensor readings
- JSON data stream
- System status updates

## Customization

### Modifying Theremin Pitch Range
Edit the `map()` function call within `theremin_update()` in `theremin.cpp` to change the output frequency range.

### Adjusting LED Patterns
Modify `getInteractiveColor()` and `getAttractColor()` functions for different color schemes.

## Performance Notes

- **`tone()` Audio**: Relies on standard Arduino library for audio.
- **LED Updates**: Optimized timing prevents audio glitches.
- **Sensor Processing**: Non-blocking algorithms maintain responsiveness.

## License

Open source - feel free to modify and distribute.

## Version History

- **v3.0**: Refactored to use `tone()` for robust audio output and simplified the modular structure.
- **v2.0**: Enhanced modular system with multiple theremin variants.
- **v1.0**: Original dual-mode LED and basic audio system.

---

*For technical support or questions, refer to the inline code documentation.*