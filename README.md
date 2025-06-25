# Solar Shrine Interactive Installation

An interactive installation using Arduino ultrasonic sensors, LED lighting effects, audio feedback, and TouchDesigner for visual effects. The installation features dual-mode lighting (attract/interactive), hand-proximity detection, and haptic audio feedback.

## Project Structure

```
solar-shrine-arduino/
├── arduino/                           # Arduino code variants
│   ├── sensors_fastled_effects.ino   # Advanced FastLED with attract/interactive modes
│   ├── sensors_ws2815_effects.ino    # WS2815 advanced lighting effects
│   ├── speaker_test.ino              # Audio hardware testing & integration
│   ├── sensor_serial_stepper.ino     # Motor control integration
│   ├── sensors_string_led_output.ino # Simple LED control (no libraries)
│   ├── sensors_string_output.ino     # Basic sensor output
│   └── sensors_json_output/          # JSON output variants
│       ├── sensors_json_output.ino   # JSON + audio integration
│       └── sensors_ws2815.ino        # Human hand LED mapping (83 LEDs)
├── touchdesigner/                     # TouchDesigner files and scripts
│   ├── scripts/
│   │   ├── sensor_parser.py          # Python script for parsing sensor data
│   │   └── movie_integration_parser.py # Movie integration parser
│   ├── Po Tolo V2.5.toe              # Current TouchDesigner project
│   └── solar_shrine_setup.txt        # Setup instructions for new projects
└── README.md                          # This file
```

## Hardware Setup

### Required Components

#### Core Electronics
- **Arduino Uno, Nano, or similar** (5V/16MHz recommended)
- **2x HC-SR04 ultrasonic sensors** (for hand detection)
- **USB cable** for Arduino connection and power

#### LED Lighting System
- **WS2812B or WS2815 LED strip** (20-83 LEDs depending on configuration)
  - WS2812B: 5V addressable LEDs
  - WS2815: 12V addressable LEDs (higher power, more stable)
- **Appropriate power supply** for LED strip (calculate ~20mA per LED)

#### Audio System (Optional)
- **WWZMDiB XH-M543 High Power Digital Amplifier Board**
  - TPA3116D2 chipset
  - Dual Channel 2×120W output
  - 12-24V DC power input
- **Dayton Audio DAEX32QMB-4 Quad Feet Mega Bass 32mm Exciter**
  - 40W RMS, 4 Ohm impedance
  - Haptic feedback through surface vibration
  - Frequency response: 80Hz - 20kHz

#### Motor Control (Optional)
- **L298N Motor Driver**
- **DC Motor** (for physical installation effects)

### Pin Configuration

**Standard pin assignments across all sketches:**

| Component | Arduino Pin | Notes |
|-----------|-------------|-------|
| **Ultrasonic Sensor 1 (Left)** | | |
| - Trigger | Digital 9 | Output signal |
| - Echo | Digital 10 | Input signal |
| **Ultrasonic Sensor 2 (Right)** | | |
| - Trigger | Digital 5 | Output signal |
| - Echo | Digital 6 | Input signal |
| **LED Strip Data** | Digital 3 | FastLED data pin |
| **Speaker/Amplifier** | Digital 11 | PWM output for audio |
| **Motor Control (L298N)** | | |
| - IN3 | Digital 4 | Motor direction |
| - IN4 | Digital 5 | Motor direction |
| - ENB | Digital 11 | Motor enable (PWM) |

### Power Requirements

- **Arduino**: 5V via USB or 7-12V via barrel jack
- **Sensors**: 5V from Arduino (low current)
- **WS2812B LEDs**: 5V, ~20mA per LED
- **WS2815 LEDs**: 12V, ~20mA per LED  
- **Audio Amplifier**: 12-24V DC, up to 10A for full power
- **Motor Driver**: 7-12V DC

## Arduino Code Variants

### 1. sensors_fastled_effects.ino ⭐ **RECOMMENDED**
**Advanced dual-mode lighting system with hand detection averaging**

- **Attract Mode**: Sinusoidal yellow-red fade (5-second period)
- **Interactive Mode**: Distance-based red→orange→yellow colors
- **Hand Detection**: 5-sample averaging to filter wind/dust
- **Smooth Transitions**: Trigonometric phase matching
- **TouchDesigner Integration**: JSON output with orange correlation values

**Features:**
- 10-second timeout returns to attract mode
- Maintains colors when hands move away
- Correlated color values for TouchDesigner effects
- Split LED strip control (left/right sensors)

**Hardware:** WS2812B/WS2815 LED strips, HC-SR04 sensors
**Libraries:** FastLED, ArduinoJson

### 1.1. sensors_fastled_theremin.ino ⭐ **NEW - THEREMIN INTEGRATION**
**Complete theremin + lighting system combining all features**

- **All Features from sensors_fastled_effects.ino**
- **Theremin Sound Generation**: Distance-based frequency control
- **Dual Sensor Audio**: Left hand = bass, Right hand = treble
- **Harmonic Effects**: Special vibrato when both hands detected
- **Timer Compatibility**: Uses NewTone library to avoid conflicts
- **Professional Audio**: Works with WWZMDiB XH-M543 + Dayton Audio exciter

**Audio Features:**
- Frequency range: 80Hz - 2kHz
- Smooth frequency transitions with anti-aliasing
- Musical startup sequence
- Optimized for haptic feedback through exciter

**Hardware:** All previous components + audio amplifier system
**Libraries:** FastLED, ArduinoJson, NewTone, NewPing

### 2. sensors_json_output/sensors_json_output.ino
**Audio feedback integration with sensor detection**

- **Speaker Integration**: WWZMDiB XH-M543 + Dayton Audio DAEX32QMB-4
- **Beep System**: 1kHz tones when hands detected
- **JSON Output**: Structured data for TouchDesigner
- **LED Effects**: Basic color interpolation based on sensor difference

**Features:**
- Frequency-based beep generation (no tone library required)
- Beep rate limiting (500ms minimum interval)
- Hand detection with color mapping
- Startup audio test

**Hardware:** Audio amplifier, exciter speaker, LED strip, sensors
**Libraries:** FastLED, ArduinoJson

### 3. sensors_json_output/sensors_ws2815.ino
**Human hand LED mapping for 83-LED installation**

- **Hand-Shaped LED Layout**: 6 strips mapped to fingers and palm
- **Advanced Effects**: Hand wave and finger wave animations
- **Power Management**: Optimized for 83 LEDs on 12V WS2815
- **Proximity Mapping**: Different effects based on left/right hand proximity

**LED Mapping:**
- Thumb: LEDs 0-12 (13 LEDs)
- Index: LEDs 13-27 (15 LEDs)  
- Middle: LEDs 28-43 (16 LEDs)
- Ring: LEDs 44-57 (14 LEDs)
- Pinky: LEDs 58-69 (12 LEDs)
- Palm: LEDs 70-82 (13 LEDs)

**Hardware:** WS2815 LED strips (83 LEDs), 12V power supply
**Libraries:** FastLED, ArduinoJson

### 4. sensors_ws2815_effects.ino
**Advanced WS2815 lighting effects with multiple animation modes**

- **Multiple Effect Modes**: Rainbow, sparkle, wave effects
- **Split Strip Control**: Independent left/right sensor responses
- **Breathing Effects**: Proximity-based intensity
- **Special Modes**: Rainbow when both hands very close (<5cm)

**Hardware:** WS2815 LED strips, HC-SR04 sensors
**Libraries:** FastLED, ArduinoJson

### 5. speaker_test.ino
**Comprehensive audio hardware testing**

- **Multi-Frequency Testing**: 100Hz, 1kHz, 2kHz test tones
- **Hardware Validation**: Pin state verification
- **Amplifier Testing**: Square wave generation for haptic feedback
- **Serial Diagnostics**: Step-by-step test results

**Use for:** Initial setup and troubleshooting of audio hardware

### 6. sensor_serial_stepper.ino
**Motor control integration for physical installations**

- **L298N Motor Control**: Precise motor driver integration
- **Binary Output**: Simple 0/1 values for TouchDesigner
- **Hand Detection**: Both sensors must detect for motor activation

**Hardware:** L298N motor driver, DC motor
**Libraries:** None required

### 7. sensors_string_led_output.ino
**Lightweight LED control without libraries**

- **No External Libraries**: Pure Arduino code
- **Simple LED Control**: On/off based on dual hand detection
- **Low Memory**: Minimal resource usage

**Hardware:** Basic LED strip or single LED
**Libraries:** None required

### 8. sensors_string_output.ino
**Basic sensor output for testing**

- **Raw Sensor Data**: Simple space-separated values
- **Minimal Code**: Essential sensor reading only
- **Testing Tool**: Verify sensor functionality

**Output Format:** `distance1 distance2 hands_detected`

## Setup Instructions

### Arduino Setup

1. **Install Required Libraries** (depending on sketch):
   ```
   Arduino IDE → Tools → Manage Libraries
   Search "FastLED" → Install latest version
   Search "ArduinoJson" → Install v7.x
   Search "NewPing" → Install latest version
   ```
   
   **For Theremin Integration** (sensors_fastled_theremin.ino):
   - Download **NewTone Library** manually from: [BitBucket NewTone](https://bitbucket.org/teckel12/arduino-new-tone/downloads/)
   - Extract and place in Arduino Libraries folder
   - **Why NewTone?** Avoids timer conflicts with ultrasonic sensors (proven in [Theremino project](https://projecthub.arduino.cc/tdelatorre/theremino-a-theremin-made-with-arduino-3e661f))

2. **Hardware Connections**:
   - Connect sensors to pins 5,6,9,10 as per pin configuration table
   - Connect LED strip data to pin 3
   - Connect audio output to pin 11 (if using speaker)
   - Ensure proper power supply for all components

3. **Upload Code**:
   - Choose appropriate sketch for your hardware configuration
   - **For beginners**: Start with `sensors_fastled_effects.ino`
   - **For audio**: Use `sensors_json_output/sensors_json_output.ino`
   - **For testing**: Use `speaker_test.ino` to verify audio hardware

4. **Test Setup**:
   - Open Serial Monitor (9600 baud)
   - Verify sensor readings and JSON output
   - Test LED effects by moving hands over sensors

### TouchDesigner Setup

1. **Open Project**: `touchdesigner/Po Tolo V2.5.toe`
2. **Configure Serial Connection**:
   - Set correct COM port in Serial DAT
   - Verify 9600 baud rate matches Arduino
3. **Use Parser Scripts**: Reference `scripts/sensor_parser.py` for JSON parsing
4. **Effect Mapping**: Use orange correlation values for synchronized effects

## Hardware Wiring Diagrams

### Basic Sensor + LED Setup
```
Arduino Uno        HC-SR04 (Left)    HC-SR04 (Right)    LED Strip
Pin 9        -->   Trig
Pin 10       -->   Echo  
Pin 5        -->                     Trig
Pin 6        -->                     Echo
Pin 3        -->                                        Data In
5V           -->   VCC               VCC                5V/12V+
GND          -->   GND               GND                GND
```

### Audio Integration Setup
```
Arduino Pin 11 --> WWZMDiB XH-M543 Input
XH-M543 Output --> Dayton Audio DAEX32QMB-4 Exciter
12-24V Supply  --> XH-M543 Power Input
```

## Detection Parameters

### Sensor Range
- **Minimum Distance**: 1cm (objects closer may not detect reliably)
- **Maximum Distance**: 20cm (effective hand detection range)
- **Update Rate**: ~20Hz (50ms delay between readings)

### Hand Detection Logic
- **Single Hand**: Triggers interactive mode immediately
- **Dual Hand**: Enhanced effects and special modes
- **Averaging**: 5-sample rolling average prevents false triggers
- **Timeout**: 10-second return to attract mode

### Color Mapping (Interactive Mode)
- **Far (20cm)**: Full Red `(255, 0, 0)`
- **Close (1cm)**: Full Yellow `(255, 255, 0)`
- **Mid-range**: Proportional Orange `(255, green_value, 0)`
- **TouchDesigner Values**: 0.0 (red/far) to 1.0 (yellow/close)

## Troubleshooting

### Common Issues
1. **No LED Response**: Check power supply, data pin connection, LED strip type
2. **Erratic Sensor Readings**: Ensure stable 5V supply, check wiring
3. **No Audio Output**: Verify amplifier power, speaker connections, pin 11 output
4. **TouchDesigner Not Receiving Data**: Check COM port, baud rate, JSON format

### Testing Steps
1. Run `speaker_test.ino` to verify audio hardware
2. Use Serial Monitor to verify sensor readings
3. Test with `sensors_string_output.ino` for basic functionality
4. Gradually upgrade to more complex sketches

## Development Notes

- All sketches use **9600 baud rate** for serial communication
- **50ms delay** between sensor readings for smooth operation  
- **JSON format** provides maximum flexibility for TouchDesigner integration
- **FastLED library** required for advanced lighting effects
- **Power management** critical for LED strips >20 LEDs 