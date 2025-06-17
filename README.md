# Solar Shrine Interactive Installation

An interactive installation using Arduino ultrasonic sensors and TouchDesigner for visual effects.

## Project Structure

```
solar-shrine-arduino/
├── arduino/                           # Arduino code variants
│   ├── sensors_json_output.ino       # JSON output for TouchDesigner (recommended)
│   ├── sensors_string_output.ino     # Simple space-separated values output
│   └── sensor_serial_stepper.ino     # Original version with motor control
├── touchdesigner/                     # TouchDesigner files and scripts
│   ├── scripts/
│   │   └── sensor_parser.py          # Python script for parsing sensor data
│   ├── Po Tolo V2.5.toe              # Current TouchDesigner project
│   └── solar_shrine_setup.txt        # Setup instructions for new projects
└── README.md                          # This file
```

## Hardware Setup

### Required Components
- 2x HC-SR04 ultrasonic sensors
- Arduino board (Uno, Nano, etc.)
- USB cable for Arduino connection
- Optional: L298N motor driver + DC motor (for stepper version)

### Pin Configuration
- **Sensor 1 (Left)**: TRIG=9, ECHO=10
- **Sensor 2 (Right)**: TRIG=5, ECHO=6
- **Motor Control** (stepper version only): IN3=4, IN4=5, ENB=11

## Arduino Code Variants

### 1. sensors_json_output.ino (Recommended)
- Outputs structured JSON data
- Includes hand detection logic
- Best for TouchDesigner integration
- **Requires**: ArduinoJson library

**Output format:**
```json
{"left":15,"right":8,"hands_detected":true}
```

### 2. sensors_string_output.ino
- Simple space-separated output
- Raw distance values only
- Lightweight alternative

**Output format:**
```
15 8
```

### 3. sensor_serial_stepper.ino
- Original version with motor control
- Outputs binary values (0/1)
- Controls physical motor based on sensor input

## Setup Instructions

### Arduino Setup
1. **Install required libraries** (for JSON version):
   ```
   Arduino IDE → Tools → Manage Libraries → Search "ArduinoJson" → Install v7.x
   ```

2. **Upload the code**:
   - Choose `sensors_json_output.ino` for TouchDesigner integration
   - Note the COM port number for TouchDesigner connection

3. **Test the output**:
   - Open Serial Monitor (9600 baud)
   - Verify sensor readings appear correctly

### TouchDesigner Setup
1. **Open the project**: `touchdesigner/Po Tolo V2.5.toe`
2. **Configure Serial connection**:
   - Set correct COM port in Serial DAT
   - Verify 9600 baud rate
3. **Use the parser script**: Reference `scripts/sensor_parser.py` for JSON parsing

## Sensor Data Processing

### Detection Range
- **Minimum**: 1cm
- **Maximum**: 20cm
- **Hand Detection**: Both sensors must detect objects within range

### TouchDesigner Integration
The `sensor_parser.py` script provides:
- JSON parsing with error handling
- Parameter mapping to visual effects
- Value normalization (0-1 range)
- Custom effect triggers

## Visual Effects Mapping
- **Left Sensor**: Controls left-side visual intensity
- **Right Sensor**: Controls right-side visual intensity  
- **Hands Detected**: Triggers special combined effects
- **Out of Range**: Default/idle state

## Development Notes
- All Arduino sketches use 9600 baud rate
- 10ms delay between sensor readings
- JSON version provides most flexibility for complex effects
- Motor control version available for physical installations 