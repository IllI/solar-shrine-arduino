# Solar Shrine - Modular Integration System

## 🎯 **Overview**

The **Solar Shrine Modular System** is the main integration hub that combines the best features from both the theremin-based lighting system and the DJ scratch audio system. This modular architecture is designed for easy expansion and customization of future functionality.

## 🚀 **Features**

### **Core Systems**
- ✅ **Dual-Mode LED Lighting**: Attract (sinusoidal fade) + Interactive (distance-based colors)
- ✅ **Advanced Hand Detection**: 5-sample averaging with false detection filtering
- ✅ **DJ Scratch Audio**: Left hand = Play/Pause, Right hand = Scratch control
- ✅ **TouchDesigner Integration**: Enhanced JSON output with audio state
- ✅ **Modular Architecture**: Clean separation of systems for easy expansion

### **Audio System**
- **Crystal Clear Playback**: 20kHz PWM with 4x amplification
- **Real-Time Scratching**: Variable speed forward/backward control
- **Gesture Control**: Hand movements trigger different audio modes
- **Flash Storage**: 26KB audio stored in PROGMEM (no SD card needed)

### **LED System**
- **Attract Mode**: Yellow↔Red sinusoidal fade (5-second period)
- **Interactive Mode**: Distance-based Red→Orange→Yellow colors per sensor
- **Smooth Transitions**: Phase-matched switching between modes
- **Dual-Zone Control**: Left/right halves respond to respective sensors

## 🔧 **Hardware Configuration**

### **Arduino Mega 2560 Pin Layout**
| Component | Pin | Function | Notes |
|-----------|-----|----------|-------|
| **Left Sensor** | 10 | Trigger | HC-SR04 ultrasonic |
| **Left Sensor** | 11 | Echo | HC-SR04 ultrasonic |
| **Right Sensor** | 5 | Trigger | HC-SR04 ultrasonic |
| **Right Sensor** | 6 | Echo | HC-SR04 ultrasonic |
| **LED Strip** | 3 | Data | WS2812B/WS2815 |
| **Audio Output** | 12 | PWM | Timer1 OC1B → 1K resistor → Amp |

### **Audio Hardware Stack**
- **Arduino Mega 2560**: Pin 12 PWM generation (Timer1 OC1B)
- **Wiring**: Right channel → 1K resistor → Pin 12, Left channel + Ground → Ground
- **Amplifier**: WWZMDiB XH-M543 (TPA3116D2 chip, 2x120W)
- **Exciter**: Dayton Audio DAEX32QMB-4 (40W 4Ω)

## 🎮 **Control Scheme**

### **Left Hand (Play/Pause Control)**
- **Wave over left sensor**: Toggle audio playback
- **Audio starts**: From beginning of sample
- **Visual feedback**: Serial monitor shows "AUDIO: PLAY" / "AUDIO: STOP"

### **Right Hand (Scratch Control)**
- **Quick movements**: Trigger scratch mode (forward/backward)
- **Sustained presence**: Variable speed control based on distance
- **No movement**: Return to normal playback speed
- **Scratch intensity**: More rapid movements = faster scratching

### **Combined Control**
- **Both hands**: Independent LED control + audio interaction
- **Distance mapping**: Close = bright colors, Far = dim colors
- **Mode switching**: Auto-switch between attract and interactive modes

### **Effect Rotation**
- Rotation cycles through `dj_scratch → alien → robots → mini_theremin` whenever no hands are detected for 5 seconds.
- Rotation resumes after returning to attract mode.
- You can change the rotation order in `nextEffect(...)` inside `solar_shrine_modular.ino`.

### **Testing Flags**
- `DISABLE_EFFECT_ROTATION` (bool): set to `false` to enable rotation (default now).
- `TEST_ALIEN_ONLY` (bool): when `true`, locks to the Alien effect for audio testing. Default is `false`.

## 📊 **System Architecture**

### **Modular Design**
```cpp
// =============================================================================
// HARDWARE CONFIGURATION      - Pin definitions and hardware setup
// SYSTEM CONSTANTS            - Configurable parameters
// SYSTEM STATE VARIABLES      - Runtime state management
// AUDIO SYSTEM (DJ SCRATCH)   - Audio generation and control
// SENSOR SYSTEM               - Distance measurement and averaging
// LED SYSTEM                  - Lighting effects and color management
// AUDIO CONTROL SYSTEM        - Gesture-based audio control
// JSON OUTPUT SYSTEM          - TouchDesigner integration
// MAIN SYSTEM SETUP AND LOOP  - Main program flow
// =============================================================================
```

### **Key Functions**
- `setupAudioSystem()`: Initialize Timer1 PWM for audio
- `updateAudioControl()`: Process hand gestures for audio control
- `updateLEDs()`: Handle attract/interactive lighting modes
- `sendJSONUpdate()`: Output data for TouchDesigner integration
- `readDistanceWithReset()`: Sensor reading with HC-SR04 reset fix

## 🔗 **TouchDesigner Integration**

### **JSON Output Format**
```json
{
  "left": 15,
  "right": 8,
  "hands_detected": true,
  "left_in_range": true,
  "right_in_range": true,
  "mode": "interactive",
  "audio_active": true,
  "audio_playing": true,
  "scratch_mode": false,
  "playback_speed": 5,
  "left_orange_value": 0.75,
  "right_orange_value": 0.60,
  "system": "solar_shrine_modular",
  "timestamp": 123456
}
```

### **TouchDesigner Usage**
1. **Connect Serial DAT** to Arduino's COM port (9600 baud)
2. **Add JSON DAT** to parse incoming data
3. **Use expressions** like `op('json1').result['audio_playing']` to control effects
4. **Map color values** to visual parameters using `left_orange_value` and `right_orange_value`

## 🚀 **Quick Start**

### **Installation**
1. **Open Arduino IDE**
2. **Navigate to**: `01_MAIN_SYSTEM/solar_shrine_modular/`
3. **Open**: `solar_shrine_modular.ino`
4. **Install required libraries**:
   - FastLED (latest version)
   - ArduinoJson (v7.x)
    - NewTone (for Alien theremin audio)
5. **Upload to Arduino Mega 2560**

### **Required Files**
- `solar_shrine_modular.ino` - Main program
- `audio_data.h` - DJ scratch audio samples (26KB)
- `README.md` - This documentation

### **Hardware Connections**
1. **Connect sensors** to pins 5,6,10,11 as specified
2. **Connect LED strip** data line to pin 3
3. **Connect audio**: Pin 12 → 1K resistor → Amplifier right channel
4. **Connect grounds**: Arduino GND → Amplifier left channel + ground

## 🛠️ **Customization**

### **Audio Samples**
- Replace `audio_data.h` with your own audio samples
- Use `convert_audio_progmem.py` to generate new audio data
- Maximum ~26KB for Arduino Mega 2560 flash memory

### **LED Effects**
- Modify `getInteractiveColor()` for different color schemes
- Adjust `ATTRACT_PERIOD` for different fade timing
- Change `NUM_LEDS` for different strip lengths
  
#### Theremin Fireball Visual
- Runs in `MINITHEREMIN` mode.
- Right hand controls sweep speed (closer = faster).
- Left hand controls size/brightness.
- Color palette is strictly orange→yellow across both hands.

### **Sensor Sensitivity**
- Adjust `MIN_RANGE` and `MAX_RANGE` for detection zones
- Modify `SAMPLES` for different averaging amounts
- Change detection thresholds in the main loop

## 🔧 **Troubleshooting**

### **No Audio Output**
1. Check pin 12 connection to amplifier
2. Verify 1K resistor is installed
3. Ensure amplifier has 12V power supply
4. Check Serial Monitor for "Audio System Ready" message

### **LEDs Not Working**
1. Verify pin 3 connection to LED strip
2. Check LED strip power supply (5V for WS2812B, 12V for WS2815)
3. Ensure FastLED library is properly installed
4. Try reducing `NUM_LEDS` for testing

### **Sensors Not Responding**
1. Check sensor connections on pins 5,6,10,11
2. Verify sensors have 5V power and ground
3. Test individual sensors with basic sensor test sketch
4. Check for interference from other electronics

## 📈 **Performance Characteristics**

### **Timing**
- **Main Loop**: 50Hz (20ms cycle time)
- **JSON Updates**: 10Hz (100ms intervals)
- **Audio Sampling**: 8kHz effective rate
- **LED Updates**: Real-time with FastLED

### **Memory Usage**
- **Program**: ~35KB (including 26KB audio data)
- **RAM**: ~8KB for variables and buffers
- **Available**: ~25KB program space for future expansion

## 🎯 **Future Expansion**

### **Planned Modules**
- **Motor Control**: Stepper motor positioning
- **Additional Sensors**: Temperature, humidity, light
- **Network Integration**: WiFi/Ethernet connectivity
- **Advanced Audio**: Multiple audio tracks, effects
- **Extended LED**: Matrix displays, additional strips

### **Architecture Benefits**
- **Clean Separation**: Each system is independently manageable
- **Easy Integration**: New modules follow established patterns
- **Minimal Conflicts**: Careful timer and resource management
- **Scalable Design**: Modular approach supports complex additions

---

## 🎉 **Success!**

Your Solar Shrine Modular System is ready for action! This integration provides:
- ✅ **Proven LED effects** from the theremin system
- ✅ **High-quality audio** from the DJ scratch system  
- ✅ **Robust sensor handling** with averaging and filtering
- ✅ **TouchDesigner integration** with comprehensive JSON output
- ✅ **Modular architecture** ready for future expansion

**Enjoy your interactive audio-visual installation!** 🎶✨ 