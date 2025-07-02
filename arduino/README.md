# Solar Shrine Arduino - Modular System Architecture

## 🎯 **READY-TO-COMPILE SYSTEM**
**Main Production System:** `01_MAIN_SYSTEM/solar_shrine_theremin/solar_shrine_theremin.ino`

---

## 📁 **Modular Project Structure**

This project is now organized into **independent, compilable modules**. Each folder contains a complete Arduino sketch that can be opened and compiled without conflicts.

```
solar-shrine-arduino/arduino/
├── 01_MAIN_SYSTEM/                    # 🚀 PRODUCTION READY
│   └── solar_shrine_theremin/
│       └── solar_shrine_theremin.ino  # ⭐ MAIN SYSTEM - COMPILE THIS
├── 02_lighting_systems/               # 💡 LED Control Systems
│   └── fastled_dual_mode/
│       └── fastled_dual_mode.ino      # Attract/Interactive lighting
├── 03_audio_systems/                  # 🔊 Audio Components
│   └── speaker_test/
│       └── speaker_test.ino           # Audio hardware testing
├── 04_motor_control/                  # ⚙️ Motor Systems
│   └── stepper_control/
│       └── stepper_control.ino        # Stepper motor control
├── 05_basic_sensors/                  # 📡 Sensor Testing
│   └── sensor_output/
│       └── sensor_output.ino          # Basic sensor readings
├── 06_json_integration/               # 🔗 TouchDesigner Integration
│   └── basic_json/
│       └── basic_json.ino             # JSON output for TouchDesigner
├── 99_ARCHIVE/                        # 📦 Legacy files (safe to ignore)
└── sensors_json_output/               # 📦 Legacy folder (will be archived)
```

---

## 🚀 **Quick Start - Main System**

### **FOR IMMEDIATE USE:**
1. **Open Arduino IDE**
2. **Navigate to:** `01_MAIN_SYSTEM/solar_shrine_theremin/`
3. **Open:** `solar_shrine_theremin.ino`
4. **Install required libraries** (see below)
5. **Upload to Arduino**

### **Required Libraries for Main System:**
```
Arduino IDE → Tools → Manage Libraries:
- FastLED (latest version)
- ArduinoJson (v7.x)
- NewPing (latest version)
```

**Manual Install Required:**
- **NewTone Library**: [Download from BitBucket](https://bitbucket.org/teckel12/arduino-new-tone/downloads/)
  - Extract ZIP and place in Arduino Libraries folder
  - **Why NewTone?** Prevents timer conflicts with ultrasonic sensors

---

## 🎛️ **System Features**

### **Main System (solar_shrine_theremin.ino)**
- ✅ **Dual-Mode Lighting**: Attract (sinusoidal fade) + Interactive (distance-based)
- ✅ **Theremin Audio**: Hand-proximity sound generation
- ✅ **Hand Detection**: 5-sample averaging prevents false triggering
- ✅ **TouchDesigner Integration**: JSON output with correlation values
- ✅ **Smooth Transitions**: Trigonometric phase matching
- ✅ **Audio Hardware**: Full integration with amplifier + exciter

### **Key Features:**
- **Attract Mode**: Yellow↔Red sinusoidal fade (5-second period)
- **Interactive Mode**: Distance-based Red→Orange→Yellow colors
- **Theremin Audio**: Dual-hand harmonic generation
- **10-Second Timeout**: Auto-return to attract mode
- **Wind/Dust Filtering**: Averaged sensor readings

---

## 🔧 **Hardware Configuration**

### **Standard Pin Layout** (Used across all systems)
| Component | Pins | Description |
|-----------|------|-------------|
| **Ultrasonic Sensor 1** | 9 (Trig), 10 (Echo) | Left hand detection |
| **Ultrasonic Sensor 2** | 5 (Trig), 6 (Echo) | Right hand detection |
| **LED Strip** | 3 | WS2812B/WS2815 control |
| **Audio Output** | 11 | To WWZMDiB XH-M543 amplifier |
| **Motor Control** | 2,3,4,7 | Stepper motor (when used) |

### **Audio Hardware Stack**
- **Amplifier**: WWZMDiB XH-M543 High Power Digital Amplifier (TPA3116D2)
- **Exciter**: Dayton Audio DAEX32QMB-4 Quad Feet Mega Bass 32mm (40W 4Ω)
- **Power**: 12V+ recommended for full audio output

---

## 🎯 **Module Usage Guide**

### **For Development & Testing:**

#### **02_lighting_systems/fastled_dual_mode**
- **Purpose**: Standalone lighting effects
- **Features**: Attract/Interactive modes without audio
- **Use Case**: LED development and testing

#### **03_audio_systems/** - Audio Quality Research & Development
- **speaker_test/**: Audio hardware validation and testing
- **minimin_theremin_test/**: Proven MiniMin design with vibrato
- **musical_theremin_warm/**: Research-based musical theremin with warm sound
- **alien_sound_effect/**: Experimental synthesized effects

**Audio Quality Research:**
- **Frequency Range**: 120-1500Hz proven optimal for musical theremin
- **Waveform**: Sine waves = warm, Triangle waves = tinny
- **Vibrato**: 2-4Hz rate with <2% depth for musical expression
- **Research Sources**: [Adafruit](https://learn.adafruit.com/adafruit-arduino-lesson-10-making-sounds/pseudo-theramin), [Arduino.cc](https://www.arduino.cc/en/Tutorial/BuiltInExamples/tonePitchFollower/)

#### **04_motor_control/stepper_control**
- **Purpose**: Motor positioning control
- **Features**: Sensor-driven positioning, test sequences
- **Use Case**: Mechanical system integration

#### **05_basic_sensors/sensor_output**
- **Purpose**: Basic sensor validation
- **Features**: Raw distance readings
- **Use Case**: Sensor calibration and testing

#### **06_json_integration/basic_json**
- **Purpose**: TouchDesigner data integration
- **Features**: JSON formatted sensor output
- **Use Case**: TouchDesigner development

---

## 🔄 **Development Workflow**

### **For Component Development:**
1. **Work on individual modules** for specific features
2. **Test each component** independently
3. **Integrate successful features** into main system
4. **Archive experimental code** in `99_ARCHIVE/`

### **For System Integration:**
1. **Start with main system** (`solar_shrine_theremin.ino`)
2. **Modify specific functions** from other modules
3. **Test integrated system** thoroughly
4. **Update documentation** as needed

---

## 📚 **Library Installation Guide**

### **Standard Libraries (Arduino IDE):**
```bash
Tools → Manage Libraries → Search & Install:
- FastLED
- ArduinoJson (v7.x)
- NewPing
- Stepper (built-in)
```

### **Manual Installation (NewTone):**
1. **Download**: [NewTone ZIP from BitBucket](https://bitbucket.org/teckel12/arduino-new-tone/downloads/)
2. **Extract**: Unzip the downloaded file
3. **Install**: 
   - **Windows**: Copy to `Documents/Arduino/libraries/`
   - **Mac**: Copy to `~/Documents/Arduino/libraries/`
   - **Linux**: Copy to `~/Arduino/libraries/`
4. **Restart**: Close and reopen Arduino IDE
5. **Verify**: Check if `#include <NewTone.h>` works

---

## 🚨 **Troubleshooting**

### **Compilation Errors:**
- ✅ **"Redefinition" errors**: Make sure you're only opening ONE `.ino` file at a time
- ✅ **Library errors**: Install all required libraries for your chosen module
- ✅ **Pin conflicts**: Check pin assignments match your hardware

### **Main System Issues:**
- ✅ **No audio**: Check NewTone library installation
- ✅ **Erratic sensors**: Verify wiring and power supply
- ✅ **LED problems**: Confirm FastLED library and strip type

### **Performance Issues:**
- ✅ **Slow response**: Check sensor averaging settings
- ✅ **Audio glitches**: Verify power supply capacity
- ✅ **LED flickering**: Check power distribution

---

## 🎵 **Audio Integration Details**

### **Audio Quality Research (Based on Multiple Sources):**

**Optimal Frequency Ranges:**
- **120-1500Hz**: Research-proven musical range ([Adafruit](https://learn.adafruit.com/adafruit-arduino-lesson-10-making-sounds/pseudo-theramin), [Arduino.cc](https://www.arduino.cc/en/Tutorial/BuiltInExamples/tonePitchFollower/))
- **Below 120Hz**: Bass range, hard to hear clearly on small speakers
- **Above 1500Hz**: Becomes harsh and piercing to human ears

**Waveform Quality Analysis:**
- **Sine Waves**: Pure, warm, musical (recommended for theremin)
- **Triangle Waves**: Contains odd harmonics, sounds "tinny"
- **Square Waves**: Very buzzy, better for retro game sounds

### **Theremin Frequency Mapping (Main System):**
- **Left Hand Only**: 80Hz - 220Hz (bass range)
- **Right Hand Only**: 220Hz - 2000Hz (treble range)  
- **Both Hands**: Harmonic blending with vibrato effects
- **Distance Mapping**: Close = High frequency, Far = Low frequency

### **Musical Theremin Implementations:**
- **Main System**: NewTone library with triangle wave (functional)
- **MiniMin Test**: Proven design with gentle vibrato
- **Musical Warm**: Research-based sine wave with 120-1500Hz range
- **Alien Effect**: Experimental synthesized sounds

### **Audio Hardware Requirements:**
- **12V Power Supply** (2A+ recommended)
- **Proper grounding** between Arduino and amplifier
- **Heat dissipation** for amplifier (consider cooling)
- **Speaker Selection**: Full-range speakers recommended for 120-1500Hz response

---

## 📞 **Support & Development**

### **File Organization Benefits:**
- ✅ **No compilation conflicts** - Each module is independent
- ✅ **Modular development** - Work on features separately
- ✅ **Easy testing** - Standalone component validation
- ✅ **Version control** - Clear change tracking
- ✅ **Collaboration** - Multiple developers can work simultaneously

### **Next Steps:**
1. **Test main system** with your hardware
2. **Customize parameters** (distances, colors, frequencies)
3. **Develop new features** in separate modules
4. **Integrate successful experiments** into main system

---

**🎯 Remember: The main production system is `01_MAIN_SYSTEM/solar_shrine_theremin/solar_shrine_theremin.ino` - this is your ready-to-use, fully-featured Solar Shrine controller!** 