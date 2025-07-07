# Solar Shrine Arduino System

## ⚠️ **IMPORTANT AUDIO UPDATE**

**Previous sample playback approach using `analogWrite()` does NOT work!**
- Creates buzzer sounds instead of recognizable audio
- Due to Arduino's PWM frequency limitations (490Hz vs 8kHz+ needed)

**⚡ NEW TIMER-BASED METHOD: Technical Solution with Limitations**
- Uses Timer2 PWM at 11,025Hz frequency
- Interrupt-driven precise sample timing
- RC filter for analog audio output
- **REALITY**: Poor quality, ~1 second duration only
- **RECOMMENDATION**: Your current Mozzi system is significantly better

---

## 🎯 **System Overview**

The Solar Shrine is an interactive art installation combining:
- **Dual-mode LED lighting** (attract/interactive)
- **Hand-proximity audio generation** (Mozzi theremin - HIGH QUALITY)
- **Ultrasonic sensors** for hand detection
- **TouchDesigner integration** via JSON data

## 🚀 **Quick Start - Main System**

### **Current Production System:**
```
01_MAIN_SYSTEM/solar_shrine_theremin/solar_shrine_theremin.ino
```

This is your **working, tested system** with:
- ✅ Mozzi theremin audio (high quality)
- ✅ Dual-mode LED lighting
- ✅ TouchDesigner JSON integration
- ✅ Hand detection with filtering

### **Required Libraries:**
```bash
Arduino IDE → Tools → Manage Libraries:
- FastLED
- ArduinoJson (v7.x)
- NewPing
- Mozzi (from https://sensorium.github.io/Mozzi/)
```

---

## 🔊 **Audio Options**

### **1. Mozzi Theremin (STRONGLY RECOMMENDED)**
- **File**: `01_MAIN_SYSTEM/solar_shrine_theremin/solar_shrine_theremin.ino`
- **Quality**: High-quality synthesis
- **Memory**: Code only (~10KB)
- **Duration**: Unlimited
- **Status**: ✅ Working perfectly

### **2. Timer-Based Sample Playback (EXPERIMENTAL)**
- **File**: `03_audio_systems/timer_audio_player/timer_audio_player.ino`
- **Quality**: Poor (distorted, muffled)
- **Memory**: ~1 second duration only
- **Hardware**: Requires RC filter (1kΩ + 10µF)
- **Status**: ⚠️ Works but limited quality
- **Guide**: See `TIMER_AUDIO_GUIDE.md`

### **3. SD Card Audio (BEST FOR SAMPLES)**
- **File**: `03_audio_systems/sd_card_audio_player/sd_card_audio_player.ino`
- **Quality**: Good quality, unlimited duration
- **Memory**: External SD card
- **Status**: ✅ Working (requires SD card module)

---

## 🎵 **Audio Sample Playback Reality Check**

### **What Research Shows:**
Based on extensive Arduino forum analysis, sample playback on Arduino has **severe limitations**:

- **Quality**: Described as "distorted", "not understandable", "static with clicks"
- **Duration**: **~1 second maximum** on Arduino UNO (memory constraint)
- **Hardware**: **RC filter mandatory** or audio is harsh/unusable
- **Complexity**: Much more complex than expected

### **Timer Method (Technical Solution):**
```bash
# 1. Convert your audio properly (limited results)
python convert_audio_timer.py your_audio.wav

# 2. Add RC filter circuit (mandatory)
Pin 3 → 1kΩ resistor → Amplifier
         ↓
       10µF capacitor → Ground

# 3. Use timer_audio_player.ino (expect poor quality)
```

### **Honest Comparison:**
| Method | Quality | Duration | Complexity | Recommendation |
|--------|---------|----------|------------|----------------|
| **Your Mozzi System** | ✅ Excellent | ✅ Unlimited | ✅ Simple | 🎯 **USE THIS** |
| Timer Sample Playback | ⚠️ Poor | ❌ ~1 second | ⚠️ Complex | 🔬 Experiment only |
| SD Card Audio | ✅ Good | ✅ Unlimited | ⚠️ Hardware | 💾 If samples needed |

---

## 🔧 **Hardware Configuration**

### **Pin Assignments:**
```cpp
// Main system (works with Mozzi)
const int trigPin1 = 10;    // Sensor 1 trigger
const int echoPin1 = 11;    // Sensor 1 echo
const int trigPin2 = 5;     // Sensor 2 trigger
const int echoPin2 = 6;     // Sensor 2 echo
const int LED_PIN = 3;      // WS2812B LED strip
const int AUDIO_PIN = 9;    // Audio output (Mozzi compatible)
```

### **Timer Audio Player:**
```cpp
// Timer audio player (Pin 3 for Timer2)
const int trigPin1 = 10;    // Sensor 1 trigger
const int echoPin1 = 11;    // Sensor 1 echo
const int trigPin2 = 5;     // Sensor 2 trigger
const int echoPin2 = 6;     // Sensor 2 echo
const int LED_PIN = 2;      // LEDs (moved from 3)
const int AUDIO_PIN = 3;    // Timer2 PWM output (OC2B)
```

### **Audio Hardware:**
- **Amplifier**: WWZMDiB XH-M543 (TPA3116D2 chip)
- **Exciter**: Dayton Audio DAEX32QMB-4 (40W 4Ω)
- **Power**: 12V+ recommended
- **Filter** (Timer method only): RC filter (1kΩ + 10µF)

---

## 📁 **Module Structure**

```
solar-shrine-arduino/
├── 01_MAIN_SYSTEM/                    # 🚀 PRODUCTION READY
│   └── solar_shrine_theremin.ino      # ⭐ MAIN SYSTEM - BEST AUDIO
├── 02_lighting_systems/               # 💡 LED Testing
│   └── fastled_dual_mode.ino          # Standalone lighting
├── 03_audio_systems/                  # 🔊 Audio Components
│   ├── timer_audio_player/            # ⚠️ Experimental: Poor quality samples
│   │   └── timer_audio_player.ino     # Timer2-based audio
│   ├── sd_card_audio_player/          # ✅ Good quality with SD card
│   │   └── sd_card_audio_player.ino   # TMRpcm library
│   └── mozzi_theremin_smooth/         # ✅ Advanced Mozzi
│       └── mozzi_theremin_smooth.ino  # Professional theremin
├── 04_motor_control/                  # ⚙️ Motor Systems
├── 05_basic_sensors/                  # 📡 Sensor Testing
├── 06_json_integration/               # 🔗 TouchDesigner
└── 99_ARCHIVE/                        # 📦 Legacy files
```

---

## 🎛️ **Features**

### **Main System Features:**
- **Attract Mode**: Yellow↔Red sinusoidal fade
- **Interactive Mode**: Distance-based color changes
- **Theremin Audio**: Dual-hand frequency control (HIGH QUALITY)
- **Hand Detection**: 5-sample averaging prevents false triggers
- **TouchDesigner**: JSON output with correlation data
- **Smooth Transitions**: Trigonometric phase matching

### **Timer Audio Player Features:**
- **Sample Playback**: Recognizable but poor quality audio
- **Hand-Triggered**: Starts/stops with hand detection
- **Memory Limited**: ~1 second maximum duration
- **Hardware Required**: RC filter circuit mandatory
- **Debug Interface**: Serial commands for testing

---

## 🧪 **Testing & Development**

### **Main System Testing (RECOMMENDED):**
```bash
# 1. Upload main system
# 2. Open Serial Monitor
# 3. Wave hands near sensors
# 4. Should see JSON output + high-quality audio + LEDs
```

### **Timer Audio Testing (EXPERIMENTAL):**
```bash
# 1. Add RC filter circuit first (mandatory)
# 2. Upload timer_audio_player.ino
# 3. Send 't' in Serial Monitor
# 4. Should hear poor-quality demo audio sample
# 5. Wave hands to trigger playback
```

### **Expected Results:**
- **Main System**: Professional-quality theremin sounds
- **Timer Audio**: Distorted, muffled but recognizable audio

---

## 🔄 **Development Workflow**

### **For Audio Development:**
1. **STRONGLY RECOMMEND**: Keep using your main Mozzi system
2. **If experimenting**: Try timer method but expect poor quality
3. **For samples**: Consider SD card approach instead
4. **Never go back**: To basic `analogWrite()` method

### **For System Integration:**
1. **Test modules independently**
2. **Use main system** as integration base
3. **Add new features** incrementally
4. **Archive experimental code** in `99_ARCHIVE/`

---

## 📚 **Documentation**

- **`TIMER_AUDIO_GUIDE.md`** - Complete timer audio implementation (with limitations)
- **`AUDIO_IMPROVEMENTS.md`** - Audio quality research
- **`AUDIO_RECOMMENDATION.md`** - Audio system recommendations
- **`Solar_Shrine_Custom_Shield_Design.md`** - Hardware design

---

## 🚨 **Troubleshooting**

### **Audio Issues:**
- **No audio**: Check power supply, wiring, RC filter (timer method)
- **Buzzer sounds**: You're using `analogWrite()` method (doesn't work)
- **Distorted audio** (timer method): This is normal/expected
- **Won't compile**: Check library installation, PROGMEM syntax

### **System Issues:**
- **Erratic sensors**: Check wiring, power supply stability
- **LED problems**: Verify FastLED library, LED strip type
- **JSON errors**: Check ArduinoJson version (v7.x)

---

## 🌟 **Recent Updates**

- ✅ **Research-based assessment** of timer audio limitations
- ✅ **Realistic quality expectations** for sample playback
- ✅ **Comprehensive audio guide** (TIMER_AUDIO_GUIDE.md)
- ⚠️ **Honest comparison** of audio methods
- ✅ **Updated documentation** with realistic information

---

## 📞 **Final Recommendation**

### **For Your Solar Shrine Project:**

**🎯 KEEP YOUR CURRENT MOZZI SYSTEM** - It's superior in every way:
- Higher audio quality than timer method
- Unlimited duration vs ~1 second
- Musical responsiveness vs poor sample quality
- No additional hardware needed
- Proven to work perfectly

### **For Experimental Learning:**
- **Timer method**: Only if you want to understand Arduino audio limitations
- **SD card audio**: If you specifically need audio file playback
- **External modules**: DFPlayer Mini for high-quality MP3 playback

**Your existing system is already the best solution for your project!** 🎵 