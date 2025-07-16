# 🎧 DJ Scratch PROGMEM Audio Player

**Crystal clear voice playback with real-time DJ scratching effects controlled by sonar sensors!**

## 🎯 Features

- **🎵 High-Quality Audio**: Phase Correct PWM for crystal clear speech
- **🖐️ Gesture Control**: Left/right hand control via sonar sensors
- **🔄 Real-Time Scratching**: Variable speed forward/backward playback
- **🎶 Loop Control**: Smart looping with hand gestures
- **📢 Loud Volume**: 4x amplification with no distortion
- **💾 Flash Storage**: Audio stored in PROGMEM (no SD card needed)

## 🎮 Control Scheme

### **Left Hand (Sensor 1)** - Playback Control
- **Wave hand over sensor**: Play/Pause toggle
- **Hold hand very close (< 4cm)**: Toggle loop on/off
- **Remove hand**: Maintains current state

### **Right Hand (Sensor 2)** - Scratch Effects
- **Move hand quickly**: Trigger scratch mode
- **Forward motion**: Fast forward playback
- **Backward motion**: Reverse playback
- **Slow motion**: Return to normal speed
- **Remove hand**: Return to normal playback

## 🔧 Hardware Setup

### **Sensors (From Main System)**
- **Left Sensor**: Pins 10 (Trig), 11 (Echo)
- **Right Sensor**: Pins 5 (Trig), 6 (Echo)

### **Audio Output** (Arduino Mega 2560)
- **Pin 12**: Audio output (Timer1 OC1B PWM)
- **Wiring**: Right channel → 1K resistor → Pin 12, Left channel + Ground → Ground
- **Amplifier**: Connect to WWZMDiB XH-M543

### **Range Settings**
- **Detection Range**: 2-20cm
- **Optimal Range**: 5-15cm for best control
- **Scratch Sensitivity**: 5cm movement triggers scratch

## 🚀 Quick Start

1. **Open Arduino IDE**
2. **Navigate to**: `solar-shrine-arduino/arduino/03_audio_systems/dj_scratch_progmem/`
3. **Open**: `dj_scratch_progmem.ino`
4. **Install required libraries**:
   ```
   - NewPing (latest version)
   ```
5. **Upload to Arduino**
6. **Open Serial Monitor** (9600 baud)

## 🎵 Usage Instructions

### **Basic Playback**
1. **Wave left hand** over sensor → Audio starts playing
2. **Wave left hand again** → Audio pauses
3. **Wave left hand again** → Audio resumes

### **Loop Control**
1. **Hold left hand very close** (< 4cm) → Toggle loop on/off
2. **Serial Monitor shows**: `🔄 Loop: ON` or `🔄 Loop: OFF`

### **Scratching**
1. **Move right hand quickly** over sensor → Scratch mode activated
2. **Fast forward motion** → Fast forward playback
3. **Fast backward motion** → Reverse playback
4. **Slow motion** → Return to normal speed
5. **Remove hand** → Normal playback resumes

### **Status Monitoring**
Serial Monitor shows real-time status:
```
Status: PLAYING | Position: 45.2% | Speed: 1.50x | Loop: ON
🎵 SCRATCH! Speed: -2.30 (backward)
🎵 Normal playback resumed
```

## 🎛️ Advanced Controls

### **Scratch Sensitivity**
Adjust in code:
```cpp
const int SCRATCH_VELOCITY_THRESHOLD = 5;  // cm/reading
```
- **Lower value**: More sensitive scratching
- **Higher value**: Less sensitive scratching

### **Scratch Speed Range**
```cpp
const float MAX_SCRATCH_SPEED = 4.0;  // Maximum speed multiplier
```
- **Higher value**: Faster scratching possible
- **Lower value**: More controlled scratching

### **Detection Range**
```cpp
const float MIN_RANGE = 2.0;  // cm
const float MAX_RANGE = 20.0; // cm
```

## 🔊 Audio Quality

- **Sample Rate**: 8kHz
- **Bit Depth**: 8-bit with 4x amplification
- **PWM Frequency**: 20kHz (inaudible)
- **Audio Duration**: 3.29 seconds (26,285 samples)
- **Memory Usage**: ~26KB flash memory

## 🎯 Performance Tips

1. **Optimal Hand Position**: 8-12cm from sensors
2. **Smooth Movements**: Avoid jerky motions for best scratch control
3. **Consistent Speed**: Move hand at steady pace for scratching
4. **Hand Timeout**: 200ms timeout prevents accidental triggers

## 🐛 Troubleshooting

### **No Audio Output**
- Check RC filter connections (1kΩ + 10nF)
- Verify pin 9 connection to amplifier
- Ensure amplifier is powered (12V recommended)

### **Scratching Not Working**
- Move hand faster over right sensor
- Adjust `SCRATCH_VELOCITY_THRESHOLD` in code
- Check sensor connections (pins 5,6)

### **Play/Pause Not Working**
- Check left sensor connections (pins 10,11)
- Verify hand is within 2-20cm range
- Watch Serial Monitor for hand detection

### **Audio Quality Issues**
- Ensure proper RC filter (1kΩ resistor + 10nF capacitor)
- Check amplifier input connections
- Verify 12V power supply for amplifier

## 🔄 Integration with Main System

This DJ scratch system can be integrated into the main Solar Shrine system:

1. **Replace** `solar_shrine_theremin.ino` with DJ scratch version
2. **Keep** existing LED and JSON output code
3. **Modify** audio section to use DJ scratch controls
4. **Maintain** compatibility with TouchDesigner integration

## 📊 Technical Details

- **Timer1 Phase Correct PWM**: 20kHz frequency, 400 levels
- **Dynamic Sample Rate**: Adjusts based on scratch speed
- **Memory Efficient**: Uses volatile variables for interrupt safety
- **Real-Time Processing**: 50ms control loop for responsive scratching

## 🎉 Fun Features

- **Backward Playback**: Reverse scratching plays audio backward
- **Speed Control**: Variable speed from 0.1x to 4.0x
- **Loop Modes**: Smart looping with gesture control
- **Visual Feedback**: Serial Monitor shows all actions in real-time

---

**🎵 Ready to scratch? Upload the code and start mixing!** 