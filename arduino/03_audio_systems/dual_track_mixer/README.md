# Dual Track Audio Mixer - Arduino Mega 2560

## 🎵 **Concept**

This effect creates a real-time audio mixer that combines two independent PROGMEM audio tracks:

- **Left Hand**: Controls a looping "beat" track (background rhythm)
- **Right Hand**: Controls an "overlay" track (sound effects, melodies, etc.)
- **Real-time mixing**: Both tracks are combined in the Timer1 interrupt
- **Independent control**: Each hand sensor starts/stops/resets its respective track

## 🔧 **Hardware Requirements**

### **Arduino Mega 2560 Pin Assignments:**
- **Pin 10**: Left sensor trigger (beat track control)
- **Pin 11**: Left sensor echo
- **Pin 5**: Right sensor trigger (overlay track control)
- **Pin 6**: Right sensor echo
- **Pin 12**: Audio output (Timer1 OC1B)

### **Audio Output Circuit:**
```
Arduino Pin 12 → 1kΩ resistor → Amplifier Input
                              ↓
                         Optional: 10nF capacitor → GND
```

## 📁 **Current Audio Setup**

**✅ READY TO USE!** Your system is already configured with:

- **Beat Track**: `beat1Data` from `beat_data.h` (7.73 seconds, loops continuously)
- **Overlay Track**: `singingData` from `overlay_data.h` (6.44 seconds, one-shot)

## 🚀 **Quick Start**

1. **Upload to Arduino Mega 2560**
2. **Test the system:**
   - Hold left hand over sensor → Beat starts looping
   - Hold right hand over sensor → Singing overlay plays on top
   - Remove hands → Tracks stop and reset

## 📁 **Adding New Audio Files** 

### **Step 1: Convert Your Audio Files**

```bash
# Navigate to project root
cd solar-shrine-arduino

# Convert your beat track (should loop well)
python convert_audio_progmem.py your_beat.wav

# Convert your overlay track (one-shot sound)  
python convert_audio_progmem.py your_overlay.wav
```

### **Step 2: Update the Code**

The conversion script generates variables with names based on your filename:

**Example:** `beat.wav` → `beat1Data`, `BEAT1_SAMPLE_COUNT`, etc.

Update these lines in `dual_track_mixer.ino`:
```c
// Change these to match your new audio file variable names:
AudioTrack beatTrack = {0, 0, false, true, 5, your_beat1Data, YOUR_BEAT1_SAMPLE_COUNT};
AudioTrack overlayTrack = {0, 0, false, false, 5, your_overlay1Data, YOUR_OVERLAY1_SAMPLE_COUNT};

// And in setup():
Serial.print(YOUR_BEAT1_SAMPLE_COUNT); 
Serial.print(YOUR_BEAT1_DURATION);
Serial.print(YOUR_OVERLAY1_SAMPLE_COUNT);
Serial.print(YOUR_OVERLAY1_DURATION);
```

## 🎮 **How It Works**

### **Beat Track (Left Hand):**
- **Activation**: Wave left hand over sensor
- **Behavior**: Starts playing and loops continuously
- **Deactivation**: Remove left hand → beat stops and resets to beginning

### **Overlay Track (Right Hand):**
- **Activation**: Wave right hand over sensor  
- **Behavior**: Plays once from beginning to end
- **Deactivation**: Remove right hand → overlay stops and resets to beginning
- **Retriggering**: Wave hand again to restart overlay from beginning

### **Audio Mixing:**
- Both tracks are mixed in real-time during the Timer1 interrupt
- Samples are averaged to prevent clipping
- Only active tracks contribute to the final output

## 📊 **Memory Considerations**

### **Arduino Mega 2560:**
- **Flash Memory**: 256KB total
- **Available for audio**: ~200KB (after code overhead)
- **Recommended**: 
  - Beat track: 5-15 seconds (looping)
  - Overlay track: 1-5 seconds (one-shot)

### **Size Planning:**
```
Duration (seconds) × 8000 samples/sec = Total samples
Total samples ÷ 1024 = File size in KB

Example:
- 10-second beat: 10 × 8000 = 80,000 samples = ~78KB
- 3-second overlay: 3 × 8000 = 24,000 samples = ~23KB
- Total: ~101KB (well within Mega limits)
```

## 🎯 **Audio File Recommendations**

### **Beat Track Characteristics:**
- **Duration**: 2-10 seconds
- **Content**: Drum loops, bass lines, rhythmic patterns
- **Looping**: Should loop seamlessly (start/end should match)
- **Volume**: Moderate level (will be background)

### **Overlay Track Characteristics:**
- **Duration**: 0.5-5 seconds  
- **Content**: Sound effects, melodies, vocal clips, percussion hits
- **One-shot**: Complete sound that doesn't need to loop
- **Volume**: Can be louder (will be foreground)

## 🚀 **Installation Instructions**

1. **Prepare Audio Files**:
   ```bash
   python convert_audio_progmem.py your_beat.wav
   python convert_audio_progmem.py your_overlay.wav
   ```

2. **Copy Files**:
   ```bash
   cp beat_data.h arduino/03_audio_systems/dual_track_mixer/
   cp overlay_data.h arduino/03_audio_systems/dual_track_mixer/
   ```

3. **Upload to Arduino**:
   - Open `dual_track_mixer.ino` in Arduino IDE
   - Select "Arduino Mega 2560"
   - Upload to your board

4. **Connect Hardware**:
   - Connect sensors to pins 10/11 and 5/6
   - Connect pin 12 to your amplifier
   - Power up and test!

## 🎵 **Usage Examples**

### **Example 1: Hip-Hop Beat + Scratch**
- **Beat**: 4-bar drum loop (4 seconds)
- **Overlay**: Scratch sound effects (1-2 seconds each)
- **Result**: Create live hip-hop mixes

### **Example 2: Ambient + Percussion**
- **Beat**: Atmospheric drone (8 seconds)
- **Overlay**: Various percussion hits (0.5-1 second each)
- **Result**: Build ambient soundscapes

### **Example 3: Melody + Bass**
- **Beat**: Bass line loop (2 seconds)
- **Overlay**: Melodic phrases (2-4 seconds each)
- **Result**: Layer musical elements

## 🐛 **Troubleshooting**

### **No Audio Output:**
- ✅ Verify pin 12 connection to amplifier
- ✅ Check that both audio files are present
- ✅ Ensure proper power supply to amplifier

### **Only One Track Playing:**
- ✅ Check sensor connections (pins 10/11 and 5/6)
- ✅ Verify both hands are detected (check Serial Monitor)
- ✅ Ensure both audio files have correct variable names

### **Audio Quality Issues:**
- ✅ Check for clipping (reduce audio levels before conversion)
- ✅ Verify proper RC filter on output (1kΩ + 10nF)
- ✅ Ensure stable power supply

### **Compilation Errors:**
- ✅ Make sure both `beat_data.h` and `overlay_data.h` are present
- ✅ Check that variable names match exactly
- ✅ Verify files aren't corrupted (should be ~KB in size)

## 📈 **Serial Monitor Output**

When running, you'll see debug output like:
```
Dual Track Mixer Ready - Mega Version
Left Hand: Beat Track (loops)
Right Hand: Overlay Track (one-shot)
Beat track: 16000 samples, 2.00s
Overlay track: 8000 samples, 1.00s
START: BEAT
Beat: PLAY | Overlay: STOP | L:5.2cm R:25.1cm
START: OVERLAY  
Beat: PLAY | Overlay: PLAY | L:4.8cm R:3.7cm
STOP: OVERLAY
Beat: PLAY | Overlay: STOP | L:6.1cm R:22.3cm
```

## 🎉 **Integration with Main System**

This effect can be integrated into your main `solar_shrine_modular` system as one of the rotating audio effects. The dual-track mixing approach could also be adapted for other creative audio combinations.

**Ready to create your own dual-track audio mixes!** 🎵