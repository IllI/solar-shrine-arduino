# Solar Shrine Audio Improvements

## Volume Control & Smooth Audio Solutions

Your theremin was experiencing two main issues:
1. **Too loud** - NewTone has no volume control
2. **Choppy sound** - Abrupt frequency changes and no smoothing

Here are the **recommended solutions** from best to good:

---

## 🔥 **BEST SOLUTION: Mozzi Library** (Professional Quality)

### Why Mozzi is Perfect for Your Theremin:
- ✅ **Built-in volume control** with smooth envelopes
- ✅ **ADSR envelope generators** (Attack, Decay, Sustain, Release)
- ✅ **Multiple waveform types** (sine, triangle, sawtooth)
- ✅ **Audio filters** for warmth and character
- ✅ **Smooth control interpolation**
- ✅ **Vibrato and LFO support**
- ✅ **Professional synthesizer-quality audio**

### Installation:
1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search for **"Mozzi"**
4. Install the latest version

### New File Location:
```
02_lighting_systems/mozzi_theremin_smooth/mozzi_theremin_smooth.ino
```

### Key Improvements in Mozzi Version:
- **ADSR Envelope**: Smooth attack/release prevents clicks
- **Volume Control**: True amplitude control from 0-100%
- **Frequency Smoothing**: Buttery smooth pitch transitions
- **Vibrato**: Natural-sounding frequency modulation
- **Low-pass Filter**: Adds warmth and character
- **Multiple Smoothing Stages**: For sensors, frequency, and volume

---

## 🔥 **GOOD ALTERNATIVE: Volume3 Library** (10-bit Volume)

If you want to stick with NewTone but add volume control:

### Installation:
1. Download from: https://github.com/connornishijima/arduino-volume3
2. Install via Library Manager: Search "Volume3"

### Usage:
```cpp
#include "Volume3.h"

void setup() {
  // No vol.begin() needed!
}

void loop() {
  vol.tone(9, 440, 512); // pin, frequency, volume (0-1023)
}
```

### Benefits:
- ✅ **10-bit volume control** (0-1023 levels)
- ✅ **No extra components needed**
- ✅ **Easy drop-in replacement**
- ✅ **100kHz PWM frequency** (inaudible)

---

## 🔥 **ALTERNATIVE: Volume1 Library** (8-bit Volume + Fades)

Another excellent option with fade effects:

### Installation:
```
Library Manager → Search "Volume" (by Connor Nishijima)
```

### Usage:
```cpp
#include "Volume.h"
Volume vol;

void setup() {
  vol.begin();
}

void loop() {
  vol.tone(440, 128);     // frequency, volume (0-255)
  vol.fadeOut(2000);      // 2-second fade out
  vol.delay(1000);        // Use vol.delay() instead of delay()
}
```

### Benefits:
- ✅ **8-bit volume control** (0-255 levels)
- ✅ **Built-in fade functions**
- ✅ **Smooth volume transitions**
- ✅ **Volume sliding effects**

---

## 📊 **Library Comparison**

| Library | Volume Bits | Frequency Range | Features | Audio Quality |
|---------|-------------|-----------------|----------|---------------|
| **Mozzi** | 16-bit | 1Hz - 8kHz+ | ADSR, Filters, LFO, Multiple waves | **EXCELLENT** |
| **Volume3** | 10-bit (1023) | 1Hz - 4.2kHz | Simple volume control | **GOOD** |
| **Volume1** | 8-bit (255) | 120Hz - 5kHz | Volume + Fade effects | **GOOD** |
| **NewTone** | No control | 31Hz - 65kHz | Basic square wave | **BASIC** |

---

## 🎛️ **Hardware Volume Control (Any Library)**

For external volume control, add this circuit:

```
Arduino Pin → 1kΩ Resistor → 10kΩ Potentiometer → Speaker
                                     ↓
                                   Ground
```

**Or use a digital potentiometer** like MCP4131 for software control.

---

## 🔧 **Upgrading Your Current System**

### Option 1: Quick Fix (Keep NewTone)
- Add **Volume3** library for volume control
- Add **frequency smoothing** in software
- Upgrade to Volume3 in your current code

### Option 2: Professional Upgrade (Recommended)
- Switch to **Mozzi** library completely
- Use the new `mozzi_theremin_smooth.ino` file
- Get professional synthesizer-quality audio

### Option 3: Hybrid Approach
- Use **Volume1** for volume control and fades
- Add manual frequency smoothing
- Keep most of your existing code structure

---

## 🎵 **Audio Output Pins**

| Library | Default Pin | Notes |
|---------|-------------|-------|
| **Mozzi** | Pin 9 | Can be configured |
| **Volume3** | Pin 9, 10 | Uses Timer1 |
| **Volume1** | Pin 5, 6 | Uses Timer0 |
| **NewTone** | Any pin | Pin specified in function |

---

## 🚀 **Recommended Next Steps**

1. **Try the Mozzi version first** - It's the best solution
2. **Install Mozzi library** via Library Manager
3. **Upload the new smooth theremin code**
4. **Connect a potentiometer to A0** for volume control (optional)
5. **Enjoy professional-quality audio!**

The Mozzi version will give you:
- 🔇 **Perfect volume control**
- 🎵 **Silky smooth frequency transitions**
- 🎶 **Beautiful musical envelopes**
- 🎸 **Rich, warm sound character**

Your visitors will be amazed by the improvement! 