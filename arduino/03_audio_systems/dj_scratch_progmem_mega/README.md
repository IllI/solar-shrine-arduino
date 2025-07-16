# DJ Scratch PROGMEM - Arduino Mega 2560 Version

## Hardware Requirements

### Arduino Mega 2560 Pin Assignments:
- **Pin 10**: Left sensor trigger (TRIG1)
- **Pin 11**: Left sensor echo (ECHO1) - *matches your existing wiring*
- **Pin 5**: Right sensor trigger (TRIG2)
- **Pin 6**: Right sensor echo (ECHO2)
- **Pin 12**: Audio output (Timer1 OC1B) - **NEW connection needed**

### Key Differences from Arduino Uno Version:
- **Audio pin changed** from pin 9 to pin 12 (Timer1 OC1B on Mega)
- **Left sensor pins** kept as 10/11 to match your existing hardware
- **Timer1 configuration** updated for OC1B instead of OC1A

## Hardware Connections

### Existing Wiring (Keep as is):
- Pin 9: Your existing audio output connection
- Pin 11: Your existing sensor echo connection

### New Wiring Required:
- **Pin 12 → Audio Amplifier Input** (replaces pin 11 connection)

### Complete Wiring Diagram:
```
Arduino Mega 2560 Connections:
├── Pin 10 → Left Sensor Trigger
├── Pin 11 → Left Sensor Echo (existing)
├── Pin 5  → Right Sensor Trigger  
├── Pin 6  → Right Sensor Echo
├── Pin 12 → Audio Amplifier Input (NEW)
└── Pin 9  → Keep existing connection
```

## Timer1 PWM on Arduino Mega 2560

The Mega has three Timer1 PWM outputs:
- **Pin 11**: OC1A (used for your sensor echo)
- **Pin 12**: OC1B (used for audio output)
- **Pin 13**: OC1C (available if needed)

## Installation Instructions

1. **Open Arduino IDE**
2. **Navigate to this folder**: `dj_scratch_progmem_mega/`
3. **Open**: `dj_scratch_progmem_mega.ino`
4. **Upload to Arduino Mega 2560**
5. **Connect pin 12** to your audio amplifier

## Control Scheme

- **Left Hand**: Wave to play/pause audio
- **Right Hand**: Quick movements trigger scratch effects
- **Both Hands**: Scratch mode with variable speed control

## Audio Features

- **Crystal clear playback** using Timer1 PWM
- **Real-time scratching** with forward/backward control
- **Variable speed**: 0.1x to 4.0x playback speed
- **26KB audio stored** in PROGMEM flash memory

## Troubleshooting

### No Audio Output:
- Verify pin 12 is connected to amplifier input
- Check that Timer1 is not being used by other code
- Ensure proper power supply to amplifier

### Sensor Issues:
- Verify sensor connections on pins 10/11 and 5/6
- Check that sensors have proper 5V power and ground

### Compilation Errors:
- Make sure you're only opening this folder in Arduino IDE
- Don't open both Uno and Mega versions simultaneously 