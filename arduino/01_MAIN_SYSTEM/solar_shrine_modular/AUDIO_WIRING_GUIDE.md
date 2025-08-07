# Solar Shrine - Audio Effects Wiring and Pin Configuration

This document provides a definitive guide to the audio output configuration for the Solar Shrine project, ensuring all developers understand the hardware constraints and software solutions.

## 1. The Golden Rule: Pin 12 is for Audio

Due to the custom-soldered protoboard shield, **all audio effects MUST use Pin 12 for output.** The shield has a 1KΩ current-limiting resistor physically soldered between Pin 12 and the audio output jack. This hardware is fixed and cannot be changed.

**Any attempt to configure audio on a different pin will result in no sound.**

### Confirmed Pin Assignments

Refer to the `CUSTOM_SHIELD_WIRING.md` for a complete hardware overview. The key takeaway for audio is:

- **Pin 12**: Sole audio output pin.
- **Pin 10 & 11**: Left ultrasonic sensor.
- **Pin 8**: Unused by the core audio/sensor system, but was previously considered for the DJ Scratch effect. This is not possible due to the shield wiring.

## 2. Managing Timer Conflicts on Pin 12

Three different audio effects share Pin 12. This is possible because they are never active simultaneously. However, they use different timer mechanisms, which requires careful software management.

### Effect #1: Theremin

- **Library:** `NewTone()`
- **Pin Usage:** Directly drives Pin 12.
- **Timer Usage:** `NewTone()` uses Timer2. It does not conflict with the ultrasonic sensors.
- **Status:** Works correctly on Pin 12.

### Effect #2: DJ Scratch

- **Method:** Custom Timer1 PWM
- **Pin Usage:** Configures Timer1 to output a PWM signal on Pin 12 (OC1B).
- **Timer Usage:** Uses Timer1. This **conflicts** with the Mozzi library, but since the DJ Scratch and Alien effects are mutually exclusive, this is managed by the main state machine.
- **Status:** Works correctly on Pin 12.

### Effect #3: Alien (Mozzi)

- **Library:** `Mozzi`
- **Pin Usage:** Configured for `STANDARD_PLUS` mode, which uses Pin 9 for audio. **This is a problem.**
- **Timer Usage:** Mozzi uses Timer1 for its audio generation loop (`audioHook()`). This timer is essential for Mozzi to function.

## 3. The Alien Effect: A Special Case

The Alien effect, which uses the **Mozzi** library, presents a unique challenge due to a hardware timer conflict.

### The Timer1 Conflict

- **Mozzi** requires **Timer1** for audio synthesis.
- The **Left Ultrasonic Sensor** is wired to **Pin 10**, which is a critical pin for Timer1 on the Arduino Mega.

**Result:** It is physically impossible to run Mozzi and read from the left sensor at the same time. When the `ALIEN` effect is active, Mozzi takes over Timer1, and the left sensor stops working correctly.

### The Software Workaround (And What to Expect)

To allow the Alien effect to produce sound despite this limitation, the following software solution is in place:

1.  **Left Sensor is Intentionally Disabled:** When `currentEffect` in `solar_shrine_modular.ino` is `ALIEN`, the code **stops reading the left sensor**. This is not a bug; it is a required workaround.

2.  **Volume is Set to Maximum:** Because the left sensor cannot be used for volume control, the `alien_effect.cpp` code is designed to **default to full volume** whenever the `ALIEN` effect is active. The pitch can still be controlled with the right hand.

**In summary: When you switch to the Alien effect, you will hear sound, and you can change its pitch with your right hand. Your left hand will have no effect on the volume, and the sensor value will appear fixed because it is not being read. This is the correct and expected behavior for the current hardware.**

This guide should prevent future confusion regarding pin assignments and provide a clear path for debugging or extending the audio system.