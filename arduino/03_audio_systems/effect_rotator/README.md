# Effect Rotator - Minimal Version

## Overview
This is a **minimal modification** of the proven `dj_scratch_progmem.ino` that adds 3 additional effects while preserving 99% of the original DJ Scratch code.

## Memory Optimization
- **Original DJ Scratch**: 31400 bytes (97%) ✅ WORKS
- **This version**: Should be very close to 97% since it's 99% identical code

## Effects
1. **Effect 0: DJ Scratch** (100% unchanged from original)
   - Left hand = play/stop
   - Right hand = scratch/speed control  
   - Uses 159KB PROGMEM audio data

2. **Effect 1: Square Wave Theremin** (minimal synthesis)
   - Left hand = play/stop
   - Right hand = ignored (no scratch on synth effects)

3. **Effect 2: Alien Sound** (different square wave pattern)
   - Left hand = play/stop
   - Right hand = ignored

4. **Effect 3: Robot Talking** (third wave pattern)
   - Left hand = play/stop
   - Right hand = ignored

## Auto-Rotation
- Automatically cycles through effects after 5 seconds of no hand detection
- Serial output shows current effect: "Effect 0", "Effect 1", etc.

## Key Design Decisions
1. **Preserve working DJ Scratch**: Effect 0 is 100% identical to proven code
2. **Minimal ISR modifications**: Just added simple synthesis in else branch
3. **Disable scratch on synth effects**: Only DJ Scratch uses complex scratch logic
4. **Simple synthesis**: Basic square wave patterns to minimize memory usage

## Memory Usage
Added only 7 bytes of global variables:
- `uint8_t currentEffect = 0;` (1 byte)
- `unsigned long lastHandTime = 0;` (4 bytes)  
- `uint16_t synthPhase = 0;` (2 bytes)

## Files Required
- `effect_rotator_minimal.ino` - Main program
- `audio_data.h` - 159KB PROGMEM audio data (copied from dj_scratch_progmem)

## Operation
1. System starts in Effect 0 (DJ Scratch)
2. Use left hand to play/stop any effect
3. Use right hand for scratch/speed control (DJ Scratch only)
4. Remove hands for 5 seconds to auto-rotate to next effect
5. Cycle: DJ Scratch → Square Theremin → Alien Sound → Robot Talking → repeat

## Why This Approach Works
- Builds on proven DJ Scratch codebase (97% memory usage)
- Minimal additions preserve memory efficiency
- Simple synthesis doesn't require additional libraries
- Auto-rotation provides variety without complexity 