# LED Hand Mapping for Solar Shrine Modular

This document explains how the 120-LED strip is mapped into two human-hand shapes for granular light control by finger.

Total layout:
- Total LEDs: 120 (0..119)
- Left hand: indices 0..59
- Right hand: indices 60..119
- Each hand is partitioned into five contiguous finger blocks: THUMB, INDEX, MIDDLE, RING, PINKY.

The mapping lives in `led_mapping.h` and provides helpers to set per-finger colors and to paint an entire hand.

## Visual Reference

The layout is based on this photo (stored alongside the sketch):

![LED Hands Layout](led_hands_layout.jpg)

If your physical daisy-chain order differs from the defaults, update the lengths in `led_mapping.h` to match your soldered segments.

## Default Finger Block Lengths

Per hand (summing to 60):

- THUMB: 7
- PALM: 5
- INDEX: 13
- MIDDLE: 13
- RING: 12
- PINKY: 10

These values are chosen to roughly match the photographed build and are easy to tweak. Adjust both hands symmetrically or independently.

## API Overview (led_mapping.h)

- enum HandSide { LEFT_HAND, RIGHT_HAND }
- enum Segment { THUMB, PALM, INDEX, MIDDLE, RING, PINKY }
- void setSegmentColor(HandSide side, Segment segment, const CRGB& color)
- void paintHandFlatColor(HandSide side, const CRGB& color)
- void debug_show_finger_blocks() — paints distinct colors per finger on both hands for verification
- void build_default_traversal(uint16_t* outIndices, uint16_t& outLen)
- void build_spatial_map(float* xOut, float* yOut)  // normalized x,y per LED

## Quick Test

Uncomment in setup():

```
// debug_show_finger_blocks();
// FastLED.show();
// delay(1000);
```

You should see five colored bands per hand (left: 0..59, right: 60..119). If a finger’s segment doesn’t align, edit LEFT_FINGER_LENGTHS and RIGHT_FINGER_LENGTHS in `led_mapping.h`.

## Using in Effects

From any effect update, you can drive specific fingers:

```
setFingerColor(LEFT_HAND, INDEX, CRGB::Orange);
setFingerColor(RIGHT_HAND, MIDDLE, CRGB::Cyan);
FastLED.show();
```

Keep LED writes lightweight inside audio loops. Prefer batching LED changes and calling FastLED.show() once per frame in the main loop.

