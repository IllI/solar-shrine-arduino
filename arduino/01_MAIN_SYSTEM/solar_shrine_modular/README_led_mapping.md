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
- uint16_t hand_xy_to_index(HandSide side, uint8_t col, uint8_t row) // grid cell to LED index
- uint8_t hand_num_rows(HandSide side), uint8_t hand_num_columns()   // grid dimensions per hand
- uint16_t left_seq_to_index(uint8_t seq1Based), right_seq_to_index(uint8_t seq1Based)

Direction reference (from wrist toward fingertips):
- Left hand: THUMB BASE→TIP (bottom→top), PALM BASE→TIP, INDEX TIP→BASE, MIDDLE BASE→TIP, RING TIP→BASE, PINKY TIP→BASE.
- Right hand: THUMB is TIP→BASE (top→bottom), PALM BASE→TIP, INDEX TIP→BASE, MIDDLE BASE→TIP, RING TIP→BASE, PINKY BASE→TIP.
### Left-hand sequential index labels

Using lengths: PINKY 10, RING 12, MIDDLE 13, INDEX 13, PALM 5, THUMB 7.
Serpentine (TIP↔BASE per direction above):

- 1..10  : Left PINKY (tip→base)
- 11..22 : Left RING (base→tip)
- 23..35 : Left MIDDLE (tip→base)
- 36..48 : Left INDEX (base→tip)
For a small centered circle on the left hand, the expected LEDs are:
- Ring: 12–15
- Middle: 30–34
- Index: 38–41
If you do not observe these indices lighting, adjust the left-hand segment directions in `led_mapping.h` to match your wiring.
- 49..53 : Left PALM (base→tip)
- 54..60 : Left THUMB (base→tip)

These labels let you visually verify which LED numeric index should light for a small circle near the center (around the middle/index mid-rows).
### Grid Mapping

We model each hand as a discrete grid where columns correspond to segments and rows to LED positions along a segment (0 at fingertip/tip, increasing toward base depending on actual wiring direction; handled internally). Use `hand_xy_to_index(side, col, row)` to paint shapes by grid coordinates, independent of the serpentine wiring.

### Sequential Mapping (1..60 per hand)

For cases where you want to light specific sequential indices (e.g., Ring 12–15, Middle 30–34, Index 38–41 to form a small circle), use:

```
uint16_t i1 = left_seq_to_index(12);
uint16_t i2 = left_seq_to_index(30);
uint16_t i3 = left_seq_to_index(38);
```

This mapping is consistent with the left-hand run-lengths and directions listed above.


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

