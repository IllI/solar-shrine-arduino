#ifndef LED_MAPPING_H
#define LED_MAPPING_H

#include <Arduino.h>
#include <FastLED.h>

// This module maps a 120-LED strip into two hands (left/right), 
// grouped by fingers for granular control.
//
// Assumptions:
// - Total LEDs: 120 (0..119)
// - Left hand uses indices [0..59], right hand uses [60..119]
// - Within each hand, the chain is physically contiguous; exact order may vary.
// - Finger block sizes below are a practical starting point and are easy to tweak.
//
// If your physical daisy-chain order differs, adjust the per-finger lengths below
// and re-upload. Use debug_show_finger_blocks() to verify visually.

#ifndef NUM_LEDS
#define NUM_LEDS 120
#endif

// Forward declaration of LED buffer from the main sketch
extern CRGB leds[NUM_LEDS];

enum HandSide { LEFT_HAND = 0, RIGHT_HAND = 1 };
enum Finger   { THUMB = 0, INDEX = 1, MIDDLE = 2, RING = 3, PINKY = 4 };

// Per-finger LED counts for each hand. These add up to 60 per hand.
// Tune these to match your exact solder layout. The defaults below are
// estimated from the photo; adjust if the debug view does not align.
static const uint8_t LEFT_FINGER_LENGTHS[5]  = { 10, 12, 14, 12, 12 }; // thumb..pinky = 60
static const uint8_t RIGHT_FINGER_LENGTHS[5] = { 10, 12, 14, 12, 12 }; // mirror lengths

// Starting indices for each finger block are derived from consecutive sums
inline uint16_t handBaseIndex(HandSide side) {
  return (side == LEFT_HAND) ? 0 : (NUM_LEDS / 2);
}

inline const uint8_t* fingerLengths(HandSide side) {
  return (side == LEFT_HAND) ? LEFT_FINGER_LENGTHS : RIGHT_FINGER_LENGTHS;
}

inline uint16_t fingerStartIndex(HandSide side, Finger finger) {
  const uint8_t* lengths = fingerLengths(side);
  uint16_t start = handBaseIndex(side);
  for (uint8_t f = 0; f < (uint8_t)finger; ++f) {
    start += lengths[f];
  }
  return start;
}

inline uint8_t fingerLength(HandSide side, Finger finger) {
  return fingerLengths(side)[(uint8_t)finger];
}

// Paint helpers
inline void setFingerColor(HandSide side, Finger finger, const CRGB& color) {
  uint16_t start = fingerStartIndex(side, finger);
  uint16_t len   = fingerLength(side, finger);
  for (uint16_t i = 0; i < len; ++i) {
    leds[start + i] = color;
  }
}

inline void paintHandFlatColor(HandSide side, const CRGB& color) {
  for (uint8_t f = 0; f < 5; ++f) {
    setFingerColor(side, (Finger)f, color);
  }
}

// Debug view: shows each finger block with a distinct color so you can verify mapping
inline void debug_show_finger_blocks() {
  const CRGB debugColors[5] = {
    CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::Yellow, CRGB::Magenta
  };
  // Left hand blocks
  for (uint8_t f = 0; f < 5; ++f) {
    setFingerColor(LEFT_HAND, (Finger)f, debugColors[f]);
  }
  // Right hand blocks (same palette)
  for (uint8_t f = 0; f < 5; ++f) {
    setFingerColor(RIGHT_HAND, (Finger)f, debugColors[f]);
  }
}

// ===============================
// Custom traversal path utilities
// ===============================
enum FingerDirection { BASE_TO_TIP = 0, TIP_TO_BASE = 1 };

inline void appendFingerToPath(HandSide side, Finger finger, FingerDirection dir,
                               uint16_t* out, uint16_t& cursor) {
  uint16_t start = fingerStartIndex(side, finger);
  uint8_t len = fingerLength(side, finger);
  if (dir == BASE_TO_TIP) {
    for (uint8_t i = 0; i < len; ++i) out[cursor++] = start + i;
  } else {
    for (int i = len - 1; i >= 0; --i) out[cursor++] = start + i;
  }
}

// Build a default serpentine path across both hands to match the photographed layout.
// You can tweak this order if your solder path differs.
inline void build_default_traversal(uint16_t* outIndices, uint16_t& outLen) {
  outLen = 0;
  // Left hand sequence from the photo: start at top of left pinky (tip),
  // descend to base; hop to index base and climb up; then descend middle; then ring; end at thumb base->tip.
  appendFingerToPath(LEFT_HAND, PINKY, TIP_TO_BASE, outIndices, outLen);   // pinky tip -> base
  appendFingerToPath(LEFT_HAND, INDEX, BASE_TO_TIP, outIndices, outLen);   // index base -> tip
  appendFingerToPath(LEFT_HAND, MIDDLE, TIP_TO_BASE, outIndices, outLen);  // middle tip -> base
  appendFingerToPath(LEFT_HAND, RING, BASE_TO_TIP, outIndices, outLen);    // ring base -> tip
  appendFingerToPath(LEFT_HAND, THUMB, BASE_TO_TIP, outIndices, outLen);   // thumb base -> tip

  // Right hand (mirrored continuation): thumb BASE->TIP, middle BASE->TIP, index TIP->BASE, index BASE->TIP, ring TIP->BASE, pinky BASE->TIP
  appendFingerToPath(RIGHT_HAND, THUMB, BASE_TO_TIP, outIndices, outLen);
  appendFingerToPath(RIGHT_HAND, MIDDLE, BASE_TO_TIP, outIndices, outLen);
  appendFingerToPath(RIGHT_HAND, INDEX, TIP_TO_BASE, outIndices, outLen);
  appendFingerToPath(RIGHT_HAND, INDEX, BASE_TO_TIP, outIndices, outLen);
  appendFingerToPath(RIGHT_HAND, RING, TIP_TO_BASE, outIndices, outLen);
  appendFingerToPath(RIGHT_HAND, PINKY, BASE_TO_TIP, outIndices, outLen);

  // Safety: if not all LEDs were covered due to length mismatches, fill remaining in order
  while (outLen < NUM_LEDS) {
    outIndices[outLen] = outLen;
    ++outLen;
  }
}

#endif // LED_MAPPING_H

