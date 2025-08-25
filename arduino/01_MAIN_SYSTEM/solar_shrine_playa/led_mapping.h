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
// Six segments per hand, including PALM strip between INDEX and THUMB
enum Segment  { THUMB = 0, PALM = 1, INDEX = 2, MIDDLE = 3, RING = 4, PINKY = 5, SEGMENT_COUNT = 6 };

// Per-finger LED counts for each hand. These add up to 60 per hand.
// Tune these to match your exact solder layout. The defaults below are
// estimated from the photo; adjust if the debug view does not align.
// Lengths per segment (thumb, palm, index, middle, ring, pinky) per hand; sums to 60
static const uint8_t LEFT_SEGMENT_LENGTHS[SEGMENT_COUNT]  = { 7, 5, 13, 13, 12, 10 };
static const uint8_t RIGHT_SEGMENT_LENGTHS[SEGMENT_COUNT] = { 7, 5, 13, 13, 12, 10 };

// Starting indices for each finger block are derived from consecutive sums
inline uint16_t handBaseIndex(HandSide side) {
  return (side == LEFT_HAND) ? 0 : (NUM_LEDS / 2);
}

inline const uint8_t* segmentLengths(HandSide side) { return (side == LEFT_HAND) ? LEFT_SEGMENT_LENGTHS : RIGHT_SEGMENT_LENGTHS; }
inline uint16_t segmentStartIndex(HandSide side, Segment seg) {
  const uint8_t* lengths = segmentLengths(side);
  uint16_t start = handBaseIndex(side);
  for (uint8_t s = 0; s < (uint8_t)seg; ++s) start += lengths[s];
  return start;
}
inline uint8_t segmentLength(HandSide side, Segment seg) { return segmentLengths(side)[(uint8_t)seg]; }

// Paint helpers
inline void setSegmentColor(HandSide side, Segment seg, const CRGB& color) {
  uint16_t start = segmentStartIndex(side, seg);
  uint16_t len   = segmentLength(side, seg);
  for (uint16_t i = 0; i < len; ++i) {
    leds[start + i] = color;
  }
}

inline void paintHandFlatColor(HandSide side, const CRGB& color) {
  for (uint8_t s = 0; s < SEGMENT_COUNT; ++s) setSegmentColor(side, (Segment)s, color);
}

// Debug view: shows each finger block with a distinct color so you can verify mapping
inline void debug_show_finger_blocks() {
  const CRGB debugColors[5] = {
    CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::Yellow, CRGB::Magenta
  };
  // Left hand blocks
  for (uint8_t s = 0; s < SEGMENT_COUNT; ++s) setSegmentColor(LEFT_HAND, (Segment)s, debugColors[s % 5]);
  // Right hand blocks
  for (uint8_t s = 0; s < SEGMENT_COUNT; ++s) setSegmentColor(RIGHT_HAND, (Segment)s, debugColors[s % 5]);
}

// ===============================
// Custom traversal path utilities
// ===============================
enum FingerDirection { BASE_TO_TIP = 0, TIP_TO_BASE = 1 };

inline void appendSegmentToPath(HandSide side, Segment seg, FingerDirection dir,
                               uint16_t* out, uint16_t& cursor) {
  uint16_t start = segmentStartIndex(side, seg);
  uint8_t len = segmentLength(side, seg);
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
  // Left hand serpentine (reverted to previous closer version):
  // Start: top of PINKY -> down to base; loop to bottom of RING -> up; loop to top of MIDDLE -> down;
  // then loop to bottom of INDEX -> up; then PALM up; then THUMB up.
  appendSegmentToPath(LEFT_HAND, PINKY,  TIP_TO_BASE, outIndices, outLen); // pinky down
  appendSegmentToPath(LEFT_HAND, RING,   BASE_TO_TIP, outIndices, outLen); // ring up
  appendSegmentToPath(LEFT_HAND, MIDDLE, TIP_TO_BASE, outIndices, outLen); // middle down
  appendSegmentToPath(LEFT_HAND, INDEX,  BASE_TO_TIP, outIndices, outLen); // index up
  appendSegmentToPath(LEFT_HAND, PALM,   BASE_TO_TIP, outIndices, outLen); // palm up
  appendSegmentToPath(LEFT_HAND, THUMB,  BASE_TO_TIP, outIndices, outLen); // thumb up

  // Right hand (mirrored continuation): thumb BASE->TIP, middle BASE->TIP, index TIP->BASE, index BASE->TIP, ring TIP->BASE, pinky BASE->TIP
  appendSegmentToPath(RIGHT_HAND, THUMB,  BASE_TO_TIP, outIndices, outLen);
  appendSegmentToPath(RIGHT_HAND, PALM,   BASE_TO_TIP, outIndices, outLen);
  appendSegmentToPath(RIGHT_HAND, INDEX,  TIP_TO_BASE, outIndices, outLen);
  appendSegmentToPath(RIGHT_HAND, MIDDLE, BASE_TO_TIP, outIndices, outLen);
  appendSegmentToPath(RIGHT_HAND, RING,   TIP_TO_BASE, outIndices, outLen);
  appendSegmentToPath(RIGHT_HAND, PINKY,  BASE_TO_TIP, outIndices, outLen);

  // Safety: if not all LEDs were covered due to length mismatches, fill remaining in order
  while (outLen < NUM_LEDS) {
    outIndices[outLen] = outLen;
    ++outLen;
  }
}

// ===============================
// 2D spatial mapping (x,y in 0..1)
// ===============================
inline void build_spatial_map(float* xOut, float* yOut) {
  // Horizontal positions per segment. Tunable constants to correct real-world spacing.
  // Keys are Segment enum order: { THUMB, PALM, INDEX, MIDDLE, RING, PINKY }
  // Left-hand Xs roughly match photo: thumb near center, pinky outer-left.
  static const float LEFT_SEGMENT_X[SEGMENT_COUNT]  = { 0.46f, 0.41f, 0.34f, 0.28f, 0.20f, 0.12f };
  // Right-hand Xs symmetric; keep as-is (user reported correct behavior)
  static const float RIGHT_SEGMENT_X[SEGMENT_COUNT] = { 0.54f, 0.59f, 0.66f, 0.72f, 0.80f, 0.88f };
  // Revert vertical offsets to zero (use trims in visual if needed)
  static const float LEFT_SEGMENT_Y_OFFSET[SEGMENT_COUNT]  = { 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f };
  static const float RIGHT_SEGMENT_Y_OFFSET[SEGMENT_COUNT] = { 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f };

  auto clamp01 = [](float v) -> float { if (v < 0.0f) return 0.0f; if (v > 1.0f) return 1.0f; return v; };

  // Left order from outer to inner: PINKY, RING, MIDDLE, INDEX, PALM, THUMB
  Segment leftOrder[SEGMENT_COUNT]  = { PINKY, RING, MIDDLE, INDEX, PALM, THUMB };
  // Right order from inner to outer: THUMB, PALM, INDEX, MIDDLE, RING, PINKY
  Segment rightOrder[SEGMENT_COUNT] = { THUMB, PALM, INDEX, MIDDLE, RING, PINKY };

  // Directions per left segment (match documented index ranges):
  // THUMB up, PALM up, INDEX up, MIDDLE down, RING up, PINKY down
  FingerDirection leftDir[SEGMENT_COUNT]  = { BASE_TO_TIP, BASE_TO_TIP, BASE_TO_TIP, TIP_TO_BASE, BASE_TO_TIP, TIP_TO_BASE };
  FingerDirection rightDir[SEGMENT_COUNT] = { TIP_TO_BASE, BASE_TO_TIP, TIP_TO_BASE, BASE_TO_TIP, TIP_TO_BASE, BASE_TO_TIP };

  // Fill left hand
  for (uint8_t s = 0; s < SEGMENT_COUNT; ++s) {
    Segment seg = leftOrder[s];
    float x = LEFT_SEGMENT_X[(uint8_t)seg];
    uint16_t start = segmentStartIndex(LEFT_HAND, seg);
    uint8_t len = segmentLength(LEFT_HAND, seg);
    for (uint8_t i = 0; i < len; ++i) {
      uint8_t pos = (leftDir[s] == BASE_TO_TIP) ? i : (len - 1 - i);
      xOut[start + i] = x;
      float y = (float)pos / (float)(len - 1);
      y += LEFT_SEGMENT_Y_OFFSET[(uint8_t)seg];
      yOut[start + i] = clamp01(y);
    }
  }

  // Fill right hand
  for (uint8_t s = 0; s < SEGMENT_COUNT; ++s) {
    Segment seg = rightOrder[s];
    float x = RIGHT_SEGMENT_X[(uint8_t)seg];
    uint16_t start = segmentStartIndex(RIGHT_HAND, seg);
    uint8_t len = segmentLength(RIGHT_HAND, seg);
    for (uint8_t i = 0; i < len; ++i) {
      uint8_t pos = (rightDir[s] == BASE_TO_TIP) ? i : (len - 1 - i);
      xOut[start + i] = x;
      float y = (float)pos / (float)(len - 1);
      y += RIGHT_SEGMENT_Y_OFFSET[(uint8_t)seg];
      yOut[start + i] = clamp01(y);
    }
  }
}

// ===============================
// Discrete grid mapping per hand (columns = segments, rows = max segment length)
// ===============================

inline uint8_t hand_num_columns() { return SEGMENT_COUNT; }

inline uint8_t hand_num_rows(HandSide side) {
  const uint8_t* lengths = segmentLengths(side);
  uint8_t maxLen = 0;
  for (uint8_t s = 0; s < SEGMENT_COUNT; ++s) if (lengths[s] > maxLen) maxLen = lengths[s];
  return maxLen; // rows measured from 0 (tip) to maxLen-1 (base)
}

inline Segment column_to_segment(HandSide side, uint8_t col) {
  Segment leftOrder[SEGMENT_COUNT]  = { PINKY, RING, MIDDLE, INDEX, PALM, THUMB };
  Segment rightOrder[SEGMENT_COUNT] = { THUMB, PALM, INDEX, MIDDLE, RING, PINKY };
  return (side == LEFT_HAND) ? leftOrder[col] : rightOrder[col];
}

inline FingerDirection segment_direction(HandSide side, Segment seg) {
  // Must match the authoritative left-hand directions used elsewhere:
  // THUMB up, PALM up, INDEX up, MIDDLE down, RING up, PINKY down
  FingerDirection leftDir[SEGMENT_COUNT]  = { BASE_TO_TIP, BASE_TO_TIP, BASE_TO_TIP, TIP_TO_BASE, BASE_TO_TIP, TIP_TO_BASE };
  FingerDirection rightDir[SEGMENT_COUNT] = { BASE_TO_TIP, BASE_TO_TIP, TIP_TO_BASE, BASE_TO_TIP, TIP_TO_BASE, BASE_TO_TIP };
  return (side == LEFT_HAND) ? leftDir[(uint8_t)seg] : rightDir[(uint8_t)seg];
}

// Map grid (col,row) -> LED index; returns 0xFFFF if out-of-bounds for that segment
inline uint16_t hand_xy_to_index(HandSide side, uint8_t col, uint8_t row) {
  Segment seg = column_to_segment(side, col);
  uint8_t len = segmentLength(side, seg);
  if (row >= len) return 0xFFFF;
  uint16_t start = segmentStartIndex(side, seg);
  FingerDirection dir = segment_direction(side, seg);
  if (dir == BASE_TO_TIP) {
    return start + row; // row 0 is BASE in this orientation
  } else {
    // TIP_TO_BASE: row 0 should be TIP
    return start + (len - 1 - row);
  }
}

// ===============================
// Sequential index mapping helpers (1-based within a hand)
// ===============================

// Left hand order and cumulative lengths for 1..60 indexing
inline uint16_t left_seq_to_index(uint8_t seq1Based) {
  if (seq1Based < 1 || seq1Based > 60) return 0xFFFF;
  // Segment order: PINKY(10), RING(12), MIDDLE(13), INDEX(13), PALM(5), THUMB(7)
  const Segment order[SEGMENT_COUNT] = { PINKY, RING, MIDDLE, INDEX, PALM, THUMB };
  const uint8_t cum[SEGMENT_COUNT]   = { 10, 22, 35, 48, 53, 60 };
  uint8_t segIdx = 0;
  while (seq1Based > cum[segIdx]) segIdx++;
  uint8_t prevCum = (segIdx == 0) ? 0 : cum[segIdx - 1];
  uint8_t offsetInSeg = (uint8_t)(seq1Based - prevCum - 1); // 0-based within segment (serpentine progression)

  Segment seg = order[segIdx];
  uint8_t len = segmentLength(LEFT_HAND, seg);
  uint16_t start = segmentStartIndex(LEFT_HAND, seg);
  FingerDirection dir = segment_direction(LEFT_HAND, seg);

  if (dir == BASE_TO_TIP) {
    // Sequencing increases from base upwards for BASE_TO_TIP
    uint8_t rowFromBase = offsetInSeg;
    return start + rowFromBase;
  } else { // TIP_TO_BASE
    // Sequencing increases from tip downward for TIP_TO_BASE
    uint8_t rowFromTip = offsetInSeg;
    return start + (len - 1 - rowFromTip);
  }
}

// Right hand order and cumulative lengths for 1..60 indexing
inline uint16_t right_seq_to_index(uint8_t seq1Based) {
  if (seq1Based < 1 || seq1Based > 60) return 0xFFFF;
  // Segment order: THUMB(7), PALM(5), INDEX(13), MIDDLE(13), RING(12), PINKY(10)
  const Segment order[SEGMENT_COUNT] = { THUMB, PALM, INDEX, MIDDLE, RING, PINKY };
  const uint8_t cum[SEGMENT_COUNT]   = { 7, 12, 25, 38, 50, 60 };
  uint8_t segIdx = 0;
  while (seq1Based > cum[segIdx]) segIdx++;
  uint8_t prevCum = (segIdx == 0) ? 0 : cum[segIdx - 1];
  uint8_t offsetInSeg = (uint8_t)(seq1Based - prevCum - 1);

  Segment seg = order[segIdx];
  uint8_t len = segmentLength(RIGHT_HAND, seg);
  uint16_t start = segmentStartIndex(RIGHT_HAND, seg);
  FingerDirection dir = segment_direction(RIGHT_HAND, seg);

  if (dir == BASE_TO_TIP) {
    uint8_t rowFromBase = offsetInSeg;
    return start + rowFromBase;
  } else {
    uint8_t rowFromTip = offsetInSeg;
    return start + (len - 1 - rowFromTip);
  }
}

#endif // LED_MAPPING_H
