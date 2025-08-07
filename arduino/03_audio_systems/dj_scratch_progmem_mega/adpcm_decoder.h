/*
 * ADPCM Decoder for Arduino
 * Decodes 4-bit ADPCM audio data to 8-bit PCM in real-time
 * Optimized for memory-constrained microcontrollers
 */

#ifndef ADPCM_DECODER_H
#define ADPCM_DECODER_H

#include <avr/pgmspace.h>
#include "audio_data_adpcm.h"

class ADPCMDecoder {
private:
  int16_t predicted_sample;
  uint8_t step_index;
  bool high_nibble;
  uint16_t byte_index;
  
public:
  ADPCMDecoder() {
    reset();
  }
  
  void reset() {
    predicted_sample = 0;
    step_index = 0;
    high_nibble = false;
    byte_index = 0;
  }
  
  uint8_t decode_next_sample() {
    // Check bounds
    if (byte_index >= AUDIO_ADPCM_SIZE) {
      return 128; // Return silence if we've reached the end
    }
    
    // Get the current ADPCM byte
    uint8_t adpcm_byte = pgm_read_byte(&audioDataADPCM[byte_index]);
    
    // Extract the 4-bit nibble
    uint8_t nibble;
    if (high_nibble) {
      nibble = (adpcm_byte >> 4) & 0x0F;
      byte_index++; // Move to next byte after processing high nibble
    } else {
      nibble = adpcm_byte & 0x0F;
    }
    high_nibble = !high_nibble;
    
    // Decode the nibble
    return decode_nibble(nibble);
  }
  
  uint16_t get_position() {
    return byte_index * 2 + (high_nibble ? 1 : 0);
  }
  
  void set_position(uint16_t sample_pos) {
    byte_index = sample_pos / 2;
    high_nibble = (sample_pos % 2) == 1;
    
    // Clamp to valid range
    if (byte_index >= AUDIO_ADPCM_SIZE) {
      byte_index = AUDIO_ADPCM_SIZE - 1;
      high_nibble = true;
    }
  }
  
  bool is_end() {
    return byte_index >= AUDIO_ADPCM_SIZE;
  }
  
private:
  uint8_t decode_nibble(uint8_t nibble) {
    // Extract sign and magnitude
    uint8_t sign = nibble & 8;
    uint8_t delta = nibble & 7;
    
    // Get current step size
    uint16_t step = pgm_read_word(&adpcm_step_table[step_index]);
    
    // Calculate difference
    int16_t vpdiff = step >> 3;
    if (delta & 4) vpdiff += step;
    if (delta & 2) vpdiff += step >> 1;
    if (delta & 1) vpdiff += step >> 2;
    
    // Apply sign
    if (sign) {
      predicted_sample -= vpdiff;
    } else {
      predicted_sample += vpdiff;
    }
    
    // Clamp predicted sample
    if (predicted_sample > 32767) {
      predicted_sample = 32767;
    } else if (predicted_sample < -32768) {
      predicted_sample = -32768;
    }
    
    // Update step index
    int8_t index_adjust = pgm_read_byte(&adpcm_index_table[delta]);
    step_index += index_adjust;
    if (step_index > 88) {
      step_index = 88;
    } else if (step_index < 0) {
      step_index = 0;
    }
    
    // Convert 16-bit signed to 8-bit unsigned
    return (uint8_t)((predicted_sample / 256) + 128);
  }
};

#endif // ADPCM_DECODER_H