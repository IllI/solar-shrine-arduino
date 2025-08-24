#line 1 "C:\\Users\\cityz\\IllI\\play\\soulShine\\solar-shrine-arduino\\arduino\\01_MAIN_SYSTEM\\solar_shrine_playa\\DjScratch.cpp"
#include "DjScratch.h"
#include <avr/pgmspace.h>
#include "audio_data.h"
#include "VocoderEffect.h"

namespace DjScratch {

  // Private variables
  namespace {
    volatile int32_t sampleIndex = 0;
    volatile uint8_t sampleCounter = 0;
    volatile uint8_t playState = 0;
    volatile uint8_t playbackSpeed = 5;
    volatile int8_t scratchSpeed = 1;
    volatile bool isScratchMode = false;
  }

  void setup() {
    // This is called once during the main setup.
    // We don't need to do anything here because enter() handles the setup.
  }

  void enter() {
    Serial.println(F("Entering DJ Scratch mode..."));
    pinMode(12, OUTPUT);
    TCCR1A = _BV(COM1B1) | _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(CS10);
    ICR1 = 399; // 20kHz
    OCR1B = 200; // 50% duty cycle (silence)
    TIMSK1 = _BV(OCIE1B);

    sampleIndex = 0;
    sampleCounter = 0;
    playState = 1; // Start playing immediately
    playbackSpeed = 5;
    scratchSpeed = 1;
    isScratchMode = false;
    Serial.println(F("Mode: DJ Scratch (PROGMEM + Timer1)"));
  }

  void exit() {
    TIMSK1 &= ~_BV(OCIE1B);
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1B = 0;
    digitalWrite(12, LOW);
  }

  void update(bool leftHand, bool rightHand, float d1, float d2) {
    if (leftHand) {
      if (playState == 0) {
        playState = 1;
        Serial.println("DJ: Play");
      }
    } else {
      if (playState == 1) {
        playState = 0;
        Serial.println("DJ: Stop");
      }
    }

    if (rightHand) {
      int mappedDistance = map(d2 * 10, 20, 200, 0, 100);
      mappedDistance = constrain(mappedDistance, 0, 100);

      static int lastMappedDistance = 50;
      int distanceChange = abs(mappedDistance - lastMappedDistance);

      if (distanceChange > 20) {
        isScratchMode = true;
        scratchSpeed = map(mappedDistance, 0, 100, -3, 3);
        if (scratchSpeed == 0) scratchSpeed = 1;
      } else {
        isScratchMode = false;
        playbackSpeed = map(mappedDistance, 0, 100, 10, 1);
        playbackSpeed = constrain(playbackSpeed, 1, 10);
      }
      lastMappedDistance = mappedDistance;
    } else {
      isScratchMode = false;
      playbackSpeed = 5;
    }
  }

  void handleISR() {
    sampleCounter++;
    uint8_t currentSpeed = isScratchMode ? 2 : playbackSpeed;

    if (sampleCounter >= currentSpeed) {
      sampleCounter = 0;
      if (playState == 1) {
        if (sampleIndex < 0) sampleIndex = 0;
        if (sampleIndex >= AUDIO_SAMPLE_COUNT) sampleIndex = 0;

        uint8_t sample = pgm_read_byte(&audioData[sampleIndex]);
        int16_t amp = ((int16_t)sample - 128) * 4;
        amp = constrain(amp, -128, 127);
        

        
        OCR1B = ((uint32_t)(amp + 128) * ICR1) / 255;

        if (isScratchMode) {
          sampleIndex += scratchSpeed;
          if (sampleIndex < 0) sampleIndex = AUDIO_SAMPLE_COUNT - 1;
          if (sampleIndex >= AUDIO_SAMPLE_COUNT) sampleIndex = 0;
        } else {
          sampleIndex++;
          if (sampleIndex >= AUDIO_SAMPLE_COUNT) sampleIndex = 0;
        }
      } else {
        OCR1B = ICR1 / 2; // Silence
      }
    }
  }
}