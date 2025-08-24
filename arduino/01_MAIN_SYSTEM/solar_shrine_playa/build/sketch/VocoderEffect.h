#line 1 "C:\\Users\\cityz\\IllI\\play\\soulShine\\solar-shrine-arduino\\arduino\\01_MAIN_SYSTEM\\solar_shrine_playa\\VocoderEffect.h"
#ifndef VOCODER_EFFECT_H
#define VOCODER_EFFECT_H

#include <Arduino.h>
#include <MozziGuts.h>
#include <Oscil.h>
#include <RollingAverage.h>
#include <tables/saw2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>
#include <tables/sin2048_int8.h>
#include <avr/pgmspace.h>

class VocoderEffect {
public:
  VocoderEffect();
  void enter();
  void exit();
  void update(bool leftHand, bool rightHand, float d1, float d2);
  int audio();
  void feedAudioSample(int16_t sample);

private:
    static const int VOCODER_BANDS = 4;
    static const int BUFFER_SIZE = 16;

    Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> robotOsc1;
    Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, AUDIO_RATE> robotOsc2;
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> robotOsc3;
    Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> robotOsc4;

    volatile int16_t currentSample;
    volatile int16_t lastSample;
    RollingAverage<int, 8> audioEnvelope;

    struct SimpleBand {
      RollingAverage<int, 4> envelope;
      int level;
      int targetFreq;
    };
    SimpleBand vocoderBands[VOCODER_BANDS];

    int lowpass1, lowpass2, lowpass3;
    int highpass1, highpass2;
    int bandpass1, bandpass2;

    int robotPitch;
    int robotFormant;
    int robotIntensity;
    int vocoderVolume;
    bool vocoderActive;

    int harmonicMix;
    int voiceModulation;
    int audioActivity;

    static const int MIN_ROBOT_PITCH = 80;
    static const int MAX_ROBOT_PITCH = 600;
    static const int MIN_FORMANT = 60;
    static const int MAX_FORMANT = 200;
};

#endif // VOCODER_EFFECT_H
