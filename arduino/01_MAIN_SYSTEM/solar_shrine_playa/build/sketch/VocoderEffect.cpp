#line 1 "C:\\Users\\cityz\\IllI\\play\\soulShine\\solar-shrine-arduino\\arduino\\01_MAIN_SYSTEM\\solar_shrine_playa\\VocoderEffect.cpp"
#include "VocoderEffect.h"

VocoderEffect::VocoderEffect() :
  robotOsc1(SAW2048_DATA),
  robotOsc2(SQUARE_NO_ALIAS_2048_DATA),
  robotOsc3(SIN2048_DATA),
  robotOsc4(SAW2048_DATA),
  audioEnvelope() {
  
  robotPitch = 200;
  robotFormant = 128;
  robotIntensity = 180;
  vocoderVolume = 255;
  vocoderActive = false;
  harmonicMix = 128;
  voiceModulation = 100;
  audioActivity = 0;
  currentSample = 0;
  lastSample = 0;

  lowpass1 = lowpass2 = lowpass3 = 0;
  highpass1 = highpass2 = 0;
  bandpass1 = bandpass2 = 0;

  for (int i = 0; i < VOCODER_BANDS; i++) {
    vocoderBands[i].level = 0;
  }
  vocoderBands[0].targetFreq = 150;
  vocoderBands[1].targetFreq = 400;
  vocoderBands[2].targetFreq = 800;
  vocoderBands[3].targetFreq = 1600;
}

void VocoderEffect::enter() {
    robotOsc1.setFreq((float)robotPitch);
    robotOsc2.setFreq((float)(robotPitch * 1.5f));
    robotOsc3.setFreq((float)(robotPitch * 0.5f));
    robotOsc4.setFreq((float)(robotPitch * 2.0f));
    
    for (int i = 0; i < VOCODER_BANDS; i++) {
      vocoderBands[i].level = 0;
    }
    
    currentSample = 0;
    lastSample = 0;
    audioActivity = 0;
    
    lowpass1 = lowpass2 = lowpass3 = 0;
    highpass1 = highpass2 = 0;
    bandpass1 = bandpass2 = 0;
    
    vocoderActive = true;
    Serial.println(F("Vocoder Effect: Enhanced robot voice active"));
}

void VocoderEffect::exit() {
    vocoderActive = false;
}

void VocoderEffect::feedAudioSample(int16_t sample) {
    if (!vocoderActive) return;
    
    lastSample = currentSample;
    currentSample = sample;
    
    int sampleLevel = abs(sample);
    audioActivity = audioEnvelope.next(sampleLevel);
    
    lowpass1 += (sample - lowpass1) >> 3;
    lowpass2 += (lowpass1 - lowpass2) >> 4;
    lowpass3 += (lowpass2 - lowpass3) >> 5;
    
    highpass1 = sample - lowpass1;
    highpass2 = sample - lowpass2;
    
    bandpass1 = lowpass1 - lowpass2;
    bandpass2 = lowpass2 - lowpass3;
    
    vocoderBands[0].level = vocoderBands[0].envelope.next(abs(lowpass3) >> 2);
    vocoderBands[1].level = vocoderBands[1].envelope.next(abs(bandpass2) >> 1);
    vocoderBands[2].level = vocoderBands[2].envelope.next(abs(bandpass1));
    vocoderBands[3].level = vocoderBands[3].envelope.next(abs(highpass1) >> 1);
}

void VocoderEffect::update(bool leftHand, bool rightHand, float d1, float d2) {
    if (rightHand) {
      int distance = constrain((int)d2, 1, 20);
      robotPitch = map(distance, 1, 20, MAX_ROBOT_PITCH, MIN_ROBOT_PITCH);
      robotIntensity = map(distance, 1, 20, 255, 120);
      harmonicMix = map(distance, 1, 20, 200, 80);
      vocoderActive = true;
    } else {
      robotPitch = 200;
      robotIntensity = 150;
      harmonicMix = 120;
    }
    
    if (leftHand) {
      int distance = constrain((int)d1, 1, 20);
      robotFormant = map(distance, 1, 20, MAX_FORMANT, MIN_FORMANT);
      voiceModulation = map(distance, 1, 20, 200, 50);
      vocoderVolume = map(distance, 1, 20, 255, 100);
      vocoderActive = true;
    } else {
      robotFormant = 128;
      voiceModulation = 100;
      vocoderVolume = 180;
    }
    
    float formantMod = robotFormant / 128.0f;
    
    robotOsc1.setFreq((float)(robotPitch * formantMod));
    robotOsc2.setFreq((float)(robotPitch * 1.5f * formantMod));
    robotOsc3.setFreq((float)(robotPitch * 0.5f * formantMod));
    robotOsc4.setFreq((float)(robotPitch * 2.0f * formantMod));
}

int VocoderEffect::audio() {
    if (!vocoderActive) {
      return 0;
    }
    
    int carrier1 = robotOsc1.next();
    int carrier2 = robotOsc2.next();
    int carrier3 = robotOsc3.next();
    int carrier4 = robotOsc4.next();
    
    int vocodedOutput = 0;
    
    int bassCarrier = ((int32_t)carrier3 * vocoderBands[0].level) >> 8;
    vocodedOutput += bassCarrier >> 2;
    
    int midLowCarrier = ((int32_t)carrier1 * vocoderBands[1].level) >> 7;
    vocodedOutput += midLowCarrier >> 1;
    
    int midHighCarrier = ((int32_t)carrier2 * vocoderBands[2].level) >> 7;
    vocodedOutput += midHighCarrier >> 2;
    
    int trebleCarrier = ((int32_t)carrier4 * vocoderBands[3].level) >> 8;
    vocodedOutput += trebleCarrier >> 3;
    
    vocodedOutput = ((int32_t)vocodedOutput * robotIntensity) >> 8;
    
    int harmonics = ((int32_t)vocodedOutput * harmonicMix) >> 9;
    vocodedOutput += harmonics;
    
    int modulated = ((int32_t)vocodedOutput * voiceModulation) >> 8;
    vocodedOutput = (vocodedOutput + modulated) >> 1;
    
    vocodedOutput = constrain(vocodedOutput, -127, 127);
    
    return (vocodedOutput * 255) >> 7;
}