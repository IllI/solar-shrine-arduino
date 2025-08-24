#line 1 "C:\\Users\\cityz\\IllI\\play\\soulShine\\solar-shrine-arduino\\arduino\\01_MAIN_SYSTEM\\solar_shrine_playa\\ThereminEffect.h"
#ifndef THEREMINEFFECT_H
#define THEREMINEFFECT_H

#include <Arduino.h>
#include <MozziGuts.h>
#include <Oscil.h>
#include <tables/sin2048_int8.h>

class ThereminEffect {
public:
    ThereminEffect();
    void setup();
    void enter();
    void exit();
    void update(int rightHand, int leftHand, int d1, int d2);
    int audio();

private:
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aOscil;
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aVibrato;
    int theremin_freq;
    int theremin_vol;
};

#endif // THEREMINEFFECT_H