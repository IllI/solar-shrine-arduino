#line 1 "C:\\Users\\cityz\\IllI\\play\\soulShine\\solar-shrine-arduino\\arduino\\01_MAIN_SYSTEM\\solar_shrine_playa\\RobotsEffect.h"
#ifndef ROBOTSEFFECT_H
#define ROBOTSEFFECT_H

#include <Arduino.h>
#include <MozziGuts.h>
#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <mozzi_midi.h>
#include <ADSR.h>

class RobotsEffect {
public:
    RobotsEffect();
    void setup();
    void enter();
    void exit();
    void update(int rightHand, int leftHand, int d1, int d2);
    int audio();

private:
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aSin;
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aSin2;
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aGain;
    ADSR<AUDIO_RATE, AUDIO_RATE> aEnv;
    int step;
    char a, b, c, d, e, f, g, h;
    int notes[8];
    int current_note_index;
    unsigned long note_change_time;
};

#endif // ROBOTSEFFECT_H